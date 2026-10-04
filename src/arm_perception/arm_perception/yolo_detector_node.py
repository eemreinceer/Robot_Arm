#!/usr/bin/env python3
"""YOLO bbox + organized point cloud PCA based 6DOF object pose publisher."""
from __future__ import annotations

from pathlib import Path
import copy
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from arm_interfaces.msg import ObjectArray, ObjectPose
from arm_interfaces.srv import GetPickPose
from arm_perception.detection_2d import detection_array_from_yolo
from arm_perception.math3d import (
    homogeneous,
    matrix_from_quaternion,
    matrix_from_rpy,
    quaternion_from_matrix,
    transform_from_msg,
)
from arm_perception.pose_estimation import (
    camera_link_to_optical_points,
    estimate_pose,
    points_from_bbox,
)


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')
        self.declare_parameter('model_path', 'src/arm_perception/models/yolo_arm.pt')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('points_topic', '/camera/points')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('detection_markers_topic', '/detection_markers')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_band_m', 0.08)
        self.declare_parameter('points_are_camera_link_convention', True)
        self.declare_parameter('minimum_cluster_points', 12)
        self.declare_parameter('max_inference_hz', 10.0)
        self.declare_parameter('pick_pose_service', '/get_pick_pose')
        self.declare_parameter('pre_pick_offset_m', 0.10)
        self.declare_parameter('link6_to_grasp_xyz', [0.0075, -0.0032, 0.1023])
        self.declare_parameter('link6_to_grasp_rpy', [-1.5708, 1.0809, 1.3594])
        # Validated top-down grasp orientations covering the object zone. A
        # single fixed wrist orientation is not reachable everywhere (this 6-DOF
        # arm has limited dexterity), so we select the nearest FK-validated
        # near-vertical grasp to the object (pick-side analog of the bins'
        # drop_link6_quaternion). The fallback quaternion is the Faz-1 calibrated
        # grasp, used only if the table is missing.
        self.declare_parameter(
            'grasp_orientations_file',
            os.path.join(
                get_package_share_directory('arm_perception'),
                'config', 'grasp_orientations.yaml'))
        self.declare_parameter(
            'top_grasp_link6_quaternion', [0.749593, 0.605798, 0.207415, 0.167625])
        self.grasp_orientations = self._load_grasp_orientations(
            str(self.get_parameter('grasp_orientations_file').value))

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_points: Optional[PointCloud2] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.cached_objects: Dict[str, ObjectPose] = {}
        self.latest_pick_pose: Optional[PoseStamped] = None
        self.latest_pre_pick_pose: Optional[PoseStamped] = None
        self.last_inference_time = 0.0
        self.model = self._load_model()

        self.publisher = self.create_publisher(
            ObjectArray, self.get_parameter('detections_topic').value, 10)
        self.detections_2d_publisher = self.create_publisher(
            Detection2DArray, self.get_parameter('detections_2d_topic').value, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter('detection_markers_topic').value, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self._on_points, qos_profile_sensor_data)
        self.create_service(
            GetPickPose, self.get_parameter('pick_pose_service').value,
            self._get_pick_pose)
        self.get_logger().info(
            'Perception node ready; publishing timestamp-aligned 2D/3D detections '
            'and RViz markers')

    def _load_model(self):
        path = Path(str(self.get_parameter('model_path').value)).expanduser()
        if not path.is_file():
            self.get_logger().warning(
                f'YOLO model not found at {path}; node stays alive without inference')
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(str(path))
            self.get_logger().info(f'Loaded YOLO model: {path}')
            return model
        except Exception as exc:  # runtime dependency or model issue
            self.get_logger().error(f'Unable to load YOLO model {path}: {exc}')
            return None

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.latest_camera_info = message

    def _on_points(self, message: PointCloud2) -> None:
        self.latest_points = message

    def _on_image(self, message: Image) -> None:
        if self.model is None:
            return
        max_hz = float(self.get_parameter('max_inference_hz').value)
        now = time.monotonic()
        if max_hz > 0.0 and now - self.last_inference_time < 1.0 / max_hz:
            return
        self.last_inference_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            results = self.model.predict(
                image, verbose=False,
                conf=float(self.get_parameter('confidence_threshold').value))
            self.detections_2d_publisher.publish(
                detection_array_from_yolo(message.header, results))
            if self.latest_points is None:
                self.get_logger().warning(
                    'No point cloud yet; publishing 2D detections only',
                    throttle_duration_sec=5.0)
                return
            cloud = self.latest_points
            xyz = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            xyz = np.asarray(xyz, dtype=np.float64).reshape((-1, 3))
            if cloud.height <= 1 or xyz.shape[0] != cloud.width * cloud.height:
                self.get_logger().warning(
                    'Point cloud is not organized; skipping frame', throttle_duration_sec=5.0)
                return
            xyz = xyz.reshape((cloud.height, cloud.width, 3))
            # Gazebo RGB-D publishes native +X-forward coordinates while the
            # bridged header names the REP-103 optical frame. Normalize before
            # depth filtering, PCA and TF application.
            if bool(self.get_parameter('points_are_camera_link_convention').value):
                xyz = camera_link_to_optical_points(xyz)
            detected = self._objects_from_results(message, cloud, xyz, results)
            self.cached_objects = {item.object_id: item for item in detected.objects}
            self.publisher.publish(detected)
            self._publish_detection_markers(detected)
        except TransformException as exc:
            self.get_logger().warning(f'Camera TF unavailable: {exc}', throttle_duration_sec=5.0)
        except Exception as exc:
            self.get_logger().error(f'Perception frame failed: {exc}', throttle_duration_sec=2.0)

    def _objects_from_results(self, image: Image, cloud: PointCloud2, xyz, results) -> ObjectArray:
        target_frame = str(self.get_parameter('target_frame').value)
        tf_msg = self.tf_buffer.lookup_transform(target_frame, cloud.header.frame_id, Time())
        target_from_camera = transform_from_msg(tf_msg.transform)
        output = ObjectArray()
        output.header.stamp = image.header.stamp
        output.header.frame_id = target_frame
        if not results:
            return output
        names = results[0].names
        boxes = results[0].boxes
        if boxes is None:
            return output
        for index, box in enumerate(boxes):
            bounds = np.asarray(box.xyxy[0].cpu(), dtype=np.float64).tolist()
            cluster = points_from_bbox(
                xyz, cloud.width, cloud.height, bounds,
                depth_band_m=float(self.get_parameter('depth_band_m').value))
            try:
                pose = estimate_pose(
                    cluster,
                    minimum_points=int(self.get_parameter('minimum_cluster_points').value))
            except ValueError as exc:
                class_id = int(box.cls[0].item())
                class_name = str(names[class_id])
                self.get_logger().warning(
                    f'Dropping {class_name} bbox: cluster_points={cluster.shape[0]} '
                    f'bounds={bounds}: {exc}',
                    throttle_duration_sec=2.0)
                continue
            camera_from_object = homogeneous(
                pose.position, matrix_from_quaternion(pose.quaternion))
            target_from_object = target_from_camera @ camera_from_object
            class_id = int(box.cls[0].item())
            class_name = str(names[class_id])
            object_pose = ObjectPose()
            object_pose.object_id = f'{class_name}_{index}'
            object_pose.object_type = class_name
            object_pose.confidence = float(box.conf[0].item())
            object_pose.dimensions = pose.dimensions.astype(np.float32).tolist()
            object_pose.pose.header = output.header
            self._set_pose(object_pose.pose, target_from_object)
            output.objects.append(object_pose)
        return output

    def _get_pick_pose(self, request: GetPickPose.Request, response: GetPickPose.Response):
        detected = self.cached_objects.get(request.object_id)
        if detected is None:
            response.success = False
            response.message = f'Object not found in latest detections: {request.object_id}'
            return response
        target_from_object = self._pose_to_transform(detected.pose)
        if request.preferred_approach == 'top':
            # Use a reachable DOWNWARD wrist orientation (not the PCA orientation,
            # which is unreachable) and place the grasp_link frame ON the object.
            # Link_6 = grasp_target * inv(link6_to_grasp), so the gripper actually
            # closes over the object instead of pointing in an arbitrary
            # direction. The orientation is the nearest FK-validated near-vertical
            # grasp to the object; pick_place_node yaw-sweeps for the final angle.
            object_position = target_from_object[:3, 3]
            grasp_quaternion = self._select_grasp_quaternion(
                float(object_position[0]), float(object_position[1]))
            grasp_rotation = matrix_from_quaternion(grasp_quaternion)
            link6_to_grasp_translation = np.asarray(
                self.get_parameter('link6_to_grasp_xyz').value, dtype=np.float64)
            link6_position = object_position - grasp_rotation @ link6_to_grasp_translation
            target_from_link6 = homogeneous(link6_position, grasp_rotation)
        else:
            link6_to_grasp = homogeneous(
                np.asarray(self.get_parameter('link6_to_grasp_xyz').value, dtype=np.float64),
                matrix_from_rpy(*self.get_parameter('link6_to_grasp_rpy').value))
            target_from_link6 = target_from_object @ np.linalg.inv(link6_to_grasp)
        response.pick_pose = PoseStamped()
        response.pick_pose.header = detected.pose.header
        self._set_pose(response.pick_pose, target_from_link6)
        response.pre_pick_pose = PoseStamped()
        response.pre_pick_pose.header = detected.pose.header
        response.pre_pick_pose.pose = copy.deepcopy(response.pick_pose.pose)
        response.pre_pick_pose.pose.position.z += float(
            self.get_parameter('pre_pick_offset_m').value)
        self.latest_pick_pose = copy.deepcopy(response.pick_pose)
        self.latest_pre_pick_pose = copy.deepcopy(response.pre_pick_pose)
        self._publish_detection_markers(self._cached_object_array(detected.pose.header))
        response.success = True
        response.message = 'Pick pose resolved in Link_6 frame'
        return response

    def _cached_object_array(self, header) -> ObjectArray:
        output = ObjectArray()
        output.header = header
        output.objects.extend(self.cached_objects.values())
        return output

    def _publish_detection_markers(self, detections: ObjectArray) -> None:
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        for index, detected in enumerate(detections.objects):
            markers.markers.append(self._object_marker(detected, index))
        if self.latest_pre_pick_pose is not None:
            markers.markers.append(self._grasp_marker(
                self.latest_pre_pick_pose, marker_id=0, name="pre_grasp",
                color=(0.0, 0.85, 1.0, 0.9)))
        if self.latest_pick_pose is not None:
            markers.markers.append(self._grasp_marker(
                self.latest_pick_pose, marker_id=1, name="grasp",
                color=(0.1, 1.0, 0.1, 0.95)))
        self.marker_publisher.publish(markers)

    @staticmethod
    def _object_marker(detected: ObjectPose, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = detected.pose.header
        marker.ns = "detected_objects"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = detected.pose.pose
        dimensions = YoloDetectorNode._marker_dimensions(detected.dimensions)
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]
        r, g, b, a = YoloDetectorNode._class_color(detected.object_type)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.lifetime.sec = 1
        return marker

    @staticmethod
    def _grasp_marker(
        pose_stamped: PoseStamped, marker_id: int, name: str,
        color: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "grasp_pose"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.08
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = name
        marker.lifetime.sec = 1
        return marker

    @staticmethod
    def _marker_dimensions(dimensions) -> Tuple[float, float, float]:
        fallback = (0.05, 0.05, 0.05)
        if len(dimensions) < 3:
            return fallback
        values = tuple(float(value) for value in dimensions[:3])
        if any(value <= 0.0 for value in values):
            return fallback
        return values

    @staticmethod
    def _class_color(object_type: str) -> Tuple[float, float, float, float]:
        colors = {
            "red_box": (1.0, 0.05, 0.03, 0.55),
            "yellow_cylinder": (1.0, 0.85, 0.02, 0.55),
            "blue_cube": (0.05, 0.25, 1.0, 0.55),
        }
        return colors.get(object_type, (0.7, 0.7, 0.7, 0.45))

    def _load_grasp_orientations(
        self, path: str) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Load the validated (at_xy, quaternion) grasp-orientation table.

        Returns a list of (xy[2], quaternion[xyzw]); empty if the file is
        missing or malformed, in which case the fallback quaternion is used."""
        try:
            document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
            entries = []
            for item in document['grasp_orientations']:
                xy = np.asarray(item['at_xy'], dtype=np.float64)
                quaternion = np.asarray(item['quaternion'], dtype=np.float64)
                entries.append((xy, quaternion))
            self.get_logger().info(
                f'Loaded {len(entries)} validated grasp orientations from {path}')
            return entries
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f'Could not load grasp orientations ({exc}); using fallback quaternion')
            return []

    def _select_grasp_quaternion(self, x: float, y: float) -> np.ndarray:
        """Nearest validated grasp orientation to the object (x, y), or fallback."""
        if not self.grasp_orientations:
            return np.asarray(
                self.get_parameter('top_grasp_link6_quaternion').value, dtype=np.float64)
        target = np.array([x, y], dtype=np.float64)
        xy, quaternion = min(
            self.grasp_orientations, key=lambda e: float(np.linalg.norm(e[0] - target)))
        return quaternion

    @staticmethod
    def _pose_to_transform(message: PoseStamped) -> np.ndarray:
        pose = message.pose
        return homogeneous(
            [pose.position.x, pose.position.y, pose.position.z],
            matrix_from_quaternion([
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]))

    @staticmethod
    def _set_pose(message: PoseStamped, transform: np.ndarray) -> None:
        position = transform[:3, 3]
        quaternion = quaternion_from_matrix(transform[:3, :3])
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
