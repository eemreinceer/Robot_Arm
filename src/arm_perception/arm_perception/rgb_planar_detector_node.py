#!/usr/bin/env python3
"""YOLO bbox to target-frame object poses using a calibrated table homography.

EYE-IN-HAND UYARISI (2026-07-26): Robot Arm kamerası kolun üzerinde. Tek sabit
homografi yalnızca YAKALANDIĞI eklem pozunda geçerlidir; kol kımıldarsa
piksel→masa eşlemesi değişir ve düğüm sessizce yanlış poz yayınlar. Bu yüzden
kalibrasyon dosyası gözlem pozunu (`observation_joint_positions`) ve toleransı
zorunlu tutar; /joint_states bu pozdan saparsa kare DÜŞÜRÜLÜR.

Kalıcı çözüm (henüz açık iş): ölçülmüş hand-eye transformu + kare başına TF
lookup ile poz-bağımsız projeksiyon. Bu düğüm o gelene kadar "tek gözlem
pozundan bak, sonra hareket et" akışını güvenli tutar.
"""
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from vision_msgs.msg import Detection2DArray

from arm_interfaces.msg import ObjectArray, ObjectPose
from arm_perception.detection_2d import detection_array_from_yolo
from arm_perception.planar_projection import bbox_to_object_position, object_dimensions


class RgbPlanarDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('rgb_planar_detector_node')
        self.declare_parameter('model_path', 'src/arm_perception/models/yolo_arm.pt')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_inference_hz', 10.0)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.latest_joint_positions = None
        (self.homography, self.table_z_m, self.dimensions_by_class,
         self.observation_pose, self.joint_tolerance_rad) = self._load_calibration()
        self.model = self._load_model()
        self.publisher = self.create_publisher(
            ObjectArray, str(self.get_parameter('detections_topic').value), 10)
        self.detections_2d_publisher = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_2d_topic').value), 10)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state, 10)
        self.get_logger().info(
            'RGB planar detector ready (no depth/pointcloud subscription). '
            'EYE-IN-HAND: homografi yalnızca gözlem pozunda geçerli, tolerans '
            '{:.3f} rad, izlenen eklemler: {}'.format(
                self.joint_tolerance_rad, sorted(self.observation_pose)))

    def _load_calibration(self):
        path = Path(str(self.get_parameter('calibration_file').value)).expanduser()
        if not path.is_file():
            raise RuntimeError('calibration_file must point to a measured homography YAML')
        with path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        # Gözlem pozu ZORUNLU: eye-in-hand'de homografi ondan ayrılamaz.
        # Boş/eksik bırakılamaz — ölçülmemiş kalibrasyonla çalışmak yerine
        # düğüm başlamayı reddeder.
        observation_pose = config.get('observation_joint_positions')
        if not isinstance(observation_pose, dict) or not observation_pose:
            raise RuntimeError(
                'calibration_file must list observation_joint_positions '
                '(joint name -> radians): the arm pose the homography was '
                'captured at. The Robot Arm camera is eye-in-hand, so a homography '
                'without its pose is unusable.')
        try:
            observation_pose = {
                str(name): float(value) for name, value in observation_pose.items()}
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                'observation_joint_positions values must be numeric radians') from exc
        tolerance = config.get('joint_tolerance_rad')
        if tolerance is None:
            raise RuntimeError(
                'calibration_file must set joint_tolerance_rad: how far the arm '
                'may sit from the observation pose before the homography is '
                'considered invalid.')
        tolerance = float(tolerance)
        if not tolerance > 0.0:
            raise RuntimeError('joint_tolerance_rad must be positive')
        return (
            config['image_to_target_plane_homography'],
            float(config['table_z_m']),
            config['object_dimensions_m'],
            observation_pose,
            tolerance)

    def _on_joint_state(self, message: JointState) -> None:
        # JointState kısmi olabilir (bazı yayıncılar eklem altkümesi gönderir);
        # gelen isimleri biriktir, silme.
        if self.latest_joint_positions is None:
            self.latest_joint_positions = {}
        for name, position in zip(message.name, message.position):
            self.latest_joint_positions[str(name)] = float(position)

    def _at_observation_pose(self) -> bool:
        """True yalnızca kol homografinin yakalandığı pozdaysa."""
        if self.latest_joint_positions is None:
            self.get_logger().warning(
                'No /joint_states yet: cannot confirm the arm is at the '
                'observation pose, dropping frame (eye-in-hand camera).',
                throttle_duration_sec=5.0)
            return False
        for name, expected in self.observation_pose.items():
            actual = self.latest_joint_positions.get(name)
            if actual is None:
                self.get_logger().warning(
                    'Observation joint {} missing from /joint_states, dropping '
                    'frame'.format(name),
                    throttle_duration_sec=5.0)
                return False
            if abs(actual - expected) > self.joint_tolerance_rad:
                self.get_logger().warning(
                    'Arm off observation pose ({}: {:.3f} vs {:.3f} rad, tol '
                    '{:.3f}); table homography invalid here, dropping frame.'.format(
                        name, actual, expected, self.joint_tolerance_rad),
                    throttle_duration_sec=2.0)
                return False
        return True

    def _load_model(self):
        path = Path(str(self.get_parameter('model_path').value)).expanduser()
        if not path.is_file():
            raise RuntimeError('YOLO model not found: ' + str(path))
        from ultralytics import YOLO
        return YOLO(str(path))

    def _on_image(self, message: Image) -> None:
        max_hz = float(self.get_parameter('max_inference_hz').value)
        now = time.monotonic()
        if max_hz > 0.0 and now - self.last_inference_time < 1.0 / max_hz:
            return
        if not self._at_observation_pose():
            return
        self.last_inference_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            results = self.model.predict(
                image, verbose=False,
                conf=float(self.get_parameter('confidence_threshold').value))
            self.detections_2d_publisher.publish(
                detection_array_from_yolo(message.header, results))
            output = ObjectArray()
            output.header.stamp = message.header.stamp
            output.header.frame_id = str(self.get_parameter('target_frame').value)
            if results and results[0].boxes is not None:
                for index, box in enumerate(results[0].boxes):
                    class_id = int(box.cls[0].item())
                    class_name = str(results[0].names[class_id])
                    dimensions = object_dimensions(class_name, self.dimensions_by_class)
                    bbox = np.asarray(box.xyxy[0].cpu(), dtype=np.float64).tolist()
                    position = bbox_to_object_position(
                        bbox, self.homography, self.table_z_m, dimensions[2])
                    detected = ObjectPose()
                    detected.object_id = '{}_{}'.format(class_name, index)
                    detected.object_type = class_name
                    detected.confidence = float(box.conf[0].item())
                    detected.dimensions = list(dimensions)
                    detected.pose.header = output.header
                    detected.pose.pose.position.x = position[0]
                    detected.pose.pose.position.y = position[1]
                    detected.pose.pose.position.z = position[2]
                    detected.pose.pose.orientation.w = 1.0
                    output.objects.append(detected)
            self.publisher.publish(output)
        except (KeyError, ValueError) as exc:
            self.get_logger().warning(
                'Dropping RGB detection without class calibration: {}'.format(exc),
                throttle_duration_sec=2.0)
        except Exception as exc:
            self.get_logger().error(
                'RGB planar perception frame failed: {}'.format(exc),
                throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbPlanarDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
