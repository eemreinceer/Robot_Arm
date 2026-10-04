from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from arm_interfaces.msg import ObjectPose
from arm_perception.yolo_detector_node import YoloDetectorNode


def test_object_marker_matches_detected_object_pose_and_dimensions():
    detected = ObjectPose()
    detected.object_type = "red_box"
    detected.pose.header.frame_id = "base_link"
    detected.pose.pose.position.x = 0.12
    detected.pose.pose.position.y = -0.34
    detected.pose.pose.position.z = 0.56
    detected.pose.pose.orientation.w = 1.0
    detected.dimensions = [0.11, 0.12, 0.13]

    marker = YoloDetectorNode._object_marker(detected, marker_id=7)

    assert marker.header.frame_id == "base_link"
    assert marker.ns == "detected_objects"
    assert marker.id == 7
    assert marker.type == Marker.CUBE
    assert marker.action == Marker.ADD
    assert marker.pose.position.x == detected.pose.pose.position.x
    assert marker.pose.position.y == detected.pose.pose.position.y
    assert marker.pose.position.z == detected.pose.pose.position.z
    assert marker.scale.x == detected.dimensions[0]
    assert marker.scale.y == detected.dimensions[1]
    assert marker.scale.z == detected.dimensions[2]
    assert marker.color.r > marker.color.g


def test_grasp_marker_uses_pose_stamped_frame_and_pose():
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.20
    pose.pose.position.y = -0.10
    pose.pose.position.z = 0.30
    pose.pose.orientation.w = 1.0

    marker = YoloDetectorNode._grasp_marker(
        pose, marker_id=1, name="grasp", color=(0.1, 1.0, 0.1, 0.95))

    assert marker.header.frame_id == "base_link"
    assert marker.ns == "grasp_pose"
    assert marker.id == 1
    assert marker.type == Marker.ARROW
    assert marker.pose.position.x == pose.pose.position.x
    assert marker.pose.position.y == pose.pose.position.y
    assert marker.pose.position.z == pose.pose.position.z
    assert marker.color.a == 0.95
