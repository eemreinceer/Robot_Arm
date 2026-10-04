#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pytest
import time
import math

try:
    from arm_interfaces.msg import ObjectArray
except ImportError:
    ObjectArray = None

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from gz_gt import spawn_model, remove_model, get_gt_pose_in_base_link, calculate_pose_error, get_quat_from_rpy

class PoseEstimationTestNode(Node):
    def __init__(self):
        super().__init__('test_3d_pose_estimation')
        self.latest_msg = None
        if ObjectArray is not None:
            self.sub = self.create_subscription(
                ObjectArray,
                '/detected_objects',
                self.callback,
                10
            )

    def callback(self, msg):
        self.latest_msg = msg

@pytest.fixture
def rclpy_setup():
    rclpy.init()
    node = PoseEstimationTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.mark.parametrize("model_name, world_pos, rpy", [
    # Red Box yaw variations
    ('red_box', (0.4, 0.1, 0.65), (0.0, 0.0, 0.0)),
    ('red_box', (0.4, 0.1, 0.65), (0.0, 0.0, math.pi/4)),
    ('red_box', (0.4, 0.1, 0.65), (0.0, 0.0, math.pi/2)),
    
    # Blue Cube yaw variations
    ('blue_cube', (0.5, 0.0, 0.65), (0.0, 0.0, 0.0)),
    ('blue_cube', (0.5, 0.0, 0.65), (0.0, 0.0, math.pi/4)),
    ('blue_cube', (0.5, 0.0, 0.65), (0.0, 0.0, math.pi/2)),
    
    # Yellow Cylinder tilt variations (yaw-symmetric, testing tilting)
    ('yellow_cylinder', (0.3, -0.2, 0.65), (0.0, 0.0, 0.0)),
    ('yellow_cylinder', (0.3, -0.2, 0.65), (math.pi/12, 0.0, 0.0)),  # 15 degrees roll tilt
    ('yellow_cylinder', (0.3, -0.2, 0.65), (0.0, math.pi/12, 0.0)),  # 15 degrees pitch tilt
])
def test_3d_pose_accuracy(rclpy_setup, model_name, world_pos, rpy):
    node = rclpy_setup
    
    x, y, z = world_pos
    roll, pitch, yaw = rpy
    spawn_model(model_name, x, y, z, roll, pitch, yaw)
    
    start = time.time()
    node.latest_msg = None
    detected_obj = None
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_msg:
            for obj in node.latest_msg.objects:
                if obj.object_type == model_name:
                    detected_obj = obj
                    break
        if detected_obj:
            break
            
    remove_model(model_name)
    
    assert detected_obj is not None, f"Model {model_name} was spawned but not detected by YOLO."
    
    gt_pos, gt_euler = get_gt_pose_in_base_link(x, y, z, roll, pitch, yaw)
    gt_quat = get_quat_from_rpy(roll, pitch, yaw)
    
    pred_pos = (detected_obj.pose.pose.position.x, detected_obj.pose.pose.position.y, detected_obj.pose.pose.position.z)
    pred_quat = (detected_obj.pose.pose.orientation.x, detected_obj.pose.pose.orientation.y, 
                 detected_obj.pose.pose.orientation.z, detected_obj.pose.pose.orientation.w)
                 
    pos_err, ori_err = calculate_pose_error(pred_pos, pred_quat, gt_pos, gt_quat, model_name)
    
    assert pos_err < 20.0, f"Position error too high for {model_name}: {pos_err:.2f} mm"
    assert ori_err < 15.0, f"Orientation error too high for {model_name}: {ori_err:.2f} deg"
