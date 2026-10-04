#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pytest
import time
from sensor_msgs.msg import Image

try:
    from arm_interfaces.msg import ObjectArray
except ImportError:
    ObjectArray = None

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from gz_gt import spawn_model, remove_model

class YoloTestNode(Node):
    def __init__(self):
        super().__init__('test_yolo_detection')
        self.detections = []
        if ObjectArray is not None:
            self.sub = self.create_subscription(
                ObjectArray,
                '/detected_objects',
                self.callback,
                10
            )

    def callback(self, msg):
        self.detections.append(msg)

@pytest.fixture
def rclpy_setup():
    rclpy.init()
    node = YoloTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_yolo_dependencies():
    assert ObjectArray is not None, "arm_interfaces/msg/ObjectArray not found. Build arm_interfaces."

def test_yolo_detection_contract(rclpy_setup):
    """
    Test that YOLO detects objects  
    and complies with the ObjectArray contract.
    """
    node = rclpy_setup
    
    # Spawn objects for testing
    spawn_model('red_box', 0.4, 0.1, 0.65, 0, 0, 0)
    spawn_model('yellow_cylinder', 0.3, -0.2, 0.65, 0, 0, 0)
    
    # Wait for up to 5 seconds to receive a message
    start = time.time()
    while time.time() - start < 5.0 and len(node.detections) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    remove_model('red_box')
    remove_model('yellow_cylinder')
        
    assert len(node.detections) > 0, "No /detected_objects messages received."
        
    msg = node.detections[-1]
    assert msg.header.frame_id == "base_link", f"Invalid frame_id: {msg.header.frame_id}"
    assert len(msg.objects) > 0, "No objects detected in the scene."
    
    found_box = False
    found_cyl = False
    
    for obj in msg.objects:
        assert 0.0 <= obj.confidence <= 1.0, f"Confidence out of bounds: {obj.confidence}"
        assert obj.dimensions.x > 0 and obj.dimensions.y > 0 and obj.dimensions.z > 0, "Invalid dimensions"
        
        if obj.object_type == 'red_box':
            found_box = True
        elif obj.object_type == 'yellow_cylinder':
            found_cyl = True
            
    assert found_box, "red_box was not detected"
    assert found_cyl, "yellow_cylinder was not detected"
