#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from arm_interfaces.action import PickAndPlace
from geometry_msgs.msg import Pose
import time

class TestPickAndPlace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_pick_and_place_client')
        self.action_client = ActionClient(self.node, PickAndPlace, 'pick_and_place')

    def tearDown(self):
        self.node.destroy_node()

    def test_action_server_exists(self):
        # Check if action server is running (with a short timeout)
        server_exists = self.action_client.wait_for_server(timeout_sec=1.0)
        self.node.get_logger().info(f"Action server status: {'Available' if server_exists else 'Unavailable'}")
        
    def test_nominal_pick_place(self):
        # Nominale pick place testi
        # pick_pose: (0.45, 0.0, 0.625)
        # place_pose: (0.45, 0.25, 0.625)
        self.node.get_logger().info("Scaffolding nominal pick and place test.")
        
    def test_action_preemption(self):
        # Preemption testi
        self.node.get_logger().info("Scaffolding action preemption test.")

if __name__ == '__main__':
    unittest.main()
