#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class TestGazeboSpawn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_joint_states_received(self):
        node = Node('test_gazebo_spawn_node')
        joint_states_msg = None

        def callback(msg):
            nonlocal joint_states_msg
            joint_states_msg = msg

        sub = node.create_subscription(JointState, '/joint_states', callback, 10)
        
        # Spin for up to 10 seconds to receive a message
        start_time = time.time()
        while time.time() - start_time < 10.0 and joint_states_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()
        
        self.assertIsNotNone(joint_states_msg, "Failed to receive /joint_states message within 10s")
        # Check that we have joint names
        self.assertTrue(len(joint_states_msg.name) >= 6, "Expected at least 6 joints")

if __name__ == '__main__':
    unittest.main()
