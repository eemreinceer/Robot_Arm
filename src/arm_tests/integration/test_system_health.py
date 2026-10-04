#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from arm_interfaces.action import PickAndPlace
import time
import threading

class TestSystemHealth(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_system_health_node')
        self.msg_times = []
        self.lock = threading.Lock()

        # Subscribe to joint states to measure frequency
        self.joint_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10
        )

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spinning in a background thread
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.spin_thread.join(timeout=1.0)

    def _joint_callback(self, msg):
        with self.lock:
            self.msg_times.append(time.time())

    def test_joint_states_frequency(self):
        self.node.get_logger().info("Waiting for /joint_states publishers...")
        start_wait = time.time()
        while self.node.count_publishers('/joint_states') == 0:
            if time.time() - start_wait > 10.0:
                self.fail("Timed out waiting for /joint_states publishers to connect")
            time.sleep(0.1)

        self.node.get_logger().info("Measuring /joint_states publication frequency over 2.0 seconds...")
        with self.lock:
            self.msg_times.clear()
        
        # Wait for 2 seconds to accumulate messages
        time.sleep(2.0)
        
        with self.lock:
            times = list(self.msg_times)
            
        self.node.get_logger().info(f"Received {len(times)} joint state messages")
        self.assertTrue(len(times) > 0, "No /joint_states messages received at all")
        
        if len(times) >= 2:
            time_span = times[-1] - times[0]
            if time_span > 0:
                frequency = (len(times) - 1) / time_span
                self.node.get_logger().info(f"Measured joint state frequency: {frequency:.2f} Hz")
                self.assertTrue(frequency >= 10.0, f"Expected /joint_states frequency to be >= 10Hz, but it was {frequency:.2f}Hz")
            else:
                self.fail("Time span between messages was 0")
        else:
            self.fail("Not enough messages to calculate frequency (need at least 2)")

    def test_move_group_and_pick_place_availability(self):
        # 1. Check pick_place_node action server (/pick_and_place)
        self.node.get_logger().info("Checking if /pick_and_place action server is available...")
        pick_place_client = ActionClient(self.node, PickAndPlace, '/pick_and_place')
        pick_place_ready = pick_place_client.wait_for_server(timeout_sec=5.0)
        
        self.node.get_logger().info(f"pick_place_node action server availability: {pick_place_ready}")
        self.assertTrue(pick_place_ready, "pick_place_node action server /pick_and_place is not available")

        # 2. Check move_group service (/compute_ik)
        self.node.get_logger().info("Checking if move_group /compute_ik service is available...")
        ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        ik_ready = ik_client.wait_for_service(timeout_sec=5.0)
        
        self.node.get_logger().info(f"move_group /compute_ik service availability: {ik_ready}")
        self.assertTrue(ik_ready, "move_group /compute_ik service is not available")

if __name__ == '__main__':
    unittest.main()
