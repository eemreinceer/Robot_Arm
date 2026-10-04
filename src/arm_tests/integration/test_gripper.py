#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import ParallelGripperCommand
import time
import threading

class TestGripper(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_gripper_node')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Start spinning in a background thread
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.latest_gripper_pos = None
        self.lock = threading.Lock()
        self.action_client = ActionClient(self.node, ParallelGripperCommand, '/gripper_controller/gripper_cmd')

        # Subscribe to joint states to monitor gripper joint
        self.joint_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10
        )

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.spin_thread.join(timeout=1.0)

    def _joint_callback(self, msg):
        with self.lock:
            # Look for gripper joint (usually gripper_joint1, gripper_joint_1, or Gripper_joint1)
            for name, pos in zip(msg.name, msg.position):
                if 'gripper' in name.lower() and ('joint1' in name.lower() or 'joint_1' in name.lower()):
                    self.latest_gripper_pos = pos
                    break

    def test_gripper_open_close(self):
        # 1. Wait for action server
        self.node.get_logger().info("Waiting for gripper action server...")
        server_available = self.action_client.wait_for_server(timeout_sec=10.0)
        self.assertTrue(server_available, "Gripper action server not available")

        # 2. Wait to receive initial joint state
        self.node.get_logger().info("Waiting for initial gripper state...")
        start_wait = time.time()
        while time.time() - start_wait < 5.0:
            with self.lock:
                if self.latest_gripper_pos is not None:
                    break
            time.sleep(0.1)

        with self.lock:
            self.assertIsNotNone(self.latest_gripper_pos, "Failed to get initial gripper joint position")
            initial_pos = self.latest_gripper_pos

        self.node.get_logger().info(f"Initial gripper joint position: {initial_pos:.4f} rad")

        # 3. Send Close command
        self.node.get_logger().info("Sending Close command to gripper (position=0.0)...")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['gripper_joint1']
        goal.command.position = [0.0]
        goal.command.effort = [50.0]

        send_goal_future = self.action_client.send_goal_async(goal)
        
        # Wait for goal handle
        start_wait = time.time()
        while not send_goal_future.done():
            if time.time() - start_wait > 5.0:
                self.fail("Timed out waiting for goal handle from gripper action server")
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "Gripper goal was rejected by action server")

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        start_wait = time.time()
        while not get_result_future.done():
            if time.time() - start_wait > 10.0:
                self.fail("Timed out waiting for gripper Close result")
            time.sleep(0.1)

        result = get_result_future.result().result
        self.node.get_logger().info(f"Gripper Close finished. Success/Reached: {result.reached_goal}")

        # Verify joint position is close to 0.0
        time.sleep(1.0)  # Wait for physics to settle
        with self.lock:
            closed_pos = self.latest_gripper_pos

        self.node.get_logger().info(f"Gripper closed position: {closed_pos:.4f} rad")
        # Checking ±0.01 tolerance from 0.0
        self.assertAlmostEqual(closed_pos, 0.0, delta=0.01, msg=f"Gripper did not close fully. Current pos: {closed_pos:.4f}")

        # 4. Send Open command
        self.node.get_logger().info("Sending Open command to gripper (position=0.05)...")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['gripper_joint1']
        goal.command.position = [0.05]
        goal.command.effort = [30.0]

        send_goal_future = self.action_client.send_goal_async(goal)
        
        # Wait for goal handle
        start_wait = time.time()
        while not send_goal_future.done():
            if time.time() - start_wait > 5.0:
                self.fail("Timed out waiting for goal handle from gripper action server")
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "Gripper goal was rejected by action server")

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        start_wait = time.time()
        while not get_result_future.done():
            if time.time() - start_wait > 10.0:
                self.fail("Timed out waiting for gripper Open result")
            time.sleep(0.1)

        result = get_result_future.result().result
        self.node.get_logger().info(f"Gripper Open finished. Success/Reached: {result.reached_goal}")

        # Verify joint position is close to 0.05
        time.sleep(1.0)  # Wait for physics to settle
        with self.lock:
            opened_pos = self.latest_gripper_pos

        self.node.get_logger().info(f"Gripper opened position: {opened_pos:.4f} rad")
        # Checking ±0.01 tolerance from 0.05
        self.assertAlmostEqual(opened_pos, 0.05, delta=0.01, msg=f"Gripper did not open fully. Current pos: {opened_pos:.4f}")

if __name__ == '__main__':
    unittest.main()
