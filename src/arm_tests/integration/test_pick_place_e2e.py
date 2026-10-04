#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from arm_interfaces.action import PickAndPlace
import time
import threading

class TestPickPlaceE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_pick_place_e2e_node')
        self.initial_joints = None
        self.latest_joints = None
        self.max_joints_seen = None
        self.min_joints_seen = None
        self.received_phases = []
        self.lock = threading.Lock()

        # Subscribe to joint states to monitor movement
        self.joint_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10
        )

        self.action_client = ActionClient(self.node, PickAndPlace, '/pick_and_place')

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
            # We only care about the 6 arm joints
            arm_joint_indices = []
            arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            # Map names to values
            joint_values = []
            for name in arm_joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_values.append(msg.position[idx])
            
            if len(joint_values) == 6:
                if self.initial_joints is None:
                    self.initial_joints = joint_values
                self.latest_joints = joint_values

                # Track min and max joints seen during test
                if self.max_joints_seen is None:
                    self.max_joints_seen = list(joint_values)
                    self.min_joints_seen = list(joint_values)
                else:
                    for idx in range(6):
                        self.max_joints_seen[idx] = max(self.max_joints_seen[idx], joint_values[idx])
                        self.min_joints_seen[idx] = min(self.min_joints_seen[idx], joint_values[idx])

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"Feedback received: Phase: {feedback.current_phase}, Progress: {feedback.progress:.2f}")
        with self.lock:
            self.received_phases.append(feedback.current_phase)

    def test_e2e_pick_and_place(self):
        # 1. Wait for action server
        self.node.get_logger().info("Waiting for /pick_and_place action server...")
        server_available = self.action_client.wait_for_server(timeout_sec=10.0)
        self.assertTrue(server_available, "Action server /pick_and_place not available")

        # 2. Wait to receive initial joint states
        self.node.get_logger().info("Waiting for initial joint states...")
        start_wait = time.time()
        while time.time() - start_wait < 5.0:
            with self.lock:
                if self.initial_joints is not None:
                    break
            time.sleep(0.1)
        
        with self.lock:
            self.assertIsNotNone(self.initial_joints, "Failed to get initial joint states")
            initial_pos = list(self.initial_joints)
        
        self.node.get_logger().info(f"Initial arm joint positions: {initial_pos}")

        # 3. Create Goal
        goal = PickAndPlace.Goal()
        goal.object_id = 'test_object'
        
        # Calibrated workspace positions for Link_6 (theta = 90.0)
        goal.pick_pose.position.x = 0.253227
        goal.pick_pose.position.y = -0.007999
        goal.pick_pose.position.z = 0.168658
        goal.pick_pose.orientation.x = 0.749593
        goal.pick_pose.orientation.y = 0.605798
        goal.pick_pose.orientation.z = 0.207415
        goal.pick_pose.orientation.w = 0.167625

        goal.place_pose.position.x = 0.403227
        goal.place_pose.position.y = 0.242001
        goal.place_pose.position.z = 0.168658
        goal.place_pose.orientation.x = 0.749593
        goal.place_pose.orientation.y = 0.605798
        goal.place_pose.orientation.z = 0.207415
        goal.place_pose.orientation.w = 0.167625

        goal.speed_scale = 0.5
        goal.return_home = True

        # 4. Send Goal
        self.node.get_logger().info("Sending goal to /pick_and_place...")
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        
        # Wait for goal handle
        start_wait = time.time()
        while not send_goal_future.done():
            if time.time() - start_wait > 5.0:
                self.fail("Timed out waiting for goal handle from action server")
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "Goal was rejected by action server")
        self.node.get_logger().info("Goal accepted by action server")

        # 5. Wait for result (timeout 60s)
        self.node.get_logger().info("Waiting for action result...")
        get_result_future = goal_handle.get_result_async()
        
        start_action = time.time()
        action_timeout = 60.0
        while not get_result_future.done():
            if time.time() - start_action > action_timeout:
                self.node.get_logger().error("Action timed out! Attempting to cancel goal...")
                cancel_future = goal_handle.cancel_goal_async()
                time.sleep(2.0)
                self.fail(f"Action did not complete within {action_timeout} seconds")
            time.sleep(0.2)

        action_result = get_result_future.result()
        result = action_result.result
        
        self.node.get_logger().info(f"Action finished. Success: {result.success}, Message: {result.message}")
        self.assertTrue(result.success, f"Action failed with message: {result.message}")

        # 6. Verify feedback phases were received
        with self.lock:
            phases = list(self.received_phases)
        self.node.get_logger().info(f"Phases traversed: {phases}")
        
        # Check that we received key phases
        self.assertTrue(len(phases) > 0, "No feedback phases were received")
        self.assertIn("moving_to_pre_pick", phases, "Phase 'moving_to_pre_pick' not traversed")
        self.assertIn("closing_gripper", phases, "Phase 'closing_gripper' not traversed")
        self.assertIn("moving_to_pre_place", phases, "Phase 'moving_to_pre_place' not traversed")
        self.assertIn("opening_gripper", phases, "Phase 'opening_gripper' not traversed")

        # 7. Check if joint states changed (verifying movement)
        with self.lock:
            max_seen = list(self.max_joints_seen) if self.max_joints_seen is not None else None
            min_seen = list(self.min_joints_seen) if self.min_joints_seen is not None else None
            
        self.assertIsNotNone(max_seen, "Failed to record max joint values during test")
        self.assertIsNotNone(min_seen, "Failed to record min joint values during test")
        
        # Calculate joint ranges (max - min) to ensure robot moved during execution
        ranges = [max_val - min_val for max_val, min_val in zip(max_seen, min_seen)]
        max_range = max(ranges)
        self.node.get_logger().info(f"Max joint range of motion: {max_range:.4f} rad")
        self.assertTrue(max_range > 0.05, f"Expected robot to move, but max joint range of motion was only {max_range:.4f} rad")

if __name__ == '__main__':
    unittest.main()
