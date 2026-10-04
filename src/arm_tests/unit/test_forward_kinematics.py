#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

# Import custom FK service once generated
try:
    from arm_interfaces.srv import SolveFk
except ImportError:
    SolveFk = None

class TestForwardKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_forward_kinematics_node')
        cls.fk_client = cls.node.create_client(GetPositionFK, '/compute_fk')
        if SolveFk is not None:
            cls.custom_fk_client = cls.node.create_client(SolveFk, '/fk_solve')
        else:
            cls.custom_fk_client = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Wait for services to be ready before each test to ensure discovery
        self.fk_client.wait_for_service(timeout_sec=5.0)
        if self.custom_fk_client is not None:
            self.custom_fk_client.wait_for_service(timeout_sec=5.0)

    def _call_moveit_fk(self, joint_angles, link_name='Link_6'):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [link_name]
        
        joint_state = JointState()
        joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_state.position = [float(val) for val in joint_angles]
        
        req.robot_state.joint_state = joint_state
        
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            res = future.result()
            if res and res.error_code.val == 1 and len(res.pose_stamped) > 0:
                return res.pose_stamped[0].pose
        return None

    def _call_custom_fk(self, joint_angles):
        if self.custom_fk_client is None:
            self.skipTest("SolveFk service type not imported (arm_interfaces not built with Phase 2 definitions)")
            
        req = SolveFk.Request()
        req.joint_angles = [float(val) for val in joint_angles]
        
        future = self.custom_fk_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            res = future.result()
            if res and res.success:
                return res.tcp_pose.pose
        return None

    def test_services_availability(self):
        # Verify services are ready
        self.assertTrue(self.fk_client.wait_for_service(timeout_sec=1.0), "MoveIt2 /compute_fk service not available")
        if self.custom_fk_client is not None:
            self.assertTrue(self.custom_fk_client.wait_for_service(timeout_sec=1.0), "Custom /fk_solve service not available")

    def test_home_fk(self):
        # Home joints: all zeros
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Call MoveIt2 FK
        moveit_pose = self._call_moveit_fk(home_joints)
        self.assertIsNotNone(moveit_pose, "MoveIt2 FK failed for home configuration")
        self.node.get_logger().info(f"Home pose from MoveIt2: {moveit_pose.position.x:.6f}, {moveit_pose.position.y:.6f}, {moveit_pose.position.z:.6f}")
        
        # Call Custom FK
        custom_pose = self._call_custom_fk(home_joints)
        self.assertIsNotNone(custom_pose, "Custom FK failed for home configuration")
        self.node.get_logger().info(f"Home pose from Custom solver: {custom_pose.position.x:.6f}, {custom_pose.position.y:.6f}, {custom_pose.position.z:.6f}")
        
        # Verify delta is within 0.1 mm
        dx = abs(custom_pose.position.x - moveit_pose.position.x)
        dy = abs(custom_pose.position.y - moveit_pose.position.y)
        dz = abs(custom_pose.position.z - moveit_pose.position.z)
        
        self.node.get_logger().info(f"Home FK delta: dx={dx*1000:.4f}mm, dy={dy*1000:.4f}mm, dz={dz*1000:.4f}mm")
        
        self.assertTrue(dx < 1e-4, f"Home X mismatch: dx = {dx*1000:.4f}mm")
        self.assertTrue(dy < 1e-4, f"Home Y mismatch: dy = {dy*1000:.4f}mm")
        self.assertTrue(dz < 1e-4, f"Home Z mismatch: dz = {dz*1000:.4f}mm")

    def test_random_poses_fk(self):
        np.random.seed(42) # For repeatability
        
        num_samples = 100
        tolerance = 1e-4 # 0.1mm
        
        failures = 0
        mismatch_count = 0
        
        for i in range(num_samples):
            # Random joints within URDF safety limits (e.g. -1.5 to 1.5 rad to stay in safe zone)
            q = np.random.uniform(-1.5, 1.5, 6)
            
            moveit_pose = self._call_moveit_fk(q)
            custom_pose = self._call_custom_fk(q)
            
            if moveit_pose is None or custom_pose is None:
                failures += 1
                continue
                
            dx = abs(custom_pose.position.x - moveit_pose.position.x)
            dy = abs(custom_pose.position.y - moveit_pose.position.y)
            dz = abs(custom_pose.position.z - moveit_pose.position.z)
            
            if dx >= tolerance or dy >= tolerance or dz >= tolerance:
                mismatch_count += 1
                self.node.get_logger().error(f"Mismatch at sample {i}: q={q.tolist()}\n"
                                             f"MoveIt: [{moveit_pose.position.x:.6f}, {moveit_pose.position.y:.6f}, {moveit_pose.position.z:.6f}]\n"
                                             f"Custom: [{custom_pose.position.x:.6f}, {custom_pose.position.y:.6f}, {custom_pose.position.z:.6f}]\n"
                                             f"delta: dx={dx*1000:.4f}mm, dy={dy*1000:.4f}mm, dz={dz*1000:.4f}mm")
        
        self.assertEqual(failures, 0, f"Kinematics service failures encountered during sampling: {failures}/{num_samples}")
        self.assertEqual(mismatch_count, 0, f"FK mismatches found (>0.1mm): {mismatch_count}/{num_samples}")

if __name__ == '__main__':
    unittest.main()
