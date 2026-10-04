#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

# Import custom IK service once generated
try:
    from arm_interfaces.srv import SolveIk
except ImportError:
    SolveIk = None

class TestInverseKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_inverse_kinematics_node')
        cls.fk_client = cls.node.create_client(GetPositionFK, '/compute_fk')
        if SolveIk is not None:
            cls.custom_ik_client = cls.node.create_client(SolveIk, '/ik_solve')
        else:
            cls.custom_ik_client = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Wait for services to be ready before each test to ensure discovery
        self.fk_client.wait_for_service(timeout_sec=5.0)
        if self.custom_ik_client is not None:
            self.custom_ik_client.wait_for_service(timeout_sec=5.0)

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

    def _call_custom_ik(self, target_pose, seed_angles=None):
        if self.custom_ik_client is None:
            self.skipTest("SolveIk service type not imported (arm_interfaces not built with Phase 2 definitions)")
            
        req = SolveIk.Request()
        req.target_pose = target_pose
        if seed_angles is not None:
            req.seed_angles = [float(val) for val in seed_angles]
        else:
            req.seed_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req.solver = 'dls' # Damped Least Squares
        
        future = self.custom_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            return future.result()
        return None

    def test_services_availability(self):
        # Verify services are ready
        self.assertTrue(self.fk_client.wait_for_service(timeout_sec=1.0), "MoveIt2 /compute_fk service not available")
        if self.custom_ik_client is not None:
            self.assertTrue(self.custom_ik_client.wait_for_service(timeout_sec=1.0), "Custom /ik_solve service not available")

    def test_ik_round_trip(self):
        # 200 random joint samples
        np.random.seed(24) # Seed for reproducibility
        num_samples = 200
        success_count = 0
        tolerance = 1e-3 # 1.0mm
        
        for i in range(num_samples):
            # Generate random joint values (in safe range)
            q_src = np.random.uniform(-1.5, 1.5, 6)
            
            # 1. Compute FK using MoveIt2 to get a guaranteed reachable pose
            target_pose = self._call_moveit_fk(q_src)
            if target_pose is None:
                continue
                
            # 2. Call Custom IK service
            ik_res = self._call_custom_ik(target_pose, seed_angles=q_src + np.random.normal(0, 0.1, 6))
            if ik_res is None or not ik_res.success:
                continue
                
            # 3. Call FK on the resulting joints from custom IK solver
            q_sol = ik_res.joint_angles
            resulting_pose = self._call_moveit_fk(q_sol)
            if resulting_pose is None:
                continue
                
            # 4. Compare resulting pose with target pose
            dx = abs(resulting_pose.position.x - target_pose.position.x)
            dy = abs(resulting_pose.position.y - target_pose.position.y)
            dz = abs(resulting_pose.position.z - target_pose.position.z)
            
            error = np.sqrt(dx**2 + dy**2 + dz**2)
            
            if error < tolerance:
                success_count += 1
            else:
                self.node.get_logger().warn(
                    f"IK solution found but exceeded error threshold at sample {i}:\n"
                    f"  Target: [{target_pose.position.x:.6f}, {target_pose.position.y:.6f}, {target_pose.position.z:.6f}]\n"
                    f"  Result: [{resulting_pose.position.x:.6f}, {resulting_pose.position.y:.6f}, {resulting_pose.position.z:.6f}]\n"
                    f"  Error: {error*1000:.4f}mm"
                )

        success_rate = (success_count / num_samples) * 100.0
        self.node.get_logger().info(f"Custom IK Solver success rate: {success_rate:.2f}% ({success_count}/{num_samples})")
        
        # Verify success rate is greater than 95%
        self.assertTrue(success_rate >= 95.0, f"IK success rate ({success_rate:.2f}%) below threshold of 95%")

    def test_out_of_workspace_graceful_failure(self):
        # Target position far outside workspace (e.g. 5 meters away)
        unreachable_pose = Pose()
        unreachable_pose.position.x = 5.0
        unreachable_pose.position.y = 5.0
        unreachable_pose.position.z = 5.0
        unreachable_pose.orientation.x = 0.0
        unreachable_pose.orientation.y = 0.0
        unreachable_pose.orientation.z = 0.0
        unreachable_pose.orientation.w = 1.0
        
        ik_res = self._call_custom_ik(unreachable_pose)
        
        self.assertIsNotNone(ik_res, "Custom IK Solver service failed to respond for unreachable pose")
        self.assertFalse(ik_res.success, "Custom IK Solver reported success for an unreachable pose far outside the workspace")
        self.node.get_logger().info("Graceful failure verification passed (reported success=False as expected)")

if __name__ == '__main__':
    unittest.main()
