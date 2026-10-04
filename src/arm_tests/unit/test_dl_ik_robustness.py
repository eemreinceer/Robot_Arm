#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pytest
import numpy as np
import time
from geometry_msgs.msg import Pose

try:
    from arm_interfaces.srv import SolveIk
except ImportError:
    SolveIk = None

class RobustnessTestNode(Node):
    def __init__(self):
        super().__init__('dl_ik_robustness_test')
        if SolveIk is not None:
            self.client = self.create_client(SolveIk, '/dl_ik_solve')
        else:
            self.client = None

    def call_ik(self, target_pose):
        req = SolveIk.Request()
        req.target_pose = target_pose
        req.seed_angles = [0.0]*6
        req.solver = 'dl'
        
        start_time = time.perf_counter()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        latency_ms = (time.perf_counter() - start_time) * 1000.0
        
        if not future.done():
            self.get_logger().error("Service call timed out")
            return None, latency_ms
            
        return future.result(), latency_ms

@pytest.fixture
def rclpy_setup():
    rclpy.init()
    node = RobustnessTestNode()
    if node.client is not None:
        node.client.wait_for_service(timeout_sec=5.0)
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_dl_ik_service_available(rclpy_setup):
    assert SolveIk is not None, "arm_interfaces/srv/SolveIk not found. Package needs to be built."
    assert rclpy_setup.client.service_is_ready(), "/dl_ik_solve service is not available."

def test_dl_ik_inference_latency(rclpy_setup):
    """Test that the Deep Learning IK inference latency is less than 5ms."""
    node = rclpy_setup
    if not node.client.service_is_ready():
        pytest.skip("/dl_ik_solve not available")
        
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.0
    target_pose.position.z = 0.4
    target_pose.orientation.w = 1.0

    # Warmup
    node.call_ik(target_pose)
    
    latencies = []
    for _ in range(50):
        res, lat = node.call_ik(target_pose)
        if res and res.success:
            latencies.append(lat)
            
    assert len(latencies) > 0, "All IK calls failed"
    avg_latency = np.mean(latencies)
    assert avg_latency < 5.0, f"Average latency {avg_latency:.2f}ms exceeds 5ms requirement"

@pytest.mark.parametrize("noise_sigma", [0.001, 0.005, 0.010])
def test_dl_ik_noise_robustness(rclpy_setup, noise_sigma):
    """Test the robustness of DL IK to noisy inputs (1mm, 5mm, 10mm)."""
    node = rclpy_setup
    if not node.client.service_is_ready():
        pytest.skip("/dl_ik_solve not available")
        
    base_x = 0.3
    base_y = 0.0
    base_z = 0.4
    
    success_count = 0
    total = 20
    
    np.random.seed(42)
    for _ in range(total):
        target_pose = Pose()
        target_pose.position.x = base_x + np.random.normal(0, noise_sigma)
        target_pose.position.y = base_y + np.random.normal(0, noise_sigma)
        target_pose.position.z = base_z + np.random.normal(0, noise_sigma)
        target_pose.orientation.w = 1.0
        
        res, _ = node.call_ik(target_pose)
        if res and res.success:
            success_count += 1
            
    success_rate = success_count / total
    # Even with noise, the neural net should return *some* prediction without crashing.
    # It might have a higher FK error internally, but the node itself should succeed in returning values.
    assert success_rate > 0.8, f"DL IK success rate {success_rate*100}% is too low for noise sigma {noise_sigma}m"
