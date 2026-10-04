#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import os
from pathlib import Path
import threading
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

# Import custom services
try:
    from arm_interfaces.srv import SolveFk, SolveIk
except ImportError:
    SolveFk = None
    SolveIk = None

class KinematicsBenchmarker(Node):
    def __init__(self):
        super().__init__('kinematics_benchmarker')
        
        # Clients
        self.moveit_fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.moveit_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if SolveFk is not None:
            self.custom_fk_client = self.create_client(SolveFk, '/fk_solve')
        else:
            self.custom_fk_client = None
            
        if SolveIk is not None:
            self.custom_ik_client = self.create_client(SolveIk, '/ik_solve')
        else:
            self.custom_ik_client = None

    def wait_for_services(self):
        self.get_logger().info("Waiting for kinematics services...")
        self.moveit_fk_client.wait_for_service()
        self.moveit_ik_client.wait_for_service()
        if self.custom_fk_client:
            self.custom_fk_client.wait_for_service()
        if self.custom_ik_client:
            self.custom_ik_client.wait_for_service()
        self.get_logger().info("All services available!")

    def call_service(self, client, req):
        future = client.call_async(req)
        start_time = time.perf_counter()
        while not future.done():
            time.sleep(0.0001)
        latency = (time.perf_counter() - start_time) * 1000.0 # in milliseconds
        res = future.result()
        return res, latency

    def benchmark(self, num_samples=1000):
        if self.custom_fk_client is None or self.custom_ik_client is None:
            self.get_logger().error("Custom SolveFk/SolveIk services not available (arm_interfaces build needed). Running only MoveIt2 benchmark.")
        
        self.get_logger().info(f"Starting benchmark with {num_samples} samples...")
        
        # Random inputs
        np.random.seed(42)
        joint_samples = np.random.uniform(-1.5, 1.5, (num_samples, 6))
        
        # Latency lists (ms)
        latencies = {
            'moveit_fk': [],
            'moveit_ik': [],
            'custom_fk': [],
            'custom_ik': []
        }
        
        success = {
            'moveit_fk': 0,
            'moveit_ik': 0,
            'custom_fk': 0,
            'custom_ik': 0
        }
        
        for i in range(num_samples):
            q = joint_samples[i]
            
            # --- 1. MoveIt2 FK ---
            req_fk = GetPositionFK.Request()
            req_fk.header.frame_id = 'base_link'
            req_fk.fk_link_names = ['Link_6']
            js = JointState()
            js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            js.position = [float(val) for val in q]
            req_fk.robot_state.joint_state = js
            
            res_mfk, lat_mfk = self.call_service(self.moveit_fk_client, req_fk)
            latencies['moveit_fk'].append(lat_mfk)
            if res_mfk and res_mfk.error_code.val == 1:
                success['moveit_fk'] += 1
                target_pose = res_mfk.pose_stamped[0].pose
            else:
                target_pose = None

            # --- 2. Custom FK ---
            if self.custom_fk_client:
                req_cfk = SolveFk.Request()
                req_cfk.joint_angles = [float(val) for val in q]
                res_cfk, lat_cfk = self.call_service(self.custom_fk_client, req_cfk)
                latencies['custom_fk'].append(lat_cfk)
                if res_cfk and res_cfk.success:
                    success['custom_fk'] += 1

            # Skip IK tests if FK failed to provide target pose
            if target_pose is None:
                continue

            # --- 3. MoveIt2 IK ---
            req_mik = GetPositionIK.Request()
            req_mik.ik_request.group_name = 'arm'
            req_mik.ik_request.pose_stamped.header.frame_id = 'base_link'
            req_mik.ik_request.pose_stamped.pose = target_pose
            req_mik.ik_request.ik_link_name = 'Link_6'
            req_mik.ik_request.timeout.sec = 1
            
            res_mik, lat_mik = self.call_service(self.moveit_ik_client, req_mik)
            latencies['moveit_ik'].append(lat_mik)
            if res_mik and res_mik.error_code.val == 1:
                success['moveit_ik'] += 1

            # --- 4. Custom IK ---
            if self.custom_ik_client:
                req_cik = SolveIk.Request()
                req_cik.target_pose = target_pose
                req_cik.seed_angles = [float(val) for val in q]
                req_cik.solver = 'dls'
                
                res_cik, lat_cik = self.call_service(self.custom_ik_client, req_cik)
                latencies['custom_ik'].append(lat_cik)
                if res_cik and res_cik.success:
                    success['custom_ik'] += 1

            if (i + 1) % 100 == 0:
                self.get_logger().info(f"Processed {i+1}/{num_samples} samples...")

        # Calculate statistics
        stats = {}
        for key, lat_list in latencies.items():
            if len(lat_list) == 0:
                stats[key] = {
                    'avg': 0.0, 'median': 0.0, 'p95': 0.0, 'p99': 0.0, 'min': 0.0, 'max': 0.0, 'success_rate': 0.0
                }
                continue
            arr = np.array(lat_list)
            stats[key] = {
                'avg': np.mean(arr),
                'median': np.median(arr),
                'p95': np.percentile(arr, 95),
                'p99': np.percentile(arr, 99),
                'min': np.min(arr),
                'max': np.max(arr),
                'success_rate': (success[key] / num_samples) * 100.0
            }

        self.generate_report(stats, num_samples)

    def generate_report(self, stats, num_samples):
        report = f"""# Kinematics Performance Benchmark Report

Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}  
Number of samples: {num_samples}  

## Latency and Success Rate Analysis

| Solver / Service | Success Rate | Avg Latency (ms) | Median Latency (ms) | p95 Latency (ms) | p99 Latency (ms) | Min Latency (ms) | Max Latency (ms) |
|---|---|---|---|---|---|---|---|
| **MoveIt2 FK** (`/compute_fk`) | {stats['moveit_fk']['success_rate']:.2f}% | {stats['moveit_fk']['avg']:.4f} | {stats['moveit_fk']['median']:.4f} | {stats['moveit_fk']['p95']:.4f} | {stats['moveit_fk']['p99']:.4f} | {stats['moveit_fk']['min']:.4f} | {stats['moveit_fk']['max']:.4f} |
| **Custom FK** (`/fk_solve`) | {stats['custom_fk']['success_rate']:.2f}% | {stats['custom_fk']['avg']:.4f} | {stats['custom_fk']['median']:.4f} | {stats['custom_fk']['p95']:.4f} | {stats['custom_fk']['p99']:.4f} | {stats['custom_fk']['min']:.4f} | {stats['custom_fk']['max']:.4f} |
| **MoveIt2 IK** (`/compute_ik`) | {stats['moveit_ik']['success_rate']:.2f}% | {stats['moveit_ik']['avg']:.4f} | {stats['moveit_ik']['median']:.4f} | {stats['moveit_ik']['p95']:.4f} | {stats['moveit_ik']['p99']:.4f} | {stats['moveit_ik']['min']:.4f} | {stats['moveit_ik']['max']:.4f} |
| **Custom IK** (`/ik_solve`) | {stats['custom_ik']['success_rate']:.2f}% | {stats['custom_ik']['avg']:.4f} | {stats['custom_ik']['median']:.4f} | {stats['custom_ik']['p95']:.4f} | {stats['custom_ik']['p99']:.4f} | {stats['custom_ik']['min']:.4f} | {stats['custom_ik']['max']:.4f} |

## Key Findings

1. **FK Performance:** Compare the Custom analytical/numerical formulation computation speed against MoveIt2's setup.
2. **IK Robustness & Latency:** Custom numerical Jacobian (Damped Least Squares) method success rates vs. OMPL/MoveIt2's solver configuration.
"""
        # Save report
        report_dir = Path(__file__).resolve().parents[1] / 'benchmark_results'
        report_dir.mkdir(parents=True, exist_ok=True)
        report_path = report_dir / 'kinematics_report.md'
        
        with report_path.open('w', encoding='utf-8') as f:
            f.write(report)
            
        print(f"\n--- Benchmark Results Report generated at: {report_path} ---")
        print(report)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsBenchmarker()
    
    # Spin in thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.wait_for_services()
        node.benchmark(num_samples=1000)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
