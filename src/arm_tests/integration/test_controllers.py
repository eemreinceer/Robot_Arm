#!/usr/bin/env python3
"""
Controller Response Test
Koştur: ros2 run arm_tests test_controllers.py
Gereksinim: Gazebo + controllers çalışıyor olmalı
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import statistics


class ControllerTest(Node):
    def __init__(self):
        super().__init__('controller_test')
        self.joint_states_received = 0
        self.latencies = []
        self.last_cmd_time = None

        self.sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def _joint_cb(self, msg):
        if self.last_cmd_time is not None:
            latency_ms = (time.time() - self.last_cmd_time) * 1000
            self.latencies.append(latency_ms)
        self.joint_states_received += 1

    def run_tests(self):
        results = []

        # Test 1: /joint_states yayınlanıyor mu?
        rclpy.spin_once(self, timeout_sec=2.0)
        results.append(('/joint_states topic active', self.joint_states_received > 0))

        # Test 2: Controller gecikme testi (10 ölçüm)
        for _ in range(10):
            traj = JointTrajectory()
            traj.joint_names = ['joint_1', 'joint_2', 'joint_3',
                                 'joint_4', 'joint_5', 'joint_6']
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pt.time_from_start.sec = 1
            traj.points = [pt]
            self.last_cmd_time = time.time()
            self.pub.publish(traj)
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latencies:
            p95 = sorted(self.latencies)[int(len(self.latencies) * 0.95)]
            results.append((f'Controller p95 latency < 100ms (actual: {p95:.1f}ms)', p95 < 100.0))

        return results


def main():
    rclpy.init()
    node = ControllerTest()
    results = node.run_tests()

    print('\n=== Controller Response Test Results ===')
    passed = 0
    for name, result in results:
        status = '✅ PASS' if result else '❌ FAIL'
        print(f'  {status}  {name}')
        if result:
            passed += 1

    print(f'\n{passed}/{len(results)} tests passed')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    exit(main())
