#!/usr/bin/env python3
"""
MoveIt2 Planning Integration Test
Gereksinim: move_group çalışıyor olmalı
Koştur: ros2 run arm_tests test_moveit_planning.py
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import time


class MoveItPlanningTest(Node):
    def __init__(self):
        super().__init__('moveit_planning_test')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def run_tests(self):
        results = []

        # Test 1: move_group servisi erişilebilir mi?
        available = self.ik_client.wait_for_service(timeout_sec=5.0)
        results.append(('move_group IK service reachable', available))

        # Test 2: Basit bir IK çözümü
        if available:
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.pose_stamped.header.frame_id = 'base_link'
            req.ik_request.pose_stamped.pose.position.x = 0.3
            req.ik_request.pose_stamped.pose.position.y = 0.0
            req.ik_request.pose_stamped.pose.position.z = 0.3
            req.ik_request.pose_stamped.pose.orientation.w = 1.0
            req.ik_request.timeout.sec = 1

            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result():
                success = (future.result().error_code.val == 1)  # SUCCESS = 1
                results.append(('IK solve for reachable pose', success))
            else:
                results.append(('IK solve for reachable pose', False))

        return results


def main():
    rclpy.init()
    node = MoveItPlanningTest()
    results = node.run_tests()

    print('\n=== MoveIt2 Planning Test Results ===')
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
