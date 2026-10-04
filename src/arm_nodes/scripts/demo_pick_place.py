#!/usr/bin/env python3
"""Send a fixed PickAndPlace action goal for smoke testing."""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from arm_interfaces.action import PickAndPlace


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


# Workspace coordinates calibrated on 2026-05-28 via tf2_echo base_link tool_link
# Home position: (-0.176, -0.400, 0.564)
# Pick pose: slightly lower than home Z
# Place pose: Y-offset from pick pose
PICK_X, PICK_Y, PICK_Z = 0.253227, -0.007999, 0.168658
PLACE_X, PLACE_Y, PLACE_Z = 0.403227, 0.242001, 0.168658
# Orientation from find_valid_pick_pose.py for theta = 90.0
ORI_X, ORI_Y, ORI_Z, ORI_W = 0.749593, 0.605798, 0.207415, 0.167625


class DemoPickPlaceClient(Node):
    def __init__(self):
        super().__init__('demo_pick_place')
        self._client = ActionClient(self, PickAndPlace, '/pick_and_place')

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/pick_and_place action server is not available')
            return 1

        goal = PickAndPlace.Goal()
        goal.object_id = 'demo_box'
        goal.pick_pose = make_pose(PICK_X, PICK_Y, PICK_Z, ORI_X, ORI_Y, ORI_Z, ORI_W)
        goal.place_pose = make_pose(PLACE_X, PLACE_Y, PLACE_Z, ORI_X, ORI_Y, ORI_Z, ORI_W)
        goal.speed_scale = 0.5
        goal.return_home = True

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'success={result.success} message="{result.message}" execution_time={result.execution_time:.2f}s'
        )
        return 0 if result.success else 2

    def _feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'phase={feedback.current_phase} progress={feedback.progress:.2f}')


def main():
    rclpy.init()
    node = DemoPickPlaceClient()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
