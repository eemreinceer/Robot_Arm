#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pytest
import time
from action_msgs.msg import GoalStatusArray, GoalStatus

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))
from gz_gt import spawn_model, remove_model

class FullPipelineTestNode(Node):
    def __init__(self):
        super().__init__('test_full_pipeline')
        self.latest_status = None
        self.success_events = 0
        self.sub = self.create_subscription(
            GoalStatusArray,
            '/pick_and_place/_action/status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                # To prevent double counting the same goal, we track the goal ID
                goal_id = bytes(status.goal_info.goal_id.uuid).hex()
                if not hasattr(self, 'completed_goals'):
                    self.completed_goals = set()
                if goal_id not in self.completed_goals:
                    self.completed_goals.add(goal_id)
                    self.success_events += 1

@pytest.fixture
def rclpy_setup():
    rclpy.init()
    node = FullPipelineTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_full_pipeline_success(rclpy_setup):
    """
    Test the full perception -> pick pipeline 5 times.
    Waits for the autonomous pick node to complete the goal.
    """
    node = rclpy_setup
    
    total_runs = 5
    models = ['red_box', 'yellow_cylinder', 'blue_cube', 'red_box', 'blue_cube']
    poses = [(0.4, 0.1, 0.65), (0.3, -0.2, 0.65), (0.5, 0.0, 0.65), (0.35, 0.2, 0.65), (0.45, -0.1, 0.65)]
    
    for i in range(total_runs):
        model_name = models[i]
        x, y, z = poses[i]
        
        spawn_model(model_name, x, y, z, 0, 0, 0)
        
        # Wait up to 30s for the action to succeed
        start = time.time()
        initial_successes = node.success_events
        succeeded = False
        
        while time.time() - start < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.success_events > initial_successes:
                succeeded = True
                break
                
        remove_model(model_name)
        assert succeeded, f"Pipeline failed on run {i+1} with {model_name}. Action did not reach STATUS_SUCCEEDED."
        
        # Brief pause before next cycle
        time.sleep(2.0)
        
    assert node.success_events >= total_runs, f"Expected {total_runs} successes, got {node.success_events}"
