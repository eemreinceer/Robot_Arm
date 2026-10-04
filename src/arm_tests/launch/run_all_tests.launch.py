import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Launch file that triggers colcon test for our test package
    return LaunchDescription([
        ExecuteProcess(
            cmd=['colcon', 'test', '--packages-select', 'arm_tests', '--event-handlers', 'console_cohesion+'],
            output='screen'
        )
    ])
