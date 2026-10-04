"""
rsp.launch.py — Robot State Publisher
Sadece robot_description'ı yayınlar. Diğer launch dosyaları include eder.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_arm_description = get_package_share_directory('arm_description')

    # Argümanlar
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # XACRO → URDF dönüşümü
    xacro_file = os.path.join(pkg_arm_description, 'urdf', 'arm.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Simülasyon saatini kullan'
        ),
        robot_state_publisher,
    ])
