"""
display.launch.py — RViz'de robotu görselleştir (Gazebo olmadan)
Kullanım: ros2 launch arm_description display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_arm_description = get_package_share_directory('arm_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    xacro_file = os.path.join(pkg_arm_description, 'urdf', 'arm.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot State Publisher
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

    # Gripper başlangıç pozisyonları: açık değerlerin yarısı
    jsp_params = {
        'zeros': {
            'gripper_joint1': -0.2868,
            'gripper_joint2': 0.2368,
        }
    }

    # Joint State Publisher GUI (eklemleri elle hareket ettir)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[jsp_params],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_gui'), 'true')),
    )

    # Joint State Publisher (GUI olmadan)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[jsp_params],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_gui'), 'false')),
    )

    # RViz2
    rviz_config = os.path.join(pkg_arm_description, 'config', 'arm.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Sim zamanı kullan'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Joint State Publisher GUI aç'),
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
