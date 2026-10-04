"""Robot Arm kol — SADECE donanim bring-up (Jetson Nano, Humble).

`arm_bringup/launch/real_hardware.launch.py`'nin cikarilmis hali: burada
move_group, pick_place_node ve RViz YOK. Sebebi kasitli:

  * Nano'nun MoveIt'i PC'dekinden eski bir API'de (`moveit/...*.h` basliklari,
    `computeCartesianPath` imzasinda jump_threshold). arm_nodes orada ancak
    commit'siz uyumluluk yamalariyla derleniyor. Donanimi surmek icin MoveIt'e
    ihtiyac yok, o yuzden bu launch ona hic dokunmuyor.
  * RViz ve rqt PC tarafinda calisir (Nano'da RViz surunur). ROS_DOMAIN_ID ayni
    ve iki makine ayni yerel agda (10.42.0.x) oldugu surece DDS kendiliginden
    bulur.

Kalibrasyon dogrulamasi icin MoveIt'ten daha iyi bir arac zaten var:
`rqt_joint_trajectory_controller` — slider dogrudan joint_trajectory_controller'a
gider, arada planlayici/IK yoktur, yani invert/zero_offset/kanal hatalari
gizlenmeden gorunur.

Kullanim (Nano konteynerinde), servo rayi KAPALIYKEN:
    ros2 launch robot_arm_description hardware_bringup.launch.py
    # linkleri elle referans poza getir, sonra:
    ros2 param set /robot_arm_hardware_safety reference_positions \
        "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    ros2 param set /robot_arm_hardware_safety armed true
    ros2 run controller_manager spawner robot_arm_controller
    ros2 run controller_manager spawner robot_arm_gripper_controller
    # servo rayi EN SON acilir

Donanim notu: tasima UART (`/dev/ttyTHS1`), USB CDC bu GD32 klonunda calismiyor —
bkz. firmware/stm32_servo_ctrl/README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory("robot_arm_description")

    serial_device = LaunchConfiguration("serial_device")
    baud_rate = LaunchConfiguration("baud_rate")
    mock_serial = LaunchConfiguration("mock_serial")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("robot_arm_description"), "urdf",
                    "robot_arm_real.urdf.xacro",
                ]),
                " serial_device:=", serial_device,
                " baud_rate:=", baud_rate,
                " mock_serial:=", mock_serial,
            ]),
            value_type=str,
        )
    }

    controllers_yaml = os.path.join(
        description_share, "config", "robot_arm_controllers_real.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": False}],
    )

    spawn_motion = LaunchConfiguration("spawn_motion_controllers")

    def spawner(controller, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller,
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
            condition=condition,
        )

    broadcaster = spawner("joint_state_broadcaster")
    arm_controller = spawner("robot_arm_controller", IfCondition(spawn_motion))
    gripper_controller = spawner("robot_arm_gripper_controller", IfCondition(spawn_motion))

    # Sirali spawn: broadcaster once ayaga kalksin ki controller'lar state
    # arayuzunu hazir bulsun.
    return LaunchDescription([
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("mock_serial", default_value="false"),
        # A joint_trajectory_controller that goes ACTIVE while the hardware is
        # still disarmed latches a hold target from the stale disarmed state,
        # and pushes it the moment the operator arms. Default to broadcaster
        # only: arm first, then spawn these two by hand.
        DeclareLaunchArgument("spawn_motion_controllers", default_value="false"),
        robot_state_publisher,
        ros2_control_node,
        broadcaster,
        RegisterEventHandler(
            OnProcessExit(target_action=broadcaster, on_exit=[arm_controller])),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])),
    ])
