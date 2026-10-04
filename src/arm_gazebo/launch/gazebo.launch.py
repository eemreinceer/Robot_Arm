import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    arm_gazebo_share = get_package_share_directory("arm_gazebo")
    arm_description_share = get_package_share_directory("arm_description")

    # World seçilebilir (Faz-1: pick_and_place.world; sorting: sorting.world)
    world = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([arm_gazebo_share, "worlds", world])
    xacro_file = os.path.join(arm_description_share, "urdf", "arm.urdf.xacro")

    # Set GZ_SIM_RESOURCE_PATH to locate meshes and models
    gz_resource_path = ""
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"] + ":"
    
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        gz_resource_path +
        os.path.dirname(arm_description_share) + ":" +
        os.path.join(arm_gazebo_share, "models")
    )

    # Process Xacro
    robot_description_content = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    robot_description = {"robot_description": robot_description_content}

    # Gazebo Sim launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )

    # Use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true"
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="pick_and_place.world",
        description="arm_gazebo/worlds içindeki world dosya adı (sorting: sorting.world)",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn Robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "6dof_arm",
            # Yükseklik artık URDF'teki world_joint (z=0.6) ile sabit;
            # base world'e fixed bağlı olduğu için -z offset KULLANILMAZ.
        ],
        output="screen",
    )

    # Ros Gz Bridge
    bridge_config = os.path.join(arm_gazebo_share, "config", "gz_bridge.yaml")
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # Controller Spawners
    # --controller-manager-timeout 60: yavaş WSL/Gazebo RTF'inde gz_ros2_control
    # plugin'i controller_manager'ı default 10 sn içinde hazır edemeyebiliyordu;
    # spawner zaman aşımıyla BAŞARISIZ çıkıyor, OnProcessExit çıkış koduna
    # bakmadığı için zincir yine de ilerliyor → joint_state_broadcaster aktive
    # OLMUYOR → /joint_states yok → RViz'de hareketli link'lerin TF'i kırılıyor.
    # 60 sn bekleme bunu ağır yükte bile deterministik kılar.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    # Delay controller spawners until after the robot has spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
    ])
