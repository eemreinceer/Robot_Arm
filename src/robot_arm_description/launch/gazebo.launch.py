import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("robot_arm_description")
    xacro_file = os.path.join(share, "urdf", "robot_arm.urdf.xacro")

    # package:// mesh URI'leri share'in ÜST dizininden, model:// URI'leri ise
    # models dizininin KENDİSİNDEN çözülür — ikisi farklı kök ister, o yüzden
    # ikisi de eklenir. (Tahta modeli model:// kullanıyor.)
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(
        p for p in (gz_resource_path,
                    os.path.dirname(share),
                    os.path.join(share, "models")) if p
    )

    # Kamera montaj transformu launch'tan geçer. Gerçek kolda bu ÖLÇÜLMEDİ ve
    # xacro varsayılanı kasten sıfırdır (kamera bileğin içinde = aşikâr yanlış).
    # Sim'de ise bilinen bir değer verilebilir ve bu, hand-eye zincirinin geri
    # bulması gereken YER GERÇEĞİ olur: sim provası çözülen X'i buraya verilen
    # değerle karşılaştırır. Gerçek donanım için bir sayı ima etmez.
    camera_mount_xyz = LaunchConfiguration("camera_mount_xyz")
    declare_camera_mount_xyz = DeclareLaunchArgument(
        "camera_mount_xyz", default_value="0 0 0",
        description="camera_mount_joint xyz (m). Sim provasında yer gerçeği.")
    camera_mount_rpy = LaunchConfiguration("camera_mount_rpy")
    declare_camera_mount_rpy = DeclareLaunchArgument(
        "camera_mount_rpy", default_value="0 0 0",
        description="camera_mount_joint rpy (rad). Sim provasında yer gerçeği.")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " camera_mount_xyz:='", camera_mount_xyz, "'",
                " camera_mount_rpy:='", camera_mount_rpy, "'",
            ]), value_type=str
        )
    }

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    world = LaunchConfiguration("world")
    declare_world = DeclareLaunchArgument("world", default_value="robot_arm_test.world")
    world_path = PathJoinSubstitution([share, "worlds", world])

    # Server ve GUI ayrı süreçlerdir. Birleşik `gz sim -r` kullanıldığında GUI'nin
    # kapanması server'ı da bitirip ROS düğümlerini sahipsiz bırakıyordu.
    headless = LaunchConfiguration("headless")
    declare_headless = DeclareLaunchArgument("headless", default_value="false")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-r -s ", world_path]}.items(),
    )

    gz_gui = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=["gz", "sim", "-g", "--force-version", "8"],
            output="screen",
            condition=UnlessCondition(headless),
        )],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "robot_arm"],
        output="screen",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(share, "config", "gz_bridge.yaml"),
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # Ağır yükte gz_ros2_control geç hazır olur → 60s timeout (ana kolda kanıtlı)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_gripper_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_headless,
        declare_camera_mount_xyz,
        declare_camera_mount_rpy,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        jsb_spawner,
        arm_spawner,
        gripper_spawner,
    ])
