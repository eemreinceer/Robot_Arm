"""Launch the read-only Robot Arm browser console gateway on the operator PC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('arm_perception'), 'config', 'web_console.yaml']
    )
    # Kalibrasyon kimligi ve olcum kayit defteri paket icinden cozulur; YAML'a
    # mutlak yol yazmak makineden makineye tasinmaz.
    camera_config_dir = PathJoinSubstitution(
        [FindPackageShare('arm_perception'), 'config']
    )
    measurement_registry = PathJoinSubstitution(
        [FindPackageShare('arm_perception'), 'config',
         'measurement_registry.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument(
                'bind_address',
                default_value='127.0.0.1',
                description=(
                    'HTTP bind address. Keep loopback unless the robot LAN is trusted.'
                ),
            ),
            Node(
                package='arm_perception',
                executable='web_console',
                name='web_console',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'bind_address': LaunchConfiguration('bind_address'),
                        'camera_config_dir': camera_config_dir,
                        'measurement_registry': measurement_registry,
                    },
                ],
            ),
        ]
    )
