"""Real RGB-only perception preparation for Jetson Nano + IMX219."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # camera_info_file geçilmezse node CameraInfo'yu SESSİZCE yayınlamaz
    # (fail-closed). Paketli kalibrasyonu default yaparak bunu kapatıyoruz;
    # başka bir kamera için argümanla üzerine yazılır.
    default_camera_info = PathJoinSubstitution([
        FindPackageShare('arm_perception'), 'config', 'imx219_640x480.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('sensor_id', default_value='0'),
        # IMX219 fiziksel olarak TERS monte (2026-07-26 doğrulandı) → nvvidconv
        # flip-method=2 (180° döndürme). Deployment gerçeği, node default'u değil.
        DeclareLaunchArgument('flip_method', default_value='2'),
        DeclareLaunchArgument(
            'camera_info_file', default_value=default_camera_info),
        Node(
            package='arm_perception', executable='csi_camera_node',
            name='csi_camera', output='screen',
            parameters=[{
                'sensor_id': LaunchConfiguration('sensor_id'),
                'flip_method': LaunchConfiguration('flip_method'),
                'camera_info_file': LaunchConfiguration('camera_info_file'),
            }]),
        Node(
            package='arm_perception', executable='rgb_planar_detector_node',
            name='rgb_planar_detector', output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }]),
    ])
