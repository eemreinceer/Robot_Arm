from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = PACKAGE_ROOT / 'deploy' / 'pi5'


def _unit(name):
    return (DEPLOY_ROOT / name).read_text(encoding='utf-8')


def test_camera_unit_pins_custom_libcamera_and_direct_calibrated_node():
    unit = _unit('robot-arm-camera.service')

    assert ('GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/'
            'gstreamer-1.0') in unit
    assert 'ROBOT_ARM_RUNTIME_ROOT=/opt/robot_arm' in unit
    assert 'EnvironmentFile=-/etc/robot_arm/robot_arm.env' in unit
    assert 'csi_camera_node" --ros-args' in unit
    assert 'imx219_640x480_pi5.yaml' in unit
    assert '/home/' not in unit
    assert 'ros2 launch' not in unit
    assert 'Restart=always' in unit
    assert 'KillMode=control-group' in unit


def test_dashboard_unit_starts_with_camera_but_exposes_no_motion_stack():
    unit = _unit('robot-arm-dashboard.service')

    assert 'Wants=network-online.target robot-arm-camera.service' in unit
    assert 'ROBOT_ARM_BIND_ADDR=127.0.0.1' in unit
    assert 'EnvironmentFile=-/etc/robot_arm/robot_arm.env' in unit
    assert 'ros2 run arm_perception web_console' in unit
    assert '/home/' not in unit
    assert 'ros2_control' not in unit
    assert 'real_hardware' not in unit


def test_pi5_deploy_files_are_installed_with_package():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert 'deploy/pi5/*' in setup
