"""
IMX219 CSI kamerayi TEK BASINA, kalibrasyonu yuklu olarak baslatir.

Neden ayri bir launch: gunluk prosedur yalniz kamerayi kaldiriyor (detector
yok). Launch, `camera_info_file` parametresini her zaman paketli kalibrasyona
baglar; elle yazilan `ros2 run ... -p ...` komutundaki unutma sinifini kapatir.

Bu launch kalibrasyon dosyasini paketin `share/config` dizininden cozer, yani
parametre varsayilan olarak DOGRU. Fail-closed davranis degismedi: baska bir
kamera takilirsa `camera_info_file` argumani ile ustune yazilmalidir.

Cozunurluk/flip degerleri kalibrasyonun yakalandigi degerlerle ayni olmak
zorunda; node uyusmazlikta CameraInfo yayinlamayi reddeder.

2026-07-28 ve 2026-07-29 cold boot'larinda (Jetson Nano) kamera prosesi
DDS/RMW hazir olmadan node kurarken exit 1 ile dustu. Abone topic'i listede
tuttugu icin bu sessiz kaldi. Ayni race herhangi bir platformda olabilecegi
icin OnProcessExit tabanli tekrar zinciri korunuyor: varsayilan olarak en
fazla iki tekrar, beser saniye ara ve butce bitince gorunur launch shutdown.

2026-08-19: hedef platform Jetson Nano'dan Raspberry Pi 5'e tasindi
(csi_camera_node artik libcamera arka ucunu kullaniyor, bkz. 2658331/c0fa0c9).
Bu launch'in varsayilanlari artik Pi 5'in fiziksel montajina ve kalibrasyonuna
gore: flip_method=0, camera_info_file=imx219_640x480_pi5.yaml.
"""

import os
from dataclasses import dataclass, field

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.events import Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_LOGGER = get_logger('imx219_camera.launch')


@dataclass
class _RetryState:
    max_restarts: int
    delay_s: float
    starts: int = field(default=0)

    def begin_attempt(self):
        self.starts += 1
        return self.starts

    def can_restart(self):
        # starts=1 ve max_restarts=2 -> iki yeni baslatma hakki vardir.
        return self.starts <= self.max_restarts


def _non_negative_int(value, name):
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f'{name} tam sayi olmali: {value!r}') from error
    if parsed < 0:
        raise RuntimeError(f'{name} negatif olamaz: {parsed}')
    return parsed


def _non_negative_float(value, name):
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f'{name} sayi olmali: {value!r}') from error
    if parsed < 0.0:
        raise RuntimeError(f'{name} negatif olamaz: {parsed}')
    return parsed


def _camera_exited(event, context, state):
    """Retry a long-lived camera process only while the bounded budget remains."""
    if context.is_shutdown:
        return []

    returncode = getattr(event, 'returncode', 'unknown')
    if state.can_restart():
        next_attempt = state.starts + 1
        total_attempts = state.max_restarts + 1
        _LOGGER.warning(
            f'CSI camera exited unexpectedly (code {returncode}); retrying '
            f'in {state.delay_s:.1f}s (attempt {next_attempt}/{total_attempts})')
        return [
            TimerAction(
                period=state.delay_s,
                actions=[
                    OpaqueFunction(
                        function=_start_camera,
                        args=[state]),
                ]),
        ]

    reason = (
        'CSI camera failed after '
        f'{state.starts} attempts (last exit code {returncode}); '
        'bounded restart budget exhausted')
    _LOGGER.error(reason)
    return [EmitEvent(event=Shutdown(reason=reason))]


def _start_camera(context, state=None):
    if state is None:
        state = _RetryState(
            max_restarts=_non_negative_int(
                LaunchConfiguration('max_startup_restarts').perform(context),
                'max_startup_restarts'),
            delay_s=_non_negative_float(
                LaunchConfiguration('startup_retry_delay_s').perform(context),
                'startup_retry_delay_s'))

    attempt = state.begin_attempt()
    _LOGGER.info(
        f'Starting CSI camera (attempt {attempt}/{state.max_restarts + 1})')

    # Pi 5'te DOGRU libcamera GStreamer eklentisi /usr/local altindadir (yerel
    # derlenmis v0.7.2+rpt: PiSP pipeline handler + IPA). Kullanicinin ~/.bashrc'si
    # bunu GST_PLUGIN_PATH'e koyar, ama .bashrc yalniz interaktif kabuklarda
    # okunur -- SSH komutu, systemd unit'i ya da cron ile baslatildiginda node
    # apt'nin libcamera 0.2.0'ina duser, onun IPA dizini bostur ve
    # `cameras() is empty` ile SIFIR KARE yayinlar. 2026-08-21'de bu 40 dakika
    # yakti. Ortami kabuga birakmayip burada kuruyoruz; dizin yoksa (Jetson,
    # x86 gelistirme makinesi) ekleme yapilmaz ve davranis degismez.
    env_actions = []
    local_gst = '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0'
    if os.path.isdir(local_gst):
        existing = os.environ.get('GST_PLUGIN_PATH', '')
        if local_gst not in existing.split(':'):
            env_actions.append(SetEnvironmentVariable(
                'GST_PLUGIN_PATH',
                f'{local_gst}:{existing}' if existing else local_gst))

    return env_actions + [
        Node(
            package='arm_perception',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[{
                'sensor_id': LaunchConfiguration('sensor_id'),
                'flip_method': LaunchConfiguration('flip_method'),
                'output_width': LaunchConfiguration('output_width'),
                'output_height': LaunchConfiguration('output_height'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_info_file': LaunchConfiguration('camera_info_file'),
                'camera_name': LaunchConfiguration('camera_name'),
                'first_frame_timeout_s': LaunchConfiguration(
                    'first_frame_timeout_s'),
            }],
            on_exit=lambda event, exit_context: _camera_exited(
                event, exit_context, state)),
    ]


def generate_launch_description():
    default_camera_info = PathJoinSubstitution([
        FindPackageShare('arm_perception'), 'config', 'imx219_640x480_pi5.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        # Pi 5 rig'inin fiziksel montaji (2026-08-19 dogrulandi, gorsel karsilastirma
        # ile) Jetson'unkinden FARKLI: donme yok -> videoflip method=none. Kalibrasyon
        # (imx219_640x480_pi5.yaml) da bu flip ile alindi. Jetson'un flip=2 degeri ve
        # kalibrasyonu (imx219_640x480.yaml) referans icin repoda duruyor ama artik
        # varsayilan degil.
        DeclareLaunchArgument('flip_method', default_value='0'),
        DeclareLaunchArgument('output_width', default_value='640'),
        DeclareLaunchArgument('output_height', default_value='480'),
        DeclareLaunchArgument('frame_rate', default_value='30'),
        DeclareLaunchArgument(
            'camera_name', default_value='',
            description='libcamera kamera adi; bos = ilk kamera. Argus '
                        'yolunda kullanilmaz (orada sensor_id gecerlidir).'),
        DeclareLaunchArgument(
            'first_frame_timeout_s', default_value='15.0',
            description='Bu sure icinde tek kare gelmezse node olur ve '
                        'sinirli yeniden baslatma butcesi devreye girer. '
                        '0 = kapali.'),
        DeclareLaunchArgument(
            'max_startup_restarts',
            default_value='2',
            description='Unexpected camera exits after the first attempt.'),
        DeclareLaunchArgument(
            'startup_retry_delay_s',
            default_value='5.0',
            description='Delay between bounded camera startup attempts.'),
        DeclareLaunchArgument(
            'camera_info_file', default_value=default_camera_info,
            description='Kalibrasyon YAML. Bos verilirse CameraInfo YAYINLANMAZ.'),
        OpaqueFunction(function=_start_camera),
    ])
