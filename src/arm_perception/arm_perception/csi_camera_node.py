#!/usr/bin/env python3
"""Raspberry Pi Camera V2 karelerini Jetson Argus ya da Pi 5 libcamera uzerinden yayinla."""
import os
import time

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def validate_flip_method(value) -> int:
    """Return a validated nvvidconv flip-method value."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'flip_method tam sayi olmali: {value!r}')
    flip_method = value
    if flip_method not in range(4):
        raise ValueError(
            f'flip_method 0..3 araliginda olmali, gelen: {flip_method}')
    return flip_method


def validate_camera_info_contract(
        data: dict, output_width: int, output_height: int,
        configured_flip_method: int) -> int:
    """Validate resolution and orientation bound to an intrinsics YAML."""
    configured_flip = validate_flip_method(configured_flip_method)
    if 'flip_method' not in data:
        raise ValueError(
            'kalibrasyon YAML flip_method icermiyor; yonu bilinmeyen '
            'intrinsics kullanilamaz')
    calibration_flip = validate_flip_method(data['flip_method'])
    if calibration_flip != configured_flip:
        raise ValueError(
            f'kalibrasyon flip_method={calibration_flip}, kamera '
            f'flip_method={configured_flip}; CameraInfo yayinlanmayacak')

    width = int(data['image_width'])
    height = int(data['image_height'])
    if (width, height) != (int(output_width), int(output_height)):
        raise ValueError(
            f'kalibrasyon {width}x{height} icin, ama yayin '
            f'{output_width}x{output_height}; CameraInfo yayinlanmayacak')
    return calibration_flip


def camera_info_from_calibration(
        data: dict, output_width: int, output_height: int,
        configured_flip_method: int) -> CameraInfo:
    """Build CameraInfo only after the resolution/flip contract passes."""
    validate_camera_info_contract(
        data, output_width, output_height, configured_flip_method)
    info = CameraInfo()
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
    info.k = [float(v) for v in data['camera_matrix']['data']]
    info.d = [float(v) for v in data['distortion_coefficients']['data']]
    info.r = [float(v) for v in data['rectification_matrix']['data']]
    info.p = [float(v) for v in data['projection_matrix']['data']]
    return info


def argus_gstreamer_pipeline(
        sensor_id: int, capture_width: int, capture_height: int,
        output_width: int, output_height: int, frame_rate: int,
        flip_method: int) -> str:
    """Build an OpenCV appsink pipeline for nvarguscamerasrc."""
    return (
        'nvarguscamerasrc sensor-id={sensor_id} ! '
        'video/x-raw(memory:NVMM), width=(int){capture_width}, '
        'height=(int){capture_height}, framerate=(fraction){frame_rate}/1, '
        'format=(string)NV12 ! nvvidconv flip-method={flip_method} ! '
        'video/x-raw, width=(int){output_width}, height=(int){output_height}, '
        'format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    ).format(
        sensor_id=sensor_id, capture_width=capture_width,
        capture_height=capture_height, frame_rate=frame_rate,
        flip_method=flip_method, output_width=output_width,
        output_height=output_height)


# nvvidconv flip-method -> GStreamer videoflip method. Sayilar ayni SEYI
# ifade etmiyor, bu yuzden esleme acikca yazilir:
#   0 none | 1 saat yonunun TERSI 90 | 2 180 derece | 3 saat yonunde 90
_VIDEOFLIP_BY_NVVIDCONV = {
    0: 'none',
    1: 'counterclockwise',
    2: 'rotate-180',
    3: 'clockwise',
}


def libcamera_gstreamer_pipeline(
        capture_width: int, capture_height: int,
        output_width: int, output_height: int, frame_rate: int,
        flip_method: int, camera_name: str = '') -> str:
    """Build an OpenCV appsink pipeline for libcamerasrc (Raspberry Pi 5).

    Argus yolunun karsiligi. Iki yol arasindaki farklar KASITLI olarak burada
    toplanir, node govdesine sizmaz:

    * `libcamerasrc` sensor-id almaz, `camera-name` alir; bos birakilirsa ilk
      kamera secilir.
    * Dondurme `nvvidconv flip-method` ile degil `videoflip method` ile yapilir
      ve sayilar ayni anlama GELMEZ -- esleme `_VIDEOFLIP_BY_NVVIDCONV`.
      flip_method parametresi nvvidconv anlamini korur, cunku kalibrasyon
      sozlesmesi (`validate_camera_info_contract`) o sayiya bagli.
    * Olcekleme NVMM icinde degil `videoscale` ile yapilir; ISP davranisi
      Jetson'inkinden FARKLIDIR, dolayisiyla intrinsics ve algi tabani bu
      platformda YENIDEN olculmelidir (Jetson'da alinan degerler tasinmaz).
    * `libcamerasrc` sonrasi caps'e `format=(string)NV12` ACIKCA verilir.
      2026-08-19'da Pi 5 + IMX219'da OLCULDU: format alani BOS birakilinca
      libcamera 1640x1232 icin ISP'yi devre disi birakip ham SBGGR/RAW
      akisi seciyor (sensor'un bu cozunurlukteki dogal modu budur), ve
      `videoconvert`/`videoflip` bu formati islemedigi icin pipeline
      "Internal data stream error" ile aciliyor. NV12 istendiginde ayni
      cozunurlukte ISP devreye giriyor ("1640x1232-NV12/Rec709").

    ⚠ Bring-up'ta hala KONTROL EDILECEK: gercek kare hizi ve flip yonu servo
    ile canli goruntude dogrulanmadi.
    """
    validate_flip_method(flip_method)
    source = 'libcamerasrc'
    if camera_name:
        source += f' camera-name={camera_name}'
    return (
        '{source} ! '
        'video/x-raw, width=(int){capture_width}, '
        'height=(int){capture_height}, framerate=(fraction){frame_rate}/1, '
        'format=(string)NV12 ! '
        'videoflip method={videoflip} ! videoconvert ! videoscale ! '
        'video/x-raw, width=(int){output_width}, height=(int){output_height}, '
        'format=(string)BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    ).format(
        source=source, capture_width=capture_width,
        capture_height=capture_height, frame_rate=frame_rate,
        videoflip=_VIDEOFLIP_BY_NVVIDCONV[int(flip_method)],
        output_width=output_width, output_height=output_height)


def detect_backend() -> str:
    """Platformdan arka ucu kestir: Tegra ise argus, degilse libcamera.

    `/etc/nv_tegra_release` yalniz L4T kurulumlarinda bulunur. Kestirim
    yanlissa `backend` parametresiyle acikca verilir; node hangi yolu
    sectigini her acilista LOGLAR, cunku sessizce yanlis arka uce dusmek
    "kamera acilmiyor" olarak gorunur ve saatler yakar.
    """
    return 'argus' if os.path.exists('/etc/nv_tegra_release') else 'libcamera'


class CsiCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('csi_camera_node')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('capture_width', 1640)
        self.declare_parameter('capture_height', 1232)
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('flip_method', 0)
        # 'argus' (Jetson) | 'libcamera' (Raspberry Pi 5) | 'auto'
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('camera_name', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('reopen_delay_s', 2.0)
        # Kamera hic kare vermiyorsa node'un SESSIZCE ayakta kalmamasi icin.
        # 2026-08-21'de olculdu: yanlis libcamera'ya dusen node topic'i acar,
        # surec yasar, %0.1 CPU yakar ve tek kare basmaz -- disaridan "calisiyor"
        # gorunur. Bu sure icinde tek bir kare bile gelmezse node OLUR; launch'un
        # sinirli yeniden baslatma butcesi devreye girer ve tukenince gurultuyle
        # kapanir. 0 = watchdog kapali.
        self.declare_parameter('first_frame_timeout_s', 15.0)
        # Kalibrasyon PC'de uretilir (scripts/solve_camera_intrinsics.py),
        # repoya commit'lenir ve buraya yol olarak verilir.
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # Pipeline acildiktan sonra parametreyi degistirmek nvvidconv'i
        # yeniden kurmaz. Bu nedenle gecersiz degeri node baslangicinda reddet.
        self.flip_method = validate_flip_method(
            self.get_parameter('flip_method').value)
        backend = str(self.get_parameter('backend').value).strip().lower()
        if backend not in ('auto', 'argus', 'libcamera'):
            raise ValueError(
                f"backend 'auto', 'argus' ya da 'libcamera' olmali: {backend!r}")
        self.backend = detect_backend() if backend == 'auto' else backend

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            qos_profile_sensor_data)
        self.camera_info = self._load_camera_info()
        self.capture = None
        self.last_open_attempt = 0.0
        self.started_at = time.monotonic()
        self.frames_published = 0
        self.first_frame_timeout_s = float(
            self.get_parameter('first_frame_timeout_s').value)
        frame_rate = max(1, int(self.get_parameter('frame_rate').value))
        self.timer = self.create_timer(1.0 / frame_rate, self._publish_frame)
        self.get_logger().info(
            f'CSI camera publisher: backend={self.backend} '
            f'(parametre: {backend}), flip_method={self.flip_method}')

    def _load_camera_info(self):
        """Kalibrasyon YAML'ini CameraInfo'ya cevirir.

        Dosya verilmemisse CameraInfo YAYINLANMAZ. Bilincli tercih: bos ya da
        birim bir CameraInfo yayinlamak, asagi akistaki her seye 'bu kamera
        kalibre' demek olur ve hatayi sessizce metrik sonuclara tasir.
        """
        path = str(self.get_parameter('camera_info_file').value).strip()
        if not path:
            self.get_logger().warning(
                'camera_info_file verilmedi — CameraInfo YAYINLANMAYACAK. '
                'Kalibrasyon icin: scripts/capture_calib_images.py + '
                'scripts/solve_camera_intrinsics.py')
            return None
        if not os.path.exists(path):
            self.get_logger().error(f'camera_info_file bulunamadi: {path}')
            return None
        try:
            with open(path) as handle:
                data = yaml.safe_load(handle)
            out_w = int(self.get_parameter('output_width').value)
            out_h = int(self.get_parameter('output_height').value)
            info = camera_info_from_calibration(
                data, out_w, out_h, self.flip_method)
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
            self.get_logger().error(
                f'camera_info_file okunamadi ({path}): {error}')
            return None
        self.get_logger().info(
            f'CameraInfo yuklendi: {path} ({info.width}x{info.height}, '
            f'flip {self.flip_method}, '
            f'rms {data.get("calibration_rms_px", "?")} px)')
        return info

    def _pipeline(self) -> str:
        value = lambda name: int(self.get_parameter(name).value)
        if self.backend == 'libcamera':
            return libcamera_gstreamer_pipeline(
                value('capture_width'), value('capture_height'),
                value('output_width'), value('output_height'),
                value('frame_rate'), self.flip_method,
                str(self.get_parameter('camera_name').value).strip())
        return argus_gstreamer_pipeline(
            value('sensor_id'), value('capture_width'), value('capture_height'),
            value('output_width'), value('output_height'), value('frame_rate'),
            self.flip_method)

    def _ensure_capture(self) -> bool:
        if self.capture is not None and self.capture.isOpened():
            return True
        now = time.monotonic()
        delay = float(self.get_parameter('reopen_delay_s').value)
        if now - self.last_open_attempt < delay:
            return False
        self.last_open_attempt = now
        if self.capture is not None:
            self.capture.release()
        self.capture = cv2.VideoCapture(self._pipeline(), cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            source = ('nvarguscamerasrc' if self.backend == 'argus'
                      else 'libcamerasrc')
            self.get_logger().error(
                f'IMX219 {source} uzerinden acilamadi; yeniden denenecek',
                throttle_duration_sec=5.0)
            return False
        self.get_logger().info(f'IMX219 akisi acildi (backend={self.backend})')
        return True

    def _publish_frame(self) -> None:
        self._check_frame_watchdog()
        if not self._ensure_capture():
            return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning(
                'kare okunamadi; akis yeniden acilacak')
            self.capture.release()
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        self.publisher.publish(message)
        self.frames_published += 1
        if self.camera_info is not None:
            # Ayni header: abonelerin goruntu ile intrinsics'i eslestirmesi
            # zaman damgasina dayanir.
            self.camera_info.header = message.header
            self.info_publisher.publish(self.camera_info)

    def _check_frame_watchdog(self) -> None:
        """Hic kare gelmeden gecen sureyi denetle ve gerekirse GURULTUYLE ol.

        Sessiz basarisizligin bedeli olculdu (2026-08-21): topic acik, surec
        canli, sifir kare -- ve arizanin nerede oldugu ancak `ros2 topic hz`
        elle kosulunca goruldu. Node'un kendisi bunu soylemeli.
        """
        if self.frames_published or self.first_frame_timeout_s <= 0.0:
            return
        waited = time.monotonic() - self.started_at
        if waited < self.first_frame_timeout_s:
            return
        self.get_logger().error(
            f'{waited:.1f} s icinde TEK KARE gelmedi (backend={self.backend}). '
            'libcamera yolunda en sik sebep: GST_PLUGIN_PATH dogru libcamera '
            'eklentisini gostermiyor ve apt surumunun IPA dizini bos. '
            'Kontrol: gst-launch-1.0 libcamerasrc ! fakesink')
        raise SystemExit(1)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
