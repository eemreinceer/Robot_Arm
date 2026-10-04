#!/usr/bin/env python3
"""
Robot Arm vizyon-encoder düğümü — eye-in-hand kameradan board pozu.

Kamera (KOLA MONTE, eye-in-hand) SABİT bir düz checkerboard'a bakar; bu düğüm
her karede board'un kamera frame'indeki pozunu (rvec/tvec → PoseStamped + TF)
çıkarır. Encoder olmayan bu kolda "sanal encoder"ın ilk halkası budur:
kamera↔board pozu → (hand-eye ile) TCP pozu → (IK ile) eklem kestirimi
(sonraki fazlar).

Bu düğüm kameranın sabit olduğunu VARSAYMAZ: poz her zaman canlı kamera
frame'inde (camera_optical_frame) yayınlanır, dünyaya bağlanma işi TF'e bırakılır
— yani eye-in-hand ile doğru çalışır. AMA zinciri kapatan halka HENÜZ YOK:
camera_link, link_5'e ÖLÇÜLMEMİŞ (şu an 0) bir transformla bağlı
(robot_arm_camera.xacro). Ölçüm/hand-eye kalibrasyonu yapılmadan bu pozdan
base_link'e geçen hiçbir sonuç geçerli değildir.

TASARIM KARARLARI:
* Intrinsics DOSYADAN DEĞİL, canlı /camera/camera_info'dan alınır — yayıncı
  (csi_camera_node) hangi K/D'yi, hangi flip/çözünürlükte yayınlıyorsa PnP onunla
  tutarlı olur; iki ayrı kaynak sessizce ayrışamaz.
* REPROJECTION KAPISI (reproj_gate_px): board pozu ancak reprojection hatası
  eşiğin altındaysa "güvenilir" sayılıp PoseStamped + TF olarak yayınlanır.
  Offline doğrulama (bkz. run_board_pnp_offline.py) temiz karelerde ~0.2px,
  uç/eğik karelerde 4-6px verdi → eşik bu ikisini ayırır, kötü pozu reddeder.

  🔴 EŞİK 2026-07-28'de 0.5 → 1.0 PX YÜKSELTİLDİ, CANLI ÖLÇÜMLE.
  Eski 0.5px değeri cepheden çekilmiş KALİBRASYON karelerinden türetilmişti ve
  gerçek çalışma geometrisini reddediyordu. Tahta masaya yatık, kamera eye-in-hand
  ve ona eğik bakarken ölçüm (40 kare):

      reprojeksiyon   ort 0.72px (0.54-0.82)   <- eski kapı hepsini REDDEDERDİ
      pos x std       0.037 mm
      pos y std       0.167 mm
      pos z std       0.100 mm
      açı sapması     ort 0.11 derece, maks 0.30 derece

  Aynı sahnede dik duran tahtayla karşılaştırma: reprojeksiyon 0.21px (daha iyi)
  ama pos z std 0.401mm ve açı sapması maks 1.05 derece (3-4x DAHA KÖTÜ).

  DERS: reprojeksiyon poz kalitesinin vekili DEĞİLDİR. Düzlemsel PnP cepheden
  bakışta dejenere rejime yaklaşır (derinlik/eğim zayıf kısıtlanır); eğik bakış
  perspektif ipucu verip pozu daha iyi koşullandırır. Yüksek reprojeksiyon burada
  gürültü değil, kare kenarlarındaki distorsiyon artığı — sistematik, dolayısıyla
  tekrarlanabilirliği bozmuyor.

  Kapı yine de anlamlı: gerçek yanlış eşleşme (ters köşe sıralaması) sentetik
  ölçümde 3.39px verdi. 1.0px, çalışma rejimini (<0.85px) geçirip gerçek
  hatayı (3px+) reddeder.
* `/vision_encoder/observation` kabul için kullanılan TEK atomik kontrattır;
  stamp görüntünün stamp'idir ve pose/kalite/tahta geometrisi aynı kareden gelir.
  Eski `board_pose`, `detected` ve `reprojection_px` topic'leri yalnız RViz ve
  debug içindir; kabul verisi olarak eşleştirilmemelidir.
* reprojection_px her tespitte (kapı geçsin geçmesin) ayrı debug topic'inde
  yayınlanır ki eşik canlıda ölçüyle ayarlanabilsin.

SINIRLAR (board_pnp.py'den miras): detektör sıralaması ancak renk paritesi
180° dönüşü ayırt eden bir checkerboard geometrisinde kararlıdır. Güncel 6x9
iç-köşe tahta bu kapıyı geçer; eski 6x8 tahta geçmez. PnP reprojection marjı
bu kararı veremez. Ayrıca tüm board kadrajda olmalı. Bunlar canlıda ısırırsa
ChArUco'ya geçilir (make_charuco_board.py hazır).
"""
import math

from arm_perception.board_pnp import (
    checkerboard_ordering_ambiguous,
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
    estimate_board_pose,
)
from arm_interfaces.msg import BoardObservation
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(rmat):
    """
    3x3 dönme matrisi → (x, y, z, w) quaternion.

    tf_transformations bağımlılığından kaçınmak için elle: Shepperd yöntemi,
    en büyük köşegen bileşeninden türeterek sayısal kararlılığı korur.
    """
    m = rmat
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def build_board_observation(image_header, camera_frame, result, accepted,
                            board_cols, board_rows, square_size_mm):
    """Build one frame-attributed acceptance observation.

    ``detected`` means the pose passed every producer-side acceptance gate, not
    merely that OpenCV found corners.  When it is false, pose and reprojection
    deliberately remain at their message defaults as required by the shared
    interface contract.
    """
    observation = BoardObservation()
    observation.header.stamp = image_header.stamp
    observation.header.frame_id = camera_frame or image_header.frame_id
    observation.detected = bool(accepted and result is not None)
    observation.board_cols = int(board_cols)
    observation.board_rows = int(board_rows)
    observation.square_size_mm = float(square_size_mm)
    observation.corners_expected = int(board_cols) * int(board_rows)
    observation.corners_detected = (
        int(result.get('n_corners', 0)) if result is not None else 0)

    if not observation.detected:
        return observation

    rmat, _ = cv2.Rodrigues(result['rvec'])
    qx, qy, qz, qw = rotation_matrix_to_quaternion(rmat)
    tvec = np.asarray(result['tvec']).reshape(3)
    observation.pose.position.x = float(tvec[0])
    observation.pose.position.y = float(tvec[1])
    observation.pose.position.z = float(tvec[2])
    observation.pose.orientation.x = qx
    observation.pose.orientation.y = qy
    observation.pose.orientation.z = qz
    observation.pose.orientation.w = qw
    observation.reprojection_px = float(result['reproj_px'])
    return observation


class VisionEncoderNode(Node):
    """Publish atomic acceptance observations plus legacy debug topics."""

    def __init__(self):
        super().__init__('vision_encoder_node')
        # Bu geometri POZ hedefinindir; intrinsics'in çekildiği eski tahtayla
        # aynı olmak zorunda değildir (K nesne ölçeğinden bağımsızdır).
        # cols/rows = İÇ köşe sayısı. square_size_mm güncel baskıdan ölçülür;
        # pose ölçeği doğrudan buna bağlıdır.
        self.declare_parameter('board_cols', DEFAULT_BOARD_COLS)
        self.declare_parameter('board_rows', DEFAULT_BOARD_ROWS)
        self.declare_parameter('square_size_mm', DEFAULT_SQUARE_SIZE_MM)
        # Reprojection kapısı: bu pikselin üstündeki pozlar reddedilir (yayınlanmaz).
        # Offline: temiz ~0.2px, uç kareler 4-6px. 0.5 makul başlangıç; canlıda
        # reprojection_px topic'ini izleyip ayarla.
        # 1.0px: canlı çalışma rejimi <0.85px, gerçek yanlış eşleşme 3px+.
        # Gerekçe ve ölçüm için modül docstring'ine bak.
        self.declare_parameter('reproj_gate_px', 1.0)
        # Mesafe akıl-sağlığı: board bu aralık dışındaysa (m) tespiti güvenme.
        self.declare_parameter('min_distance_m', 0.05)
        self.declare_parameter('max_distance_m', 2.0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter(
            'observation_topic', '/vision_encoder/observation')
        self.declare_parameter('board_frame', 'vision_board')
        self.declare_parameter('publish_tf', True)

        self.board_cols = int(self.get_parameter('board_cols').value)
        self.board_rows = int(self.get_parameter('board_rows').value)
        self.square_size_mm = float(self.get_parameter('square_size_mm').value)
        corners_expected = self.board_cols * self.board_rows
        if not (1 <= self.board_cols <= 255 and 1 <= self.board_rows <= 255):
            raise ValueError('board_cols/board_rows uint8 araliginda olmali')
        if corners_expected > 65535:
            raise ValueError('beklenen kose sayisi uint16 araligini asmamali')
        if self.square_size_mm <= 0.0:
            raise ValueError('square_size_mm pozitif olmali')
        self.ordering_ambiguous = checkerboard_ordering_ambiguous(
            self.board_cols, self.board_rows)
        self.reproj_gate_px = float(self.get_parameter('reproj_gate_px').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.max_distance_m = float(self.get_parameter('max_distance_m').value)
        self.board_frame = str(self.get_parameter('board_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = 'camera_optical_frame'

        self.pose_pub = self.create_publisher(
            PoseStamped, '/vision_encoder/board_pose', 10)
        self.reproj_pub = self.create_publisher(
            Float32, '/vision_encoder/reprojection_px', 10)
        self.detected_pub = self.create_publisher(
            Bool, '/vision_encoder/detected', 10)
        self.observation_pub = self.create_publisher(
            BoardObservation,
            str(self.get_parameter('observation_topic').value),
            10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # CameraInfo görüntüyle aynı QoS'ta (sensor data / best_effort) gelir.
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'vision_encoder hazır — board {self.board_cols}x{self.board_rows} '
            f'iç köşe, {self.square_size_mm}mm, reproj kapısı '
            f'{self.reproj_gate_px}px. Intrinsics /camera/camera_info bekleniyor.')
        if self.ordering_ambiguous:
            self.get_logger().warning(
                'Checkerboard geometrisi 180 derece detektor siralamasini '
                'ayirt edemiyor; pose yonelimi operasyonel olarak ambiguous. '
                '6x9 ic-kose tahta veya kimlikli hedef kullanin.')

    def _on_camera_info(self, msg):
        """K/D'yi canlı intrinsics'ten al. Sıfır K (kalibresiz) reddedilir."""
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if k[0, 0] <= 0.0:
            self.get_logger().warning(
                'CameraInfo kalibresiz görünüyor (fx<=0) — PnP çalıştırılamaz. '
                'csi_camera_node camera_info_file ile başlatılmalı.',
                throttle_duration_sec=10.0)
            return
        self.camera_matrix = k
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    def _on_image(self, msg):
        if self.camera_matrix is None:
            self.observation_pub.publish(build_board_observation(
                msg.header, msg.header.frame_id, None, False,
                self.board_cols, self.board_rows, self.square_size_mm))
            self.get_logger().warning(
                'Intrinsics henüz gelmedi (/camera/camera_info) — kare atlandı.',
                throttle_duration_sec=5.0)
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = estimate_board_pose(
            gray, self.board_cols, self.board_rows, self.square_size_mm,
            self.camera_matrix, self.dist_coeffs)

        detected = Bool()
        detected.data = result is not None
        self.detected_pub.publish(detected)
        if result is None:
            self.observation_pub.publish(build_board_observation(
                msg.header, self.camera_frame, None, False,
                self.board_cols, self.board_rows, self.square_size_mm))
            self.get_logger().info(
                'board tespit edilmedi (kadrajda değil / kısmi / aşırı eğik)',
                throttle_duration_sec=3.0)
            return

        reproj = float(result['reproj_px'])
        distance = float(result['distance_m'])
        # reprojection'ı kapıdan bağımsız yayınla (eşiği canlıda ayarlamak için).
        self.reproj_pub.publish(Float32(data=reproj))

        # Kapılar: reprojection + mesafe akıl-sağlığı.
        if reproj > self.reproj_gate_px:
            self.observation_pub.publish(build_board_observation(
                msg.header, self.camera_frame, result, False,
                self.board_cols, self.board_rows, self.square_size_mm))
            self.get_logger().info(
                f'poz REDDEDİLDİ: reproj {reproj:.2f}px > kapı '
                f'{self.reproj_gate_px}px (mesafe {distance*100:.1f}cm)',
                throttle_duration_sec=2.0)
            return
        if not (self.min_distance_m <= distance <= self.max_distance_m):
            self.observation_pub.publish(build_board_observation(
                msg.header, self.camera_frame, result, False,
                self.board_cols, self.board_rows, self.square_size_mm))
            self.get_logger().warning(
                f'poz REDDEDİLDİ: mesafe {distance*100:.1f}cm aralık dışı '
                f'[{self.min_distance_m*100:.0f}, {self.max_distance_m*100:.0f}]cm',
                throttle_duration_sec=2.0)
            return

        observation = build_board_observation(
            msg.header, self.camera_frame, result, True,
            self.board_cols, self.board_rows, self.square_size_mm)
        self.observation_pub.publish(observation)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.camera_frame
        pose.pose = observation.pose
        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.camera_frame
            tf.child_frame_id = self.board_frame
            tf.transform.translation.x = observation.pose.position.x
            tf.transform.translation.y = observation.pose.position.y
            tf.transform.translation.z = observation.pose.position.z
            tf.transform.rotation = observation.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        self.get_logger().info(
            f'board pozu: mesafe {distance*100:.1f}cm  reproj {reproj:.2f}px  '
            f'köşe {result["n_corners"]}  flip {result["flipped"]}  '
            f'ordering_stable {result["ordering_stable"]}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = VisionEncoderNode()
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
