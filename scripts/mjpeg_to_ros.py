#!/usr/bin/env python3
"""Nano'nun MJPEG akisini PC tarafinda ROS 2 topic'i olarak yayinlar.

NEDEN VAR
  PC (Jazzy) ile Nano (Humble) arasinda DDS kesfi Tailscale uzerinden
  CALISMIYOR -- 2026-08-18'de olculdu: PC'de yalnizca kendi `/parameter_events`
  ve `/rosout`'u gorunuyor. Web konsolu ise PC'de kosuyor (Nano konteyneri
  Python 3.6, konsol 3.7+ hedefliyor) ve kamerayi ROS topic'inden okuyor.
  Bu kopru o boslugu HTTP uzerinden kapatir: `mjpeg_bridge` zaten Nano'da
  8080'de yayinda.

⚠ OLCUM ICIN KULLANMA -- KALITE 100 HARIC
  Buradaki kareler `mjpeg_bridge` tarafindan JPEG'e sikistirilmistir; kamera
  dugumunun yayinladigi ham kareler degildir. Sikistirmanin kose tespitine
  etkisi 2026-08-18'de OLCULDU (ayni kare, sikistirilmis ve ham, kose kose):

    q= 80 (kopru varsayilani) : medyan 0.059 px, p90 0.125
    q= 95                     : medyan 0.014 px, p90 0.031
    q=100                     : medyan 0.005 px, p90 0.010

  Kiyas icin: ayni kurulumda kare-arasi kose gurultusu ~0.072 px ve pesinde
  olunan sistematik artik 0.19-0.31 px. Yani VARSAYILAN kalitede kopru
  olcumu bozar, q=100'de katkisi gurultunun onda biri kalir ve ihmal
  edilebilir. Kalibrasyon gibi hassas isler icin koprunun kaynagini
  `-p jpeg_quality:=100` ile kaldirmak SART.

  Zaman damgasi her halukarda PC saatidir, yakalama ani degil -- zamanlamaya
  bagli olcumler buradan yapilmaz.

KULLANIM
  python3 scripts/mjpeg_to_ros.py \
      --url http://robot-arm.local:8080/stream \
      --info-json data/camera/nano_imx219_info.json
"""

import argparse
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

BOUNDARY_MARKER = b'\xff\xd8'          # JPEG SOI
END_MARKER = b'\xff\xd9'               # JPEG EOI


def build_camera_info(spec, stamp, frame_id):
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = int(spec['width'])
    info.height = int(spec['height'])
    info.distortion_model = str(spec['distortion_model'])
    info.d = [float(v) for v in spec['d']]
    info.k = [float(v) for v in spec['k']]
    info.r = [float(v) for v in spec['r']]
    info.p = [float(v) for v in spec['p']]
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=os.environ.get('ROBOT_ARM_MJPEG_URL'),
                    help='MJPEG stream URL; may also be set with '
                         'ROBOT_ARM_MJPEG_URL')
    ap.add_argument('--info-json', required=True,
                    help='Nano /camera/camera_info alanlarini tasiyan JSON; '
                         'intrinsics ELDE UYDURULMAZ, cihazdan alinir')
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--info-topic', default='/camera/camera_info')
    ap.add_argument('--frame-id', default='')
    args = ap.parse_args()
    if not args.url:
        ap.error('--url or ROBOT_ARM_MJPEG_URL is required')

    spec = json.load(open(args.info_json))
    frame_id = args.frame_id or spec.get('frame_id', 'camera_optical_frame')

    rclpy.init()
    node = Node('mjpeg_to_ros')
    image_pub = node.create_publisher(Image, args.image_topic,
                                      qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, args.info_topic,
                                     qos_profile_sensor_data)
    node.get_logger().info(
        f'{args.url} -> {args.image_topic} (JPEG kopru, OLCUM ICIN DEGIL)')

    stream = urllib.request.urlopen(args.url, timeout=10)
    buffer = b''
    frames = 0
    started = time.time()
    try:
        while rclpy.ok():
            chunk = stream.read(4096)
            if not chunk:
                node.get_logger().error('akis kapandi')
                break
            buffer += chunk
            start = buffer.find(BOUNDARY_MARKER)
            end = buffer.find(END_MARKER, start + 2)
            if start < 0 or end < 0:
                continue
            jpeg, buffer = buffer[start:end + 2], buffer[end + 2:]
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            stamp = node.get_clock().now().to_msg()
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = frame_id
            message.height, message.width = rgb.shape[:2]
            message.encoding = 'rgb8'
            message.is_bigendian = 0
            message.step = message.width * 3
            message.data = rgb.tobytes()
            image_pub.publish(message)
            info_pub.publish(build_camera_info(spec, stamp, frame_id))

            frames += 1
            if frames % 150 == 0:
                node.get_logger().info(
                    f'{frames} kare, {frames / (time.time() - started):.1f} FPS')
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
