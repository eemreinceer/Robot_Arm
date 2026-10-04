#!/usr/bin/env python3
"""Kamera akışlarını düşük hızda yeniden yayınlar (kayıt için).

Sim RGBD kamerası ~30 Hz basıyor: /camera/image 26.8 MB/s, /camera/points
~150 MB/s (ölçüm, 640x480). Ham hızda kaydedilen 80 saniyelik bir bag ~17 GB
olur. Bu düğüm topic'leri örnekleyip `<topic>_throttle` adıyla yeniden yayınlar;
`record_sim_mcap.sh` throttle'lı adları kaydeder.

ros-jazzy-topic-tools kurulu olmadığı için bağımlılıksız yerel karşılığı.

    python3 scripts/sensor_throttle.py \
        --spec /camera/image:sensor_msgs/msg/Image:5.0

Varsayılan spec listesi sim kamerasının üç akışıdır.
"""

import argparse
import importlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

DEFAULT_SPECS = [
    "/camera/image:sensor_msgs/msg/Image:10.0",
    "/camera/depth/image_raw:sensor_msgs/msg/Image:5.0",
    "/camera/points:sensor_msgs/msg/PointCloud2:2.0",
    "/camera/camera_info:sensor_msgs/msg/CameraInfo:10.0",
]


def import_msg(type_str: str):
    pkg, kind, name = type_str.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


class Throttle(Node):
    def __init__(self, specs):
        super().__init__("sensor_throttle")
        self.counts = {}
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        for spec in specs:
            topic, type_str, rate = spec.rsplit(":", 2)
            msg_type = import_msg(type_str)
            out_topic = f"{topic}_throttle"
            pub = self.create_publisher(msg_type, out_topic, qos)
            period_ns = int(1e9 / float(rate))
            state = {"last": 0, "period": period_ns, "pub": pub, "n": 0}
            self.counts[out_topic] = state
            self.create_subscription(
                msg_type, topic,
                lambda msg, s=state: self._relay(msg, s),
                qos,
            )
            self.get_logger().info(f"{topic} → {out_topic} @ {rate} Hz")

    def _relay(self, msg, state):
        now = self.get_clock().now().nanoseconds
        if now - state["last"] < state["period"]:
            return
        state["last"] = now
        state["n"] += 1
        state["pub"].publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--spec", action="append", default=None,
                    help="topic:mesaj/tipi:hz — birden çok kez verilebilir")
    args = ap.parse_args()

    rclpy.init()
    node = Throttle(args.spec or DEFAULT_SPECS)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for name, s in node.counts.items():
            node.get_logger().info(f"{name}: {s['n']} mesaj")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
