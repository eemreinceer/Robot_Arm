#!/usr/bin/env python3
"""Calisan robotun /robot_description'ini dosyaya yazar.

Sifir-poz hayaleti icin ikinci bir robot_state_publisher'a URDF lazim. Bunu
xacro ile yeniden uretmek yerine CALISAN robotun yayinladigini aliyoruz:
(1) konteynerde xacro kurulu olmayabilir, (2) daha onemlisi, hayaletin canli
modelle BIREBIR ayni URDF'ten gelmesi garanti olur -- ayri uretilen bir URDF
sessizce farkli olursa hizalama olcumu bozulur.

Topic TRANSIENT LOCAL yayinlanir, yani gec baglanan da son mesaji alir.

Kullanim:
    python3 dump_robot_description.py /tmp/robot_arm_ref.urdf [--timeout 15]
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out")
    ap.add_argument("--topic", default="/robot_description")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("robot_description_dumper")
    received = {}

    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(
        String, args.topic, lambda m: received.setdefault("urdf", m.data), qos)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and "urdf" not in received:
        if node.get_clock().now().nanoseconds > deadline:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(f"{args.topic} {args.timeout}s icinde gelmedi — "
                     "robot_state_publisher ayakta mi, DDS ayarlari dogru mu?")
        rclpy.spin_once(node, timeout_sec=0.2)

    urdf = received["urdf"]
    if "<robot" not in urdf:
        sys.exit(f"{args.topic} icerigi URDF'e benzemiyor ({len(urdf)} bayt)")
    with open(args.out, "w") as f:
        f.write(urdf)
    print(f"{args.out} yazildi ({len(urdf)} bayt)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
