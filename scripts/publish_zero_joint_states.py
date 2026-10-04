#!/usr/bin/env python3
"""Sifir-poz hayaleti icin damgali JointState yayinlar.

Neden ayri bir script: `ros2 topic pub` header.stamp'i 0 birakir.
robot_state_publisher TF'i o damgayla yayinlar ve tf2 sifir damgali donusumu
kullanilabilir bir kayit saymaz -- hayalet RViz'de hic gorunmez. Burada her
mesaj gercek saatle damgalanir.

Kullanim:
    python3 publish_zero_joint_states.py [--topic /ref/joint_states] [--rate 10]
"""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/ref/joint_states")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--positions", default=None,
                    help="virgullu 6 deger; varsayilan hepsi 0.0")
    args = ap.parse_args()

    positions = [0.0] * len(JOINTS)
    if args.positions:
        parts = [float(v) for v in args.positions.split(",")]
        if len(parts) != len(JOINTS):
            raise SystemExit(f"--positions {len(JOINTS)} deger olmali")
        positions = parts

    rclpy.init()
    node = Node("robot_arm_zero_joint_state_publisher")
    pub = node.create_publisher(JointState, args.topic, 10)

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = positions
        pub.publish(msg)

    node.create_timer(1.0 / args.rate, tick)
    node.get_logger().info(
        f"{args.topic} uzerinde sifir poz yayinlaniyor ({args.rate} Hz)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
