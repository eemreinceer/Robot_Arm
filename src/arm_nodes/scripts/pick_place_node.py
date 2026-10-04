#!/usr/bin/env python3
"""Placeholder Python entry point.

The production implementation is the C++ `pick_place_node`. This script exists so
launch/test tooling has a Python hook while the ROS2 interface package is being
integrated.
"""

import rclpy


def main():
    rclpy.init()
    node = rclpy.create_node('pick_place_node_py_placeholder')
    node.get_logger().warn('Use the C++ pick_place_node executable for pick/place execution.')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
