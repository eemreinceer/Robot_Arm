#!/usr/bin/env python3
"""Capture a Gazebo YOLO dataset with projected 6DOF ground truth."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time
from typing import Dict, Sequence

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ament_index_python.packages import get_package_share_directory
from arm_perception.domain_randomization import (
    augment_image_and_bbox,
    sample_augmentation,
    yolo_label,
)

CLASSES = [
    {'name': 'red_box', 'model_type': 'box', 'dimensions': [0.05, 0.05, 0.05], 'extents': [0.05, 0.05, 0.05]},
    {'name': 'yellow_cylinder', 'model_type': 'cylinder', 'dimensions': [0.025, 0.05], 'extents': [0.05, 0.05, 0.05]},
    {'name': 'blue_cube', 'model_type': 'box', 'dimensions': [0.04, 0.04, 0.04], 'extents': [0.04, 0.04, 0.04]},
]


class CaptureDataset(Node):
    def __init__(self, args) -> None:
        super().__init__('capture_perception_dataset')
        self.args = args
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_info = None
        self.model_root = Path(get_package_share_directory('arm_gazebo')) / 'models'
        self.create_subscription(Image, args.image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._on_camera_info, qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        self.latest_image = message

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def wait_until_ready(self) -> None:
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and (self.latest_image is None or self.camera_info is None):
            if time.monotonic() > deadline:
                raise RuntimeError('Camera topics are unavailable')
            rclpy.spin_once(self, timeout_sec=0.2)

    def spawn_object(self, name: str, model: Dict, pose: Pose) -> None:
        model_file = self.model_root / model['name'] / 'model.sdf'
        if not model_file.is_file():
            raise RuntimeError(f'Gazebo dataset model is missing: {model_file}')
        request = (
            f'sdf_filename: "{model_file}" name: "{name}" '
            f'pose: {{position: {{x: {pose.position.x} y: {pose.position.y} z: {pose.position.z}}} '
            f'orientation: {{x: {pose.orientation.x} y: {pose.orientation.y} '
            f'z: {pose.orientation.z} w: {pose.orientation.w}}}}}')
        command = [
            'gz', 'service', '-s', '/world/pick_and_place_world/create',
            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', request]
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'Unable to spawn {name}: {result.stderr.strip()}')

    def remove_object(self, name: str) -> None:
        command = [
            'gz', 'service', '-s', '/world/pick_and_place_world/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', f'name: "{name}" type: MODEL']
        subprocess.run(command, check=False, capture_output=True, text=True)

    def wait_for_fresh_image(self, previous_stamp: int, minimum_frames: int = 1):
        deadline = time.monotonic() + 5.0
        frames = 0
        image = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            stamp = self.latest_image.header.stamp
            current = stamp.sec * 1_000_000_000 + stamp.nanosec
            if current != previous_stamp:
                previous_stamp = current
                frames += 1
                image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                if frames >= minimum_frames:
                    return image
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for camera image')


def object_color_mask(image: np.ndarray, object_type: str):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation = hsv[:, :, 0], hsv[:, :, 1]
    if object_type == 'red_box':
        hue_match = (hue < 12) | (hue > 168)
    elif object_type == 'yellow_cylinder':
        hue_match = (hue > 18) & (hue < 42)
    elif object_type == 'blue_cube':
        hue_match = (hue > 108) & (hue < 132)
    else:
        raise ValueError(f'Unsupported synthetic dataset object: {object_type}')
    return (hue_match & (saturation > 75)).astype(np.uint8)


def bbox_from_frame_difference(before: np.ndarray, after: np.ndarray, object_type: str):
    difference = cv2.absdiff(after, before)
    gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
    mask = ((gray > 20) & (object_color_mask(after, object_type) > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    components = [stats[index] for index in range(1, count) if stats[index, cv2.CC_STAT_AREA] >= 25]
    if not components:
        raise RuntimeError(f'Spawned {object_type} is not visible in the rendered camera frame')
    x, y, width, height, _ = max(components, key=lambda item: item[cv2.CC_STAT_AREA])
    return float(x), float(y), float(x + width - 1), float(y + height - 1)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy)


def random_pose(rng: np.random.Generator, model: Dict) -> Pose:
    pose = Pose()
    pose.position.x = float(rng.uniform(0.32, 0.45))
    pose.position.y = float(rng.uniform(-0.12, 0.12))
    pose.position.z = 0.60 + float(model['extents'][2]) / 2.0
    # Keep dynamic table objects stable so the recorded spawn pose remains valid GT.
    yaw = float(rng.uniform(-np.pi, np.pi))
    qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, yaw)
    pose.orientation.x, pose.orientation.y = float(qx), float(qy)
    pose.orientation.z, pose.orientation.w = float(qz), float(qw)
    return pose


def pose_relative_to_base(world_pose: Pose, base_world_xyz: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x = world_pose.position.x - float(base_world_xyz[0])
    pose.position.y = world_pose.position.y - float(base_world_xyz[1])
    pose.position.z = world_pose.position.z - float(base_world_xyz[2])
    pose.orientation = world_pose.orientation
    return pose


def choose_split(rng: np.random.Generator) -> str:
    value = rng.random()
    return 'train' if value < 0.70 else ('val' if value < 0.85 else 'test')


def prepare_output(root: Path, append: bool = False) -> None:
    if root.exists() and not append:
        shutil.rmtree(root)
    for split in ('train', 'val', 'test'):
        (root / 'images' / split).mkdir(parents=True, exist_ok=True)
        (root / 'labels' / split).mkdir(parents=True, exist_ok=True)
    (root / 'gt').mkdir(parents=True, exist_ok=True)
    config = {
        'path': str(root.resolve()),
        'train': 'images/train', 'val': 'images/val', 'test': 'images/test',
        'names': {index: item['name'] for index, item in enumerate(CLASSES)}}
    (root / 'data.yaml').write_text(yaml.safe_dump(config, sort_keys=False))


def parse_args():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=package / 'datasets' / 'pick_objects')
    parser.add_argument('--samples', type=int, default=3000)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--settle-seconds', type=float, default=0.7)
    parser.add_argument('--fresh-frames', type=int, default=3)
    parser.add_argument('--max-attempts', type=int, default=10)
    parser.add_argument('--append', action='store_true')
    parser.add_argument('--base-world-x', type=float, default=0.0)
    parser.add_argument('--base-world-y', type=float, default=0.0)
    parser.add_argument('--base-world-z', type=float, default=0.6)
    parser.add_argument('--image-topic', default='/camera/image')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.settle_seconds < 0.5:
        raise ValueError('--settle-seconds must be at least 0.5 for the 8 Hz camera')
    if args.fresh_frames < 1:
        raise ValueError('--fresh-frames must be positive')
    if args.max_attempts < 1:
        raise ValueError('--max-attempts must be positive')
    rng = np.random.default_rng(args.seed)
    prepare_output(args.output, append=args.append)
    rclpy.init()
    node = CaptureDataset(args)
    annotations = args.output / 'gt' / 'annotations.jsonl'
    start_index = 0
    if args.append and annotations.is_file():
        with annotations.open(encoding='utf-8') as existing:
            start_index = sum(1 for _ in existing)
    try:
        node.wait_until_ready()
        node.remove_object('pick_object')
        with annotations.open('a', encoding='utf-8') as output:
            for captured in range(args.samples):
                index = start_index + captured
                model_id = index % len(CLASSES)
                model = CLASSES[model_id]
                for attempt in range(args.max_attempts):
                    name = f'dataset_{index:06d}_{model["name"]}_{attempt:02d}'
                    world_pose = random_pose(rng, model)
                    base_pose = pose_relative_to_base(
                        world_pose, [args.base_world_x, args.base_world_y, args.base_world_z])
                    previous = node.latest_image.header.stamp
                    previous_stamp = previous.sec * 1_000_000_000 + previous.nanosec
                    background = node.wait_for_fresh_image(previous_stamp, args.fresh_frames)
                    previous = node.latest_image.header.stamp
                    previous_stamp = previous.sec * 1_000_000_000 + previous.nanosec
                    node.spawn_object(name, model, world_pose)
                    try:
                        deadline = time.monotonic() + args.settle_seconds
                        while time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        image = node.wait_for_fresh_image(previous_stamp, args.fresh_frames)
                        bbox = bbox_from_frame_difference(background, image, model['name'])
                        break
                    except RuntimeError as exc:
                        if attempt + 1 >= args.max_attempts:
                            raise
                        node.get_logger().warning(
                            f'Resampling {model["name"]} after invisible attempt {attempt + 1}: {exc}')
                    finally:
                        node.remove_object(name)
                augmentation = sample_augmentation(rng)
                image, bbox = augment_image_and_bbox(image, bbox, augmentation)
                height, width = image.shape[:2]
                split = choose_split(rng)
                stem = f'{index:06d}_{model["name"]}'
                cv2.imwrite(str(args.output / 'images' / split / f'{stem}.jpg'), image)
                label = yolo_label(model_id, bbox, width, height)
                (args.output / 'labels' / split / f'{stem}.txt').write_text(label + '\n')
                record = {
                    'image': f'images/{split}/{stem}.jpg', 'class': model['name'],
                    'bbox_xyxy': list(map(float, bbox)),
                    'pose_base_link': {
                        'position': [base_pose.position.x, base_pose.position.y, base_pose.position.z],
                        'orientation': [base_pose.orientation.x, base_pose.orientation.y,
                                        base_pose.orientation.z, base_pose.orientation.w]},
                    'dimensions': model['dimensions'],
                    'augmentation': augmentation.__dict__}
                output.write(json.dumps(record) + '\n')
                output.flush()
                node.get_logger().info(f'Captured {captured + 1}/{args.samples}: {stem}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
