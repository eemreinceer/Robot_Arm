"""Spawn the class sorting bins and a randomized six-object Gazebo scene."""
from __future__ import annotations

import argparse
from pathlib import Path
import random
import subprocess
import time
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from arm_interfaces.msg import ObjectArray
from arm_interfaces.srv import SolveIk
from arm_perception.gazebo_pose import wait_for_model_pose
from arm_perception.sorting_config import BinSpec, load_sorting_config

LEGACY_WORLD_MODELS = ('pick_object', 'place_tray')
OBJECTS = {
    'red_box': {'half_height': 0.025},
    'yellow_cylinder': {'half_height': 0.025},
    'blue_cube': {'half_height': 0.020},
}


class SortingSceneNode(Node):
    def __init__(self, args) -> None:
        super().__init__('demo_sorting_scene')
        self.args = args
        self.config = load_sorting_config(args.config)
        self.perception_share = Path(get_package_share_directory('arm_perception'))
        self.gazebo_share = Path(get_package_share_directory('arm_gazebo'))
        self.ik_client = self.create_client(SolveIk, args.ik_service)
        self.latest_detections: ObjectArray | None = None
        self.latest_detections_time = 0.0
        self.object_done_time = 0.0
        self.last_completed_object = ''
        self.create_subscription(
            ObjectArray, args.detections_topic, self._on_detections, 10)
        self.create_subscription(
            String, args.object_done_topic, self._on_object_done, 10)
        self.active_object_publisher = self.create_publisher(
            String, args.active_object_topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.scene_ready_publisher = self.create_publisher(
            Bool, args.scene_ready_topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def _call_gazebo(
        self,
        service: str,
        reqtype: str,
        request: str,
        timeout_ms: int = 3000,
        require_success: bool = True,
        attempts: int = 1,
    ) -> None:
        command = [
            'gz', 'service', '-s', service, '--reqtype', reqtype,
            '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', request,
        ]
        last_result = None
        for _ in range(attempts):
            last_result = subprocess.run(
                command, check=False, capture_output=True, text=True)
            accepted = 'data: true' in last_result.stdout.lower()
            if last_result.returncode == 0 and (accepted or not require_success):
                return
            time.sleep(0.2)
        assert last_result is not None
        raise RuntimeError(
            f'Gazebo service rejected request: {service}: '
            f'{last_result.stdout.strip()} {last_result.stderr.strip()}')

    def _on_detections(self, message: ObjectArray) -> None:
        if message.objects:
            self.latest_detections = message
            self.latest_detections_time = time.monotonic()

    def _on_object_done(self, message: String) -> None:
        self.last_completed_object = message.data
        self.object_done_time = time.monotonic()
        self.get_logger().info(f'Object completion signal: {message.data}')

    def list_models(self) -> set[str]:
        result = subprocess.run(
            ['gz', 'model', '--list'], check=False, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'Unable to list Gazebo models: {result.stderr.strip()}')
        return {
            line.strip()[2:] for line in result.stdout.splitlines()
            if line.strip().startswith('- ')
        }

    def publish_scene_ready(self, ready: bool) -> None:
        message = Bool()
        message.data = ready
        self.scene_ready_publisher.publish(message)

    def publish_active_object(self, model_name: str, object_type: str) -> None:
        message = String()
        message.data = f'{model_name} {object_type}'.strip()
        self.active_object_publisher.publish(message)

    def remove_model(self, name: str) -> None:
        self._call_gazebo(
            f'/world/{self.args.world}/remove/blocking',
            'gz.msgs.Entity',
            f'name: "{name}" type: MODEL',
            timeout_ms=1000,
            require_success=False,
        )

    def wait_model_absent(self, name: str, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if name not in self.list_models():
                return True
            time.sleep(0.2)
        return name not in self.list_models()

    def spawn_model(self, name: str, model_file: Path, xyz: Iterable[float]) -> None:
        x, y, z = xyz
        if not model_file.is_file():
            raise RuntimeError(f'Missing Gazebo model: {model_file}')
        if name in self.list_models():
            self.get_logger().warning(
                f'Removing stale Gazebo model before respawn: {name}')
            self.remove_model(name)
            if not self.wait_model_absent(name):
                raise RuntimeError(f'Unable to remove stale Gazebo model before respawn: {name}')
        request = (
            f'sdf_filename: "{model_file}" name: "{name}" '
            f'pose: {{position: {{x: {x} y: {y} z: {z}}} '
            'orientation: {w: 1.0}}'
        )
        last_error = None
        for _ in range(6):
            try:
                self._call_gazebo(
                    f'/world/{self.args.world}/create/blocking',
                    'gz.msgs.EntityFactory',
                    request,
                    timeout_ms=5000,
                )
            except RuntimeError as exc:
                last_error = exc
            if name in self.list_models():
                return
            time.sleep(0.5)
        raise RuntimeError(f'Unable to spawn Gazebo model {name}: {last_error}')

    def validate_spawned_pose(
        self,
        model_name: str,
        expected_xyz: tuple[float, float, float],
        tolerance_m: float = 0.015,
    ) -> None:
        pose = wait_for_model_pose(
            model_name, world=self.args.world, attempts=12, delay_s=0.25)
        error = (
            (pose.xyz[0] - expected_xyz[0]) ** 2
            + (pose.xyz[1] - expected_xyz[1]) ** 2
            + (pose.xyz[2] - expected_xyz[2]) ** 2
        ) ** 0.5
        self.get_logger().info(
            f'Gazebo GT spawn pose: {model_name} world_center=('
            f'{pose.xyz[0]:.4f}, {pose.xyz[1]:.4f}, {pose.xyz[2]:.4f}) '
            f'expected=({expected_xyz[0]:.4f}, {expected_xyz[1]:.4f}, '
            f'{expected_xyz[2]:.4f}) error={error * 1000.0:.1f}mm')
        if error > tolerance_m:
            raise RuntimeError(
                f'Gazebo spawn pose mismatch for {model_name}: '
                f'{error:.3f}m > {tolerance_m:.3f}m')

    def validate_bin_ik(self, bin_spec: BinSpec) -> None:
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'IK service unavailable: {self.args.ik_service}')
        request = SolveIk.Request()
        request.target_pose = bin_spec.place_pose()
        request.seed_angles = list(bin_spec.ik_seed)
        request.solver = 'dls'
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'IK request timed out for {bin_spec.object_type}')
        response = future.result()
        if not response.success:
            raise RuntimeError(f'IK failed for {bin_spec.object_type}: {response.message}')
        radial = (bin_spec.spawn_world_xyz[0] ** 2 + bin_spec.spawn_world_xyz[1] ** 2) ** 0.5
        self.get_logger().info(
            f'IK PASS {bin_spec.object_type}: radial={radial:.3f}m '
            f'position_error={response.position_error * 1000.0:.3f}mm')

    def validate_bins(self) -> None:
        for bin_spec in self.config.bins.values():
            self.validate_bin_ik(bin_spec)

    def reset_scene(self) -> None:
        self.publish_scene_ready(False)
        self.publish_active_object('', '')
        existing_models = self.list_models()
        targets = list(LEGACY_WORLD_MODELS)
        targets.extend(bin_spec.model_name for bin_spec in self.config.bins.values())
        targets.extend(
            f'sorting_{object_type}_{index:02d}'
            for object_type in OBJECTS for index in range(10)
        )
        for name in targets:
            if name in existing_models:
                self.remove_model(name)
                if hasattr(self, 'wait_model_absent') and not self.wait_model_absent(name):
                    self.get_logger().warning(
                        f'Gazebo model still present after reset remove: {name}')

    def spawn_bins(self) -> None:
        bins_share = Path(get_package_share_directory(self.config.models.bins_package))
        for bin_spec in self.config.bins.values():
            model_file = (
                bins_share / self.config.models.bins_dir
                / bin_spec.model_name / 'model.sdf'
            )
            self.spawn_model(bin_spec.model_name, model_file, bin_spec.spawn_world_xyz)

    def _sample_object_xy(
        self,
        rng: random.Random,
        occupied: list[tuple[float, float]],
    ) -> tuple[float, float]:
        x_range, y_range = self.config.object_zone_world
        for _ in range(200):
            x = rng.uniform(*x_range)
            y = rng.uniform(*y_range)
            if all(
                ((x - other_x) ** 2 + (y - other_y) ** 2) ** 0.5
                >= self.config.minimum_object_spacing_m
                for other_x, other_y in occupied
            ):
                return x, y
        raise RuntimeError('Unable to place sorting objects with configured spacing')

    def object_sequence(self):
        for index in range(self.args.objects_per_class):
            for object_type in OBJECTS:
                yield object_type, index

    def spawn_one_object(
        self, object_type: str, index: int, rng: random.Random,
        occupied: list[tuple[float, float]],
    ) -> str:
        objects_share = Path(get_package_share_directory(self.config.models.objects_package))
        model_file = objects_share / self.config.models.objects_dir / object_type / 'model.sdf'
        x, y = self._sample_object_xy(rng, occupied)
        occupied.append((x, y))
        z = self.config.table_world_z + self.config.half_height(object_type)
        model_name = f'sorting_{object_type}_{index:02d}'
        self.spawn_model(model_name, model_file, (x, y, z))
        if hasattr(self, 'validate_spawned_pose'):
            self.validate_spawned_pose(model_name, (x, y, z))
        self.get_logger().info(
            f'Spawned {model_name} at world=({x:.3f}, {y:.3f}, {z:.3f})')
        return model_name

    def wait_for_object_done(self, spawned_at: float, model_name: str) -> None:
        deadline = time.monotonic() + self.args.object_timeout
        ignored_signal = ''
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.object_done_time >= spawned_at:
                if self.last_completed_object != model_name:
                    if self.last_completed_object != ignored_signal:
                        self.get_logger().warning(
                            f'Object gate ignored: expected {model_name}, '
                            f'signal={self.last_completed_object}')
                        ignored_signal = self.last_completed_object
                    continue
                self.get_logger().info(
                    f'Object gate PASS: {model_name} completed '
                    f'(signal={self.last_completed_object})')
                return
        raise RuntimeError(
            f'Object gate failed: {model_name} was not completed within '
            f'{self.args.object_timeout:.1f}s')

    def spawn_objects_sequentially(self) -> None:
        rng = random.Random(self.args.seed)
        for object_type, index in self.object_sequence():
            self.publish_scene_ready(False)
            self.latest_detections = None
            self.latest_detections_time = 0.0
            spawned_at = time.monotonic()
            model_name = self.spawn_one_object(object_type, index, rng, [])
            self.publish_active_object(model_name, object_type)
            if not self.args.skip_detection_gate:
                self.wait_for_scene_ready(spawned_at)
            self.publish_scene_ready(True)
            self.wait_for_object_done(spawned_at, model_name)
        # Leave the autonomous consumer gated after the final object.  Keeping
        # this true lets the last transient-local active object be selected
        # again after this process exits.
        self.publish_scene_ready(False)
        self.publish_active_object('', '')

    def _unsorted_detections(self):
        if self.latest_detections is None:
            return []
        return [
            detected for detected in self.latest_detections.objects
            if detected.object_type in self.config.bins
            and not any(
                bin_spec.contains(detected.pose.pose.position, margin=0.01)
                for bin_spec in self.config.bins.values()
            )
        ]

    def wait_for_scene_ready(self, spawned_at: float) -> None:
        settle_deadline = time.monotonic() + self.args.settle_seconds
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        deadline = time.monotonic() + self.args.detection_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            detections = self._unsorted_detections()
            if (
                self.latest_detections_time >= spawned_at
                and len(detections) >= self.args.minimum_detections
            ):
                classes = ', '.join(
                    sorted({detected.object_type for detected in detections}))
                self.get_logger().info(
                    f'Detection gate PASS: {len(detections)} unsorted object(s), '
                    f'classes=[{classes}]')
                return
        raise RuntimeError(
            f'Detection gate failed: no fresh /detected_objects message with at least '
            f'{self.args.minimum_detections} unsorted object(s) after '
            f'{self.args.detection_timeout:.1f}s')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    parser.add_argument('--world', default='pick_and_place_world')
    parser.add_argument('--ik-service', default='/ik_solve')
    parser.add_argument('--objects-per-class', type=int, default=2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--skip-ik-validation', action='store_true')
    parser.add_argument('--skip-detection-gate', action='store_true')
    parser.add_argument('--settle-seconds', type=float, default=2.0)
    parser.add_argument('--detection-timeout', type=float, default=30.0)
    parser.add_argument('--minimum-detections', type=int, default=1)
    parser.add_argument('--detections-topic', default='/detected_objects')
    parser.add_argument('--scene-ready-topic', default='/sorting_scene_ready')
    parser.add_argument('--object-done-topic', default='/sorting_object_done')
    parser.add_argument('--active-object-topic', default='/sorting_active_object')
    parser.add_argument('--object-timeout', type=float, default=240.0)
    parser.add_argument('--reset-only', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.objects_per_class < 2:
        raise ValueError('--objects-per-class must be at least 2')
    if args.settle_seconds < 0.5:
        raise ValueError('--settle-seconds must be at least 0.5')
    if args.detection_timeout <= 0.0:
        raise ValueError('--detection-timeout must be positive')
    if args.minimum_detections < 1:
        raise ValueError('--minimum-detections must be positive')
    if args.object_timeout <= 0.0:
        raise ValueError('--object-timeout must be positive')
    rclpy.init()
    node = SortingSceneNode(args)
    try:
        if args.dry_run:
            if not args.skip_ik_validation:
                node.validate_bins()
            node.get_logger().info('Sorting scene dry-run PASS')
            return
        # Remove Phase 1 assets before IK validation so sorting-mode
        # perception cannot dispatch a stale legacy object meanwhile.
        node.reset_scene()
        if not args.skip_ik_validation:
            node.validate_bins()
        if args.reset_only:
            node.get_logger().info('Sorting scene reset complete')
            return
        node.spawn_bins()
        node.spawn_objects_sequentially()
        node.get_logger().info('Sorting scene complete: all objects spawned and placed sequentially')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
