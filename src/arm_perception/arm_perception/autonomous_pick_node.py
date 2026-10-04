#!/usr/bin/env python3
"""Select the best perception result and trigger the pick-and-place action."""
from __future__ import annotations

import subprocess
import time
from typing import Iterable

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from arm_interfaces.action import PickAndPlace
from arm_interfaces.msg import ObjectArray
from arm_interfaces.srv import GetPickPose
from arm_perception.math3d import matrix_from_quaternion
from arm_perception.sorting_config import BinSpec, load_sorting_config


OBJECT_HALF_HEIGHT = {
    'red_box': 0.025,
    'yellow_cylinder': 0.025,
    'blue_cube': 0.020,
}


class AutonomousPickNode(Node):
    def __init__(self) -> None:
        super().__init__('autonomous_pick_node')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('pick_pose_service', '/get_pick_pose')
        self.declare_parameter('pick_action', '/pick_and_place')
        self.declare_parameter('place_xyz', [0.30, 0.30, 0.65])
        self.declare_parameter('speed_scale', 0.30)
        self.declare_parameter('cooldown_seconds', 5.0)
        self.declare_parameter('sort_all', False)
        self.declare_parameter('sorting_bins_file', '')
        self.declare_parameter('sorting_scene_ready_topic', '/sorting_scene_ready')
        self.declare_parameter('sorting_object_done_topic', '/sorting_object_done')
        self.declare_parameter('sorting_active_object_topic', '/sorting_active_object')
        self.declare_parameter('simulation_sync_enabled', False)
        # SİM KISA YOLU (varsayılan KAPALI): True yapılırsa bir sınıf algılanınca
        # o sınıfın modelleri gz set_pose ile kutuya ışınlanır; gerçek pick ATLANIR.
        # Normal modda robot hareketi çalışır; sim görsel nesne sonucu fizik
        # motoru attach/drop davranışına bırakılmadan action sonunda set_pose ile
        # senkronlanır.
        self.declare_parameter('simulation_fast_sort_enabled', False)
        self.declare_parameter('simulation_post_place_sync_enabled', True)
        self.declare_parameter('simulation_place_hover_offset_m', 0.20)
        self.declare_parameter('sorting_cooldown_seconds', 5.0)
        self.declare_parameter('simulation_world', 'pick_and_place_world')
        self.declare_parameter('simulation_objects_per_class', 2)
        self.declare_parameter('simulation_exact_pose_enabled', False)
        self.declare_parameter('base_world_yaw_rad', 1.5707963267948966)
        self.declare_parameter('pre_pick_offset_m', 0.10)
        self.declare_parameter('pre_place_offset_m', 0.10)
        self.declare_parameter('link6_to_grasp_xyz', [0.0075, -0.0032, 0.1023])
        self.declare_parameter(
            'grasp_orientations_file',
            str(get_package_share_directory('arm_perception') + '/config/grasp_orientations.yaml'))
        self.declare_parameter(
            'top_grasp_link6_quaternion', [0.749593, 0.605798, 0.207415, 0.167625])
        self.declare_parameter('bin_floor_thickness_m', 0.015)
        self.declare_parameter('place_floor_clearance_m', 0.005)
        self.busy = False
        self.last_completed = 0.0
        self.selected_object_id = ''
        self.selected_object_type = ''
        self.active_sorting_model = ''
        self.active_sorting_type = ''
        self.active_goal_handle = None
        self.simulation_synced_for_goal = False
        self.object_done_published_for_goal = False
        self.simulation_synced_models: set[str] = set()
        self._active_model_fail_count: int = 0
        self._max_consecutive_fails: int = 3
        # Bound the silent "stale detection, resetting" retry: a persistently bad
        # perception pose (e.g. the arm hovering over the object after a failed
        # grasp reports z far above the table) used to loop forever and hard-lock
        # the whole sorting run on that object. After this many consecutive sanity
        # rejects we escalate through _finish so loop protection engages instead.
        self._sanity_reject_count: int = 0
        self._max_sanity_rejects: int = 8
        # Pozisyon-only gz_ros2_control yerçekimi droop'unu telafi eder: pick
        # descent hedefinin ~18-21mm yukarısında steady-state kalıyor (P=1.0,
        # bkz. ros2_controllers.yaml). Hedef z'yi droop kadar aşağı çekip
        # gripper'ı gerçek nesne yüksekliğine indirir. SADECE pick'e uygulanır
        # (place log'da err≈0, dokunulmaz). Gerçek donanımda (kapalı-çevrim
        # servo, droop yok) 0.0'a çekilmeli.
        self.declare_parameter('pick_descent_droop_compensation_m', 0.018)
        self.sort_all = bool(self.get_parameter('sort_all').value)
        config_path = str(self.get_parameter('sorting_bins_file').value) or None
        self.sorting_config = load_sorting_config(config_path)
        self.bins: dict[str, BinSpec] = (
            self.sorting_config.bins if self.sort_all else {})
        self.grasp_orientations = self._load_grasp_orientations(
            str(self.get_parameter('grasp_orientations_file').value))
        self.sorting_scene_ready = not self.sort_all
        self._sorting_complete_logged = False
        self._warned_unknown_types: set[str] = set()
        self.pick_pose_client = self.create_client(
            GetPickPose, self.get_parameter('pick_pose_service').value)
        self.pick_client = ActionClient(
            self, PickAndPlace, self.get_parameter('pick_action').value)
        self.create_subscription(
            ObjectArray, self.get_parameter('detections_topic').value,
            self._on_detections, 10)
        self.object_done_publisher = self.create_publisher(
            String, self.get_parameter('sorting_object_done_topic').value, 10)
        if self.sort_all:
            self.create_subscription(
                String, self.get_parameter('sorting_active_object_topic').value,
                self._on_active_object,
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            self.create_subscription(
                Bool, self.get_parameter('sorting_scene_ready_topic').value,
                self._on_sorting_scene_ready,
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
        mode = 'sorting' if self.sort_all else 'single-place'
        fast_sort = bool(self.get_parameter('simulation_fast_sort_enabled').value)
        exact_pose = (
            bool(self.get_parameter('simulation_exact_pose_enabled').value)
            if self.has_parameter('simulation_exact_pose_enabled') else False
        )
        self.get_logger().info(
            f'Autonomous perception-to-pick loop ready: mode={mode} '
            f'fast_sort={fast_sort} exact_pose={exact_pose} '
            f'sim_sync={bool(self.get_parameter("simulation_sync_enabled").value)} '
            f'post_place_sync={self._post_place_sync_enabled()}')

    def _on_active_object(self, message: String) -> None:
        parts = message.data.split()
        self.active_sorting_model = parts[0] if parts else ''
        self.active_sorting_type = parts[1] if len(parts) > 1 else ''
        self._active_model_fail_count = 0
        self._sanity_reject_count = 0
        if self.active_sorting_model:
            self.get_logger().info(
                f'Active sorting object: {self.active_sorting_model} '
                f'type={self.active_sorting_type}')

    def _on_sorting_scene_ready(self, message: Bool) -> None:
        self.sorting_scene_ready = message.data
        state = 'ready' if message.data else 'waiting'
        self.get_logger().info(f'Sorting scene state: {state}')

    def _on_detections(self, message: ObjectArray) -> None:
        if self.sort_all and not self.sorting_scene_ready:
            return
        if self.busy or not message.objects:
            return
        if (
            self.sort_all
            and bool(self.get_parameter('simulation_fast_sort_enabled').value)
            and self._simulation_fast_sort_complete()
        ):
            return
        cooldown_parameter = 'sorting_cooldown_seconds' if self.sort_all else 'cooldown_seconds'
        cooldown = float(self.get_parameter(cooldown_parameter).value)
        if time.monotonic() - self.last_completed < cooldown:
            return
        candidates = list(self._candidate_objects(message.objects))
        if not candidates:
            if self.sort_all and not self._sorting_complete_logged:
                self.get_logger().info('Sorting complete: no known unsorted detections remain')
                self._sorting_complete_logged = True
            return
        self._sorting_complete_logged = False
        selected = max(candidates, key=lambda item: item.confidence)
        if not self.pick_pose_client.service_is_ready():
            self.get_logger().warning('Waiting for /get_pick_pose service', throttle_duration_sec=5.0)
            return
        self.busy = True
        self.selected_object_id = selected.object_id
        self.selected_object_type = selected.object_type
        self.simulation_synced_for_goal = False
        self.object_done_published_for_goal = False
        if self.sort_all and bool(self.get_parameter('simulation_fast_sort_enabled').value):
            self.simulation_synced_for_goal = self._sync_simulation_object_to_bin()
            message = (
                'Simulation fast-sort completed'
                if self.simulation_synced_for_goal
                else 'Simulation fast-sort skipped: no unsorted Gazebo model remains'
            )
            self._finish(message, completed=self.simulation_synced_for_goal)
            return
        request = GetPickPose.Request()
        request.object_id = selected.object_id
        request.preferred_approach = 'top'
        future = self.pick_pose_client.call_async(request)
        future.add_done_callback(self._on_pick_pose)

    def _candidate_objects(self, objects: Iterable):
        for detected in objects:
            if not self.sort_all:
                yield detected
                continue
            if self.active_sorting_type and detected.object_type != self.active_sorting_type:
                continue
            if detected.object_type not in self.bins:
                if detected.object_type not in self._warned_unknown_types:
                    self.get_logger().warning(
                        f'Skipping unknown sorting class: {detected.object_type}')
                    self._warned_unknown_types.add(detected.object_type)
                continue
            if any(
                bin_spec.contains(detected.pose.pose.position, margin=0.01)
                for bin_spec in self.bins.values()
            ):
                continue
            yield detected

    def _on_pick_pose(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self._finish(f'Pick pose rejected: {response.message}')
                return
            pick_pose = response.pick_pose.pose
            if self._exact_pose_enabled():
                exact_pick = self._exact_pick_pose()
                if exact_pick is None:
                    self.busy = False
                    self.last_completed = time.monotonic()
                    return
                pick_pose = exact_pick
            else:
                # Sanity check: pick z in base_link must be near table level.
                # Valid range is ~0.02-0.10 m; values above 0.15 m indicate a
                # stale/occluded detection (e.g. arm returning from previous place).
                pick_z = response.pick_pose.pose.position.z
                if pick_z > 0.15:
                    self._sanity_reject_count += 1
                    if self._sanity_reject_count >= self._max_sanity_rejects:
                        self.get_logger().error(
                            f'Pick pose sanity: z={pick_z:.3f}m > 0.15 rejected '
                            f'{self._sanity_reject_count}x consecutively — giving up on '
                            f'this object instead of looping forever')
                        self._sanity_reject_count = 0
                        self._finish('Pick aborted: persistent stale/occluded detection')
                        return
                    self.get_logger().warning(
                        f'Pick pose sanity: z={pick_z:.3f}m > 0.15 (base_link) — '
                        f'stale detection, resetting '
                        f'({self._sanity_reject_count}/{self._max_sanity_rejects})')
                    self.busy = False
                    self.last_completed = time.monotonic()
                    return
                self._sanity_reject_count = 0
            if not self.pick_client.server_is_ready():
                self._finish('Waiting for /pick_and_place action server')
                return
            goal = PickAndPlace.Goal()
            goal.object_id = self.selected_object_id
            droop = float(
                self.get_parameter('pick_descent_droop_compensation_m').value)
            if droop != 0.0:
                pick_pose.position.z -= droop
                self.get_logger().info(
                    f'Pick droop compensation: -{droop:.3f}m '
                    f'-> pick z={pick_pose.position.z:.4f}')
            goal.pick_pose = pick_pose
            goal.place_pose = self._place_pose(self.selected_object_type)
            self.get_logger().info(
                f'Sorting dispatch: selected_id={self.selected_object_id} '
                f'type={self.selected_object_type} active_model={self.active_sorting_model} '
                f'place=({goal.place_pose.position.x:.3f}, '
                f'{goal.place_pose.position.y:.3f}, {goal.place_pose.position.z:.3f})')
            goal.speed_scale = float(self.get_parameter('speed_scale').value)
            # The pick/place action now owns the full 12-state sequence,
            # including RETREAT and final HOME after gripper detach settle.
            goal.return_home = self._return_home_after_goal()
            future = self.pick_client.send_goal_async(goal, feedback_callback=self._feedback)
            future.add_done_callback(self._on_goal_response)
        except Exception as exc:
            self._finish(f'Unable to request pick pose: {exc}')

    def _return_home_after_goal(self) -> bool:
        return True

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self._finish('Pick-and-place goal rejected')
                return
            self.active_goal_handle = handle
            handle.get_result_async().add_done_callback(self._on_result)
        except Exception as exc:
            self._finish(f'Unable to send pick-and-place goal: {exc}')

    def _feedback(self, feedback) -> None:
        phase = feedback.feedback.current_phase
        self.get_logger().info(
            f'Pick feedback: {phase} '
            f'{feedback.feedback.progress * 100.0:.1f}%')
        if (
            phase == 'STATE_6_CLOSE_GRIPPER'
            and self.sort_all
            and bool(self.get_parameter('simulation_fast_sort_enabled').value)
            and not self.simulation_synced_for_goal
        ):
            self.simulation_synced_for_goal = self._sync_simulation_object_to_bin()
            if self.simulation_synced_for_goal and self.active_goal_handle is not None:
                self.get_logger().info('Simulation fast-sort: canceling transport phases')
                self.active_goal_handle.cancel_goal_async()

    def _on_result(self, future) -> None:
        try:
            result = future.result().result
            sync_required = (
                result.success
                and self.sort_all
                and not self.simulation_synced_for_goal
                and self._post_place_sync_enabled()
            )
            if (
                result.success
                and self.sort_all
                and not self.simulation_synced_for_goal
                and (
                    bool(self.get_parameter('simulation_fast_sort_enabled').value)
                    or self._post_place_sync_enabled()
                )
            ):
                self.simulation_synced_for_goal = self._sync_simulation_object_to_bin()
            completed = bool(result.success)
            if sync_required:
                completed = bool(self.simulation_synced_for_goal)
            self._finish(
                f'Pick result: success={result.success} {result.message}',
                completed=completed,
            )
        except Exception as exc:
            self._finish(f'Unable to receive pick result: {exc}')

    def _set_simulation_model_pose(self, model_name: str, bin_spec: BinSpec) -> bool:
        x, y, z = bin_spec.spawn_world_xyz
        request = f'name: "{model_name}" position: {{x: {x} y: {y} z: {z + 0.05}}} '
        request += 'orientation: {w: 1.0}'
        world = str(self.get_parameter('simulation_world').value)
        command = [
            'gz', 'service', '-s', f'/world/{world}/set_pose/blocking',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', request,
        ]
        for _ in range(4):
            try:
                result = subprocess.run(
                    command, check=False, capture_output=True, text=True, timeout=8.0)
            except subprocess.TimeoutExpired:
                continue
            if result.returncode == 0 and 'data: true' in result.stdout.lower():
                return True
            time.sleep(0.2)
        return False

    def _simulation_fast_sort_complete(self) -> bool:
        objects_per_class = int(self.get_parameter('simulation_objects_per_class').value)
        return len(self.simulation_synced_models) >= len(self.bins) * objects_per_class

    def _post_place_sync_enabled(self) -> bool:
        return (
            self.sort_all
            and bool(self.get_parameter('simulation_sync_enabled').value)
            and bool(self.get_parameter('simulation_post_place_sync_enabled').value)
        )

    def _sync_simulation_object_to_bin(self) -> bool:
        if not bool(self.get_parameter('simulation_sync_enabled').value):
            return False
        bin_spec = self.bins[self.selected_object_type]
        count = int(self.get_parameter('simulation_objects_per_class').value)
        model_names = []
        if self.active_sorting_model:
            model_names.append(self.active_sorting_model)
        if self.selected_object_id.startswith('sorting_'):
            model_names.append(self.selected_object_id)
        model_names.extend(
            f'sorting_{self.selected_object_type}_{index:02d}'
            for index in range(count)
        )
        for model_name in model_names:
            if model_name in self.simulation_synced_models:
                continue
            if not self._set_simulation_model_pose(model_name, bin_spec):
                self.get_logger().warning(f'Simulation sync failed for {model_name}')
                continue
            self.simulation_synced_models.add(model_name)
            self.get_logger().info(
                f'Simulation sync: {model_name} -> {bin_spec.model_name}')
            return True
        return False

    def _publish_object_done(self) -> None:
        if not self.sort_all or self.object_done_published_for_goal:
            return
        message = String()
        message.data = self.active_sorting_model or self.selected_object_id or self.selected_object_type
        self.object_done_publisher.publish(message)
        self.object_done_published_for_goal = True
        self.get_logger().info(f'Object done signal published: {message.data}')

    def _finish(self, message: str, completed: bool = False) -> None:
        self.get_logger().info(message)
        if completed:
            self._active_model_fail_count = 0
            self._publish_object_done()
        elif self.sort_all and self.active_sorting_model:
            self._active_model_fail_count += 1
            if self._active_model_fail_count >= self._max_consecutive_fails:
                self.get_logger().warning(
                    f'Loop protection: {self.active_sorting_model} failed '
                    f'{self._active_model_fail_count}x; not signalling object done')
                self._active_model_fail_count = 0
        self.busy = False
        self.active_goal_handle = None
        self.last_completed = time.monotonic()

    def _place_pose(self, object_type: str = '') -> Pose:
        if self.sort_all:
            bin_spec = self.bins[object_type]
            pose = None
            if hasattr(self, '_exact_pose_enabled') and self._exact_pose_enabled():
                exact_pose = self._exact_place_pose(object_type, bin_spec)
                if exact_pose is not None:
                    pose = exact_pose
            if pose is None:
                pose = bin_spec.place_pose()
            if hasattr(self, '_post_place_sync_enabled') and self._post_place_sync_enabled():
                pose.position.z += float(
                    self.get_parameter('simulation_place_hover_offset_m').value)
            return pose
        xyz = self.get_parameter('place_xyz').value
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        return pose

    def _exact_pose_enabled(self) -> bool:
        return self.sort_all and bool(self.get_parameter('simulation_exact_pose_enabled').value)

    def _active_or_selected_model_name(self) -> str:
        if self.active_sorting_model:
            return self.active_sorting_model
        if self.selected_object_id.startswith('sorting_'):
            return self.selected_object_id
        return ''

    def _world_to_base_xyz(self, world_xyz: tuple[float, float, float]) -> np.ndarray:
        base_world = self.sorting_config.base_world_xyz
        wx, wy, wz = world_xyz
        bx, by, bz = base_world
        # Inverse world->base planar rotation. Old robot default is +90 deg;
        # Robot Arm profile passes 0 because its world_joint is unrotated.
        yaw = float(self.get_parameter('base_world_yaw_rad').value)
        dx, dy = wx - bx, wy - by
        cosine, sine = np.cos(yaw), np.sin(yaw)
        return np.asarray(
            (cosine * dx + sine * dy, -sine * dx + cosine * dy, wz - bz),
            dtype=np.float64)

    def _select_grasp_quaternion(self, x: float, y: float) -> np.ndarray:
        if not self.grasp_orientations:
            return np.asarray(
                self.get_parameter('top_grasp_link6_quaternion').value,
                dtype=np.float64)
        target = np.asarray((x, y), dtype=np.float64)
        nearest = min(
            self.grasp_orientations,
            key=lambda item: float(np.linalg.norm(item[0] - target)))
        return nearest[1].copy()

    def _link6_pose_for_grasp_world(
        self,
        grasp_world_xyz: tuple[float, float, float],
        quaternion_override: tuple[float, float, float, float] | None = None,
    ) -> Pose:
        grasp_base = self._world_to_base_xyz(grasp_world_xyz)
        quaternion = (
            np.asarray(quaternion_override, dtype=np.float64)
            if quaternion_override is not None
            else self._select_grasp_quaternion(float(grasp_base[0]), float(grasp_base[1]))
        )
        rotation = matrix_from_quaternion(quaternion)
        link6_to_grasp = np.asarray(
            self.get_parameter('link6_to_grasp_xyz').value, dtype=np.float64)
        link6_base = grasp_base - rotation @ link6_to_grasp
        pose = Pose()
        pose.position.x = float(link6_base[0])
        pose.position.y = float(link6_base[1])
        pose.position.z = float(link6_base[2])
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return pose

    def _exact_pick_pose(self) -> Pose | None:
        try:
            from arm_perception.gazebo_pose import wait_for_model_pose
        except Exception as exc:
            self.get_logger().warning(f'Exact Gazebo pick pose unavailable: {exc}')
            return None
        model_name = self._active_or_selected_model_name()
        if not model_name:
            self.get_logger().warning(
                'Exact Gazebo pick pose unavailable: no active sorting model name')
            return None
        world = str(self.get_parameter('simulation_world').value)
        try:
            model_pose = wait_for_model_pose(model_name, world=world, attempts=6)
        except Exception as exc:
            self.get_logger().warning(f'Exact Gazebo pick pose failed for {model_name}: {exc}')
            return None
        pick_pose = self._link6_pose_for_grasp_world(model_pose.xyz)
        pre_z = pick_pose.position.z + float(self.get_parameter('pre_pick_offset_m').value)
        self.get_logger().info(
            f'Exact pick target: model={model_name} type={self.selected_object_type} '
            f'world_center=({model_pose.xyz[0]:.4f}, {model_pose.xyz[1]:.4f}, '
            f'{model_pose.xyz[2]:.4f}) base_link6=({pick_pose.position.x:.4f}, '
            f'{pick_pose.position.y:.4f}, {pick_pose.position.z:.4f}) '
            f'pre_pick_z={pre_z:.4f}')
        return pick_pose

    def _exact_place_pose(self, object_type: str, bin_spec: BinSpec) -> Pose | None:
        try:
            from arm_perception.gazebo_pose import wait_for_model_pose
        except Exception as exc:
            self.get_logger().warning(f'Exact Gazebo place pose unavailable: {exc}')
            return None
        world = str(self.get_parameter('simulation_world').value)
        try:
            bin_pose = wait_for_model_pose(bin_spec.model_name, world=world, attempts=6)
        except Exception as exc:
            self.get_logger().warning(
                f'Exact Gazebo bin pose failed for {bin_spec.model_name}: {exc}')
            return None
        try:
            half_height = self.sorting_config.half_height(object_type)
        except (KeyError, AttributeError):
            half_height = OBJECT_HALF_HEIGHT.get(object_type, 0.025)
        floor_thickness = float(self.get_parameter('bin_floor_thickness_m').value)
        clearance = float(self.get_parameter('place_floor_clearance_m').value)
        grasp_world = (
            bin_pose.xyz[0],
            bin_pose.xyz[1],
            bin_pose.xyz[2] + floor_thickness + half_height + clearance,
        )
        place_pose = self._link6_pose_for_grasp_world(
            grasp_world, quaternion_override=bin_spec.drop_link6_quaternion)
        half_x = bin_spec.inner_size_xyz[0] / 2.0
        half_y = bin_spec.inner_size_xyz[1] / 2.0
        pre_z = place_pose.position.z + float(self.get_parameter('pre_place_offset_m').value)
        self.get_logger().info(
            f'Exact place target: bin={bin_spec.model_name} type={object_type} '
            f'bin_world_center=({bin_pose.xyz[0]:.4f}, {bin_pose.xyz[1]:.4f}, '
            f'{bin_pose.xyz[2]:.4f}) inner_x=[{bin_pose.xyz[0] - half_x:.4f}, '
            f'{bin_pose.xyz[0] + half_x:.4f}] inner_y=[{bin_pose.xyz[1] - half_y:.4f}, '
            f'{bin_pose.xyz[1] + half_y:.4f}] release_grasp_world=({grasp_world[0]:.4f}, '
            f'{grasp_world[1]:.4f}, {grasp_world[2]:.4f}) base_link6=('
            f'{place_pose.position.x:.4f}, {place_pose.position.y:.4f}, '
            f'{place_pose.position.z:.4f}) pre_place_z={pre_z:.4f}')
        return place_pose

    def _load_grasp_orientations(self, path: str):
        try:
            import yaml
            from pathlib import Path
            document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
            entries = []
            for item in document.get('grasp_orientations', []):
                entries.append((
                    np.asarray(item['at_xy'], dtype=np.float64),
                    np.asarray(item['quaternion'], dtype=np.float64),
                ))
            self.get_logger().info(f'Loaded {len(entries)} exact-pose grasp orientations')
            return entries
        except Exception as exc:
            self.get_logger().warning(
                f'Could not load exact-pose grasp orientations ({exc}); using fallback')
            return []


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousPickNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
