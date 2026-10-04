#!/usr/bin/env python3
"""Simulation-only fixed-joint grasp bridge.

Gazebo Harmonic does not reliably grasp runtime-spawned objects with contact
physics alone. This node listens to the gripper joint and emulates a temporary
fixed joint between grasp_link and the selected sorting model: on a fresh close
edge it latches the closest graspable object, then keeps that model at a fixed
translation/orientation offset relative to grasp_link until the gripper opens.

The autonomy layer is intentionally unaware of this node. It sends normal
open/close gripper commands; this node only provides sim-only attach/detach
semantics for real-motion sorting runs where fast-sort teleporting is disabled.
"""
from __future__ import annotations

import math
import re
import subprocess
import threading
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
PoseState = Tuple[float, float, float, float, float, float, float]


def _quat_normalize(q: Quat) -> Quat:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def _quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def _rotate_vector(q: Quat, v: Vector3) -> Vector3:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _xyz(state: PoseState) -> Vector3:
    return (state[0], state[1], state[2])


def _quat(state: PoseState) -> Quat:
    return (state[3], state[4], state[5], state[6])


class SimGraspNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_grasp_node')
        self.declare_parameter('simulation_world', 'pick_and_place_world')
        self.declare_parameter('gripper_joint', 'gripper_joint1')
        # -0.30 (was -0.15): the gripper closing ONTO an object stalls at ~-0.23
        # (the object blocks full closure), which never reached the old -0.15 ->
        # attach only fired LATE after the empty gripper finished closing (post
        # lift), snapping the object up from ~8 cm below -> visible disconnect /
        # fling. -0.30 sits between open_arm_threshold (-0.35) and the on-object
        # stall (~-0.23), so attach fires the moment the gripper grips, pre-lift,
        # co-located with grasp_link.
        self.declare_parameter('attach_threshold', -0.30)
        self.declare_parameter('open_arm_threshold', -0.35)
        self.declare_parameter('closed_threshold', 0.09)
        self.declare_parameter('detach_threshold', -0.09)
        self.declare_parameter('graspable_prefix', 'sorting_')
        self.declare_parameter('grasp_frame', 'grasp_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('active_object_topic', '/sorting_active_object')
        self.declare_parameter('release_topic', '/sim_grasp_release')
        self.declare_parameter('grasp_radius', 0.10)
        # Timer rate for gripper-edge polling. Carry itself is handled by the
        # temporary DetachableJoint bridge, not by per-tick teleport following.
        self.declare_parameter('follow_rate_hz', 8.0)
        self.declare_parameter('pose_poll_period_s', 0.3)
        self.declare_parameter('center_snap_retries', 3)
        self.declare_parameter('center_snap_settle_s', 0.08)
        self.declare_parameter('bridge_create_retries', 2)

        self.declare_parameter('workspace_radius_m', 1.1)

        self.world = str(self.get_parameter('simulation_world').value)
        self.gripper_joint = str(self.get_parameter('gripper_joint').value)
        self.attach_threshold = float(self.get_parameter('attach_threshold').value)
        self.open_arm_threshold = float(self.get_parameter('open_arm_threshold').value)
        self.closed_threshold = float(self.get_parameter('closed_threshold').value)
        self.detach_threshold = float(self.get_parameter('detach_threshold').value)
        self.prefix = str(self.get_parameter('graspable_prefix').value)
        self.grasp_frame = str(self.get_parameter('grasp_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.active_object_topic = str(self.get_parameter('active_object_topic').value)
        self.release_topic = str(self.get_parameter('release_topic').value)
        self.grasp_radius = float(self.get_parameter('grasp_radius').value)
        self.pose_poll_period = float(self.get_parameter('pose_poll_period_s').value)
        self.center_snap_retries = int(self.get_parameter('center_snap_retries').value)
        self.center_snap_settle = float(self.get_parameter('center_snap_settle_s').value)
        self.bridge_create_retries = int(self.get_parameter('bridge_create_retries').value)
        self.workspace_radius = float(self.get_parameter('workspace_radius_m').value)
        rate = float(self.get_parameter('follow_rate_hz').value)

        self.gripper_position: Optional[float] = None
        self.held_model: Optional[str] = None
        self.active_model: str = ''
        self._armed = False
        self._held_saw_closed = False
        self._held_local_xyz: Vector3 = (0.0, 0.0, 0.0)
        self._held_local_quat: Quat = (0.0, 0.0, 0.0, 1.0)
        self._poses: Dict[str, PoseState] = {}
        self._poses_lock = threading.Lock()
        self._running = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)
        self.create_subscription(
            String,
            self.active_object_topic,
            self._on_active_object,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(String, self.release_topic, self._on_release_request, 10)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self._reader = threading.Thread(target=self._pose_reader_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f'sim_grasp_node ready (world={self.world}, gripper={self.gripper_joint}, '
            f'prefix={self.prefix!r}, radius={self.grasp_radius} m, '
            f'active_topic={self.active_object_topic}). Sim-only fixed-joint attach enabled.')

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            idx = msg.name.index(self.gripper_joint)
        except ValueError:
            return
        self.gripper_position = msg.position[idx]
        self.get_logger().info(f'Joint states updated: gripper_position={self.gripper_position:.3f}', throttle_duration_sec=2.0)

    def _on_active_object(self, msg: String) -> None:
        parts = msg.data.split()
        self.active_model = parts[0] if parts else ''
        if self.active_model:
            self.get_logger().info(f'Active grasp model filter: {self.active_model}')

    def _on_release_request(self, msg: String) -> None:
        if self.held_model is None:
            self.get_logger().info(
                f'Release request ignored: no held model (request={msg.data!r})')
            return
        grasp_pose = self._grasp_link_world_pose()
        if grasp_pose is None:
            with self._poses_lock:
                state = self._poses.get(self.held_model)
            if state is None:
                self.get_logger().error(
                    f'Release request failed: no grasp TF and no pose for {self.held_model}')
                return
            grasp_pose = (_xyz(state), _quat(state))
        self.get_logger().info(
            f'Release request received ({msg.data!r}); detaching held model {self.held_model}')
        self._detach_model(grasp_pose)

    def _pose_reader_loop(self) -> None:
        topic = f'/world/{self.world}/pose/info'
        while self._running and rclpy.ok():
            try:
                out = subprocess.run(
                    ['gz', 'topic', '-e', '-t', topic, '-n', '1'],
                    check=False, capture_output=True, text=True, timeout=10.0).stdout
            except subprocess.TimeoutExpired:
                continue
            except Exception as e:
                self.get_logger().error(f'_pose_reader_loop error: {e}')
                import time
                time.sleep(1.0)
                continue
            poses: Dict[str, PoseState] = {}
            for block in re.split(r'\bpose\s*{', out):
                name_m = re.search(r'name:\s*"([^"]+)"', block)
                pos_m = re.search(
                    r'position\s*{[^}]*?x:\s*([-\d.eE]+)[^}]*?y:\s*([-\d.eE]+)'
                    r'[^}]*?z:\s*([-\d.eE]+)', block, re.DOTALL)
                if not name_m or not pos_m:
                    continue
                orient_block = re.search(r'orientation\s*{([^}]*)}', block, re.DOTALL)
                orientation = orient_block.group(1) if orient_block else ''
                qx = self._field(orientation, 'x', 0.0)
                qy = self._field(orientation, 'y', 0.0)
                qz = self._field(orientation, 'z', 0.0)
                qw = self._field(orientation, 'w', 1.0)
                poses[name_m.group(1)] = (
                    float(pos_m.group(1)), float(pos_m.group(2)), float(pos_m.group(3)),
                    *_quat_normalize((qx, qy, qz, qw)),
                )
            if poses:
                with self._poses_lock:
                    self._poses = poses
            threading.Event().wait(max(self.pose_poll_period, 0.05))

    @staticmethod
    def _field(text: str, name: str, default: float) -> float:
        match = re.search(rf'\b{name}:\s*([-\d.eE]+)', text)
        return float(match.group(1)) if match else default

    def _grasp_link_world_pose(self) -> Optional[Tuple[Vector3, Quat]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.grasp_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z), _quat_normalize((q.x, q.y, q.z, q.w))

    def _remove_bridge(self, bridge_name: str) -> None:
        """Cleanly tear down a grasp bridge: fire the DetachableJoint detach
        topic (so DART releases the fixed joint it added to 6dof_arm/Link_6),
        THEN remove the bridge model. gz.msgs.Empty carries no fields, so the
        payload must be empty (`-p ''`); the old `-p '""'` was parsed as a
        text-format Empty and raised `Expected identifier, got: ""`, so the
        detach never fired and the fixed joint was orphaned on Link_6."""
        topic = f'/model/{bridge_name}/detach'
        subprocess.run(
            ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
            check=False)
        time.sleep(0.1)
        req_str = f'name: "{bridge_name}" type: MODEL'
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.world}/remove',
             '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req_str],
            check=False)

    def _tick(self) -> None:
        pos = self.gripper_position
        closing = pos >= self.attach_threshold if pos is not None else False
        opened = pos <= self.open_arm_threshold if pos is not None else False
        # Never infer release from the gripper joint while carrying an object.
        # The simulated joint can drift through "open" thresholds during lift or
        # transit under slow physics, which drops the object before the bin. The
        # pick/place state machine publishes /sim_grasp_release exactly at the
        # verified place pose, so detach is command-driven only.
        release = False
        
        self.get_logger().info(
            f'Tick DIAG: pos={pos}, closing={closing}, opened={opened}, '
            f'release={release}, armed={self._armed}, held={self.held_model}, '
            f'held_saw_closed={self._held_saw_closed}',
            throttle_duration_sec=1.0)

        if pos is None:
            return
        grasp_pose = self._grasp_link_world_pose()
        if grasp_pose is None:
            self.get_logger().warning('Tick DIAG: grasp_pose is None (TF lookup failed)', throttle_duration_sec=2.0)
            return

        if opened:
            self._armed = True

        if self.held_model is None:
            if closing and self._armed:
                model, state, nearest, count = self._nearest_graspable(grasp_pose[0])
                if model and state:
                    self._attach_model(model, state, grasp_pose, nearest)
                else:
                    self.get_logger().warning(
                        f'ATTACH FAILED: gripper closed (pos={self.gripper_position:.3f}) but no '
                        f'graspable model within {self.grasp_radius:.3f} m. candidates={count}, '
                        f'nearest={nearest:.3f} m, grasp_link={tuple(round(v, 3) for v in grasp_pose[0])}',
                        throttle_duration_sec=2.0)
            return

        if pos >= self.closed_threshold:
            self._held_saw_closed = True

        if release:
            self._detach_model(grasp_pose)
            return

    def _attach_model(
        self,
        model: str,
        state: PoseState,
        grasp_pose: Tuple[Vector3, Quat],
        nearest: float,
    ) -> None:
        self.held_model = model
        self._armed = False
        self._held_saw_closed = False

        bridge_name = f'grasp_bridge_{model}'

        # CRASH GUARD (cheap, targeted): make sure no bridge of THIS name is
        # still welded to 6dof_arm/Link_6 before creating a new one — a second
        # DART 'fixed' joint on the same link aborts libgz-physics dartsim. One
        # idempotent removal (detach topic + model remove); removing a
        # non-existent bridge is a quick no-op. No model-list polling here: that
        # runs subprocesses inside the single-threaded timer callback and starves
        # the /joint_states read that gates the grasp.
        self._remove_bridge(bridge_name)

        # The bridge preserves the model<->gripper transform at the instant it
        # is created. If the gripper closes while the model center is still a few
        # centimeters away from grasp_link, that offset is carried all the way to
        # the bin and the object lands on a wall. In sorting mode the motion
        # planner already targets the model center, so make the simulation bridge
        # represent an ideal centered grasp: snap the model origin to grasp_link
        # before welding it to Link_6.
        if self._center_model_on_grasp(model, grasp_pose[0], _quat(state)):
            self.get_logger().info(
                f'ATTACH: Centered {model} on {self.grasp_frame} before bridge '
                f'(pre_snap_distance={nearest:.3f} m)')
        else:
            self.get_logger().error(
                f'ATTACH: Could not center {model}; refusing off-centre bridge '
                f'(distance={nearest:.3f} m)')
            self.held_model = None
            self._armed = True
            return

        sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{bridge_name}">
    <pose>0 0 0 0 0 0</pose>
    <link name="bridge_link">
      <inertial><mass>0.001</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
    </link>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>bridge_link</parent_link>
      <child_model>6dof_arm</child_model>
      <child_link>Link_6</child_link>
      <detach_topic>/model/{bridge_name}/detach</detach_topic>
    </plugin>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>bridge_link</parent_link>
      <child_model>{model}</child_model>
      <child_link>link</child_link>
      <detach_topic>/model/{bridge_name}/detach</detach_topic>
    </plugin>
  </model>
</sdf>'''

        import tempfile
        import os
        sdf_path = os.path.join(tempfile.gettempdir(), f"{bridge_name}.sdf")
        with open(sdf_path, 'w') as f:
            f.write(sdf_content)
            
        req_str = f'sdf_filename: "{sdf_path}" name: "{bridge_name}"'
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_str
        ]
        created = False
        last_stdout = ''
        last_stderr = ''
        for attempt in range(1, max(self.bridge_create_retries, 1) + 1):
            res = subprocess.run(cmd, check=False, capture_output=True, text=True)
            last_stdout = res.stdout.strip()
            last_stderr = res.stderr.strip()
            if res.returncode == 0 and 'data: true' in res.stdout.lower():
                created = True
                break
            self.get_logger().warning(
                f'ATTACH: Bridge create attempt {attempt}/{max(self.bridge_create_retries, 1)} '
                f'failed for {bridge_name}: rc={res.returncode} out={last_stdout} err={last_stderr}')
            self._remove_bridge(bridge_name)
            time.sleep(0.1)

        if created:
            self.get_logger().info(
                f'ATTACH: Created bridge model {bridge_name} '
                f'(distance={nearest:.3f} m)'
            )
        else:
            self.get_logger().error(
                f'ATTACH: Failed to create bridge model {bridge_name}; refusing false hold. '
                f'out={last_stdout} err={last_stderr}')
            self.held_model = None
            self._armed = True
            self._held_saw_closed = False

    def _center_model_on_grasp(self, model: str, xyz: Vector3, quat: Quat) -> bool:
        attempts = max(self.center_snap_retries, 1)
        for attempt in range(1, attempts + 1):
            if self._set_model_pose(model, xyz, quat):
                if self.center_snap_settle > 0.0:
                    time.sleep(self.center_snap_settle)
                return True
            self.get_logger().warning(
                f'ATTACH: Center snap attempt {attempt}/{attempts} failed for {model}')
            time.sleep(0.05)
        return False

    def _set_model_pose(self, model: str, xyz: Vector3, quat: Quat) -> bool:
        x, y, z = xyz
        qx, qy, qz, qw = _quat_normalize(quat)
        request = (
            f'name: "{model}" '
            f'position: {{x: {x} y: {y} z: {z}}} '
            f'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}'
        )
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world}/set_pose/blocking',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', request,
        ]
        res = subprocess.run(cmd, check=False, capture_output=True, text=True)
        return res.returncode == 0 and 'data: true' in res.stdout.lower()

    def _detach_model(self, grasp_pose: Tuple[Vector3, Quat]) -> None:
        model = self.held_model
        if model is None:
            return
            
        bridge_name = f'grasp_bridge_{model}'

        # Wake up physics by explicitly detaching first. gz.msgs.Empty has no
        # fields: the payload MUST be empty (`-p ''`). The old `-p '""'` was
        # parsed as text-format and raised `Expected identifier, got: ""`, so
        # the DetachableJoint never detached and its fixed joint stayed welded
        # to Link_6 — the root cause of the next-attach dartsim crash.
        topic = f'/model/{bridge_name}/detach'
        subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''], check=False)
        time.sleep(0.1)
        
        # gz.msgs.Entity fields are flat (name/type) — there is NO `entity`
        # wrapper. The old `entity: {name:..., type: MODEL}` form raised a
        # protobuf parse error so the remove SILENTLY FAILED: the bridge model
        # persisted and the object stayed welded to the arm (dragged around).
        # Live-verified: flat `name: "..." type: MODEL` returns data: true and
        # actually removes the bridge.
        req_str = f'name: "{bridge_name}" type: MODEL'
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_str
        ]
        res = subprocess.run(cmd, check=False, capture_output=True, text=True)

        # Honest success check: only claim removal when gz actually replied
        # data: true (returncode 0 alone hid the failure above).
        if res.returncode == 0 and 'data: true' in res.stdout:
            if self._set_model_pose(model, grasp_pose[0], grasp_pose[1]):
                x, y, z = grasp_pose[0]
                self.get_logger().info(
                    f'DETACH: Snapped {model} to release pose '
                    f'({x:.4f}, {y:.4f}, {z:.4f}) before physics handoff.')
            else:
                self.get_logger().warning(
                    f'DETACH: Could not snap {model} to release pose before physics handoff.')
            self.get_logger().info(f'DETACH: Removed bridge model {bridge_name}. Object is now governed by physics.')
            # Apply a tiny wrench to wake the object up!
            wrench_req = f'entity: {{name: "{model}", type: MODEL}}, wrench: {{force: {{z: -0.0001}}}}'
            subprocess.run(['gz', 'topic', '-t', f'/world/{self.world}/wrench', '-m', 'gz.msgs.EntityWrench', '-p', wrench_req], check=False)
        else:
            self.get_logger().error(
                f'DETACH: Failed to remove bridge model {bridge_name}: '
                f'rc={res.returncode} out={res.stdout.strip()} err={res.stderr.strip()}')

        self.held_model = None
        self._armed = False
        self._held_saw_closed = False

    def _nearest_graspable(
        self, grasp_xyz: Vector3
    ) -> Tuple[Optional[str], Optional[PoseState], float, int]:
        with self._poses_lock:
            poses = dict(self._poses)
        candidates = []
        if self.active_model:
            state = poses.get(self.active_model)
            if state is not None and self.active_model.startswith(self.prefix):
                candidates.append((self.active_model, state))
        if not candidates:
            candidates = [(name, state) for name, state in poses.items() if name.startswith(self.prefix)]

        best_name: Optional[str] = None
        best_state: Optional[PoseState] = None
        best_d2 = self.grasp_radius ** 2
        nearest_any = float('inf')
        for name, state in candidates:
            x, y, z = _xyz(state)
            d2 = (x - grasp_xyz[0]) ** 2 + (y - grasp_xyz[1]) ** 2 + (z - grasp_xyz[2]) ** 2
            nearest_any = min(nearest_any, d2)
            if d2 <= best_d2:
                best_name, best_state, best_d2 = name, state, d2
        return best_name, best_state, (nearest_any ** 0.5 if candidates else float('inf')), len(candidates)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimGraspNode()
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
