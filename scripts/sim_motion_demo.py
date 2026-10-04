#!/usr/bin/env python3
"""Sim koreografisi — MCAP kaydı için tekrarlanabilir hareket üretir.

Gazebo sim ayakta ve controller'lar aktifken çalıştırılır:

    ros2 launch robot_arm_description gazebo.launch.py headless:=true
    python3 scripts/sim_motion_demo.py

Kol için tek bir JointTrajectory, gripper için ayrı bir JointTrajectory
yayınlanır; ikisi de aynı t=0'dan başlar, böylece kayıtta gripper açma/kapama
kol hareketiyle örtüşür. Sim'de joint_6_mirror ayrı sürülür (gz-sim mimic
desteklemez), bkz. config/robot_arm_controllers.yaml.

Açılar URDF limitlerinin %80'i ile sınırlıdır (joint_1..4: ±1.52, joint_5:
-1.52..0.86, joint_6: -0.25..0.35). SADECE SİM — gerçek donanımda
çalıştırılmamalıdır.
"""

import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GRIPPER_JOINTS = ["joint_6", "joint_6_mirror"]

# URDF limitleri (rad) — koreografi bunların %80'ini kullanır.
ARM_LIMITS = {
    "joint_1": (-1.52, 1.52),
    "joint_2": (-1.52, 1.52),
    "joint_3": (-1.52, 1.52),
    "joint_4": (-1.52, 1.52),
    "joint_5": (-1.52, 0.86),
}
GRIPPER_OPEN = 0.28   # joint_6 üst limit 0.35
GRIPPER_CLOSED = -0.20  # joint_6 alt limit -0.25

MARGIN = 0.8


def _dur(t: float) -> Duration:
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def _point(positions, t: float) -> JointTrajectoryPoint:
    p = JointTrajectoryPoint()
    p.positions = [float(x) for x in positions]
    p.velocities = [0.0] * len(positions)
    p.time_from_start = _dur(t)
    return p


def build_arm_trajectory(dwell: float, step: float):
    """Eklem-eklem tarama + birleşik pick benzeri pozlar.

    Döndürülen (traj, süre) — süre son noktanın time_from_start'ı.
    """
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)

    home = [0.0] * len(ARM_JOINTS)
    q = list(home)
    t = dwell
    traj.points.append(_point(q, t))

    # 1) Tek tek her eklem: 0 → +%80 → −%80 → 0
    for idx, name in enumerate(ARM_JOINTS):
        lo, hi = ARM_LIMITS[name]
        for target in (hi * MARGIN, lo * MARGIN, 0.0):
            q = list(q)
            q[idx] = target
            t += step
            traj.points.append(_point(q, t))
            t += dwell
            traj.points.append(_point(q, t))

    # 2) Birleşik pozlar — pick & place benzeri bir gezinme
    poses = [
        [0.6, -0.5, 0.7, 0.0, -0.4],    # yaklaşma
        [0.6, -0.9, 1.0, 0.0, -0.6],    # alçal (kavrama noktası)
        [0.6, -0.5, 0.7, 0.0, -0.4],    # kaldır
        [-0.7, -0.5, 0.7, 0.3, -0.4],   # taşı
        [-0.7, -0.9, 1.0, 0.3, -0.6],   # bırakma noktası
        [-0.7, -0.5, 0.7, 0.3, -0.4],   # geri çekil
        [0.0, 0.0, 0.0, 0.0, 0.0],      # home
    ]
    for pose in poses:
        t += step
        traj.points.append(_point(pose, t))
        t += dwell
        traj.points.append(_point(pose, t))

    # 3) Yavaş sinüs süpürme — sürekli, düzgün hız profili
    sweep_period = 8.0
    cycles = 2
    samples = 40
    for k in range(1, cycles * samples + 1):
        phase = 2.0 * math.pi * k / samples
        q = [
            1.0 * math.sin(phase),
            -0.5 * (1.0 - math.cos(phase)) / 2.0,
            0.7 * math.sin(phase / 2.0),
            0.4 * math.sin(phase),
            -0.3 * (1.0 - math.cos(phase)) / 2.0,
        ]
        t += sweep_period / samples
        traj.points.append(_point(q, t))

    t += step
    traj.points.append(_point(home, t))
    t += dwell
    traj.points.append(_point(home, t))
    return traj, t


def build_gripper_trajectory(total: float, period: float):
    """Kol hareketi boyunca tekrar eden aç/kapa döngüsü."""
    traj = JointTrajectory()
    traj.joint_names = list(GRIPPER_JOINTS)
    t = 0.0
    state = GRIPPER_OPEN
    traj.points.append(_point([0.0, 0.0], 0.5))
    t = 0.5
    while t + period <= total:
        t += period / 2.0
        traj.points.append(_point([state, -state], t))
        t += period / 2.0
        traj.points.append(_point([state, -state], t))
        state = GRIPPER_CLOSED if state == GRIPPER_OPEN else GRIPPER_OPEN
    traj.points.append(_point([0.0, 0.0], total))
    return traj


class MotionDemo(Node):
    def __init__(self, dwell: float, step: float, gripper_period: float):
        # Headless sim gerçek zamandan hızlı koşuyor (ölçüm: RTF ~3, /clock
        # ~1.6 kHz). Bekleme duvar saatiyle yapılırsa hareket biteli çok sonra
        # durulur ve kayıt boş veriyle şişer → sim saati kullanılır.
        super().__init__(
            "robot_arm_sim_motion_demo",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/robot_arm_controller/joint_trajectory", qos
        )
        self.grip_pub = self.create_publisher(
            JointTrajectory, "/robot_arm_gripper_controller/joint_trajectory", qos
        )
        self.arm_traj, self.total = build_arm_trajectory(dwell, step)
        self.grip_traj = build_gripper_trajectory(self.total, gripper_period)

    def run(self) -> float:
        # Abonelerin (JTC) bağlanması için kısa bekleme. Bu bekleme DUVAR
        # saatiyle: discovery sim saatinden bağımsız, ve headless sim gerçek
        # zamandan hızlı aktığı için sim saati 5 s'yi anında tüketiyor.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (self.arm_pub.get_subscription_count() > 0
                    and self.grip_pub.get_subscription_count() > 0):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().warn("controller aboneleri görünmedi, yine de yayınlanıyor")

        self.arm_pub.publish(self.arm_traj)
        self.grip_pub.publish(self.grip_traj)
        self.get_logger().info(
            f"koreografi yayınlandı: kol {len(self.arm_traj.points)} nokta, "
            f"gripper {len(self.grip_traj.points)} nokta, süre {self.total:.1f} s"
        )
        return self.total


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dwell", type=float, default=0.6,
                    help="her poz sonrası bekleme (s)")
    ap.add_argument("--step", type=float, default=1.6,
                    help="pozlar arası geçiş süresi (s)")
    ap.add_argument("--gripper-period", type=float, default=6.0,
                    help="gripper aç/kapa döngü periyodu (s)")
    ap.add_argument("--print-duration-only", action="store_true",
                    help="hareketi yayınlamadan toplam süreyi yaz ve çık")
    args = ap.parse_args()

    if args.print_duration_only:
        _, total = build_arm_trajectory(args.dwell, args.step)
        print(f"{total:.1f}")
        return

    rclpy.init()
    node = MotionDemo(args.dwell, args.step, args.gripper_period)
    try:
        total = node.run()
        end = node.get_clock().now().nanoseconds + int((total + 2.0) * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
