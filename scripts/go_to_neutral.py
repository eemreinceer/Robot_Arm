#!/usr/bin/env python3
"""Robot Arm kol — enerji verildikten SONRA servoları kontrollü olarak nötre götür.

Neden ayrı bir araç: aktivasyon referansı, kolun elle bırakıldığı fiziksel
pozdur (encoder yok). Ray açıldığında servolar o referansı
tutar, yani kol nötrde DEĞİLDİR. Nötr, ölçümlerin (limit, zero_offset) ortak
başlangıç noktası olduğu için oraya gidilmesi gerekir; ama bu gidiş, tek
komutla değil, eklem eklem ve hız sınırlı yapılır.

Kullanım (Nano, bringup ayakta + ARM edilmiş + servo rayı AÇIK):
    python3 go_to_neutral.py [--vel 0.05] [--step 0.02] [--order 1,2,3,4,5,6]

Varsayılan davranış eklem eklemdir: her eklem için önce ne yapacağını yazar,
operatör Enter'a basmadan HİÇBİR komut gitmez. Hareket sırasında herhangi bir
tuş rampayı durdurur ve eklem bulunduğu yerde tutulur.

GÜVENLİK
  * Gerçek e-stop YAZILIM DEĞİLDİR: servo ray kesme anahtarı elinin altında
    olsun. Buradaki "tuşa bas ve dur" yalnız komut üretimini durdurur.
  * Komutlar servo_calibration.yaml'daki min/max_rad clamp'inin dışına asla
    çıkmaz; adım başına hareket üst sınırı 0.10 rad'dır.
  * Nötre giderken eklem mekanik olarak takılırsa DUR: servo zorlar, ısınır.
    Nötr pozun fiziksel olarak erişilebilir olduğunu gözle doğrula.
  * Encoder yok: /joint_states teslim edilen komutun tahminidir. Eklemin
    gerçekten nötre oturduğuna GÖZLE karar ver.
"""

import argparse
import os
import select
import sys
import termios
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import yaml
except ImportError:
    yaml = None

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GRIPPER_JOINT = "joint_6"
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
MAX_STEP = 0.10  # rad, tek komutta izin verilen en büyük hedef değişimi
NEUTRAL = 0.0    # rad; kalibrasyon eşlemesinde 1500 µs'e karşılık gelir
CATCHUP_TOL = 0.01   # rad; teslimatın "yetişti" sayıldığı eşik
# Donanımın teslimat tavanı: servo_calibration.yaml max_velocity_rad_s
# (belirtilmezse 0.1 rad/s). --vel bunun ALTINDA kalmalı, yoksa JTC 0.12 rad
# takip toleransını aşar ve pozisyon tutmaya geçer.
DELIVERY_CEILING = 0.1

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CALIB = os.path.join(
    REPO_ROOT, "src", "robot_arm_description", "config", "servo_calibration.yaml")


def load_hard_clamps(path):
    """servo_calibration.yaml'daki min/max_rad = bu aracın AŞILAMAZ sınırı."""
    clamps = {}
    if yaml is not None and os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        for ch in data.get("channels", []):
            clamps[ch["joint"]] = (float(ch["min_rad"]), float(ch["max_rad"]))
    for j in ALL_JOINTS:
        clamps.setdefault(j, (-1.57, 1.57))
    return clamps


def raw_terminal(fd):
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ICANON | termios.ECHO)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return old


def key_pressed(fd):
    return bool(select.select([fd], [], [], 0)[0]) and bool(os.read(fd, 1))


class NeutralMover(Node):
    def __init__(self, vel):
        super().__init__("robot_arm_go_to_neutral")
        self.vel = vel
        self.state = {}
        self.targets = None  # /joint_states görülene dek komut YOK
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_states, 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/robot_arm_controller/joint_trajectory", 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, "/robot_arm_gripper_controller/joint_trajectory", 10)

    def _on_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.state[name] = pos

    def wait_for_states(self, timeout_s=10.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(j in self.state for j in ALL_JOINTS):
                # Hedefler MEVCUT pozdan başlar; asla "nötrdeyiz" varsayılmaz.
                self.targets = {j: self.state[j] for j in ALL_JOINTS}
                return True
        return False

    def send(self, joint):
        """Seçili eklemin dahil olduğu controller'a tek noktalı hedef bas."""
        if joint == GRIPPER_JOINT:
            names, pub = [GRIPPER_JOINT], self.grip_pub
        else:
            names, pub = ARM_JOINTS, self.arm_pub
        dist = max(abs(self.targets[j] - self.state.get(j, self.targets[j]))
                   for j in names)
        dur = max(dist / self.vel, 0.4)
        traj = JointTrajectory()
        traj.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [self.targets[j] for j in names]
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur % 1.0) * 1e9)
        traj.points = [pt]
        pub.publish(traj)


def ramp_to_neutral(node, joint, step, clamps, fd):
    """Eklemi adım adım nötre götürür. Tuşa basılırsa durur ve False döner.

    Adımlar SAATLE değil, teslimatla ilerler: bir sonraki adım ancak
    /joint_states komut edilen değere yetiştiğinde basılır. Sebebi ölçüldü
    (2026-07-22, mock): donanımın teslimat tavanı 0.1 rad/s
    (`max_velocity_rad_s`), JTC'nin takip toleransı ise 0.12 rad. Açık
    döngüde bundan hızlı komut basılırsa hata birikir, JTC "Holding position
    due to state tolerance violation" deyip trajectory'yi bırakır ve eklem
    yolun ortasında kalır.
    """
    lo, hi = clamps[joint]
    goal = min(max(NEUTRAL, lo), hi)
    if abs(goal - NEUTRAL) > 1e-9:
        print(f"  ⚠ {joint}: nötr {NEUTRAL:+.3f} clamp dışında, "
              f"{goal:+.3f} hedefleniyor.")

    while True:
        current = node.targets[joint]
        delta = goal - current
        if abs(delta) < 1e-4:
            print(f"  ✓ {joint} nötrde ({current:+.3f} rad).")
            return True

        move = max(-step, min(step, delta))
        node.targets[joint] = min(max(current + move, lo), hi)
        node.send(joint)

        # Teslimatın yetişmesini bekle. Zaman aşımı payı cömert: hız tavanı
        # 0.1 rad/s olsa bile bir adım en fazla step/0.1 saniye sürmeli.
        timeout = max(4.0 * abs(move) / node.vel, 3.0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if key_pressed(fd):
                print(f"\n  ■ DURDURULDU — {joint} {node.targets[joint]:+.3f} "
                      "rad'da tutuluyor.")
                return False
            if abs(node.state.get(joint, 1e9) - node.targets[joint]) < CATCHUP_TOL:
                break
        else:
            delivered = node.state.get(joint, float("nan"))
            print(f"\n  ■ TESLİMAT TAKİP ETMİYOR — komut "
                  f"{node.targets[joint]:+.3f}, teslim {delivered:+.3f} rad.")
            print("    Muhtemel sebep: JTC tolerans ihlaliyle pozisyon tutuyor,"
                  " UART hatası, ya da disarm. Devam etme, logu oku.")
            print("    NOT: /joint_states komut tahminidir — mekanik takılmayı"
                  " GÖSTERMEZ, onu gözle kontrol et.")
            return False
        print(f"    {joint} → {node.targets[joint]:+.3f} rad", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=float, default=0.02,
                    help="adım boyu rad (vars. 0.02)")
    ap.add_argument("--vel", type=float, default=0.05,
                    help="hedef hız tavanı rad/s (vars. 0.05 — ilk hareket)")
    ap.add_argument("--order", default="1,2,3,4,5,6",
                    help="eklem sırası (vars. 1,2,3,4,5,6 = Faz 9 sırası)")
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    args = ap.parse_args()
    if args.step > MAX_STEP:
        sys.exit(f"adım {MAX_STEP} rad'ı aşamaz")
    if args.vel <= 0:
        sys.exit("--vel pozitif olmalı")
    if args.vel >= DELIVERY_CEILING:
        sys.exit(f"--vel {DELIVERY_CEILING} rad/s teslimat tavanının altında "
                 "olmalı, yoksa JTC takip toleransını aşar ve pozisyon tutar")

    try:
        order = [ALL_JOINTS[int(n) - 1] for n in args.order.split(",")]
    except (ValueError, IndexError):
        sys.exit("--order 1..6 arası virgülle ayrılmış eklem numaraları olmalı")

    clamps = load_hard_clamps(args.calib)

    print(__doc__)
    print("Onay listesi: servo ray kesme anahtarı elinin altında mı? Kol")
    print("mekanik destekli mi? Çalışma alanı boş mu? Donanım ARM edilmiş mi?")
    if input("Hepsi tamamsa EVET yaz: ").strip().upper() != "EVET":
        sys.exit("İptal edildi.")

    rclpy.init()
    node = NeutralMover(args.vel)
    print("/joint_states bekleniyor...")
    if not node.wait_for_states():
        sys.exit("6 eklemin joint_states verisi gelmedi — bringup ayakta mı, "
                 "donanım ARM edilmiş mi? (disarmed state BAYATTIR)")

    print("\nBaşlangıç pozu (komut tahmini, encoder değil):")
    for j in ALL_JOINTS:
        print(f"  {j:<9}{node.targets[j]:+7.3f} rad")

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    done, skipped, stopped = [], [], []
    try:
        for joint in order:
            travel = NEUTRAL - node.targets[joint]
            print(f"\n── {joint}: {node.targets[joint]:+.3f} → {NEUTRAL:+.3f} "
                  f"rad ({travel:+.3f} rad yol, ~{abs(travel) / args.vel:.0f} s)")
            choice = input("   Enter = başlat, s = atla, q = çık: ").strip().lower()
            if choice == "q":
                break
            if choice == "s":
                skipped.append(joint)
                continue

            print("   Hareket başlıyor — durdurmak için herhangi bir tuş.")
            raw_terminal(fd)
            try:
                ok = ramp_to_neutral(node, joint, args.step, clamps, fd)
            finally:
                termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
            (done if ok else stopped).append(joint)
            if not ok:
                if input("   Devam etmek için Enter, çıkmak için q: "
                         ).strip().lower() == "q":
                    break
    except KeyboardInterrupt:
        print("\nKesildi — son komut korunuyor, servo rayı hâlâ ENERJİLİ.")
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
        print("\n── Özet")
        print(f"  nötre ulaşan : {', '.join(done) if done else '-'}")
        print(f"  durdurulan   : {', '.join(stopped) if stopped else '-'}")
        print(f"  atlanan      : {', '.join(skipped) if skipped else '-'}")
        for j in ALL_JOINTS:
            print(f"  {j:<9}{node.targets[j]:+7.3f} rad")
        if len(done) == len(ALL_JOINTS):
            print("\nAltı eklem de nötrde. Limit ölçümü buradan başlatılabilir:")
            print("  python3 scripts/find_joint_limits.py")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
