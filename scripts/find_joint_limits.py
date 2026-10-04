#!/usr/bin/env python3
"""Robot Arm kol — eklem limitlerini KÜÇÜK ADIMLARLA bularak kaydet (Faz 6/11 köprüsü).

MoveIt limitleri TÜKETİR, ölçmez: bu araç limitler henüz yokken kolu güvenle
gezdirmek için planlayıcıyı atlar ve doğrudan joint_trajectory_controller'a
küçük artımlı hedefler yollar. RViz (robot_arm_gui) sadece izleme içindir.

Kullanım (Nano, hardware_bringup.launch.py ayaktayken, aynı konteyner/domain):
    python3 find_joint_limits.py [--step 0.05] [--vel 0.1] [--margin 0.05]

Tuşlar:
    1..6        eklem seç (6 = gripper)
    + / -       seçili eklemi bir adım oynat
    a           adım boyu döngüsü (0.01 / 0.02 / 0.05 / 0.10 rad)
    n  (veya [) şu anki komutu bu eklemin MIN'i olarak kaydet
    m  (veya ]) şu anki komutu bu eklemin MAX'ı olarak kaydet
    0           seçili eklemi yavaşça 0.0'a götür (kayıtlı limitler içinde)
    p           kayıt tablosunu yazdır
    q           çık (eksik kayıt varsa uyarır; ikinci q ile çıkar)

Her kayıt ANINDA sonuç dosyasına yazılır — terminal kopsa da ölçüm kaybolmaz.

GÜVENLİK
  * Gerçek e-stop YAZILIM DEĞİLDİR: servo ray kesme anahtarı elinin altında olsun.
  * Yazılım asla servo_calibration.yaml'daki mevcut min/max_rad clamp'inin
    dışına komut üretmez; adım başına hareket üst sınırı 0.10 rad'dır.
  * Hedef hız --vel (vars. 0.1 rad/s) ile sınırlanır; nokta süresi buna göre
    hesaplanır (min 0.4 s).
  * Encoder yok: /joint_states teslim edilen komutun tahminidir. Limit kararını
    GÖZLE ver — mekanik stopa DAYANMADAN, kablo gerilmeden önce kaydet.
"""

import argparse
import os
import select
import sys
import termios
import time
from datetime import datetime

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
STEP_CYCLE = [0.01, 0.02, 0.05, 0.10]
MAX_STEP = 0.10  # rad, tek komutta izin verilen en büyük hedef değişimi

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


class LimitFinder(Node):
    def __init__(self, vel):
        super().__init__("robot_arm_limit_finder")
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
                # Hedefler MEVCUT pozdan başlar; asla "sıfırdayız" varsayılmaz.
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


def raw_terminal(fd):
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ICANON | termios.ECHO)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)
    return old


def fmt_table(targets, recorded, clamps, sel, step):
    lines = ["", f"  seçili: {sel}   adım: {step:.2f} rad",
             "  eklem     hedef     kayıt-min  kayıt-max  (yazılım clamp)"]
    for j in ALL_JOINTS:
        lo, hi = recorded[j]
        c_lo, c_hi = clamps[j]
        mark = ">" if j == sel else " "
        lines.append(
            f" {mark}{j:<9}{targets[j]:+7.3f}   "
            f"{'  --  ' if lo is None else f'{lo:+6.3f}'}     "
            f"{'  --  ' if hi is None else f'{hi:+6.3f}'}   "
            f"({c_lo:+5.2f}..{c_hi:+5.2f})")
    return "\n".join(lines)


def write_results(recorded, clamps, margin, out_path):
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = [
        f"# Robot Arm eklem limit ölçümü — {stamp}",
        f"# margin: {margin} rad (soft = ölçülen ∓ margin)",
        "# NOT: encoder yok; değerler komut tahminidir, gözle doğrulanmıştır.",
        "joints:"]
    snippets = ["\n# --- servo_calibration.yaml için min/max_rad ---"]
    urdf = ["\n# --- URDF (robot_arm_body.xacro): continuous -> revolute ---"]
    for j in ALL_JOINTS:
        lo, hi = recorded[j]
        c_lo, c_hi = clamps[j]
        lines.append(f"  {j}:")
        if lo is None or hi is None:
            lines.append("    measured: INCOMPLETE  # min ve/veya max kaydedilmedi")
            continue
        at_clamp = [
            name for name, v in (("min", lo), ("max", hi))
            if min(abs(v - c_lo), abs(v - c_hi)) < 1e-6]
        if at_clamp:
            lines.append(
                f"    at_software_clamp: [{', '.join(at_clamp)}]  "
                "# YAZILIM sınırı, mekanik limit ölçümü DEĞİL")
        if lo >= hi:
            lines.append(
                f"    measured_min: {lo:.3f}")
            lines.append(
                f"    measured_max: {hi:.3f}")
            lines.append(
                "    soft: INVALID  # min >= max, tuşlar ters kullanılmış")
            continue
        s_lo, s_hi = lo + margin, hi - margin
        if s_lo >= s_hi:
            lines.append(f"    measured_min: {lo:.3f}")
            lines.append(f"    measured_max: {hi:.3f}")
            lines.append("    soft: INVALID  # margin aralığı yuttu, elle karar ver")
            continue
        lines += [f"    measured_min: {lo:.3f}", f"    measured_max: {hi:.3f}",
                  f"    soft_min: {s_lo:.3f}", f"    soft_max: {s_hi:.3f}"]
        snippets.append(f"# {j}: min_rad: {s_lo:.3f}  max_rad: {s_hi:.3f}")
        if j != GRIPPER_JOINT:
            urdf.append(
                f'# {j}: <limit lower="{s_lo:.3f}" upper="{s_hi:.3f}" .../>')
    with open(out_path, "w") as f:
        f.write("\n".join(lines + snippets + urdf) + "\n")
    return out_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--vel", type=float, default=0.1,
                    help="hedef hız tavanı rad/s (vars. 0.1)")
    ap.add_argument("--margin", type=float, default=0.05,
                    help="soft limit güvenlik payı rad (vars. 0.05)")
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    if args.step > MAX_STEP:
        sys.exit(f"adım {MAX_STEP} rad'ı aşamaz")

    clamps = load_hard_clamps(args.calib)

    print(__doc__)
    print("Onay listesi: servo ray kesme anahtarı elinin altında mı? Kol")
    print("mekanik destekli mi? Çalışma alanı boş mu? Kamera/CSI ayrık mı?")
    if input("Hepsi tamamsa EVET yaz: ").strip().upper() != "EVET":
        sys.exit("İptal edildi.")

    rclpy.init()
    node = LimitFinder(args.vel)
    print("/joint_states bekleniyor...")
    if not node.wait_for_states():
        sys.exit("6 eklemin joint_states verisi gelmedi — bringup ayakta mı?")
    print("Başlangıç pozu alındı; hedefler mevcut pozdan başlıyor.")

    recorded = {j: [None, None] for j in ALL_JOINTS}
    sel = ARM_JOINTS[0]
    step_i = STEP_CYCLE.index(args.step) if args.step in STEP_CYCLE else 2
    step = STEP_CYCLE[step_i]
    goto_zero = None  # aktifse seçili eklem 0'a adım adım gider
    pending_quit = False

    out = args.out or os.path.join(
        REPO_ROOT, "runs",
        f"joint_limits_{datetime.now().strftime('%Y%m%d_%H%M')}.yaml")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    fd = sys.stdin.fileno()
    old_attrs = raw_terminal(fd)
    print(fmt_table(node.targets, recorded, clamps, sel, step))
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            if goto_zero is not None:
                j = goto_zero
                cur = node.targets[j]
                if abs(cur) <= 0.005:
                    goto_zero = None
                else:
                    nxt = cur - min(MAX_STEP / 2, abs(cur)) * (1 if cur > 0 else -1)
                    lo, hi = clamps[j]
                    node.targets[j] = min(max(nxt, lo), hi)
                    node.send(j)
                    time.sleep(0.6)
                continue
            ready, _, _ = select.select([fd], [], [], 0.05)
            if not ready:
                continue
            ch = os.read(fd, 3).decode(errors="replace")
            if not ch:
                continue
            c = ch[0]
            if c != "q":
                pending_quit = False
            if c in "123456":
                sel = ALL_JOINTS[int(c) - 1]
            elif c in "+=-_":
                sign = 1 if c in "+=" else -1
                lo, hi = clamps[sel]
                before = node.targets[sel]
                node.targets[sel] = min(max(before + sign * step, lo), hi)
                node.send(sel)
                # Sessiz doyum tuzagi: hedef yazilim sinirinda takilirsa kol
                # durur ve bu operatore MEKANIK stop gibi gorunur. Yuksek sesle
                # soyle, yoksa kendi config'imizi "olcum" diye kaydederiz.
                if abs(node.targets[sel] - before) < 1e-9:
                    print(f"\n  ⛔ {sel} YAZILIM SINIRINDA ({before:+.3f} rad) —"
                          " kol mekanik stopa DAYANMADI, komut clamp'e dayandı.")
                    print("     Bunu limit olarak kaydedersen kendi ayarını"
                          " ölçmüş olursun, robotu değil.")
            elif c == "a":
                step_i = (step_i + 1) % len(STEP_CYCLE)
                step = STEP_CYCLE[step_i]
            elif c in "[nm]":
                slot = 0 if c in "[n" else 1
                value = node.targets[sel]
                lo, hi = clamps[sel]
                recorded[sel][slot] = value
                if min(abs(value - lo), abs(value - hi)) < 1e-6:
                    print(f"\n  ⚠ {sel} {'MIN' if slot == 0 else 'MAX'} kaydı"
                          f" tam yazılım sınırında ({value:+.3f} rad).")
                    print("     Dosyaya 'at_software_clamp' diye işaretlenecek:"
                          " bu mekanik limit ölçümü DEĞİLDİR.")
                lo_rec, hi_rec = recorded[sel]
                if lo_rec is not None and hi_rec is not None and lo_rec >= hi_rec:
                    print(f"\n  ⚠ {sel}: min ({lo_rec:+.3f}) >= max"
                          f" ({hi_rec:+.3f}) — tuşlar ters kullanılmış olabilir."
                          " Eklemi doğru uca götürüp yeniden kaydet.")
                write_results(recorded, clamps, args.margin, out)
            elif c == "0":
                goto_zero = sel
            elif c == "p":
                pass  # tablo aşağıda her tuşta basılıyor
            elif c == "q":
                missing = [j for j in ALL_JOINTS
                           if recorded[j][0] is None or recorded[j][1] is None]
                if missing and not pending_quit:
                    pending_quit = True
                    print(f"\n  ⚠ EKSİK kayıt: {', '.join(missing)} — yine de"
                          " çıkmak için tekrar q, devam için başka tuş.")
                    continue
                break
            else:
                continue
            print(fmt_table(node.targets, recorded, clamps, sel, step))
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
        path = write_results(recorded, clamps, args.margin, out)
        print(f"\nSonuç yazıldı: {path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
