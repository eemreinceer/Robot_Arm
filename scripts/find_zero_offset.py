#!/usr/bin/env python3
"""Robot Arm kol — `zero_offset_rad` ölç: servo nötrü ile URDF sıfırı arasındaki fark.

NE ÖLÇÜYORUZ
  URDF'nin "sıfır poz" dediği fiziksel konfigürasyon ile servo nötrünün
  (1500 µs) denk geldiği konfigürasyon aynı değil — CAD export'u eğri bir
  pozdan geliyor ve horn spline'ı ~15-18°'lik adımlarla oturuyor. Aradaki
  eklem başına fark `zero_offset_rad`'dır.

NASIL
  RViz'de URDF sıfır pozunda SAYDAM BİR HAYALET model durur (bkz.
  `scripts/rviz_pc.sh --ghost`). Canlı model seninle beraber hareket eder,
  hayalet durur. Her eklemi hayaletin üstüne oturana kadar gezdir, sonra `r`
  ile kaydet. O andaki komut değeri doğrudan o kanalın `zero_offset_rad`'ıdır.

  Neden birebir o değer: sürücü `pulse = f(q + zero_offset)` uygular. Şu anda
  offset 0 iken eklem q* komutunda URDF sıfırına oturuyorsa, offset'i q*
  yaptığımızda q=0 komutu aynı darbeyi üretir. Yani `zero_offset_rad = q*`.

Kullanım (Nano, bringup ayakta + ARM edilmiş + servo rayı AÇIK):
    python3 find_zero_offset.py [--step 0.05] [--vel ...]

Hız varsayılanı servo_calibration.yaml'daki `max_velocity_rad_s`'ten türer
(tavanın %60'ı). Sabit yazılmıyor: iki yer ayrışırsa ya gereksiz yavaş kalır
ya da teslimattan hızlı komut basıp JTC takip toleransını aşar.

Tuşlar:
    1..6        eklem seç (6 = gripper)
    + / -       seçili eklemi bir adım oynat
    a           adım boyu döngüsü (0.01 / 0.02 / 0.05 / 0.10 rad)
    r           BU EKLEM HAYALETE OTURDU → offset olarak kaydet
    x           seçili eklemin kaydını sil
    0           seçili eklemi yavaşça 0.0'a götür
    p           tabloyu yazdır
    q           çık (eksik kayıt varsa uyarır; ikinci q ile çıkar)

Her kayıt ANINDA dosyaya yazılır — terminal kopsa da ölçüm kaybolmaz.

GÜVENLİK
  * Gerçek e-stop YAZILIM DEĞİLDİR: servo ray kesme anahtarı elinin altında.
  * Komutlar servo_calibration.yaml limitinin dışına çıkmaz; adım ≤ 0.10 rad.
  * Hız --vel ile sınırlı ve donanım tavanının (`max_velocity_rad_s`)
    ALTINDA tutulur, yoksa JTC takip toleransını aşıp pozisyon tutmaya geçer.
  * Encoder yok: /joint_states komut tahminidir. Eklemin hayalete oturduğuna
    GÖZLE karar ver.
  * Komisyon zarfı: arming referansından ±commissioning_range_rad dışına
    çıkamazsın. Eklem oturmadan komut ilerlemiyorsa araç bunu söyler —
    zarfı geçici yükseltip stack'i yeniden başlatmak gerekir.
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
MAX_STEP = 0.10
FALLBACK_CEILING = 0.1   # arm_hardware'in max_velocity_rad_s varsayilani

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CALIB = os.path.join(
    REPO_ROOT, "src", "robot_arm_description", "config", "servo_calibration.yaml")


def load_delivery_ceiling(path):
    """Donanimin gercek hiz tavani: en dusuk max_velocity_rad_s.

    Sabit yazilmiyor cunku config'te degistirilebilir ve iki yer sessizce
    ayrisirsa arac ya gereksiz yavas kosar ya da teslimattan hizli komut
    basip JTC'nin 0.12 rad takip toleransini asar.
    """
    ceiling = None
    if yaml is not None and os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        values = [float(ch.get("max_velocity_rad_s", FALLBACK_CEILING))
                  for ch in data.get("channels", [])]
        if values:
            ceiling = min(values)
    return ceiling if ceiling and ceiling > 0 else FALLBACK_CEILING


def load_hard_clamps(path):
    """Aşılamaz sınır: limit_min/max_rad varsa o, yoksa min/max_rad."""
    clamps = {}
    if yaml is not None and os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        for ch in data.get("channels", []):
            lo = float(ch.get("limit_min_rad", ch["min_rad"]))
            hi = float(ch.get("limit_max_rad", ch["max_rad"]))
            clamps[ch["joint"]] = (lo, hi)
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


def fmt_table(targets, recorded, clamps, sel, step):
    lines = ["", f"  seçili: {sel}   adım: {step:.2f} rad",
             "  eklem     komut    kayıtlı offset      derece   (clamp)"]
    for j in ALL_JOINTS:
        off = recorded[j]
        c_lo, c_hi = clamps[j]
        mark = ">" if j == sel else " "
        off_s = "    --   " if off is None else f"{off:+8.3f} "
        deg_s = "    --  " if off is None else f"{off * 57.2958:+7.1f}°"
        lines.append(
            f" {mark}{j:<9}{targets[j]:+7.3f}   {off_s}  {deg_s}"
            f"   ({c_lo:+5.2f}..{c_hi:+5.2f})")
    done = sum(1 for j in ALL_JOINTS if recorded[j] is not None)
    lines.append(f"  kayıtlı: {done}/6")
    return "\n".join(lines)


def write_results(recorded, out_path):
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = [
        f"# Robot Arm zero_offset ölçümü — {stamp}",
        "# zero_offset_rad = eklemin URDF sıfır pozuna oturduğu KOMUT değeri.",
        "# Sürücü pulse = f(q + zero_offset) uygular, yani bu değer doğrudan",
        "# servo_calibration.yaml'a yazılır.",
        "# NOT: encoder yok; hizalama GÖZLE yapıldı, RViz hayalet modeline karşı.",
        "joints:"]
    snippet = ["\n# --- servo_calibration.yaml için ---"]
    for j in ALL_JOINTS:
        off = recorded[j]
        lines.append(f"  {j}:")
        if off is None:
            lines.append("    zero_offset_rad: INCOMPLETE  # kaydedilmedi")
            continue
        lines.append(f"    zero_offset_rad: {off:.4f}")
        lines.append(f"    degrees: {off * 57.2958:.2f}")
        snippet.append(f"# {j}: zero_offset_rad: {off:.4f}")
    with open(out_path, "w") as f:
        f.write("\n".join(lines + snippet) + "\n")
    return out_path


class OffsetFinder(Node):
    def __init__(self, vel):
        super().__init__("robot_arm_zero_offset_finder")
        self.vel = vel
        self.state = {}
        self.targets = None
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
                self.targets = {j: self.state[j] for j in ALL_JOINTS}
                return True
        return False

    def send(self, joint):
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--vel", type=float, default=None,
                    help="hedef hız rad/s (varsayılan: donanım tavanının %60'ı)")
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    if args.step > MAX_STEP:
        sys.exit(f"adım {MAX_STEP} rad'ı aşamaz")
    ceiling = load_delivery_ceiling(args.calib)
    if args.vel is None:
        args.vel = round(ceiling * 0.6, 4)
    if args.vel >= ceiling:
        sys.exit(f"--vel {ceiling} rad/s teslimat tavanının ALTINDA olmalı "
                 "(servo_calibration.yaml max_velocity_rad_s)")
    print(f"donanım hız tavanı {ceiling} rad/s → jog hızı {args.vel} rad/s")

    clamps = load_hard_clamps(args.calib)

    print(__doc__)
    print("Onay listesi: servo ray kesme anahtarı elinin altında mı? Kol")
    print("mekanik destekli mi? RViz'de SAYDAM HAYALET model görünüyor mu?")
    if input("Hepsi tamamsa EVET yaz: ").strip().upper() != "EVET":
        sys.exit("İptal edildi.")

    rclpy.init()
    node = OffsetFinder(args.vel)
    print("/joint_states bekleniyor...")
    if not node.wait_for_states():
        sys.exit("6 eklemin joint_states verisi gelmedi — bringup ayakta mı, "
                 "donanım ARM edilmiş mi?")

    recorded = {j: None for j in ALL_JOINTS}
    sel = ARM_JOINTS[0]
    step_i = STEP_CYCLE.index(args.step) if args.step in STEP_CYCLE else 1
    step = STEP_CYCLE[step_i]
    goto_zero = None
    pending_quit = False

    out = args.out or os.path.join(
        REPO_ROOT, "runs",
        f"zero_offset_{datetime.now().strftime('%Y%m%d_%H%M')}.yaml")
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
                if abs(node.targets[sel] - before) < 1e-9:
                    print(f"\n  ⛔ {sel} YAZILIM SINIRINDA ({before:+.3f} rad)"
                          " — komut daha ileri gitmiyor.")
                # Teslimat takip ediyor mu? Etmiyorsa komisyon zarfına
                # takılmış olabiliriz ve kol hiç kıpırdamaz.
                deadline = time.time() + max(4.0 * step / node.vel, 3.0)
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if abs(node.state.get(sel, 1e9) - node.targets[sel]) < 0.01:
                        break
                else:
                    print(f"\n  ■ {sel} TESLİMAT TAKİP ETMİYOR — komut"
                          f" {node.targets[sel]:+.3f}, teslim"
                          f" {node.state.get(sel, float('nan')):+.3f} rad.")
                    print("    Muhtemel sebep: KOMİSYON ZARFI sınırı"
                          " (arming referansından ±commissioning_range_rad).")
                    print("    Ölçüm için zarfı yükseltip stack'i yeniden"
                          " başlatmak gerekebilir.")
            elif c == "a":
                step_i = (step_i + 1) % len(STEP_CYCLE)
                step = STEP_CYCLE[step_i]
            elif c == "r":
                recorded[sel] = node.targets[sel]
                write_results(recorded, out)
                print(f"\n  ✓ {sel} offset kaydedildi: {recorded[sel]:+.4f} rad"
                      f" ({recorded[sel] * 57.2958:+.1f}°)")
            elif c == "x":
                recorded[sel] = None
                write_results(recorded, out)
                print(f"\n  {sel} kaydı silindi.")
            elif c == "0":
                goto_zero = sel
            elif c == "p":
                pass
            elif c == "q":
                missing = [j for j in ALL_JOINTS if recorded[j] is None]
                if missing and not pending_quit:
                    pending_quit = True
                    print(f"\n  ⚠ EKSİK kayıt: {', '.join(missing)} — yine de"
                          " çıkmak için tekrar q, devam için başka tuş.")
                    continue
                break
            else:
                continue
            print(fmt_table(node.targets, recorded, clamps, sel, step))
    except KeyboardInterrupt:
        print("\nCtrl+C — kayitlar korunuyor.")
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
        write_results(recorded, out)
        print(f"\nSonuç: {out}")
        for j in ALL_JOINTS:
            off = recorded[j]
            print(f"  {j:<9}" + ("kaydedilmedi" if off is None
                                 else f"{off:+.4f} rad ({off * 57.2958:+.1f}°)"))
        # 2026-07-22'de bir kosu tam da boyle bosa gitti: eklemler gezdirildi
        # ama 'r'ye hic basilmadigi icin dosya alti INCOMPLETE ile yazildi.
        # Cikis hangi yoldan olursa olsun (q, Ctrl+C, hata) bunu yuksek sesle
        # soyle -- ozet satirlarinin arasinda kaynayip gitmesin.
        missing = [j for j in ALL_JOINTS if recorded[j] is None]
        if missing:
            print("\n" + "=" * 62)
            print(f"⚠ KOSU EKSIK — {len(missing)}/6 eklem kaydedilmedi:")
            print(f"  {', '.join(missing)}")
            print("  Eklemi hayalete oturttuktan sonra 'r' TUSUNA BASMAK sart;")
            print("  sadece gezdirmek kaydetmez. Bu dosya kullanilamaz.")
            print("=" * 62)
        else:
            print("\n✓ Alti eklem de kaydedildi.")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
