#!/usr/bin/env python3
"""Sim'de hand-eye örnekleri toplar: her pozda TF (A) + PnP (B).

Sim provasının yakalama ayağı. Sim **yalnız kanonik başlatıcıyla** açılır;
ad-hoc `ros2 launch` ile alınan bir prova yeniden üretilebilir sayılmaz:

    ./start_simulation.sh --hand-eye --headless          # ayrı terminal
    # controller'lar active olduktan sonra:
    python3 scripts/sim_hand_eye_capture.py
    python3 scripts/solve_hand_eye.py --samples runs/hand_eye/sim_samples.json
    python3 scripts/hand_eye_pose_set_export.py          # poz kümesini takibe al

Montaj yer gerçeği başlatıcının varsayılanıdır (`--mount-xyz=` / `--mount-rpy=`
ile değiştirilebilir); `--hand-eye` modu perception/pick_place/sorting AÇMAZ,
çünkü otonom döngü aynı controller'a yazarsa ölçüm bozulur.

Sim'de camera_mount launch'tan VERİLDİĞİ için çözülen X'in karşılaştırılacağı
bir yer gerçeği vardır — gerçek kolda mümkün olmayan tek şey bu.

⚠ SADECE SİM. Gerçek donanımda çalıştırılmamalıdır: pozlar doğrudan
JointTrajectory olarak yayınlanır, hiçbir emniyet kapısından geçmez.

KANONİK KOŞU (2026-08-12, `./start_simulation.sh --hand-eye --headless`),
yer gerçeği X = [20, 0, 60] mm:

  18 aday pozdan 5'i tahtayı kadrajda tuttu (reproj 0.050--0.077 px)
  PARK  t = [11.41, -2.06, 59.07] mm  ->  yer gerçeğinden 8.9 mm
  tahta saçılımı (çözücünün başlık metriği)     0.98 mm
  tahta saçılımı (aynı veri, DOĞRU X ile)       0.41 mm rms / 0.68 mm maks
  yöntemler arası yayılım                       3.40 mm

Zincir bu koşuda da SAĞLAM: yer gerçeği X ile sabit tahta 0.41 mm'ye yeniden
kuruluyor, yani FK, TF, PnP ve montaj kompozisyonu doğru. 9 mm zincirden değil,
POZ GEOMETRİSİNDEN geliyor — aynı gürültü çeşitli bilek pozlarında ~1.4 kat,
buradaki ulaşılabilir pozlarda ~11 kat büyüyor
(`scripts/hand_eye_noise_sweep.py`, poz kümesi `data/hand_eye/`).

⚠ DÜZELTME — bu dosyanın önceki sürümü "koşullanma elendi" diyordu; yanlıştı.
Gerekçe olarak yöntemler arası yayılımın 17 kat oynarken hatanın sabit kalması
gösterilmişti. Yayılım koşullanmanın vekili değildir: yöntemlerin birbiriyle
uyuşması, tahminin gürültüye ne kadar dayandığı hakkında bilgi vermez. Sapma
poz kümesinden bağımsız DEĞİL; kümelerin hepsi benzer şekilde kötü koşullu.

⚠ BUNUN GERÇEK KOL İÇİN SONUCU — provanın en değerli çıktısı: 8.9 mm yanlış X,
`solve_hand_eye.py`'nin başlık metriğinde yalnız 0.98 mm gösteriyor ve bu sayı
mutlak olarak MAKUL görünüyor. Gerçek kolda yer gerçeği olmadığı için böyle bir
hata bu metrikle YAKALANMAZ. Tahta saçılımı tek başına kabul kapısı yapılmamalı;
poz kümesinin duyarlılığı ayrıca kanıtlanmalı — şart:
`docs/hand_eye_pose_set_requirement.md`.
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "arm_perception"))
from arm_perception import board_pnp as bp  # noqa: E402

OUT = "runs/hand_eye/sim_samples.json"
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

# Aday poz listesi ve çıktı yolu dışarıdan verilebilir. Aşağıdaki POSES
# varsayılan provanın kümesidir; poz kümesi ARAMASI (issue #8) çok daha geniş
# bir havuzla aynı yakalama yolunu kullanır, ikinci bir kopya çıkmasın diye.
#   --poses <json>   : {"poses": [[q1..q5], ...]} veya doğrudan liste
#   --out   <yol>    : varsayılan runs/hand_eye/sim_samples.json
POSES_FILE = None
for _i, _a in enumerate(sys.argv[1:], start=1):
    if _a == "--poses" and _i + 1 < len(sys.argv):
        POSES_FILE = sys.argv[_i + 1]
    elif _a == "--out" and _i + 1 < len(sys.argv):
        OUT = sys.argv[_i + 1]

# Tahtayı kadrajda tutan küçük gezinme. Hand-eye çözümü için pozların
# DÖNÜŞ ekseni çeşitliliği şart: hepsi aynı eksen etrafındaysa X gözlenemez.
POSES = [
    # BÜYÜK AÇI KÜMESİ (2026-08-12): küçük gezinme kümesi kusursuz sim'de bile
    # 8.92 mm hata ve 60 mm yöntem yayılımı verdi. Hipotez: AX=XB küçük
    # dönüşlerde kötü koşullanıyor. Bu küme dönüş EKSENİ ve GENLİĞİ çeşitliliği
    # için kuruldu; bilek (4,5) kamerayı doğrudan döndürdüğü için ağırlıklı.
    [0.00, 0.00, 0.00, 0.00, 0.00],
    [0.35, 0.00, 0.00, 0.00, 0.00],
    [-0.35, 0.00, 0.00, 0.00, 0.00],
    [0.00, 0.00, 0.00, 0.45, 0.00],
    [0.00, 0.00, 0.00, -0.45, 0.00],
    [0.00, 0.00, 0.00, 0.00, 0.45],
    [0.00, 0.00, 0.00, 0.00, -0.45],
    [0.00, 0.00, 0.00, 0.30, 0.30],
    [0.00, 0.00, 0.00, -0.30, 0.30],
    [0.00, 0.00, 0.00, 0.30, -0.30],
    [0.00, 0.00, 0.00, -0.30, -0.30],
    [0.25, 0.00, 0.00, 0.35, 0.25],
    [-0.25, 0.00, 0.00, -0.35, 0.25],
    [0.25, 0.20, -0.20, -0.30, -0.25],
    [-0.25, -0.20, 0.20, 0.30, -0.25],
    [0.00, 0.25, -0.25, 0.40, 0.00],
    [0.00, -0.25, 0.25, -0.40, 0.00],
    [0.30, 0.15, 0.15, 0.00, 0.35],
]

if POSES_FILE:
    _d = json.load(open(POSES_FILE))
    POSES = _d["poses"] if isinstance(_d, dict) else _d
    print(f"poz listesi disaridan: {POSES_FILE}  ({len(POSES)} aday)")


def m2q(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
            w, x, y, z = (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
        elif i == 1:
            s = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
            w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
        else:
            s = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
            w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s
    return [float(x), float(y), float(z), float(w)]


class Cap(Node):
    def __init__(self):
        super().__init__("sim_hand_eye_capture", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.buf = Buffer()
        TransformListener(self.buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.img = None
        self.K = None
        self.create_subscription(Image, "/camera/image", self._i, qos)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._c, qos)
        self.pub = self.create_publisher(
            JointTrajectory, "/robot_arm_controller/joint_trajectory",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _i(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        self.img = a[:, :, :3].copy()

    def _c(self, m):
        self.K = (np.array(m.k, np.float64).reshape(3, 3),
                  np.array(m.d, np.float64).reshape(1, -1))

    def goto(self, q, secs=2.0):
        t = JointTrajectory()
        t.joint_names = list(JOINTS)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.velocities = [0.0]*len(q)
        p.time_from_start = Duration(sec=int(secs),
                                     nanosec=int((secs-int(secs))*1e9))
        t.points.append(p)
        self.pub.publish(t)

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


rclpy.init()
n = Cap()
n.spin(4.0)
if n.K is None:
    raise SystemExit("camera_info yok")
K, dist = n.K

samples, skipped = [], []
for idx, q in enumerate(POSES):
    n.goto(q)
    n.spin(4.5)                       # hareket + oturma
    n.img = None
    n.spin(1.0)                       # taze kare
    if n.img is None:
        skipped.append((idx, "kare yok")); continue
    gray = cv2.cvtColor(n.img, cv2.COLOR_RGB2GRAY)
    res = bp.estimate_board_pose(gray, bp.DEFAULT_BOARD_COLS,
                                 bp.DEFAULT_BOARD_ROWS,
                                 bp.DEFAULT_SQUARE_SIZE_MM, K, dist)
    if res is None:
        skipped.append((idx, "tahta yok")); continue
    try:
        tf = n.buf.lookup_transform("base_link", "link_5", rclpy.time.Time())
    except Exception as e:
        skipped.append((idx, f"tf: {str(e)[:40]}")); continue
    tr, ro = tf.transform.translation, tf.transform.rotation
    R, _ = cv2.Rodrigues(res["rvec"])
    samples.append({
        "pose_index": idx,
        "joints": list(q),
        "base_to_wrist": {"xyz": [tr.x, tr.y, tr.z],
                          "quat_xyzw": [ro.x, ro.y, ro.z, ro.w]},
        "cam_to_board": {"xyz": [float(v) for v in res["tvec"].ravel()],
                         "quat_xyzw": m2q(R)},
        "reproj_px": float(res["reproj_px"]),
    })
    print(f"poz {idx:2d}: reproj {res['reproj_px']:.3f} px  "
          f"mesafe {res['distance_m']:.4f} m")

n.goto([0.0]*5); n.spin(3.0)
os.makedirs(os.path.dirname(os.path.join(REPO, OUT)), exist_ok=True)
json.dump({"source": "gazebo sim", "samples": samples},
          open(os.path.join(REPO, OUT), "w"), indent=1)
print(f"\ntoplanan: {len(samples)}/{len(POSES)}  atlanan: {skipped}")
print(f"yazildi : {OUT}")
n.destroy_node(); rclpy.shutdown()
