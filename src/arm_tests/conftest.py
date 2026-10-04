"""arm_tests toplama kapıları.

`unit/` ve `integration/` dizinlerinde unit test YOKTUR: her modül canlı bir
ROS 2 grafiğine konuşuyor -- MoveIt `/compute_fk`, `/dl_ik_solve`, çalışan bir
Gazebo dünyası veya YOLO tespit node'u. Bu yığın ayakta değilken bu testler
hızlıca düşmüyor; ya import hatasıyla tüm toplamayı çökertiyor ya da
timeout'larda dakikalarca yanıyor.

Ölçüm (2026-08-07, ROS Jazzy source edilmiş, yığın kapalı):

    unit/test_dl_ik_robustness.py     5 fail, 2 s
    unit/test_forward_kinematics.py   2 fail 1 skip, 23 s
    unit/test_yolo_detection.py       2 fail, 29 s
    unit/test_inverse_kinematics.py   >90 s askıda (200 örnek x 2 s spin)
    unit/test_3d_pose_estimation.py   >90 s askıda (9 parametre x ~15 s gz)
    integration/                      collect error (arm_interfaces yok)

Bu yüzden yığın testleri, ARM_TESTS_STACK=1 yığının ayakta olduğunu söylemedikçe
toplama aşamasında atlanıyor. Marker'la deselect etmek YETMEZ: import hatası
marker uygulanmadan önce oluşuyor, o yüzden kapı `collect_ignore_glob`.

Varsayılan koşu bu nedenle `utils/` paketini çalıştırır -- saf matematik ve
hand-eye oturum araçları, canlı donanım gerektirmeyen gerçek testler.
"""

import os

_flag = os.environ.get("ARM_TESTS_STACK", "")
STACK_ENABLED = _flag.strip().lower() not in ("", "0", "false", "no")

collect_ignore_glob = []
if not STACK_ENABLED:
    collect_ignore_glob += ["unit/test_*.py", "integration/*.py"]


def pytest_report_header(config):
    """Yeşil bir koşunun neyi kapsamadığını her koşuda görünür kıl.

    Bu projede tekrar eden hata sınıfı, eksik kapsamı 'repo yeşil' diye okumak.
    Başlık satırı atlananı söyler ki yeşil, olduğundan fazlası sanılmasın.
    """
    if STACK_ENABLED:
        return (
            "arm_tests: yığın testleri AÇIK (ARM_TESTS_STACK) -- "
            "canlı ROS 2 grafiği + Gazebo + YOLO gerekiyor"
        )
    return (
        "arm_tests: yığın testleri toplanmadı (unit/, integration/) -- "
        "canlı ROS 2 grafiği ister; açmak icin ARM_TESTS_STACK=1"
    )
