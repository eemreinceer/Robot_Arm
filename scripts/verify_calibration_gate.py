#!/usr/bin/env python3
"""Kalibrasyon kapısının CANLI kanıtı: firmware ile host aynı YAML'dan mı türüyor?

NE YAPAR
  Seri porttan `V?` el sıkışması, ardından `K?` parmak izi isteği. Firmware'in
  bildirdiği değeri `servo_calibration.yaml`'dan türetilenle karşılaştırır ve
  tek satır PASS/FAIL basar.

NEDEN AYRI BİR ARAÇ
  `arm_hardware` bu kapıyı aktivasyonda zaten uyguluyor (`fd26a37`), ama orada
  öğrenmek için controller_manager'ı ayağa kaldırmak, hata ayıklamak için de ROS
  log'u okumak gerekiyor. Flash'tan hemen sonra sorulacak soru bundan basit:
  "firmware doğru kalibrasyonla mı yandı?" Bu araç ROS'suz, kolu sürmeden,
  saniyeler içinde cevaplar. Ray KESİKKEN koşulabilir — hiçbir PWM üretmez.

ÜÇ SONUÇ, ÜÇÜ DE FARKLI ANLAMA GELİR
  PASS          firmware ile host aynı kalibrasyonda.
  FAIL          firmware BAŞKA bir YAML'dan yakılmış → reflash gerekir.
                Bu durumda kol sürülmemelidir: ROS radyanı bir ölçeğe göre
                çeviriyorken firmware başka bir limiti uyguluyor demektir.
  UNVERIFIED    firmware `K?` isteğini bilmiyor (eski build) → doğrulanamadı.
                "Uyuşmuyor" DEĞİLDİR. Host tarafı da bu ikisini ayırıyor
                (`STM32SystemInterface::evaluate_calibration_gate`); burada da
                ayrı tutulur, yoksa eski firmware yanlış kalibrasyon gibi
                görünür ve gereksiz reflash'a yol açar.

FLASH GEREKMEDEN DENEME
  scripts/fake_esp32.py sahte bir ESP32 açar ve üç durumu da taklit eder:

    python3 scripts/fake_esp32.py --device-file /tmp/dev --report /tmp/rep.json \
        --calibration-fingerprint 21aecfe95643057d      # PASS
    python3 scripts/fake_esp32.py ... --calibration-fingerprint deadbeefdeadbeef  # FAIL
    python3 scripts/fake_esp32.py ... --calibration-fingerprint unsupported       # UNVERIFIED

  sonra: python3 scripts/verify_calibration_gate.py --device $(cat /tmp/dev)
"""

import argparse
import os
from pathlib import Path
import select
import sys
import termios
import time

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = REPO / "src/robot_arm_description/config/servo_calibration.yaml"

# Parmak izi hesabı ÜRETEÇTEN alınır, burada YENİDEN YAZILMAZ. Üçüncü bir
# uygulama, üç yerin ayrışabileceği anlamına gelirdi; bugün C++ ile Python
# arasında tam olarak o ayrışma yaşandı (FNV offset basis'inde düşen bir
# basamak) ve ancak çapraz test yakaladı.
sys.path.insert(0, str(REPO / "firmware/esp32_servo_ctrl/tools"))
from generate_robot_config import calibration_fingerprint  # noqa: E402


def open_port(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    cc = list(attrs[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(
        fd,
        termios.TCSANOW,
        [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
         termios.B115200, termios.B115200, cc],
    )
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_line(fd, timeout_s):
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], max(0.0, deadline - time.time()))
        if not ready:
            break
        chunk = os.read(fd, 64)
        if not chunk:
            continue
        for byte in chunk:
            if byte in (0x0A, 0x0D):
                if buf:
                    return buf.decode("ascii", "replace").strip()
            else:
                buf.append(byte)
    return buf.decode("ascii", "replace").strip() if buf else None


def ask(fd, request, timeout_s):
    os.write(fd, request.encode("ascii"))
    return read_line(fd, timeout_s)


def main():
    parser = argparse.ArgumentParser(
        description="firmware ile host'un ayni kalibrasyondan turedigini dogrular")
    parser.add_argument("--device", default="/dev/ttyTHS1")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION,
                        help="host tarafinin kullandigi YAML")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="saniye; ESP32 hala boot ediyor olabilir")
    args = parser.parse_args()

    expected = calibration_fingerprint(args.calibration)
    print(f"host YAML   : {args.calibration.relative_to(REPO)}")
    print(f"host parmak izi : {expected}")

    try:
        fd = open_port(args.device)
    except OSError as error:
        print(f"\nFAIL  seri port acilamadi ({args.device}): {error}")
        print("      Cihaz dogru mu, izinler tamam mi, baska bir surec tutuyor mu?")
        return 2

    try:
        version = ask(fd, "V?\n", args.timeout)
        if version is None:
            print(f"\nUNVERIFIED  MCU {args.timeout:.0f} s icinde cevap vermedi.")
            print("            Karti besleyen guc ve UART kablolamasi kontrol edilmeli;")
            print("            bu bir kalibrasyon sonucu DEGILDIR.")
            return 3
        if not version.startswith("V1,"):
            print(f"\nUNVERIFIED  beklenmeyen el sikismasi cevabi: '{version}'")
            return 3
        print(f"firmware    : {version}")

        reply = ask(fd, "K?\n", args.timeout)
        if reply is None:
            print("\nUNVERIFIED  'K?' istegine cevap yok.")
            print("            Muhtemelen bu firmware kapiyi bilmiyor (fd26a37 oncesi).")
            return 3
        if not reply.startswith("K,"):
            # Eski firmware bilinmeyen satiri E1 ile reddeder. Bu "uyusmuyor"
            # degil "soramadik" demektir ve reflash karari farklidir.
            print(f"\nUNVERIFIED  firmware 'K?' istegini bilmiyor (cevap: '{reply}').")
            print("            Kapi bu firmware'de YOK. fd26a37 sonrasi bir build")
            print("            flash'lanmali; ondan once dogrulama yapilamaz.")
            return 3

        reported = reply[2:].strip()
        if not reported:
            print("\nUNVERIFIED  firmware bos parmak izi bildirdi ('K,').")
            return 3

        print(f"firmware parmak izi : {reported}")
        if reported == expected:
            print("\nPASS  firmware ve host ayni kalibrasyondan turuyor.")
            print("      Artik `require_calibration_match:=true` ile kosulabilir.")
            return 0

        print("\nFAIL  KALIBRASYON UYUSMUYOR — kol SURULMEMELI.")
        print(f"      host beklentisi : {expected}")
        print(f"      firmware bildirdi: {reported}")
        print("      Firmware baska bir YAML'dan yakilmis. ROS radyani bir olcege")
        print("      gore cevirirken firmware baska bir limiti uyguluyor demektir.")
        print("      Duzeltme: python3 firmware/esp32_servo_ctrl/tools/generate_robot_config.py")
        print("                sonra firmware'i yeniden flash'la ve bu araci tekrar kostur.")
        return 1
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
