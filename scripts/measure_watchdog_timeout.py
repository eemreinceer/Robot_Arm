#!/usr/bin/env python3
"""Watchdog politikasi C'nin seri tarafini olcer: akis kesilince MCU ne diyor?

NE YAPAR
  1. `P` cerceveslerini 20 Hz'de N saniye akitir (servo rayi KESIK olmali).
  2. Sonra GONDERMEYI KESER ama PORTU ACIK TUTAR ve gelen her satiri
     zaman damgasiyla kaydeder.
  3. Ilk `E3`'un gecikmesini, tekrar araligini ve toplam sayisini basar.

NEDEN AYRI BIR ARAC
  bench_servo.py akisi bitirince cikar ve portu kapatir; E3 tam da o anda
  gelmeye baslar, yani kimse okumaz. verify_stop_frame.py stop cercevesini
  olcer, timeout'u degil. Buradaki soru bunlardan farkli: "host sustuktan sonra
  MCU sustu mu, yoksa duyurdu mu, ve ne siklikta?"

NE OLCMEZ -- BU ONEMLI
  Bu arac PWM'e BAKMAZ. `E3` gormek darbenin surdugunu KANITLAMAZ; darbenin
  kesildigini de kanitlamaz. Politika C'nin fiziksel iddiasi ("PWM latch'li
  kalir") yalniz logic analyzer ile, pinde olculur:

    sigrok-cli --driver=fx2lafw --config samplerate=1m --time 8000ms \\
        --channels D0,D1 -O csv > cap.csv
    python3 scripts/measure_pwm_capture.py cap.csv --names 'GPIO13,GPIO2'

  Ayni disiplin verify_stop_frame.py'de de yaziyor: STATE=DISARMED gormek PWM'in
  kesildigi anlamina GELMEZ. Log, sinyalin yerine gecmez.

GUVENLIK
  Servo rayi KESIK olmalidir. Ray acikken bu arac kolu son komutta tutar ve
  akis kesildiginde tork basili kalir -- politika C'nin bilincli davranisi budur.
"""

import argparse
import os
import select
import sys
import termios
import time


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


def drain_lines(fd, out, started):
    """Butun bekleyen satirlari (zaman damgasiyla) out listesine ekler."""
    buf = bytearray()
    while True:
        ready, _, _ = select.select([fd], [], [], 0.0)
        if not ready:
            break
        chunk = os.read(fd, 256)
        if not chunk:
            break
        for byte in chunk:
            if byte in (0x0A, 0x0D):
                if buf:
                    out.append((time.time() - started,
                                buf.decode("ascii", "replace").strip()))
                    buf = bytearray()
            else:
                buf.append(byte)
    if buf:
        out.append((time.time() - started,
                    buf.decode("ascii", "replace").strip()))


def main():
    parser = argparse.ArgumentParser(
        description="watchdog timeout'unda MCU'nun seri davranisini olcer")
    parser.add_argument("--device", default="/dev/ttyTHS1")
    parser.add_argument("--pulse-us", type=int, default=1500,
                        help="alti kanala da gonderilecek darbe (varsayilan 1500)")
    parser.add_argument("--stream-seconds", type=float, default=3.0,
                        help="kesmeden once akis suresi")
    parser.add_argument("--listen-seconds", type=float, default=5.0,
                        help="akis kesildikten sonra dinleme suresi")
    args = parser.parse_args()

    frame = ("P" + ",".join([str(args.pulse_us)] * 6) + "\n").encode("ascii")

    try:
        fd = open_port(args.device)
    except OSError as error:
        print("port acilamadi (%s): %s" % (args.device, error))
        return 2

    # A disconnected or floating RX line leaves rubbish in the firmware's line
    # buffer, and the next real request comes back as ERR,LINE_TOO_LONG. Measured
    # 2026-08-15 after the UART harness was reseated. A bare newline closes the
    # partial line so the session starts from a known state.
    os.write(fd, b"\n")
    time.sleep(0.3)
    termios.tcflush(fd, termios.TCIFLUSH)

    started = time.time()
    lines = []
    try:
        print("akis: %.1f s @20 Hz, darbe %d us" % (args.stream_seconds, args.pulse_us))
        stream_until = started + args.stream_seconds
        frames = 0
        while time.time() < stream_until:
            os.write(fd, frame)
            frames += 1
            drain_lines(fd, lines, started)
            time.sleep(0.05)
        stopped_at = time.time() - started
        print("akis KESILDI  t=%.3f s  (%d cerceve gonderildi)" % (stopped_at, frames))

        listen_until = time.time() + args.listen_seconds
        while time.time() < listen_until:
            drain_lines(fd, lines, started)
            time.sleep(0.01)
    finally:
        os.close(fd)

    e3 = [t for t, text in lines if text == "E3"]
    acks = [t for t, text in lines if text == "OK"]
    other = [(t, text) for t, text in lines if text not in ("OK", "E3")]

    print("")
    # Without this line the run is ambiguous: zero E3 reads the same whether the
    # MCU held its tongue or was never listening. On 2026-08-15 the UART harness
    # was unplugged and the output looked like a watchdog that did not fire.
    print("ACK (OK) sayisi: %d / %d cerceve" % (len(acks), frames))
    if not acks:
        print("  UYARI: hic ACK yok -- MCU bu oturumda hic dinlemedi.")
        print("  Bu durumda E3'un gelmemesi watchdog hakkinda BIR SEY SOYLEMEZ:")
        print("  firmware P kabul etmediyse PWM hic acilmaz, watchdog da tetiklenmez.")
    print("akis kesildikten sonra gelen E3 sayisi: %d" % len(e3))
    if e3:
        print("ilk E3 gecikmesi: %.3f s (akis kesildikten sonra)" % (e3[0] - stopped_at))
        if len(e3) > 1:
            gaps = [b - a for a, b in zip(e3, e3[1:])]
            print("tekrar araligi: min %.3f  ort %.3f  max %.3f s"
                  % (min(gaps), sum(gaps) / len(gaps), max(gaps)))
        else:
            print("tekrar YOK -- tek E3 geldi (politika C oncesi davranis)")
    else:
        print("E3 HIC GELMEDI")
    if other:
        print("diger satirlar:")
        for stamp, text in other[:20]:
            print("  t=%.3f  '%s'" % (stamp, text))

    print("")
    print("HATIRLATMA: bu cikti PWM hakkinda HICBIR SEY soylemez.")
    print("Darbe surdu mu, logic analyzer kaydindan okunur.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
