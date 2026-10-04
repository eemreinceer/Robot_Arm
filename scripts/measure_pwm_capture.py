#!/usr/bin/env python3
"""sigrok-cli CSV yakalamasindan servo PWM darbe genisligi ve periyodu olcer.

2026-07-15 bu araci gerektirdi: kanal 3 hakkinda bilinen her sey bir servonun
davranisindan geriye yurutulen tahmindi ve uc kez yanlis cikti. Pinde ne oldugunu
sormak 10 dakika surdu ve uc soruyu birden cevapladi.

KONTROL KANALI OLMADAN KULLANMA: her zaman bilinen-saglam bir pini de yakala.
Ilk kayitta iki kanal da duz yuksek cikti (GND bagli degildi); kontrol olmasaydi
"pin olu" diye yanlis sonuca varilacakti.

Ayrica: analyzer kanalinin KENDISINI de bilinen-saglam bir pinde dogrula.
"""
import argparse
import statistics
import sys

parser = argparse.ArgumentParser(
    description="sigrok-cli CSV yakalamasindan servo PWM darbe genisligi ve periyot olcer",
    epilog="ornek: sigrok-cli --driver=fx2lafw --config samplerate=1m "
           "--channels D0,D1 --samples 200000 -O csv > cap.csv && "
           "%(prog)s cap.csv --names 'GPIO13 ch1,GPIO25 ch3'",
)
parser.add_argument("path", nargs="?", default="cap.csv")
parser.add_argument("--sample-rate", type=int, default=1_000_000,
                    help="yakalamadaki ornekleme hizi (Hz), sigrok ile ayni olmali")
parser.add_argument("--names", default="",
                    help="kanal etiketleri, virgulle: 'GPIO13 ch1,GPIO25 ch3'")
args = parser.parse_args()

path = args.path
sample_rate_hz = args.sample_rate

rows = []
with open(path) as handle:
    for line in handle:
        line = line.strip()
        if not line or line.startswith(";") or line.startswith("logic"):
            continue
        rows.append([int(v) for v in line.split(",")])

if not rows:
    sys.exit("no samples parsed")

labels = [n.strip() for n in args.names.split(",")] if args.names else []
names = {index: f"D{index} = {label}" for index, label in enumerate(labels) if label}

for channel in range(len(rows[0])):
    values = [r[channel] for r in rows]
    high_count = sum(values)
    print(f"\n=== {names.get(channel, channel)} ===")
    print(f"  yüksek örnek : {high_count} / {len(values)}  ({100.0 * high_count / len(values):.3f}%)")

    edges = []
    for index in range(1, len(values)):
        if values[index] != values[index - 1]:
            edges.append((index, values[index]))
    print(f"  kenar sayısı : {len(edges)}")

    if not edges:
        level = "SÜREKLI YÜKSEK" if values[0] else "SÜREKLI DÜŞÜK (düz hat)"
        print(f"  --> HİÇ DARBE YOK, hat {level}")
        continue

    rising = [i for i, v in edges if v == 1]
    falling = [i for i, v in edges if v == 0]

    widths = []
    for r in rising:
        nxt = [f for f in falling if f > r]
        if nxt:
            widths.append((nxt[0] - r) / sample_rate_hz * 1e6)
    periods = [(rising[i + 1] - rising[i]) / sample_rate_hz * 1e3
               for i in range(len(rising) - 1)]

    if widths:
        print(f"  darbe genişliği: ort {statistics.mean(widths):.1f} us  "
              f"min {min(widths):.1f}  max {max(widths):.1f}  (n={len(widths)})")
    if periods:
        print(f"  periyot        : ort {statistics.mean(periods):.2f} ms  "
              f"min {min(periods):.2f}  max {max(periods):.2f}"
              f"  -> {1000.0 / statistics.mean(periods):.1f} Hz")
