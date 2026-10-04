#!/usr/bin/env python3
"""Mekanik payını ölç: aynı hedefe FARKLI YÖNLERDEN gel (D3-b).

NE SORUYOR
  D3-a kol sabitken saf algı gürültüsünü veriyor. Bu araç bir adım öteye
  geçiyor: kol aynı hedefe farklı yaklaşma yönlerinden geldiğinde tahtanın
  ölçülen pozu ne kadar oynuyor? Algı tabanının ÜSTÜNE çıkan fark mekaniktir --
  servo tekrarlanabilirliği, dişli boşluğu, FK/zero_offset hatası.

  Bu ayrım olmadan "taban" tek bir yığın sayıdır ve hangi tarafta çalışılacağı
  bilinemez. Bugün sim verisiyle algı tarafının baskın olduğu görüldü, ama sim'de
  mekanik hata SIFIRDIR -- gerçek kolda payı ölçülmedi.

NEDEN AYRI BİR SÜRÜCÜ YAZMIYOR
  `hand_eye_session.py` (1909 satır) hareketi zaten emniyetli sürüyor: SIGINT
  sonrası yalnız sıfır dönüşü, tahta sürüklenme kapıları, komut echo'suyla
  teslim kanıtı, poz başına örnekleme. Donanıma erişimi olmayan bir oturumda o
  aracı değiştirmek, denenmemiş emniyet kodu demek olurdu. Bu script yalnız
  ONUN GİRDİSİNİ üretir ve ÇIKTISINI çözümler.

KULLANIM (kol başında, operatör kesici elde)

  1) Plan üret — hedef poz ve kaç yaklaşma yönü:
       python3 scripts/repeatability_plan.py plan \
           --target "0.10,0.05,-0.20,0.00,0.15" --approaches 4

     Her yön için `runs/hand_eye/repeat_<i>.json` yazar: [yaklasma_pozu, hedef].
     Yaklaşma pozu hedeften SADECE --approach-rad kadar sapar, yani kol hedefe
     her seferinde başka bir yönden girer ama gezinti küçük kalır.

  2) Her planı ayrı ayrı koştur (aralarda kolu gözle kontrol et):
       python3 scripts/hand_eye_session.py --poses-json runs/hand_eye/repeat_0.json \
           --out runs/hand_eye/repeat_0_session.json --samples-per-pose 5
       ... repeat_1, repeat_2, repeat_3

  3) Çözümle — algı tabanına karşı:
       python3 scripts/repeatability_plan.py analyze \
           --sessions runs/hand_eye/repeat_*_session.json \
           --perception-floor runs/hand_eye/perception_floor.json

ÇÜRÜTÜLEBİLİR TAHMİN
  Mekanik payı ihmal edilebilirse, yönler arası saçılım algı tabanına EŞİT
  çıkar. Belirgin şekilde büyükse fark mekaniktir ve büyüklüğü doğrudan
  okunur. İkisi de olabilir; ölçüm karar verir.
"""

import argparse
import glob
import json
import os

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Yaklaşma pozu hedeften bu kadar sapar. Küçük tutuluyor: amaç kolu gezdirmek
# değil, hedefe GİRİŞ YÖNÜNÜ değiştirmek. Büyük sapma, ölçülen farkı yaklaşma
# yönü yerine "uzun yol" etkisiyle kirletir.
DEFAULT_APPROACH_RAD = 0.12


def rotation_angle_deg(R):
    value = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(value, -1.0, 1.0))))


def quat_to_rotation(x, y, z, w):
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def plan(target, approaches, approach_rad, out_dir):
    """Her yaklaşma yönü için [yaklasma, hedef] poz listesi yazar."""
    os.makedirs(out_dir, exist_ok=True)
    written = []
    # Yönler eklem uzayında dolaşır: her plan farklı bir eklemi (ya da eklem
    # çiftini) ters işaretle sapıtır, böylece hedefe giriş yönü gerçekten
    # değişir -- hepsi aynı yönden gelirse ölçüm boşa çıkar.
    for index in range(approaches):
        joint = index % len(target)
        sign = 1.0 if (index // len(target)) % 2 == 0 else -1.0
        approach = list(target)
        approach[joint] = round(approach[joint] + sign * approach_rad, 4)
        path = os.path.join(out_dir, f'repeat_{index}.json')
        with open(path, 'w') as handle:
            json.dump([approach, list(target)], handle, indent=1)
        written.append((path, joint, sign))
        print(f'  {os.path.relpath(path, REPO)}  '
              f'(joint_{joint + 1} {sign:+.0f} x {approach_rad} rad)')
    print(f'\n{len(written)} plan yazildi. Her birini AYRI kostur, aralarda kolu')
    print('gozle kontrol et. Ayni hedefe farkli yonlerden gelinecek.')
    return written


def board_poses(session_path):
    """Bir oturum dosyasindan hedef pozdaki tahta olcumlerini cikarir."""
    payload = json.load(open(session_path))
    samples = payload.get('samples') or payload.get('observations') or []
    poses = []
    for sample in samples:
        board = sample.get('cam_to_board') or sample.get('board')
        if not board:
            continue
        poses.append((np.asarray(board['xyz'], float),
                      quat_to_rotation(*board['quat_xyzw'])))
    return poses


def analyze(session_paths, perception_floor_path):
    per_session = []
    for path in session_paths:
        poses = board_poses(path)
        if not poses:
            print(f'  ATLANDI (olcum yok): {os.path.relpath(path, REPO)}')
            continue
        translations = np.array([t for t, _ in poses])
        per_session.append({
            'path': path,
            'n': len(poses),
            'translation': translations.mean(axis=0),
            'rotation': poses[0][1],
            'rotations': [R for _, R in poses],
        })
        print(f'  {os.path.relpath(path, REPO)}: {len(poses)} olcum')

    if len(per_session) < 2:
        raise SystemExit('en az iki yaklasma yonu gerekli')

    # Yönler ARASI saçılım: her yönün ortalaması, ortak merkeze göre.
    centres = np.array([s['translation'] for s in per_session])
    centre = np.median(centres, axis=0)
    between_mm = np.linalg.norm(centres - centre, axis=1) * 1000.0

    reference = per_session[0]['rotation']
    between_deg = np.array([rotation_angle_deg(reference.T @ s['rotation'])
                            for s in per_session])

    print('\n=== YONLER ARASI SACILIM (algi + mekanik) ===')
    print(f'  oteleme : medyan {np.median(between_mm):.3f} mm | maks {between_mm.max():.3f}')
    print(f'  ACISAL  : medyan {np.median(between_deg):.3f} deg | maks {between_deg.max():.3f}')

    if perception_floor_path and os.path.exists(perception_floor_path):
        floor = json.load(open(perception_floor_path))
        floor_deg = floor['angular_deg']['p90']
        floor_mm = floor['translation_mm']['p90']
        print(f'\n=== ALGI TABANI (D3-a, kol sabit) ===')
        print(f'  oteleme p90 {floor_mm:.3f} mm | ACISAL p90 {floor_deg:.3f} deg')
        excess_deg = float(between_deg.max()) - floor_deg
        print('\n=== MEKANIK PAY (fark) ===')
        print(f'  acisal fazlalik: {excess_deg:+.3f} deg')
        if excess_deg <= floor_deg * 0.5:
            print('  OKUNUSU: yonler arasi sacilim algi tabaniyla ayni buyuklukte.')
            print('  Mekanik pay OLCULEBILIR DEGIL -- is algi tarafinda.')
        else:
            print('  OKUNUSU: yonler arasi sacilim tabani belirgin asiyor.')
            print('  Aradaki fark MEKANIKTIR (servo tekrarlanabilirligi / FK).')
    else:
        print('\n  ⚠ algi tabani dosyasi yok — once D3-a kosulmali,')
        print('    yoksa bu sayinin ne kadari mekanik bilinemez.')


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('plan')
    p.add_argument('--target', required=True,
                   help='5 eklem, virgullu: "q1,q2,q3,q4,q5" (rad)')
    p.add_argument('--approaches', type=int, default=4)
    p.add_argument('--approach-rad', type=float, default=DEFAULT_APPROACH_RAD)
    p.add_argument('--out-dir', default=os.path.join(REPO, 'runs', 'hand_eye'))

    a = sub.add_parser('analyze')
    a.add_argument('--sessions', nargs='+', required=True)
    a.add_argument('--perception-floor',
                   default=os.path.join(REPO, 'runs', 'hand_eye',
                                        'perception_floor.json'))

    args = parser.parse_args()
    if args.cmd == 'plan':
        target = [float(v) for v in args.target.split(',')]
        if len(target) != 5:
            raise SystemExit('hedef 5 eklem olmali')
        plan(target, args.approaches, args.approach_rad, args.out_dir)
    else:
        paths = []
        for pattern in args.sessions:
            paths.extend(sorted(glob.glob(pattern)))
        analyze(paths, args.perception_floor)


if __name__ == '__main__':
    main()
