#!/usr/bin/env python3
"""Konsolun "ne kadar guveniyoruz" sondalari -- hepsi SALT OKUNUR.

NEDEN AYRI DOSYA
  `web_console.py` zaten 1400 satir ve ROS'a bagli. Buradaki fonksiyonlarin
  hicbiri ROS'a, donanima ya da aga dokunmaz; dosya sistemi ve `/proc` okur.
  Bu sayede tamami ROS'suz test edilebilir ve konsol disinda da kullanilabilir.

NE ISE YARAR
  Bu projenin tekrar eden arizasi "kabul metrigi onemli olan hatayi goremiyor"
  (tahta sacilimi, fronto-paralel reprojeksiyon, kalibrasyon RMS'i, thin-prism
  modeli -- dort vaka). Dashboard "Kamera: saglikli" derken poz kanalinin algi
  tabaninin kac kati belirsizlik tasidigini SOYLEMEZSE ayni hatayi bes kez
  yapmis oluruz. Buradaki sondalar o sayilari yuzeye cikarir.

EMNIYET
  Hicbir fonksiyon yazmaz, hicbir cihaz acmaz. Ozellikle seri port: `/proc`
  uzerinden KIM TUTUYOR sorusunu cevaplar ama portu ACMAZ -- CP2102 uzerinden
  portu acmak ESP32'yi resetler ve motorluyken PWM'i keser (2026-08-19'da
  olculdu). Sonda arizanin kendisi olamaz.
"""
from __future__ import annotations

import glob
import json
import math
import os
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional

import yaml

# Yayindaki intrinsics ile YAML'i eslestirirken kullanilan tolerans. Kamera
# node'u degerleri float32 CameraInfo alanlarina yazip geri okudugu icin birebir
# esitlik beklenemez; 1e-3 px/katsayi farki ayni kalibrasyon demektir.
_MATCH_TOLERANCE = 1e-3


def _close(a: float, b: float, tol: float = _MATCH_TOLERANCE) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def _load_yaml(path: Path) -> Optional[dict]:
    try:
        with path.open(encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return None
    return document if isinstance(document, dict) else None


def _matrix_data(document: dict, key: str) -> list[float]:
    block = document.get(key)
    if not isinstance(block, dict):
        return []
    data = block.get('data')
    return [float(v) for v in data] if isinstance(data, list) else []


def identify_calibration(
    k: Iterable[float],
    d: Iterable[float],
    config_dir: Path,
) -> dict:
    """Yayindaki intrinsics repodaki HANGI kalibrasyon dosyasindan geliyor.

    Dosya adina GUVENMEZ -- yayindaki K/D degerlerini config dizinindeki her
    YAML ile karsilastirir. Boylece "launch dosyasi su YAML'i veriyor" degil,
    "su anda havada olan sayilar su dosyanin sayilari" denebilir. Eslesme yoksa
    bunu acikca soyler; sessizce eski/yanlis bir kalibrasyonla calismak bu
    projede zaten bir kez oldu (Jetson flip=2 YAML'i Pi 5'te).
    """
    k = [float(v) for v in k]
    d = [float(v) for v in d]
    result: dict = {
        'matched': False,
        'file': None,
        'sha256': None,
        'reason': 'CameraInfo bekleniyor',
        'candidatesChecked': 0,
    }
    if len(k) < 9:
        return result

    candidates = sorted(config_dir.glob('*.yaml')) if config_dir.is_dir() else []
    result['candidatesChecked'] = len(candidates)
    for path in candidates:
        document = _load_yaml(path)
        if document is None:
            continue
        file_k = _matrix_data(document, 'camera_matrix')
        file_d = _matrix_data(document, 'distortion_coefficients')
        if len(file_k) < 9:
            continue
        if not (_close(file_k[0], k[0]) and _close(file_k[4], k[4])
                and _close(file_k[2], k[2]) and _close(file_k[5], k[5])):
            continue
        if len(file_d) == len(d) and not all(
                _close(a, b) for a, b in zip(file_d, d)):
            continue
        result.update({
            'matched': True,
            'file': path.name,
            'sha256': _sha256(path),
            'reason': 'yayindaki intrinsics bu dosyayla birebir',
        })
        result.update(calibration_quality(document))
        return result

    result['reason'] = (
        f'yayindaki intrinsics config dizinindeki {len(candidates)} '
        'kalibrasyonun hicbiriyle uyusmuyor')
    return result


def _sha256(path: Path) -> Optional[str]:
    import hashlib
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None
    return digest[:12]


def calibration_quality(document: dict) -> dict:
    """YAML'daki kalite ve belirsizlik alanlarini cikar.

    `uncertainty_ratio` ALANI OLMAYAN kalibrasyonlar icin `None` doner ve
    konsol bunu "olculmedi" olarak gosterir -- yoklugu iyi haber sanmak bu
    projenin klasik hatasidir.
    """
    ratio = document.get('uncertainty_ratio')
    gate = document.get('uncertainty_gate_max')
    quality = {
        'fitRmsPx': _as_float(document.get('calibration_rms_px')),
        'validationMeanPx': _as_float(document.get('validation_mean_rms_px')),
        'validationMaxPx': _as_float(document.get('validation_max_rms_px')),
        'fitFrames': _as_int(document.get('calibration_frames')),
        'validationFrames': _as_int(document.get('validation_frames')),
        'principalPointPolicy': _as_text(document.get('principal_point_policy')),
        'flipMethod': _as_int(document.get('flip_method')),
        'squareSizeMm': _as_float(document.get('square_size_mm')),
        'calibratedAt': _as_text(document.get('calibration_timestamp_utc')),
        'poseSpreadMedianDeg': _as_float(document.get('pose_spread_median_deg')),
        'perceptionFloorDeg': _as_float(document.get('perception_floor_p90_deg')),
        'uncertaintyRatio': _as_float(ratio),
        'uncertaintyGateMax': _as_float(gate),
        'uncertaintyMeasuredAt': _as_text(document.get('uncertainty_measured_utc')),
    }
    quality['uncertaintyVerdict'] = _uncertainty_verdict(
        quality['uncertaintyRatio'], quality['uncertaintyGateMax'])
    return quality


def _uncertainty_verdict(ratio: Optional[float],
                         gate: Optional[float]) -> str:
    if ratio is None:
        return 'unmeasured'
    if gate is None:
        return 'unmeasured'
    return 'pass' if ratio <= gate else 'fail'


def _as_text(value) -> Optional[str]:
    """YAML tarih/saat alanlarini METNE cevir.

    PyYAML tirnaksiz `2026-08-19T13:29:20+00:00` degerini `datetime` nesnesine
    cevirir ve o nesne JSON'a serilestirilemez -- konsolun websocket gonderimi
    bu yuzden CANLI KOSUDA coktu (2026-08-21). Repodaki kalibrasyon dosyalarinin
    hepsinin tirnakli olacagina guvenilemez; sinirda cevrilir.
    """
    if value is None:
        return None
    if isinstance(value, str):
        return value
    return str(value)


def _as_float(value) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def flip_contract(expected: Optional[int], actual: Optional[int]) -> dict:
    """Kalibrasyonun cekildigi flip ile node'un uyguladigi flip ayni mi.

    Farkli olmalari intrinsics'i SESSIZCE gecersiz kilar: goruntu donuktur,
    K matrisi degildir, ve reprojeksiyon bunu gostermez cunku kalibrasyon
    karelerine degil canli goruntuye uygulanir. Jetson rig'i flip=2, Pi 5
    rig'i flip=0 kullanir; ayni repo ikisini de tasir.
    """
    if expected is None or actual is None:
        return {'status': 'unknown', 'expected': expected, 'actual': actual}
    match = int(expected) == int(actual)
    return {
        'status': 'match' if match else 'mismatch',
        'expected': int(expected),
        'actual': int(actual),
    }


def serial_port_owners(patterns: Iterable[str]) -> list[dict]:
    """Seri portu SU AN hangi surecin tuttugunu `/proc` uzerinden bul.

    Portu ACMAZ. Acmak CP2102 uzerinden ESP32'yi resetler ve motorluyken PWM'i
    keser; teshis araci arizanin sebebi olamaz.

    SINIR: yalnizca bu kullanicinin sureclerinin fd'leri okunabilir. Baska bir
    kullanicinin (ya da root'un) tuttugu port `owners` listesinde GORUNMEZ;
    bu yuzden bos liste "port serbest" DEMEK DEGILDIR ve arayuz bunu boyle
    sunmamalidir.
    """
    devices: list[str] = []
    for pattern in patterns:
        devices.extend(sorted(glob.glob(pattern)))
    if not devices:
        return []

    targets = {}
    for device in devices:
        try:
            targets[os.path.realpath(device)] = device
        except OSError:
            continue

    owners: dict[str, list[dict]] = {device: [] for device in targets.values()}
    for entry in glob.glob('/proc/[0-9]*'):
        pid = os.path.basename(entry)
        fd_dir = os.path.join(entry, 'fd')
        try:
            handles = os.listdir(fd_dir)
        except OSError:
            continue  # baska kullanicinin sureci ya da kapandi
        for handle in handles:
            try:
                target = os.readlink(os.path.join(fd_dir, handle))
            except OSError:
                continue
            if target not in targets:
                continue
            owners[targets[target]].append({
                'pid': int(pid),
                'process': _process_name(entry),
            })
            break

    return [
        {
            'device': device,
            'owners': found,
            'held': bool(found),
        }
        for device, found in sorted(owners.items())
    ]


def _process_name(proc_entry: str) -> str:
    try:
        cmdline = Path(proc_entry, 'cmdline').read_bytes()
    except OSError:
        cmdline = b''
    parts = [p for p in cmdline.split(b'\x00') if p]
    if parts:
        return ' '.join(p.decode('utf-8', 'replace') for p in parts[:2])[:120]
    try:
        return Path(proc_entry, 'comm').read_text(encoding='utf-8').strip()
    except OSError:
        return 'bilinmiyor'


class ThermalHistory:
    """Sicakligin SON DEGERI degil EGILIMI.

    2026-08-19'da iki oturum asiri isinmadan yarim kaldi. Anlik sicaklik
    tirmanisi gostermez; operatorun iptal gelmeden once yavaslama karari
    verebilmesi icin kisa bir pencere gerekiyor.
    """

    def __init__(self, window_s: float = 600.0, max_samples: int = 240) -> None:
        self.window_s = float(window_s)
        self._samples: deque[tuple[float, float]] = deque(maxlen=max_samples)

    def add(self, celsius: Optional[float], now: Optional[float] = None) -> None:
        if celsius is None:
            return
        stamp = time.monotonic() if now is None else now
        self._samples.append((stamp, float(celsius)))
        self._trim(stamp)

    def _trim(self, now: float) -> None:
        while self._samples and now - self._samples[0][0] > self.window_s:
            self._samples.popleft()

    def snapshot(self, now: Optional[float] = None) -> dict:
        stamp = time.monotonic() if now is None else now
        self._trim(stamp)
        values = [value for _, value in self._samples]
        if not values:
            return {
                'windowSeconds': self.window_s,
                'samples': [],
                'minC': None,
                'maxC': None,
                'slopeCPerMin': None,
            }
        return {
            'windowSeconds': self.window_s,
            'samples': [round(value, 1) for value in values],
            'minC': min(values),
            'maxC': max(values),
            'slopeCPerMin': self._slope(stamp),
        }

    def _slope(self, now: float) -> Optional[float]:
        """Son penceredeki egim (C/dakika). Iki ornekten azsa None."""
        if len(self._samples) < 2:
            return None
        first_t, first_v = self._samples[0]
        last_t, last_v = self._samples[-1]
        span_min = (last_t - first_t) / 60.0
        if span_min <= 0:
            return None
        return (last_v - first_v) / span_min


def load_measurement_registry(path: Path, now: Optional[datetime] = None) -> dict:
    """Kritik sabitlerin NE ZAMAN olculdugunu listele.

    Bu projede kural "iddia != olcum": kayitli uc "gercek" ve dokuz teori
    curudu. Bir sayinin ne zaman olculdugu, degerinin kendisi kadar onemli --
    ve hicbir yerde yan yana durmuyordu.

    Kayit defteri ELLE tutulur ve bu yuzden bayatlayabilir. Arayuz yasi
    gosterdigi icin bayatlik da gorunur olur; sessizce eskimekten iyidir.
    """
    document = _load_yaml(path) if path.suffix in ('.yaml', '.yml') else None
    if document is None:
        try:
            document = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return {'available': False, 'entries': [], 'path': str(path)}
    entries_in = document.get('measurements') if isinstance(document, dict) else None
    if not isinstance(entries_in, list):
        return {'available': False, 'entries': [], 'path': str(path)}

    reference = now or datetime.now(timezone.utc)
    entries = []
    for item in entries_in:
        if not isinstance(item, dict):
            continue
        measured_at = _as_text(item.get('measuredAt'))
        entries.append({
            'id': str(item.get('id', '')),
            'label': str(item.get('label', '')),
            'value': str(item.get('value', '')),
            'measuredAt': measured_at,
            'ageDays': _age_days(measured_at, reference),
            'source': str(item.get('source', '')),
            'note': str(item.get('note', '')),
        })
    entries.sort(key=lambda e: (e['ageDays'] is None, -(e['ageDays'] or 0)))
    return {'available': True, 'entries': entries, 'path': str(path)}


def _age_days(measured_at, reference: datetime) -> Optional[int]:
    if not measured_at:
        return None
    text = str(measured_at)
    for parser in (_parse_iso, _parse_date):
        parsed = parser(text)
        if parsed is not None:
            return max(0, (reference - parsed).days)
    return None


def _parse_iso(text: str) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    return parsed if parsed.tzinfo else parsed.replace(tzinfo=timezone.utc)


def _parse_date(text: str) -> Optional[datetime]:
    try:
        parsed = datetime.strptime(text[:10], '%Y-%m-%d')
    except ValueError:
        return None
    return parsed.replace(tzinfo=timezone.utc)
