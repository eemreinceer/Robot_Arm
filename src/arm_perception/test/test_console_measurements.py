"""Konsolun ölçüm güveni sondaları — hepsi ROS'suz ve donanımsız koşar."""

from pathlib import Path

from arm_perception import console_measurements as cm


_PI5 = """
image_width: 640
image_height: 480
flip_method: 0
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 497.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.17, -0.36, -0.004, 0.002, 0.18]
calibration_rms_px: 0.328
validation_mean_rms_px: 0.127
principal_point_policy: fixed_image_center
pose_spread_median_deg: 0.122
perception_floor_p90_deg: 0.056
uncertainty_ratio: 2.2
uncertainty_gate_max: 1.0
"""

_JETSON = """
image_width: 640
image_height: 480
flip_method: 2
camera_matrix:
  rows: 3
  cols: 3
  data: [498.0, 0.0, 328.5, 0.0, 496.0, 239.7, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.10, -0.20, 0.0, 0.0, 0.05]
calibration_rms_px: 0.232
"""


def _config_dir(tmp_path: Path) -> Path:
    (tmp_path / 'pi5.yaml').write_text(_PI5, encoding='utf-8')
    (tmp_path / 'jetson.yaml').write_text(_JETSON, encoding='utf-8')
    return tmp_path


def test_live_intrinsics_are_matched_to_their_source_file(tmp_path: Path):
    config = _config_dir(tmp_path)
    identity = cm.identify_calibration(
        [500.0, 0.0, 320.0, 0.0, 497.0, 240.0, 0.0, 0.0, 1.0],
        [0.17, -0.36, -0.004, 0.002, 0.18],
        config)
    assert identity['matched'] is True
    assert identity['file'] == 'pi5.yaml'
    assert identity['sha256']
    assert identity['principalPointPolicy'] == 'fixed_image_center'


def test_the_two_platform_calibrations_are_told_apart(tmp_path: Path):
    """Jetson YAML'i Pi 5'te yuklenirse bunun GORULMESI gerekir."""
    config = _config_dir(tmp_path)
    identity = cm.identify_calibration(
        [498.0, 0.0, 328.5, 0.0, 496.0, 239.7, 0.0, 0.0, 1.0],
        [0.10, -0.20, 0.0, 0.0, 0.05],
        config)
    assert identity['file'] == 'jetson.yaml'
    assert identity['flipMethod'] == 2


def test_unknown_intrinsics_are_reported_not_guessed(tmp_path: Path):
    config = _config_dir(tmp_path)
    identity = cm.identify_calibration(
        [123.0, 0.0, 10.0, 0.0, 456.0, 20.0, 0.0, 0.0, 1.0], [], config)
    assert identity['matched'] is False
    assert 'uyusmuyor' in identity['reason']


def test_missing_uncertainty_is_unmeasured_not_pass(tmp_path: Path):
    """Belirsizlik alani yoksa 'olculmedi' denir; yoklugu iyi haber degildir."""
    config = _config_dir(tmp_path)
    identity = cm.identify_calibration(
        [498.0, 0.0, 328.5, 0.0, 496.0, 239.7, 0.0, 0.0, 1.0],
        [0.10, -0.20, 0.0, 0.0, 0.05],
        config)
    assert identity['uncertaintyRatio'] is None
    assert identity['uncertaintyVerdict'] == 'unmeasured'


def test_uncertainty_above_the_gate_fails(tmp_path: Path):
    config = _config_dir(tmp_path)
    identity = cm.identify_calibration(
        [500.0, 0.0, 320.0, 0.0, 497.0, 240.0, 0.0, 0.0, 1.0],
        [0.17, -0.36, -0.004, 0.002, 0.18],
        config)
    assert identity['uncertaintyRatio'] == 2.2
    assert identity['uncertaintyVerdict'] == 'fail'


def test_flip_contract_states():
    assert cm.flip_contract(0, 0)['status'] == 'match'
    assert cm.flip_contract(0, 2)['status'] == 'mismatch'
    # Bilinmeyen ASLA 'uyumlu' sayilmaz.
    assert cm.flip_contract(0, None)['status'] == 'unknown'
    assert cm.flip_contract(None, 0)['status'] == 'unknown'


def test_thermal_history_reports_climb_rate():
    history = cm.ThermalHistory(window_s=300.0)
    for index, value in enumerate([50.0, 53.0, 56.0, 59.0]):
        history.add(value, now=index * 60.0)
    snapshot = history.snapshot(now=180.0)
    assert snapshot['samples'] == [50.0, 53.0, 56.0, 59.0]
    assert snapshot['maxC'] == 59.0
    assert abs(snapshot['slopeCPerMin'] - 3.0) < 1e-6


def test_thermal_history_drops_samples_outside_the_window():
    history = cm.ThermalHistory(window_s=100.0)
    history.add(40.0, now=0.0)
    history.add(70.0, now=500.0)
    snapshot = history.snapshot(now=500.0)
    assert snapshot['samples'] == [70.0]
    assert snapshot['slopeCPerMin'] is None


def test_thermal_history_ignores_missing_readings():
    history = cm.ThermalHistory()
    history.add(None)
    assert history.snapshot()['samples'] == []


def test_measurement_registry_ages_and_orders_by_staleness(tmp_path: Path):
    from datetime import datetime, timezone
    path = tmp_path / 'registry.yaml'
    path.write_text(
        'measurements:\n'
        '  - id: fresh\n'
        '    label: Taze\n'
        '    value: x\n'
        "    measuredAt: '2026-08-21'\n"
        '  - id: old\n'
        '    label: Eski\n'
        '    value: y\n'
        "    measuredAt: '2026-07-22'\n",
        encoding='utf-8')
    now = datetime(2026, 8, 21, tzinfo=timezone.utc)
    registry = cm.load_measurement_registry(path, now=now)
    assert registry['available'] is True
    # En eski once: bayatlik gorunur olmali.
    assert [entry['id'] for entry in registry['entries']] == ['old', 'fresh']
    assert registry['entries'][0]['ageDays'] == 30
    assert registry['entries'][1]['ageDays'] == 0


def test_measurement_registry_missing_file_is_not_fatal(tmp_path: Path):
    registry = cm.load_measurement_registry(tmp_path / 'yok.yaml')
    assert registry['available'] is False
    assert registry['entries'] == []


def test_serial_probe_reports_nothing_when_no_device_matches(tmp_path: Path):
    assert cm.serial_port_owners([str(tmp_path / 'ttyYOK*')]) == []


def test_serial_probe_finds_the_process_holding_the_device(tmp_path: Path):
    """Portu ACMADAN sahibini bulur -- acmak ESP32'yi resetlerdi."""
    device = tmp_path / 'ttyFAKE0'
    device.write_text('', encoding='utf-8')
    with device.open('r'):
        ports = cm.serial_port_owners([str(device)])
        assert len(ports) == 1
        assert ports[0]['held'] is True
        assert ports[0]['owners'][0]['pid'] > 0


def test_yaml_timestamps_are_serialisable(tmp_path: Path):
    """Tirnaksiz tarih JSON'a girerse websocket gonderimi coker (canli hata).

    PyYAML tirnaksiz ISO damgasini `datetime` nesnesine cevirir; snapshot
    dogrudan JSON'a serilestirildigi icin bu, konsolu duduruyordu.
    """
    import json
    (tmp_path / 'unquoted.yaml').write_text(
        'camera_matrix:\n'
        '  rows: 3\n'
        '  cols: 3\n'
        '  data: [500.0, 0.0, 320.0, 0.0, 497.0, 240.0, 0.0, 0.0, 1.0]\n'
        'distortion_coefficients:\n'
        '  rows: 1\n'
        '  cols: 5\n'
        '  data: [0.0, 0.0, 0.0, 0.0, 0.0]\n'
        'calibration_timestamp_utc: 2026-08-19T13:29:20.961642+00:00\n'
        'uncertainty_measured_utc: 2026-08-19\n',
        encoding='utf-8')
    identity = cm.identify_calibration(
        [500.0, 0.0, 320.0, 0.0, 497.0, 240.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        tmp_path)
    assert identity['matched'] is True
    assert isinstance(identity['calibratedAt'], str)
    json.dumps(identity)  # cokerse test kirilir


def test_registry_timestamps_are_serialisable(tmp_path: Path):
    import json
    path = tmp_path / 'registry.yaml'
    path.write_text(
        'measurements:\n'
        '  - id: unquoted\n'
        '    label: Tirnaksiz\n'
        '    value: x\n'
        '    measuredAt: 2026-08-21\n',
        encoding='utf-8')
    registry = cm.load_measurement_registry(path)
    assert isinstance(registry['entries'][0]['measuredAt'], str)
    assert registry['entries'][0]['ageDays'] is not None
    json.dumps(registry)
