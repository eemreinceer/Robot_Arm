#!/usr/bin/env python3
"""Solve and independently validate checkerboard camera intrinsics.

The fit and validation directories must come from separate runs of
``capture_calib_images.py``.  Their resolution, checkerboard geometry, square
size, and GStreamer flip-method must match.  Validation keeps K/dist fixed,
solves only the checkerboard pose with solvePnP, and reports true per-point
pixel RMS through projectPoints.

Usage:
    python3 solve_camera_intrinsics.py runs/calib_fit_20260722_1800 \
        --validation-dir runs/calib_validation_20260722_1815 \
        --principal-point-policy free \
        --square-mm 25 --out runs/imx219_640x480_candidate.yaml
"""

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime, timezone

import cv2

import numpy as np


PRINCIPAL_POINT_POLICIES = (
    'free',
    'image_center',
    'pixel_grid_center',
)


def calibration_inputs(image_size, principal_point_policy):
    """Return OpenCV flags and initial intrinsics for an explicit policy.

    ``image_center`` uses OpenCV's common ``width / 2, height / 2`` convention.
    ``pixel_grid_center`` uses the geometric centre of zero-indexed pixels,
    ``(width - 1) / 2, (height - 1) / 2``.  Neither is silently preferred:
    fixing an under-observed parameter reduces variance but can introduce bias.
    """
    if principal_point_policy not in PRINCIPAL_POINT_POLICIES:
        raise ValueError(
            f'principal-point policy gecersiz: {principal_point_policy}')
    if principal_point_policy == 'free':
        return 0, None, None

    width, height = image_size
    if principal_point_policy == 'image_center':
        cx, cy = width / 2.0, height / 2.0
    else:
        cx, cy = (width - 1.0) / 2.0, (height - 1.0) / 2.0
    camera_matrix = np.array([
        [float(width), 0.0, cx],
        [0.0, float(width), cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    distortion = np.zeros((1, 5), dtype=np.float64)
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT
    return flags, camera_matrix, distortion


def calibrate_intrinsics(
        object_points, image_points, image_size, principal_point_policy):
    """Calibrate with a named, reproducible principal-point policy."""
    flags, initial_camera_matrix, initial_distortion = calibration_inputs(
        image_size, principal_point_policy)
    return cv2.calibrateCamera(
        object_points, image_points, image_size,
        initial_camera_matrix, initial_distortion, flags=flags)


def point_rms(observed, projected) -> float:
    """Return true 2D Euclidean RMS in pixels over all observed points."""
    observed_xy = np.asarray(observed, dtype=np.float64).reshape(-1, 2)
    projected_xy = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    if observed_xy.shape != projected_xy.shape or observed_xy.size == 0:
        raise ValueError(
            'observed/projected point arrays must be non-empty and equal')
    squared_distance = np.sum((observed_xy - projected_xy) ** 2, axis=1)
    return float(math.sqrt(np.mean(squared_distance)))


def load_capture_metadata(directory: str) -> dict:
    """Load and validate the capture contract from one image directory."""
    path = os.path.join(directory, 'capture_meta.json')
    if not os.path.isfile(path):
        raise ValueError(f'capture metadata yok: {path}')
    with open(path) as handle:
        meta = json.load(handle)
    required = (
        'pattern_cols', 'pattern_rows', 'square_size_mm', 'flip_method',
        'image_width', 'image_height')
    missing = [key for key in required if key not in meta]
    if missing:
        raise ValueError(
            f'{path} zorunlu alanlari icermiyor: {", ".join(missing)}')

    raw_flip_method = meta['flip_method']
    if (isinstance(raw_flip_method, bool)
            or not isinstance(raw_flip_method, int)):
        raise ValueError(f'{path} flip_method tam sayi olmali')
    try:
        cols = int(meta['pattern_cols'])
        rows = int(meta['pattern_rows'])
        square_mm = float(meta['square_size_mm'])
        flip_method = raw_flip_method
        width = int(meta['image_width'])
        height = int(meta['image_height'])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{path} sayisal alanlari gecersiz: {error}') from error
    if cols < 3 or rows < 3:
        raise ValueError(f'{path} desen boyutu gecersiz: {cols}x{rows}')
    if square_mm <= 0.0:
        raise ValueError(f'{path} square_size_mm sifirdan buyuk olmali')
    if flip_method not in range(4):
        raise ValueError(f'{path} flip_method 0..3 olmali: {flip_method}')
    if width <= 0 or height <= 0:
        raise ValueError(f'{path} goruntu boyutu gecersiz: {width}x{height}')

    normalized = dict(meta)
    normalized.update({
        'pattern_cols': cols,
        'pattern_rows': rows,
        'square_size_mm': square_mm,
        'flip_method': flip_method,
        'image_width': width,
        'image_height': height,
    })
    return normalized


def validate_dataset_contract(fit_meta: dict, validation_meta: dict) -> None:
    """Require both datasets to represent the same camera pipeline."""
    fields = (
        'pattern_cols', 'pattern_rows', 'flip_method',
        'image_width', 'image_height')
    for field in fields:
        if fit_meta[field] != validation_meta[field]:
            raise ValueError(
                f'fit/validation {field} uyusmuyor: '
                f'{fit_meta[field]} != {validation_meta[field]}')
    if not math.isclose(
            fit_meta['square_size_mm'], validation_meta['square_size_mm'],
            rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(
            'fit/validation square_size_mm uyusmuyor: '
            f'{fit_meta["square_size_mm"]} != '
            f'{validation_meta["square_size_mm"]}')


def checkerboard_object_points(cols: int, rows: int, square_mm: float):
    """Build checkerboard object points in SI units."""
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= square_mm / 1000.0
    return points


def detect_checkerboard_corners(gray, cols: int, rows: int):
    """Detect calibration-grade corners with the sector-based detector.

    The legacy detector plus ``cornerSubPix`` showed multi-pixel systematic
    offsets on the IMX219's ISP-resampled 640x480 stream.  OpenCV's SB detector
    estimates sub-pixel corners directly and is substantially more robust to
    the aliasing visible in these captures.
    """
    flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    return cv2.findChessboardCornersSB(gray, (cols, rows), flags)


def load_checkerboard_views(
        directory: str, cols: int, rows: int,
        expected_width: int, expected_height: int):
    """Load matching-size images and retain detected checkerboard corners."""
    images = sorted(glob.glob(os.path.join(directory, '*.png')))
    if not images:
        raise ValueError(f'{directory} icinde png yok')

    image_points, used = [], []
    for path in images:
        image = cv2.imread(path)
        if image is None:
            continue
        height, width = image.shape[:2]
        if (width, height) != (expected_width, expected_height):
            raise ValueError(
                f'KARISIK BOYUT: {path} {width}x{height} != '
                f'{expected_width}x{expected_height}')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = detect_checkerboard_corners(gray, cols, rows)
        if not found:
            continue
        image_points.append(corners)
        used.append(path)
    return image_points, used, len(images)


def independent_validation_errors(
        object_points, image_points, camera_matrix, distortion):
    """Validate fixed K while solving one board pose per held-out view."""
    errors = []
    for corners in image_points:
        solved, rvec, tvec = cv2.solvePnP(
            object_points, corners, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not solved:
            errors.append(float('inf'))
            continue
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, camera_matrix, distortion)
        errors.append(point_rms(corners, projected))
    return errors


def main():
    """Solve one explicit intrinsics policy and write a candidate YAML."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dir', help='fit capture directory')
    parser.add_argument('--validation-dir', required=True,
                        help='separate capture run, never used during fitting')
    parser.add_argument('--cols', type=int, default=None)
    parser.add_argument('--rows', type=int, default=None)
    parser.add_argument('--square-mm', type=float, default=None,
                        help='optional cross-check against capture metadata')
    parser.add_argument('--out', required=True)
    parser.add_argument('--camera-name', default='imx219')
    parser.add_argument('--max-rms-px', type=float, default=1.0)
    parser.add_argument('--max-aspect-dev', type=float, default=0.02)
    parser.add_argument('--min-fit-frames', type=int, default=25)
    parser.add_argument('--min-validation-frames', type=int, default=10)
    parser.add_argument('--max-validation-mean-px', type=float, default=1.0)
    parser.add_argument('--max-validation-view-px', type=float, default=1.5)
    parser.add_argument(
        '--principal-point-policy', required=True,
        choices=PRINCIPAL_POINT_POLICIES,
        help=(
            'principal point serbestligi; bias/variance karari acik olmalidir. '
            '2026-08-21 olcumunde free politika belirsizligi 2.2x -> 4.0x '
            'buyuturken dogrulugu iyilestirmedi.'))
    args = parser.parse_args()

    if os.path.realpath(args.dir) == os.path.realpath(args.validation_dir):
        sys.exit('fit ve validation dizinleri ayni olamaz')
    try:
        fit_meta = load_capture_metadata(args.dir)
        validation_meta = load_capture_metadata(args.validation_dir)
        validate_dataset_contract(fit_meta, validation_meta)
    except ValueError as error:
        sys.exit(str(error))

    cols = fit_meta['pattern_cols']
    rows = fit_meta['pattern_rows']
    square_mm = fit_meta['square_size_mm']
    if args.cols is not None and args.cols != cols:
        sys.exit(f'--cols {args.cols}, metadata {cols}; uyusmuyor')
    if args.rows is not None and args.rows != rows:
        sys.exit(f'--rows {args.rows}, metadata {rows}; uyusmuyor')
    if args.square_mm is not None and not math.isclose(
            args.square_mm, square_mm, rel_tol=0.0, abs_tol=1e-6):
        sys.exit(
            f'--square-mm {args.square_mm}, metadata {square_mm}; uyusmuyor')

    object_points = checkerboard_object_points(cols, rows, square_mm)
    try:
        fit_points, fit_used, fit_total = load_checkerboard_views(
            args.dir, cols, rows,
            fit_meta['image_width'], fit_meta['image_height'])
        validation_points, validation_used, validation_total = (
            load_checkerboard_views(
                args.validation_dir, cols, rows,
                validation_meta['image_width'],
                validation_meta['image_height']))
    except ValueError as error:
        sys.exit(str(error))
    if len(fit_used) < 5:
        sys.exit(f'yalnizca {len(fit_used)} fit karesinde tahta bulundu')
    if not validation_used:
        sys.exit('validation karelerinin hicbirinde tahta bulunamadi')

    fit_object_points = [object_points for _ in fit_points]
    image_size = (fit_meta['image_width'], fit_meta['image_height'])
    # ASAL NOKTA POLITIKASI ACIK SECILIR.
    #
    # 2026-08-21'de 64 fit + 16 dogrulama karesiyle olculdu: serbest
    # birakildiginda asal nokta bootstrap cekilisleri arasinda 3.1 px geziniyor
    # ve bu dogrudan poz donmesine geciyor -- kalibrasyon belirsizligi algi
    # tabaninin 4.0 kati. Merkeze cakilinca 2.2 kata DUSUYOR ve dogrulama
    # reprojeksiyonu DEGISMIYOR (0.126 -> 0.127 px ortalama, maks ayni). Yani
    # veri bu parametreyi zaten belirlemiyor; serbest birakmak cozucunun oraya
    # gurultu uydurmasina izin vermekten ibaret.
    #
    # Kontrol kosuldu: asal nokta kasten 20 px sapmis bir noktaya cakilinca
    # dogruluk BOZULUYOR (0.142 px) ve yayilim dusmuyor (3.6x) -- kazanc
    # "ne sabitlersen iyi gelir" turunden bir yapaylik DEGIL.
    #
    # Farkli bir sensor/kirpma ile merkez varsayimi gecersizse ``free`` secilir;
    # sifir-indeksli piksel geometrisi gerekiyorsa ``pixel_grid_center`` secilir.
    rms, camera_matrix, distortion, rvecs, tvecs = calibrate_intrinsics(
        fit_object_points, fit_points, image_size,
        args.principal_point_policy)

    fit_errors = []
    for observed, rvec, tvec, path in zip(fit_points, rvecs, tvecs, fit_used):
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, camera_matrix, distortion)
        fit_errors.append(
            (point_rms(observed, projected), os.path.basename(path)))
    fit_errors.sort(reverse=True)

    validation_values = independent_validation_errors(
        object_points, validation_points, camera_matrix, distortion)
    validation_errors = sorted(
        zip(validation_values, (os.path.basename(p) for p in validation_used)),
        reverse=True)
    validation_mean = float(np.mean(validation_values))
    validation_max = float(np.max(validation_values))

    width, height = image_size
    print(f'fit kare sayisi  : {len(fit_used)} / {fit_total}')
    print(f'validation       : {len(validation_used)} / {validation_total}')
    print(f'goruntu boyutu   : {width}x{height}')
    print(f'flip_method      : {fit_meta["flip_method"]}')
    print(f'kare boyutu      : {square_mm} mm')
    print(f'principal point  : {args.principal_point_policy}')
    print(f'fit RMS          : {rms:.4f} px')
    print(f'validation mean  : {validation_mean:.4f} px')
    print(f'validation max   : {validation_max:.4f} px')
    print(
        f'fx, fy           : {camera_matrix[0, 0]:.2f}, '
        f'{camera_matrix[1, 1]:.2f}')
    print(
        f'cx, cy           : {camera_matrix[0, 2]:.2f}, '
        f'{camera_matrix[1, 2]:.2f}')
    print(
        'distorsiyon      : '
        f'{np.array2string(distortion.ravel(), precision=4)}')
    print('\nen kotu 3 fit karesi:')
    for error, name in fit_errors[:3]:
        print(f'  {name}: {error:.4f} px')
    print('\nen kotu 3 validation karesi:')
    for error, name in validation_errors[:3]:
        print(f'  {name}: {error:.4f} px')

    problems = []
    if rms > args.max_rms_px:
        problems.append(f'fit RMS {rms:.3f} px > {args.max_rms_px} px')
    if len(fit_used) < args.min_fit_frames:
        problems.append(
            f'yalnizca {len(fit_used)} fit karesi; '
            f'en az {args.min_fit_frames} gerekli')
    covered = fit_meta.get('covered_cells')
    if covered is not None and len(covered) < 9:
        problems.append(
            f'fit goruntusunun {9 - len(covered)} bolgesi kapsanmadi')
    if abs(camera_matrix[0, 2] - width / 2) > width * 0.15 or \
            abs(camera_matrix[1, 2] - height / 2) > height * 0.15:
        problems.append('ana nokta goruntu merkezinden sasirtici uzak')
    aspect = camera_matrix[0, 0] / camera_matrix[1, 1]
    if abs(aspect - 1.0) > args.max_aspect_dev:
        problems.append(
            f'fx/fy={aspect:.4f}; 1.0 degerinden '
            f'{abs(aspect - 1.0) * 100:.1f}% sapti')
    if len(validation_used) < args.min_validation_frames:
        problems.append(
            f'yalnizca {len(validation_used)} validation karesi; '
            f'en az {args.min_validation_frames} gerekli')
    if validation_mean > args.max_validation_mean_px:
        problems.append(
            f'validation mean {validation_mean:.3f} px > '
            f'{args.max_validation_mean_px} px')
    if validation_max > args.max_validation_view_px:
        problems.append(
            f'validation max {validation_max:.3f} px > '
            f'{args.max_validation_view_px} px')

    if problems:
        print('\n⚠ SORUNLAR:')
        for problem in problems:
            print(f'  - {problem}')
        print('Aday YAML yazilacak fakat etkinlestirilmemeli.')
    else:
        print('\n✓ Fit ve bagimsiz validation esikleri gecti.')

    projection = np.hstack([camera_matrix, np.zeros((3, 1))])
    lines = [
        f'# Robot Arm kamera intrinsics — {args.camera_name}',
        f'# Fit: {args.dir} ({len(fit_used)} kare)',
        f'# Validation: {args.validation_dir} ({len(validation_used)} kare)',
        f'# Fit RMS {rms:.4f} px | validation mean/max '
        f'{validation_mean:.4f}/{validation_max:.4f} px',
        f'image_width: {width}',
        f'image_height: {height}',
        f'camera_name: {args.camera_name}',
        f'flip_method: {fit_meta["flip_method"]}',
        f'square_size_mm: {square_mm:.6f}',
        f'principal_point_policy: {args.principal_point_policy}',
        'camera_matrix:',
        '  rows: 3', '  cols: 3',
        f'  data: [{", ".join(f"{v:.8f}" for v in camera_matrix.ravel())}]',
        'distortion_model: plumb_bob',
        'distortion_coefficients:',
        '  rows: 1', f'  cols: {distortion.size}',
        f'  data: [{", ".join(f"{v:.8f}" for v in distortion.ravel())}]',
        'rectification_matrix:',
        '  rows: 3', '  cols: 3',
        f'  data: [{", ".join(f"{v:.8f}" for v in np.eye(3).ravel())}]',
        'projection_matrix:',
        '  rows: 3', '  cols: 4',
        f'  data: [{", ".join(f"{v:.8f}" for v in projection.ravel())}]',
        f'calibration_rms_px: {rms:.6f}',
        f'calibration_frames: {len(fit_used)}',
        f'validation_mean_rms_px: {validation_mean:.6f}',
        f'validation_max_rms_px: {validation_max:.6f}',
        f'validation_frames: {len(validation_used)}',
        f'calibration_timestamp_utc: {datetime.now(timezone.utc).isoformat()}',
    ]
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')
    print(f'\n{args.out} yazildi')
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
