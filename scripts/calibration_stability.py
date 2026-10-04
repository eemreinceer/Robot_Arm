#!/usr/bin/env python3
"""Compare intrinsic-calibration uncertainty without hiding systematic shifts.

The tool uses one explicit principal-point policy for both the full-data
reference fit and every resampled fit. It reports uncertainty separately from
the pose shift introduced by a fixed principal-point policy. The output is an
atomic schema-v3 checkpoint: only ``status=complete`` is final evidence, while
``running`` or ``interrupted`` can be continued with ``--resume``.
"""

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime, timezone

import cv2

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import solve_camera_intrinsics as solver                         # noqa: E402


def rotation_gap_deg(a, b):
    """Return the shortest angular distance between two rotation matrices."""
    value = (np.trace(a.T @ b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(value, -1.0, 1.0))))


def summary(values):
    """Return the matching summary statistics used by every comparison."""
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        raise ValueError('ozet icin bos deger listesi')
    return {
        'median': float(np.median(values)),
        'p90': float(np.quantile(values, 0.90)),
        'max': float(np.max(values)),
        'mean': float(np.mean(values)),
    }


def draw_indices(frame_count, draw_count, subset, sampling, seed):
    """Generate deterministic bootstrap or without-replacement subsamples."""
    if frame_count < 1 or draw_count < 1:
        raise ValueError('frame_count ve draw_count pozitif olmali')
    if subset < 5:
        raise ValueError('her cekiliste en az 5 kare gerekli')
    replace = sampling == 'bootstrap'
    if not replace and subset > frame_count:
        raise ValueError('subsample boyutu fit kare sayisini asamaz')
    rng = np.random.default_rng(seed)
    return [
        rng.choice(frame_count, size=subset, replace=replace)
        for _ in range(draw_count)
    ]


def solve_rotations(object_points, image_points, camera_matrix, distortion):
    """Solve one checkerboard rotation for each held-out image."""
    rotations = []
    for corners in image_points:
        solved, rvec, _ = cv2.solvePnP(
            object_points, corners, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not solved:
            raise RuntimeError('validation karesinde solvePnP basarisiz')
        rotations.append(cv2.Rodrigues(rvec)[0])
    return rotations


def file_manifest(paths):
    """Return a deterministic name and SHA-256 manifest for image files."""
    entries = []
    for path in paths:
        digest = hashlib.sha256()
        with open(path, 'rb') as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b''):
                digest.update(block)
        entries.append({
            'name': os.path.basename(path),
            'sha256': digest.hexdigest(),
        })
    return entries


def atomic_write_json(path, payload):
    """Durably replace a JSON checkpoint without exposing a partial file."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = f'{path}.tmp'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def run_fingerprint(payload):
    """Hash immutable run inputs so an incompatible resume fails closed."""
    encoded = json.dumps(
        payload, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def validate_resume(checkpoint, expected_fingerprint, policies, draw_count):
    """Validate and return saved per-policy draws from a checkpoint."""
    if checkpoint.get('schema_version') != 3:
        raise ValueError('resume checkpoint schema_version=3 olmali')
    if checkpoint.get('run_fingerprint') != expected_fingerprint:
        raise ValueError('resume checkpoint girdileri bu kosuyla uyusmuyor')
    saved = checkpoint.get('policy_draws', {})
    if not isinstance(saved, dict):
        raise ValueError('resume policy_draws nesne olmali')
    unexpected = sorted(set(saved) - set(policies))
    if unexpected:
        raise ValueError(
            'resume checkpoint beklenmeyen politika iceriyor: '
            + ', '.join(unexpected))
    for policy, draws in saved.items():
        if not isinstance(draws, list) or len(draws) > draw_count:
            raise ValueError(
                f'resume {policy} draw sayisi gecersiz: '
                f'{len(draws) if isinstance(draws, list) else "liste degil"}')
    return saved


def analyze_policy(
        policy, object_points, fit_points, validation_points, image_size,
        draws, free_reference_rotations, perception_floor,
        completed_draws=None, checkpoint_callback=None):
    """Analyze one policy, optionally resuming and checkpointing each draw."""
    fit_objects = [object_points] * len(fit_points)
    rms, camera_matrix, distortion, _, _ = solver.calibrate_intrinsics(
        fit_objects, fit_points, image_size, policy)
    reference_rotations = solve_rotations(
        object_points, validation_points, camera_matrix, distortion)
    validation_errors = solver.independent_validation_errors(
        object_points, validation_points, camera_matrix, distortion)

    draw_results = list(completed_draws or [])
    if len(draw_results) > len(draws):
        raise ValueError('tamamlanan draw sayisi istenen sayiyi asamaz')
    for indices in draws[len(draw_results):]:
        sampled_points = [fit_points[int(index)] for index in indices]
        sampled_objects = [object_points] * len(sampled_points)
        sampled_rms, sampled_k, sampled_dist, _, _ = (
            solver.calibrate_intrinsics(
                sampled_objects, sampled_points, image_size, policy))
        sampled_rotations = solve_rotations(
            object_points, validation_points, sampled_k, sampled_dist)
        gaps = [
            rotation_gap_deg(reference, sampled)
            for reference, sampled in zip(
                reference_rotations, sampled_rotations)
        ]
        draw_results.append({
            'fit_rms_px': float(sampled_rms),
            'intrinsics': [
                float(sampled_k[0, 0]), float(sampled_k[1, 1]),
                float(sampled_k[0, 2]), float(sampled_k[1, 2]),
            ],
            'validation_rotation_gaps_deg': gaps,
        })
        if checkpoint_callback is not None:
            checkpoint_callback(draw_results)

    uncertainty = [
        gap
        for draw in draw_results
        for gap in draw['validation_rotation_gaps_deg']
    ]
    per_view_uncertainty = list(zip(*[
        draw['validation_rotation_gaps_deg'] for draw in draw_results
    ]))
    intrinsics = [draw['intrinsics'] for draw in draw_results]
    draw_rms = [draw['fit_rms_px'] for draw in draw_results]

    policy_shift = [
        rotation_gap_deg(free, candidate)
        for free, candidate in zip(
            free_reference_rotations, reference_rotations)
    ]
    per_view_medians = [float(np.median(values))
                        for values in per_view_uncertainty]
    result = {
        'full_fit': {
            'rms_px': float(rms),
            'camera_matrix': camera_matrix.tolist(),
            'distortion': distortion.ravel().tolist(),
            'validation_reprojection_px': summary(validation_errors),
        },
        'resampled_fit_rms_px': summary(draw_rms),
        'resampled_intrinsics': {
            'columns': ['fx', 'fy', 'cx', 'cy'],
            'values': intrinsics,
        },
        'draw_results': draw_results,
        'uncertainty_deg': summary(uncertainty),
        'per_view_median_uncertainty_deg': summary(per_view_medians),
        'pose_shift_from_free_deg': summary(policy_shift),
    }
    if perception_floor is not None:
        floor_median = float(perception_floor['angular_deg']['median'])
        floor_p90 = float(perception_floor['angular_deg']['p90'])
        median_ratio = result['uncertainty_deg']['median'] / floor_median
        p90_ratio = result['uncertainty_deg']['p90'] / floor_p90
        result['matching_floor_comparison'] = {
            'floor_median_deg': floor_median,
            'floor_p90_deg': floor_p90,
            'median_ratio': float(median_ratio),
            'p90_ratio': float(p90_ratio),
            'passes_both': bool(median_ratio <= 1.0 and p90_ratio <= 1.0),
            'note': 'matching quantiles; floor must be remeasured with the '
                    'candidate K before physical acceptance',
        }
    return result



def sigma_gate(results, sigma_deg):
    """Compare every policy's uncertainty against a declared perception sigma.

    The gate exists because reprojection RMS cannot see the error that matters:
    calibrations that agree to 0.05 px have disagreed by 0.54 deg in pose. A
    calibration is only acceptable when its resampling spread stays beside the
    perception noise it is supposed to serve, so both the median and the p90 of
    the uncertainty are compared against the declared sigma. A scalar sigma has
    no quantiles of its own, which makes this a stricter gate than the
    quantile-matched comparison against a measured floor.
    """
    if sigma_deg <= 0.0:
        raise ValueError('sigma-deg pozitif olmali')
    verdicts = {}
    for policy, result in results.items():
        uncertainty = result['uncertainty_deg']
        median_ratio = uncertainty['median'] / sigma_deg
        p90_ratio = uncertainty['p90'] / sigma_deg
        verdicts[policy] = {
            'sigma_deg': float(sigma_deg),
            'median_ratio': float(median_ratio),
            'p90_ratio': float(p90_ratio),
            'passes': bool(median_ratio <= 1.0 and p90_ratio <= 1.0),
        }
    return {
        'sigma_deg': float(sigma_deg),
        'per_policy': verdicts,
        'passes': bool(any(v['passes'] for v in verdicts.values())),
        'note': 'gate compares resampling spread with declared perception '
                'sigma; passing needs at least one policy inside the floor',
    }


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fit_dir')
    parser.add_argument('--validation-dir', required=True)
    parser.add_argument(
        '--principal-point-policy', action='append', required=True,
        choices=solver.PRINCIPAL_POINT_POLICIES,
        help='repeat for each candidate; free is always computed as reference')
    parser.add_argument('--draws', type=int, default=200)
    parser.add_argument('--sampling', choices=('bootstrap', 'subsample'),
                        default='bootstrap')
    parser.add_argument('--subset', type=int, default=0)
    parser.add_argument('--seed', type=int, default=3)
    parser.add_argument('--perception-floor-json')
    parser.add_argument(
        '--sigma-deg', type=float,
        help='declared perception sigma in degrees; applies the acceptance '
             'gate and exits non-zero when calibration uncertainty exceeds '
             'it (a scalar sigma is stricter than the quantile-matched '
             '--perception-floor-json comparison)')
    parser.add_argument('--out-json', required=True)
    parser.add_argument(
        '--resume', action='store_true',
        help='resume an interrupted schema-v3 checkpoint at --out-json')
    return parser.parse_args()


def main():
    """Run the offline policy comparison and persist restartable evidence."""
    args = parse_args()
    fit_meta = solver.load_capture_metadata(args.fit_dir)
    validation_meta = solver.load_capture_metadata(args.validation_dir)
    solver.validate_dataset_contract(fit_meta, validation_meta)
    cols = fit_meta['pattern_cols']
    rows = fit_meta['pattern_rows']
    square_mm = fit_meta['square_size_mm']
    image_size = (fit_meta['image_width'], fit_meta['image_height'])
    object_points = solver.checkerboard_object_points(cols, rows, square_mm)
    fit_points, fit_paths, fit_total = solver.load_checkerboard_views(
        args.fit_dir, cols, rows, *image_size)
    validation_points, validation_paths, validation_total = (
        solver.load_checkerboard_views(
            args.validation_dir, cols, rows, *image_size))
    if len(fit_points) < 10 or len(validation_points) < 5:
        raise SystemExit(
            f'yetersiz kare: fit {len(fit_points)}, '
            f'dogrulama {len(validation_points)}')

    if args.subset:
        subset = args.subset
    elif args.sampling == 'bootstrap':
        subset = len(fit_points)
    else:
        subset = max(5, int(round(0.8 * len(fit_points))))
    draws = draw_indices(
        len(fit_points), args.draws, subset, args.sampling, args.seed)

    perception_floor = None
    if args.perception_floor_json:
        with open(args.perception_floor_json) as handle:
            perception_floor = json.load(handle)

    policies = list(dict.fromkeys(args.principal_point_policy))
    warnings = []
    accepted = fit_meta.get('accepted')
    if accepted is not None and int(accepted) != fit_total:
        warnings.append(
            f'fit metadata accepted={accepted}, PNG count={fit_total}; '
            'merged dataset source manifest is required')
    required_pipeline_fields = (
        'camera_identifier', 'backend', 'pixel_format', 'libcamera_version')
    missing_pipeline_fields = [
        field for field in required_pipeline_fields if field not in fit_meta]
    if missing_pipeline_fields:
        warnings.append(
            'fit metadata missing pipeline identity: '
            + ', '.join(missing_pipeline_fields))

    immutable_run = {
        'opencv_version': cv2.__version__,
        'fit_dir': os.path.realpath(args.fit_dir),
        'validation_dir': os.path.realpath(args.validation_dir),
        'image_size': list(image_size),
        'board': {'cols': cols, 'rows': rows, 'square_size_mm': square_mm},
        'policies': policies,
        'sampling': {
            'method': args.sampling,
            'replace': args.sampling == 'bootstrap',
            'draws': args.draws,
            'subset': subset,
            'seed': args.seed,
            'indices': [[int(index) for index in draw] for draw in draws],
        },
        'dataset': {
            'fit_png_total': fit_total,
            'fit_detected': len(fit_paths),
            'validation_png_total': validation_total,
            'validation_detected': len(validation_paths),
            'fit_metadata': fit_meta,
            'validation_metadata': validation_meta,
            'fit_files': file_manifest(fit_paths),
            'validation_files': file_manifest(validation_paths),
        },
        'perception_floor': perception_floor,
    }
    fingerprint = run_fingerprint(immutable_run)
    policy_draws = {}
    if args.resume:
        if not os.path.isfile(args.out_json):
            raise SystemExit('--resume icin mevcut --out-json gerekli')
        with open(args.out_json) as handle:
            checkpoint = json.load(handle)
        try:
            policy_draws = validate_resume(
                checkpoint, fingerprint, policies, len(draws))
        except ValueError as error:
            raise SystemExit(str(error)) from error

    payload = {
        'schema_version': 3,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'running',
        'run_fingerprint': fingerprint,
        **immutable_run,
        'warnings': warnings,
        'policy_draws': policy_draws,
        'results': {},
        'interpretation': (
            'pose_shift_from_free is a policy-induced shift, not measured '
            'ground-truth bias; fixed principal point needs an external '
            'check'),
    }
    atomic_write_json(args.out_json, payload)

    fit_objects = [object_points] * len(fit_points)
    _, free_k, free_dist, _, _ = solver.calibrate_intrinsics(
        fit_objects, fit_points, image_size, 'free')
    free_reference_rotations = solve_rotations(
        object_points, validation_points, free_k, free_dist)
    results = {}
    try:
        for policy in policies:
            def checkpoint_callback(draw_results, current_policy=policy):
                policy_draws[current_policy] = list(draw_results)
                atomic_write_json(args.out_json, payload)

            results[policy] = analyze_policy(
                policy, object_points, fit_points, validation_points,
                image_size, draws, free_reference_rotations,
                perception_floor, policy_draws.get(policy),
                checkpoint_callback)
            payload['results'][policy] = results[policy]
            atomic_write_json(args.out_json, payload)
    except KeyboardInterrupt:
        payload['status'] = 'interrupted'
        payload['updated_utc'] = datetime.now(timezone.utc).isoformat()
        atomic_write_json(args.out_json, payload)
        raise SystemExit(
            'analiz kesildi; checkpoint --resume ile devam edebilir')

    payload['status'] = 'complete'
    payload['completed_utc'] = datetime.now(timezone.utc).isoformat()
    atomic_write_json(args.out_json, payload)

    print(f'fit {len(fit_paths)}/{fit_total}, validation '
          f'{len(validation_paths)}/{validation_total}')
    print(
        f'{args.sampling}: draws={args.draws}, subset={subset}, '
        f'seed={args.seed}')
    for policy, result in results.items():
        uncertainty = result['uncertainty_deg']
        shift = result['pose_shift_from_free_deg']
        validation = result['full_fit']['validation_reprojection_px']
        print(
            f'{policy}: uncertainty median/p90 '
            f'{uncertainty["median"]:.4f}/{uncertainty["p90"]:.4f} deg | '
            f'free-shift median {shift["median"]:.4f} deg | '
            f'validation mean/max {validation["mean"]:.4f}/'
            f'{validation["max"]:.4f} px')
    for warning in warnings:
        print(f'UYARI: {warning}')
    print(f'yazildi: {args.out_json}')

    if args.sigma_deg is not None:
        gate = sigma_gate(results, args.sigma_deg)
        payload['sigma_gate'] = gate
        atomic_write_json(args.out_json, payload)
        for policy, verdict in gate['per_policy'].items():
            state = 'GECTI' if verdict['passes'] else 'KALDI'
            print(
                f'kapi {policy}: {state} | uncertainty/sigma medyan '
                f'{verdict["median_ratio"]:.2f}x p90 '
                f'{verdict["p90_ratio"]:.2f}x '
                f'(sigma={args.sigma_deg:.4f} deg)')
        if not gate['passes']:
            raise SystemExit(
                'KAPI KALDI: hicbir politika ilan edilen algi sigmasinin '
                'icinde degil')


if __name__ == '__main__':
    main()
