"""Offline parameter experiments. This script does not change live ICP settings."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import time
from contextlib import ExitStack
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import map.icp as normal_icp
import map.kiss_icp_2d as kiss
from map.localization import ScanLocalizer
from map.scan_pipeline import register_scan
from server import SERVER_COORDS, TURTLEBOT_COORDS
from testing.compare_recorded_trials import load_scans


# One setting changes per experiment; the default values are kept in the codebase.
NORMAL = [
    ('default', {}, 'default'),
    ('kernel_005', {'kernel_distance': .05}, 'default'),
    ('kernel_020', {'kernel_distance': .20}, 'default'),
    ('uniform', {}, 'uniform'),
    ('huber_010', {}, 'huber'),
    ('cutoff_015', {'distance_threshold': .15}, 'default'),
    ('cutoff_006', {'distance_threshold': .06}, 'default'),
    ('min_pairs_30', {'point_pairs_threshold': 30}, 'default'),
]
KISS = [
    ('default', {}, 'default'),
    ('uniform', {}, 'uniform'),
    ('huber_010', {}, 'huber'),
    ('cauchy_010', {}, 'cauchy'),
    ('rmse_005', {'max_inlier_rmse': .05}, 'default'),
    ('rmse_010', {'max_inlier_rmse': .10}, 'default'),
    ('rmse_030', {'max_inlier_rmse': .30}, 'default'),
    ('ratio_060', {'min_inlier_ratio': .60}, 'default'),
    ('sigma_010', {'initial_sigma': .10}, 'default'),
    ('voxel_005', {'voxel_size': .05}, 'default'),
]
TRIALS = (4, 5, 8, 9, 10)
MEASURED_PATH = {4: 0.0, 5: 0.0, 8: 5.8, 9: 2.54, 10: 2.64}


def alternate_weights(distances, mode):
    """Huber and Cauchy use a fixed 0.10 m residual scale."""
    if mode == 'uniform':
        return np.ones_like(distances)
    if mode == 'huber':
        return np.minimum(1.0, .1 / np.maximum(distances, 1e-12))
    if mode == 'cauchy':
        return 1.0 / (1.0 + (distances / .1) ** 2)
    raise ValueError(mode)


def replay(scans, method, params, weight_mode, gate):
    localizer = ScanLocalizer() if method == 'ICP' else kiss.OdometryICP2D(kiss.ICPConfig(**params))
    original_fit = kiss.weighted_rigid_fit
    original_icp = normal_icp.icp
    status_counts = {}
    odom_previous = pose_previous = None
    odom_start = pose_start = None
    odom_path = corrected_path = 0.0
    times = []
    rmses = []
    with ExitStack() as stack:
        if gate == 'off':
            # The two implementations import the gate into their own modules.
            stack.enter_context(patch('map.scan_pipeline.below_motion_threshold', return_value=False))
            stack.enter_context(patch('map.kiss_icp_2d.below_motion_threshold', return_value=False))
        if method == 'ICP':
            def run_icp(*args, **kwargs):
                kwargs.update(params)
                return original_icp(*args, **kwargs)
            stack.enter_context(patch('map.localization.icp', side_effect=run_icp))
            if weight_mode != 'default':
                def weight(distance, _kernel):
                    return float(alternate_weights(np.asarray(distance), weight_mode))
                stack.enter_context(patch('map.icp.robust_weight', side_effect=weight))
        elif weight_mode != 'default':
            def fit(source, target, weights):
                distances = np.linalg.norm(source - target, axis=1)
                return original_fit(source, target, alternate_weights(distances, weight_mode))
            stack.enter_context(patch('map.kiss_icp_2d.weighted_rigid_fit', side_effect=fit))

        for scan, _ in scans:
            start = time.perf_counter()
            if method == 'ICP':
                result, _, _, _ = register_scan(localizer, scan, TURTLEBOT_COORDS, SERVER_COORDS)
            else:
                result = localizer.register_scan(scan)
            elapsed = 1000 * (time.perf_counter() - start)
            status_counts[result.status] = status_counts.get(result.status, 0) + 1
            if result.status not in ('first_scan', 'skipped_motion', 'empty_scan'):
                times.append(elapsed)
                if math.isfinite(result.inlier_rmse):
                    rmses.append(float(result.inlier_rmse))
            odom = np.array([scan.transform.position.x, scan.transform.position.y], dtype=float)
            pose = result.pose[:2, 2]
            if odom_start is None:
                odom_start, pose_start = odom.copy(), pose.copy()
            if odom_previous is not None:
                odom_path += float(np.linalg.norm(odom - odom_previous))
                corrected_path += float(np.linalg.norm(pose - pose_previous))
            odom_previous, pose_previous = odom.copy(), pose.copy()
    return {
        'scans': len(scans), 'updates': status_counts.get('updated', 0),
        'skips': status_counts.get('skipped_motion', 0),
        'failures': sum(n for s, n in status_counts.items() if s not in ('first_scan', 'updated', 'skipped_motion')),
        'odom_path_m': odom_path, 'corrected_path_m': corrected_path,
        'corrected_end_m': float(np.linalg.norm(pose_previous - pose_start)),
        'odom_end_m': float(np.linalg.norm(odom_previous - odom_start)),
        'median_match_ms': statistics.median(times) if times else None,
        'median_match_rmse_m': statistics.median(rmses) if rmses else None,
        'statuses': status_counts,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(Path(__file__).resolve().parent / 'results.csv'))
    args = parser.parse_args()
    output = Path(args.output)
    # Avoid replacing the user's earlier results on a rerun.
    with output.open('x', newline='', encoding='utf-8') as file:
        fields = ['trial', 'gate', 'method', 'variant', 'settings', 'scans', 'updates', 'skips',
                  'failures', 'odom_path_m', 'corrected_path_m', 'corrected_end_m', 'odom_end_m',
                  'measured_path_m', 'path_abs_error_m', 'median_match_ms', 'median_match_rmse_m', 'statuses']
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for trial in TRIALS:
            scans = load_scans(ROOT / 'testing' / 'icp_runs' / f'trial_{trial}.jsonl')
            for gate in (('on', 'off') if trial == 4 else ('on',)):
                for method, variants in [('ICP', NORMAL), ('KISS', KISS)]:
                    for name, params, weight_mode in variants:
                        print(f'Trial {trial}, gate {gate}, {method} {name}', flush=True)
                        result = replay(scans, method, params, weight_mode, gate)
                        row = {'trial': trial, 'gate': gate, 'method': method, 'variant': name,
                               'settings': json.dumps({'parameters': params, 'weight': weight_mode}),
                               'measured_path_m': MEASURED_PATH[trial],
                               'path_abs_error_m': abs(result['corrected_path_m'] - MEASURED_PATH[trial]),
                               **result}
                        row['statuses'] = json.dumps(row['statuses'])
                        writer.writerow(row)
                        file.flush()
    print(f'Wrote {output}')


if __name__ == '__main__':
    main()
