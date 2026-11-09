"""Replay recorded runs with isolated ICP parameter settings; never alter live settings."""

import argparse
import base64
import contextlib
import csv
import json
from pathlib import Path
import shutil
import statistics
import tempfile
import time
from unittest.mock import patch

from map.icp import icp
from map.icp_diagnostics import ICPDiagnostics
from map.kiss_icp_2d import ICPConfig, OdometryICP2D, pose_matrix
from map.localization import ScanLocalizer
from map.scan_pipeline import register_scan
from realityapi_pb2 import RawLidarScan
from server import SERVER_COORDS, TURTLEBOT_COORDS


TRIALS = (3, 4, 5, 7, 8, 9, 10, 11)
VARIANTS = (('normal_default', 'normal', 0.1), ('normal_kernel_005', 'normal', 0.05),
            ('normal_kernel_020', 'normal', 0.2), ('kiss_default', 'kiss', 0.10),
            ('kiss_voxel_005', 'kiss', 0.05), ('kiss_voxel_020', 'kiss', 0.20))


def load_scans(path):
    scans = []
    with path.open(encoding='utf-8') as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            item = json.loads(line)
            if item.get('device_registered', True):
                scan = RawLidarScan.FromString(base64.b64decode(item['scan_b64'], validate=True))
                scans.append((scan, item['received_utc']))
    devices = {scan.id for scan, _ in scans}
    if len(devices) != 1:
        raise ValueError(f'{path} must contain scans for exactly one registered device: {devices}')
    return scans


def run_variant(scans, name, method, parameter, output_dir):
    def tuned_icp(*args, **kwargs):
        kwargs['kernel_distance'] = parameter
        return icp(*args, **kwargs)

    localizer = ScanLocalizer() if method == 'normal' else OdometryICP2D(ICPConfig(voxel_size=parameter))
    with tempfile.TemporaryDirectory(prefix='icp-comparison-') as temporary:
        with open(Path(temporary) / 'console.txt', 'w') as quiet, contextlib.redirect_stdout(quiet):
            recorder = ICPDiagnostics(temporary, name='results')
            try:
                with patch('map.localization.icp', side_effect=tuned_icp) if method == 'normal' else contextlib.nullcontext():
                    for scan, stamp in scans:
                        start = time.perf_counter()
                        if method == 'normal':
                            result, predicted, _, reference_count = register_scan(localizer, scan, TURTLEBOT_COORDS, SERVER_COORDS)
                        else:
                            predicted = localizer.map_from_odom @ pose_matrix(scan.transform)
                            reference_count = sum(len(bucket) for bucket in localizer.local_map.voxels.values())
                            result = localizer.register_scan(scan)
                        elapsed = 1000 * (time.perf_counter() - start)
                        recorder.write(scan, result, predicted, elapsed, reference_count, received_utc=stamp)
            finally:
                recorder.close()
        destination = output_dir / f'{name}.csv'
        shutil.copyfile(recorder.csv_path, destination)
    with destination.open(newline='', encoding='utf-8') as file:
        rows = list(csv.DictReader(file))
    last = rows[-1]
    number = lambda key: float(last[key])
    return {
        'variant': name, 'scans': len(rows),
        'updates': sum(r['status'] == 'updated' for r in rows),
        'no_update': sum(r['status'] not in ('first_scan', 'updated') for r in rows),
        'odom_path_m': number('odom_path_length_m'), 'corrected_path_m': number('corrected_path_length_m'),
        'odom_end_m': number('odom_distance_from_first_m'), 'corrected_end_m': number('distance_from_first_m'),
        'odom_heading_deg': number('odom_heading_from_first_deg'),
        'corrected_heading_deg': number('heading_from_first_deg'),
        'odom_total_rotation_deg': number('odom_total_rotation_deg'),
        'corrected_total_rotation_deg': number('corrected_total_rotation_deg'),
        'odom_signed_rotation_deg': number('odom_signed_rotation_deg'),
        'corrected_signed_rotation_deg': number('corrected_signed_rotation_deg'),
        'median_ms': statistics.median(float(r['registration_time_ms']) for r in rows),
        'median_rmse_m': statistics.median(float(r['inlier_rmse_m']) for r in rows if r['inlier_rmse_m'])
        if any(r['inlier_rmse_m'] for r in rows) else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='new directory for results; will not overwrite')
    parser.add_argument('--input-dir', default='testing/icp_runs')
    args = parser.parse_args()
    destination = Path(args.output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    results = []
    for trial in TRIALS:
        scans = load_scans(Path(args.input_dir) / f'trial_{trial}.jsonl')
        for name, method, parameter in VARIANTS:
            label = f'trial_{trial}_{name}'
            print(f'Running {label}: {len(scans)} scans', flush=True)
            result = run_variant(scans, label, method, parameter, destination)
            result['trial'] = trial
            results.append(result)
            with (destination / 'summary.json').open('w') as f:
                json.dump(results, f, indent=2)
    print(f'Saved results to {destination}')


if __name__ == '__main__':
    main()
