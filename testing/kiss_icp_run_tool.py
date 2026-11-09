"""Replay recorded TurtleBot scans using the project's KISS-inspired ICP."""

import argparse
import base64
import csv
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np

from map.icp_diagnostics import ICPDiagnostics
from map.kiss_icp_2d import OdometryICP2D, pose_matrix
from realityapi_pb2 import RawLidarScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='JSONL recording saved by ICP_LOG_DIR')
    parser.add_argument('--output-dir', default='testing/icp_runs', help='directory for new replay results')
    parser.add_argument('--device', help='device ID, required for a recording with multiple devices')
    parser.add_argument('--name', help='custom name for the new replay files')
    args = parser.parse_args()
    scans = []
    skipped_unregistered = 0
    try:
        with open(args.input, encoding='utf-8') as file:
            for line_number, line in enumerate(file, 1):
                if not line.strip():
                    continue
                try:
                    item = json.loads(line)
                    scan = RawLidarScan.FromString(base64.b64decode(item['scan_b64'], validate=True))
                    if not item.get('device_registered', True):
                        skipped_unregistered += 1
                        continue
                    scans.append(scan)
                except Exception as exc:
                    raise ValueError(f'Invalid recording at line {line_number}: {exc}') from exc
        devices = {scan.id for scan in scans}
        if args.device:
            scans = [scan for scan in scans if scan.id == args.device]
        elif len(devices) > 1:
            raise ValueError(f'Multiple devices: {sorted(devices)}. Select one with --device.')
        if not scans:
            raise ValueError('No scans found for the requested device.')
        if skipped_unregistered:
            print(f'Skipped {skipped_unregistered} scans from unregistered devices.')

        localizer = OdometryICP2D()
        recorder = ICPDiagnostics(args.output_dir, name=args.name)
        try:
            for scan in scans:
                predicted = localizer.map_from_odom @ pose_matrix(scan.transform)
                reference_count = sum(len(bucket) for bucket in localizer.local_map.voxels.values())
                start = time.perf_counter()
                result = localizer.register_scan(scan)
                elapsed = (time.perf_counter() - start) * 1000
                recorder.write(scan, result, predicted, elapsed, reference_count)
        finally:
            recorder.close()
        with recorder.csv_path.open(newline='', encoding='utf-8') as file:
            rows = list(csv.DictReader(file))
        fitted = [float(row['inlier_rmse_m']) for row in rows if row['inlier_rmse_m']]
        times = [float(row['registration_time_ms']) for row in rows]
        print(f'Replayed {len(rows)} scans for {scans[0].id}; '
              f"skipped for motion: {sum(row['status'] == 'skipped_motion' for row in rows)}; "
              f"other non-updates: {sum(row['accepted'] == 'False' and row['status'] != 'skipped_motion' for row in rows)}")
        print(f'Median registration time: {statistics.median(times):.2f} ms')
        if fitted:
            print(f'Median inlier RMSE: {statistics.median(fitted):.4f} m')
        print(f"Final distance from first pose: {float(rows[-1]['distance_from_first_m']):.3f} m; "
              f"heading difference: {float(rows[-1]['heading_from_first_deg']):.2f} degrees")
        print('Closure values only measure return-to-start error if the robot physically returned to the same pose.')
        print('Fitness and residuals do not prove pose accuracy; runtime is measured on this computer.')
    except (OSError, ValueError) as exc:
        parser.exit(2, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
