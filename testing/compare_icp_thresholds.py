"""Replay trials with alternate correspondence cutoffs, without changing live defaults."""

import contextlib
import csv
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

from map.icp import icp
from map.icp_diagnostics import ICPDiagnostics
from map.kiss_icp_2d import ICPConfig, OdometryICP2D, pose_matrix
from map.localization import ScanLocalizer
from map.scan_pipeline import register_scan
from server import SERVER_COORDS, TURTLEBOT_COORDS
from testing.compare_recorded_trials import load_scans


def main():
    output = Path('testing/icp_threshold_comparison')
    output.mkdir(exist_ok=False)
    summary = []
    for trial in (4, 5, 8, 9, 10, 11):
        scans = load_scans(Path(f'testing/icp_runs/trial_{trial}.jsonl'))
        for method in ('normal', 'kiss'):
            for cutoff in (0.15, 0.30, 0.60):
                label = f'trial_{trial}_{method}_{cutoff:.2f}'
                print(label, flush=True)
                localizer = (ScanLocalizer() if method == 'normal' else
                             OdometryICP2D(ICPConfig(initial_sigma=cutoff / 3)))

                def tuned_icp(*args, **kwargs):
                    kwargs['distance_threshold'] = cutoff
                    return icp(*args, **kwargs)

                with tempfile.TemporaryDirectory() as temp:
                    with open(Path(temp) / 'log', 'w') as log, contextlib.redirect_stdout(log):
                        recorder = ICPDiagnostics(temp, name='run')
                        with patch('map.localization.icp', side_effect=tuned_icp) if method == 'normal' else contextlib.nullcontext():
                            for scan, stamp in scans:
                                start = time.perf_counter()
                                if method == 'normal':
                                    result, predicted, _, count = register_scan(localizer, scan, TURTLEBOT_COORDS, SERVER_COORDS)
                                else:
                                    predicted = localizer.map_from_odom @ pose_matrix(scan.transform)
                                    count = sum(map(len, localizer.local_map.voxels.values()))
                                    result = localizer.register_scan(scan)
                                recorder.write(scan, result, predicted, 1000 * (time.perf_counter() - start), count, received_utc=stamp)
                        recorder.close()
                    rows = list(csv.DictReader(recorder.csv_path.open()))
                    # Normal ICP's diagnostic matcher is fixed at 0.3 m; its fitness/RMSE
                    # cannot be used to compare these alternate solver cutoffs.
                    if method == 'normal':
                        for row in rows:
                            row['max_correspondence_distance_m'] = str(cutoff)
                            if cutoff != 0.30:
                                row['inlier_count'] = ''
                                row['inlier_ratio'] = ''
                                row['inlier_rmse_m'] = ''
                    with (output / f'{label}.csv').open('w', newline='') as file:
                        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
                        writer.writeheader()
                        writer.writerows(rows)
                last = rows[-1]
                summary.append({'trial': trial, 'method': method, 'initial_cutoff_m': cutoff,
                                'path_m': float(last['corrected_path_length_m']),
                                'end_m': float(last['distance_from_first_m']),
                                'heading_deg': float(last['heading_from_first_deg']),
                                'updates': sum(r['status'] == 'updated' for r in rows),
                                'skipped': sum(r['status'] == 'skipped_motion' for r in rows),
                                'other_nonupdates': sum(r['status'] not in ('updated', 'first_scan', 'skipped_motion') for r in rows),
                                'last_cutoff_m': float(last['max_correspondence_distance_m'])})
                (output / 'summary.json').write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
