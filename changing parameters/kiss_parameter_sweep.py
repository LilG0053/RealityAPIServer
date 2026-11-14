"""Replay fixed robot recordings with one KISS setting changed at a time."""

import argparse
import csv
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('parameter_replay', HERE / 'run_experiments.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)

VARIANTS = [('baseline', {})]
for parameter, values in [
    ('voxel_size', (.03, .05, .15, .20)),
    ('local_radius', (2.0, 4.0, 12.0)),
    ('max_scan_range', (3.0, 5.0, 10.0)),
    ('max_points_per_voxel', (1, 2, 10)),
    ('initial_sigma', (.05, .10, .40)),
    ('min_model_deviation', (.01, .10, .20)),
    ('min_pairs', (5, 20, 40)),
    ('min_inlier_ratio', (.20, .60, .80)),
    ('max_inlier_rmse', (.03, .05, .08, .10, .15, .30)),
    ('max_iterations', (1, 2, 3, 5, 10, 50)),
    ('translation_tolerance', (1e-4, 1e-3, 1e-2)),
    ('rotation_tolerance', (1e-4, 1e-3, 1e-2)),
]:
    VARIANTS.extend((f'{parameter}={value:g}', {parameter: value}) for value in values)

# The stationary test disables the gate to expose fitting noise. Other trials
# use the current live gate. Truth values are approximate path lengths.
TRIALS = {4: ('off', 0.0), 5: ('on', 0.0), 8: ('on', 5.8),
          9: ('on', 2.54), 10: ('on', 2.64), 11: ('on', None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='new CSV file; existing files are never overwritten')
    args = parser.parse_args()
    fields = ['trial', 'gate', 'variant', 'parameters', 'scans', 'updates', 'skips', 'failures',
              'odom_path_m', 'corrected_path_m', 'odom_end_m', 'corrected_end_m', 'measured_path_m',
              'path_abs_error_m', 'median_match_ms', 'median_match_rmse_m', 'statuses']
    with Path(args.output).open('x', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for trial, (gate, measured) in TRIALS.items():
            scans = experiment.load_scans(ROOT / 'testing' / 'icp_runs' / f'trial_{trial}.jsonl')
            for label, parameters in VARIANTS:
                print(f'Trial {trial}: {label}', flush=True)
                result = experiment.replay(scans, 'KISS', parameters, 'default', gate)
                row = {'trial': trial, 'gate': gate, 'variant': label,
                       'parameters': json.dumps(parameters), 'measured_path_m': measured,
                       'path_abs_error_m': abs(result['corrected_path_m'] - measured) if measured is not None else None,
                       **result}
                row['statuses'] = json.dumps(row['statuses'])
                writer.writerow(row)
                file.flush()
    print(f'Saved {args.output}')


if __name__ == '__main__':
    main()
