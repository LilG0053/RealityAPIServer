"""Run controlled offset and paired-tolerance comparisons without changing production code."""

import argparse
import csv
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('parameter_replay', HERE / 'run_experiments.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)
import map.kiss_icp_2d as kiss

VARIANTS = [
    ('baseline', {}, False),
    ('offset_zero', {}, True),
    ('both_tolerances_1e-4', {'translation_tolerance': .0001, 'rotation_tolerance': .0001}, False),
    ('both_tolerances_1e-3', {'translation_tolerance': .001, 'rotation_tolerance': .001}, False),
    ('both_tolerances_1e-2', {'translation_tolerance': .01, 'rotation_tolerance': .01}, False),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    fields = ['trial', 'gate', 'variant', 'parameters', 'scans', 'updates', 'skips', 'failures',
              'odom_path_m', 'corrected_path_m', 'odom_end_m', 'corrected_end_m',
              'median_match_ms', 'median_match_rmse_m', 'statuses']
    original = kiss.scan_to_robot_points
    with Path(args.output).open('x', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for trial in (4, 5, 8, 9, 10, 11):
            scans = experiment.load_scans(ROOT / 'testing' / 'icp_runs' / f'trial_{trial}.jsonl')
            gate = 'off' if trial == 4 else 'on'
            for label, parameters, zero_offset in VARIANTS:
                print(f'Trial {trial}: {label}', flush=True)
                if zero_offset:
                    def without_offset(*args, **kwargs):
                        points = original(*args, **kwargs).copy()
                        points[:, 0] += .04
                        return points
                    with patch('map.kiss_icp_2d.scan_to_robot_points', side_effect=without_offset):
                        result = experiment.replay(scans, 'KISS', parameters, 'default', gate)
                else:
                    result = experiment.replay(scans, 'KISS', parameters, 'default', gate)
                result['statuses'] = json.dumps(result['statuses'])
                writer.writerow({'trial': trial, 'gate': gate, 'variant': label,
                                 'parameters': json.dumps(parameters), **result})
                file.flush()
    print(f'Saved {args.output}')


if __name__ == '__main__':
    main()
