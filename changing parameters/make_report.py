"""Generate the readable report from the completed offline experiment CSV."""

import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
rows = list(csv.DictReader((HERE / 'results.csv').open(newline='', encoding='utf-8')))


def get(method, variant, trial, gate='on'):
    return next(row for row in rows if row['method'] == method and row['variant'] == variant
                and row['trial'] == str(trial) and row['gate'] == gate)


def result(method, variant, trial, target, gate='on', endpoint=False):
    row = get(method, variant, trial, gate)
    base = get(method, 'default', trial, gate)
    value = float(row['corrected_path_m'])
    base_value = float(base['corrected_path_m'])
    difference = abs(value - target) - abs(base_value - target)
    indicator = '' if variant == 'default' else (' ≈' if abs(difference) < .001 else (' ↑' if difference < 0 else ' ↓'))
    answer = f'{value:.3f}{indicator}'
    if endpoint:
        answer += f' / {float(row["corrected_end_m"]):.3f}'
    return answer


def table(method, variants):
    lines = ['| Changed setting | Stationary path (gate off) | Spin path | 8 path / end | 9 path | 10 path | Rejected or no-update: 5 / 8 / 9 / 10 |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for variant, description in variants:
        failures = ' / '.join(get(method, variant, t)['failures'] for t in (5, 8, 9, 10))
        lines.append('| ' + ' | '.join([description, result(method, variant, 4, 0, 'off'),
                     result(method, variant, 5, 0), result(method, variant, 8, 5.8, endpoint=True),
                     result(method, variant, 9, 2.54), result(method, variant, 10, 2.64), failures]) + ' |')
    return '\n'.join(lines)


normal = [('default', 'Default: kernel 0.10 m, cutoff 0.30 m, 10 pairs'),
          ('kernel_005', 'Kernel 0.05 m'), ('kernel_020', 'Kernel 0.20 m'),
          ('uniform', 'Uniform weights'), ('huber_010', 'Huber, 0.10 m'),
          ('cutoff_015', 'Correspondence cutoff 0.15 m'), ('cutoff_006', 'Correspondence cutoff 0.06 m'),
          ('min_pairs_30', 'Minimum 30 point pairs')]
kiss = [('default', 'Default: voxel 0.10 m, RMSE limit 0.20 m'),
        ('uniform', 'Uniform weights'), ('huber_010', 'Huber, 0.10 m'), ('cauchy_010', 'Cauchy, 0.10 m'),
        ('rmse_005', 'Maximum inlier RMSE 0.05 m'), ('rmse_010', 'Maximum inlier RMSE 0.10 m'),
        ('rmse_030', 'Maximum inlier RMSE 0.30 m'), ('ratio_060', 'Minimum inlier ratio 0.60'),
        ('sigma_010', 'Initial adaptive sigma 0.10 m'), ('voxel_005', 'Voxel size 0.05 m')]

intro = '''# ICP parameter experiments

Replayed **108 combinations** using five recorded TurtleBot trials and the current local code. Parameters and weight functions were changed only inside the offline test process. The live server, original recordings, and algorithm defaults were not edited.

**Reading the table:** All distances are meters. ↑ means closer to that trial's measured path than the default; ↓ means farther; ≈ means a difference under 1 mm. This is only a comparison of total path length, not a guarantee of a more accurate pose. Each setting changes one item from its method's default. The trial 8 second value is end distance from the estimated start: smaller is better only to the extent the robot physically returned to the same point.

- **Stationary, trial 4:** true center path 0. Gate **off** here so weights actually run. With the gate on, all settings skip 119 of 120 scans and report zero corrected path; that cannot rank the kernels.
- **Spin, trial 5:** robot center approximately stationary, so desired path is approximately 0. Gate on.
- **Trial 8:** approximately 2.9 m each way, or 5.8 m total, with an approximate return to start. Gate on.
- **Trial 9:** approximately 2.54 m total. **Trial 10:** approximately 2.64 m total. Gate on.

The gate skips a scan if odometry moved at most 2 cm and rotated at most 5° since the previous processed scan. It can hide scan-matching jitter, but also trusts odometry. Trial 8 has 292 scans and the default skips 101; trial 5 has 129 scans and the default skips 85. These are recorded scans, not a new physical-robot run.

## Normal ICP

Default robust weighting is Geman–McClure with a 0.10 m scale. The cutoff is the maximum allowed point-pair distance; the minimum pair count is an ICP matching condition, not a post-alignment pose-acceptance test.

'''
middle = '''

## KISS-inspired 2D ICP

Defaults include a 0.20 m inlier **RMSE** acceptance limit, minimum inlier ratio 0.35, initial adaptive sigma 0.20 m, and voxel size 0.10 m. RMSE is the square root of mean squared point-pair distances; it is not the same as mean squared error. The initial correspondence cutoff is `3 × sigma` and adapts later. This is the project's KISS-inspired 2D implementation, not the full upstream algorithm.

'''
end = '''

## What changed and what it means

- **Normal: uniform weighting** improved the measured path numbers for trials 8–10: trial 8 went from 5.218 m to 5.714 m against ~5.8 m, and trial 9 went from 2.439 m to 2.545 m against ~2.54 m. But it increased stationary false travel from 0.054 m to 0.105 m and spin false travel from 0.274 m to 0.824 m. Its trial 8 endpoint also worsened from 0.056 m to 0.096 m. A good total distance can conceal worse pose estimates.
- **Normal: kernel 0.05 m** reduced false travel in both stationary and spin tests, but gave worse path-distance results on trials 8 and 10 and essentially unchanged results on trial 9. **Kernel 0.20 m** moved in the opposite direction on these trials: less path-distance error but more stationary/spin drift.
- **Normal: cutoff 0.06 m** reduced stationary and spin travel, but its trial 8 end displacement rose from 0.056 m to 0.162 m and measured travel errors increased. Neither tested cutoff caused a no-update here, but other scenes may have fewer matches. Raising minimum pairs to 30 had no observed effect on these data.
- **KISS: Cauchy, 0.10 m** reduced stationary and spin false travel slightly and brought trial 8 path closer to ~5.8 m (6.394 m → 6.035 m). It also improved trial 10 path, while trial 9 changed by only 1 mm relative to its target. This is a candidate for further tests, not proof it will generalize.
- **KISS: RMSE limit 0.05 m** reduced spin path to 0.037 m, but rejected **104 attempts and accepted zero ICP updates** after initialization. It rejected 267 attempts and accepted only one update in trial 8. That apparent stability mostly follows odometry: it is not evidence that the strict setting fixed ICP.
- **KISS: RMSE limit 0.30 m and inlier ratio 0.60** had no observable effect in these trials. RMSE 0.10 m rejected some matches on trials 5 and 9, and had no effect on trials 8 and 10. Lowering the initial sigma narrows the initial correspondence cutoff from 0.60 m to 0.30 m; it had mixed results. A 0.05 m voxel reduced stationary false travel but did not improve every travel metric.

Huber and Cauchy alternatives here use a **fixed 0.10 m residual scale**; uniform gives equal weight to all accepted pairs. They are independent experimental implementations, not a claim that upstream KISS uses those settings. KISS's original adaptive weighting formula stays unchanged for its default variant.

## Limits and next step

The physical distances are approximate. Trial 8's measured total path is not a measured endpoint; likewise, a matching total distance does not verify sideways error or heading. ICP and KISS use different match references and criteria, so their RMSE and fitness are not directly interchangeable. Rejection counts must be read with path results: skipped scans are listed separately in `results.csv`, and rejected scans retain the predicted odometry pose. For normal ICP, an `updated` scan is not necessarily converged; a partial fit may have been applied before ICP stopped, so no-update counts are not a complete convergence-quality measure.

These same recordings were used to select promising settings, so this is exploratory, not a validation of general performance. Test the promising settings on **new recordings** with measured start/end positions and headings, plus stationary and spin trials. Keep the 2 cm / 5° gate constant when comparing settings. Timing in the CSV is indicative only: the normal and KISS paths include different conversion/evaluation work, so it is not a fair speed benchmark.

## Files and reproducibility

- `results.csv`: every tested combination, status counts, settings, travel and endpoint estimates.
- `run_experiments.py`: replays the original scan files with isolated parameter/weight overrides.
- Run again without overwriting existing results: `.venv/bin/python "changing parameters/run_experiments.py" --output "changing parameters/results_new.csv"`.
- This report reads `results.csv`: `.venv/bin/python "changing parameters/make_report.py"`.

The previously analyzed recordings are replayed with the **current** mounting offset and motion gate; their results need not match earlier reports generated from older code.
'''

report = intro + table('ICP', normal) + middle + table('KISS', kiss) + end
(HERE / 'REPORT.md').write_text(report, encoding='utf-8')
