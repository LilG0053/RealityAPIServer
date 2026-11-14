"""Produce an easy-to-read report from the recorded KISS sweep."""

import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
with (HERE / 'kiss_sweep_20261112.csv').open(newline='', encoding='utf-8') as source:
    rows = list(csv.DictReader(source))
with (HERE / 'kiss_followup_20261112.csv').open(newline='', encoding='utf-8') as source:
    followups = list(csv.DictReader(source))


def get(data, variant, trial):
    return next(row for row in data if row['variant'] == variant and row['trial'] == str(trial))


def cell(row, endpoint=False):
    value = f'{float(row["corrected_path_m"]):.3f}'
    if endpoint:
        value += f' / {float(row["corrected_end_m"]):.3f}'
    return value


def table(data, variants):
    lines = ['| Changed parameter | Stationary path | Spin path / end | Trial 8 path / end | Trial 9 path | Trial 10 path | Trial 11 end | Rejected: spin / 8 / 9 / 10 / 11 |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for variant in variants:
        series = {trial: get(data, variant, trial) for trial in (4, 5, 8, 9, 10, 11)}
        failures = ' / '.join(series[t]['failures'] for t in (5, 8, 9, 10, 11))
        values = [variant, cell(series[4]), cell(series[5], True), cell(series[8], True),
                  cell(series[9]), cell(series[10]), f'{float(series[11]["corrected_end_m"]):.3f}', failures]
        lines.append('| ' + ' | '.join(values) + ' |')
    return '\n'.join(lines)


report = '''# KISS-inspired ICP parameter sweep — November 12

**Main finding:** In this recorded spin, allowing fewer ICP iterations substantially reduced the accumulated apparent movement without rejecting scans. With the default 30-iteration limit, apparent robot-center travel was **0.779 m**. Limits of one, two and three iterations gave **0.258 m**, **0.465 m** and **0.656 m** respectively. That is consistent with the hypothesis that repeated fitting contributes to this trial's drift, but does not prove the cause or establish that fewer iterations improve real-world accuracy.

We replayed six existing recordings across 44 one-parameter settings (**264 runs**) and ran 30 additional offset/paired-tolerance comparisons. The original recordings and the live code and defaults were not changed. Your current motion gate was active except for stationary trial 4, where it was disabled to expose scan-matching noise.

## How to read the tables

- **Path** is cumulative estimated robot-center travel, including back-and-forth jitter; **end** is estimated straight-line displacement from the starting position. Numbers are meters. A lower end is only better if the physical end really was the start.
- Trial 4: stationary, gate off; target path approximately **0**. Trial 5: spin, gate on; robot-center path approximately **0**. Trial 8: down and back, approximately **5.8 m total**, end approximately start. Trials 9 and 10: approximately **2.54 m** and **2.64 m** traveled. Trial 11: circle ending approximately at the start; actual total path is unknown.
- The odometry baselines are approximately **0.000, 0.037, 5.141, 2.525, 2.554, 3.779 m** for trials 4, 5, 8, 9, 10 and 11. Trial 4's gate-off setting is diagnostic, not the production setting.
- Reject counts exclude motion-gate skips. When KISS rejects a match, it keeps the odometry-predicted pose. A lower path caused by many rejections can therefore simply mean ICP stopped contributing.
- “Updated” in this implementation means the quality checks accepted a result, not that convergence thresholds were reached. Especially with `max_iterations=1`, an accepted result may still be unfinished.

## Most useful findings

| Setting | Stationary path | Spin path / end | Trial 8 path / end | Trial 9 path | Trial 10 path | Rejected on spin / trial 8 |
|---|---:|---:|---:|---:|---:|---:|
| Default (30 iterations) | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0 / 0 |
| Maximum 1 iteration | 0.135 | 0.258 / 0.029 | 5.424 / 0.134 | 2.524 | 2.501 | 0 / 0 |
| Maximum 2 iterations | 0.174 | 0.465 / 0.046 | 5.640 / 0.081 | 2.531 | 2.499 | 0 / 0 |
| Maximum 3 iterations | 0.184 | 0.656 / 0.064 | 5.921 / 0.065 | 2.535 | 2.498 | 0 / 0 |
| Max range 3 m | 0.189 | 0.659 / 0.064 | 5.781 / 0.132 | 2.540 | 2.518 | 0 / 0 |
| RMSE acceptance 0.03 m | 0.197 | 0.037 / 0.021 | 5.141 / 0.135 | 2.525 | 2.554 | 104 / 268 |

- **Iterations:** Limiting to 1–3 iterations reduced spin path, and 2–3 iterations improved the trial 8 approximate endpoint. One iteration underestimated trial 8 total path (5.424 versus ~5.8 m); three estimated 5.921 m. On trial 9 the default was closer to 2.54 m than any of these limits. This is a tradeoff, not a universal winner.
- **Stopping versus convergence:** On the spin replay, all 43 accepted matches with a one-iteration limit reached that limit. At three iterations, 42 of 43 did. These are not verified converged solutions. At the default limit of 30, seven of the 43 reached the limit. The reported median inlier RMSE was about 0.0326 m at one iteration and 0.0295 m at 30; the lower fitting residual did not translate into lower spin travel. These residuals are internal match quality, not measured ground-truth accuracy.
- **Range:** Limiting range to 3 m produced 5.781 m on trial 8 and lowered spin path to 0.659 m. But it did not improve every endpoint. In this implementation `max_scan_range` also affects the adaptive-threshold rotation term, so this experiment changes both which points are included and how the threshold adapts.
- **Voxel:** A 0.03 m voxel reduced stationary false travel from 0.197 to 0.069 m, but spin travel rose from 0.779 to 0.856 m. The coarser 0.20 m setting improved the spin result but increased stationary drift. More detail is not automatically better in every situation.
- **RMSE and map radius:** A 0.03 m RMSE limit looked excellent for spin, but rejected 104 matches and accepted no updates after initialization. A 2 m local radius also rejected 104 spin matches. These are largely odometry-only outcomes, not evidence of better ICP. The 4 m radius rejected 127 attempts in trial 8 and increased trial 11 endpoint displacement to 1.373 m.
- **Little or no effect here:** Changing minimum pairs to 5, 20 or 40; minimum inlier ratio to 0.20, 0.60 or 0.80; RMSE limit to 0.15 or 0.30; or rotation tolerance alone in the tested range did not change the final path in these recordings. That does not mean these parameters cannot matter in other environments.

## Offset and paired tolerance checks

The offset experiment compares the current **−4 cm forward-axis offset** with zero offset for the same recordings and settings. It changes only the offset used by this offline replay; it does not prove which calibration is physically correct.

'''
report += table(followups, ['baseline', 'offset_zero', 'both_tolerances_1e-4', 'both_tolerances_1e-3', 'both_tolerances_1e-2'])
report += '''

On the spin, zero offset gave 0.888 m total travel and 0.092 m endpoint displacement; the configured −4 cm gave 0.779 m and 0.065 m. It improved this replay by roughly 11 cm of accumulated path, but most apparent motion remained. Because the recording has one pose per scan and lacks the original ROS acquisition timestamps and per-beam timing, this replay cannot settle whether timing, within-scan distortion, vibration, geometry, or another issue dominates.

Both tolerances at `0.01` stopped the fitting earlier in some cases: spin path fell to 0.484 m, but endpoint displacement was 0.072 m versus 0.065 m for the default, and the down-and-back result still reported 6.124 m. Looser tolerances are not identical to imposing a fixed iteration limit: both translation **and** rotation tests must be met. Note that `0.01` means 1 cm for translation and 0.01 radians (~0.57°) for rotation.

## Full one-parameter results

Each row changes only the named parameter from the current default; baseline weights, gate, offset and all other settings remain the same. The baseline has 0.10 m voxels, 8 m local radius, 8 m max scan range, five points per voxel, 0.20 m initial sigma, 0.05 m minimum model deviation, 10 minimum pairs, 0.35 minimum inlier ratio, 0.20 m maximum inlier RMSE, 30 maximum iterations, and `1e-5` translation and rotation tolerances.

'''
report += table(rows, list(dict.fromkeys(row['variant'] for row in rows)))
report += '''

## What to do next

Keep the existing default until the results are validated. Replay **1, 2, 3 and 30 iterations** on new stationary, slow-spin, fast-spin, measured straight-line and marked loop runs; record actual start/end position and heading. The recorded trials used here selected these candidates, so they are not independent validation. Track rejected, skipped, accepted and iteration-limit outcomes separately, and inspect path, endpoint and heading—not just total travel. Record raw `/scan` and `/odom` timestamps if investigating synchronization or deskew.

## Files and reproduction

- `kiss_sweep_20261112.csv`: all 264 one-parameter runs, their settings, paths, acceptance counts and timings.
- `kiss_followup_20261112.csv`: 30 paired-tolerance and offset runs.
- `kiss_parameter_sweep.py`, `kiss_followup_sweep.py`, and `kiss_sweep_report.py`: offline replay and report scripts.
- To repeat without overwriting files:

```bash
.venv/bin/python "changing parameters/kiss_parameter_sweep.py" --output "changing parameters/kiss_sweep_new.csv"
.venv/bin/python "changing parameters/kiss_followup_sweep.py" --output "changing parameters/kiss_followup_new.csv"
```

The timings are exploratory measurements on this computer; do not treat them as controlled performance benchmarks. This is the project's KISS-inspired 2D implementation, not the full upstream library.
'''
(HERE / 'KISS_PARAMETER_SWEEP.md').write_text(report, encoding='utf-8')
