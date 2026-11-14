# KISS-inspired ICP parameter sweep — November 12

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

| Changed parameter | Stationary path | Spin path / end | Trial 8 path / end | Trial 9 path | Trial 10 path | Trial 11 end | Rejected: spin / 8 / 9 / 10 / 11 |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| offset_zero | 0.210 | 0.888 / 0.092 | 6.443 / 0.141 | 2.558 | 2.531 | 0.286 | 0 / 0 / 0 / 0 / 0 |
| both_tolerances_1e-4 | 0.192 | 0.785 / 0.065 | 6.317 / 0.141 | 2.541 | 2.514 | 0.268 | 0 / 0 / 0 / 0 / 0 |
| both_tolerances_1e-3 | 0.173 | 0.885 / 0.069 | 6.383 / 0.144 | 2.534 | 2.508 | 0.278 | 0 / 0 / 0 / 0 / 0 |
| both_tolerances_1e-2 | 0.135 | 0.484 / 0.072 | 6.124 / 0.138 | 2.532 | 2.497 | 0.273 | 0 / 0 / 0 / 0 / 0 |

On the spin, zero offset gave 0.888 m total travel and 0.092 m endpoint displacement; the configured −4 cm gave 0.779 m and 0.065 m. It improved this replay by roughly 11 cm of accumulated path, but most apparent motion remained. Because the recording has one pose per scan and lacks the original ROS acquisition timestamps and per-beam timing, this replay cannot settle whether timing, within-scan distortion, vibration, geometry, or another issue dominates.

Both tolerances at `0.01` stopped the fitting earlier in some cases: spin path fell to 0.484 m, but endpoint displacement was 0.072 m versus 0.065 m for the default, and the down-and-back result still reported 6.124 m. Looser tolerances are not identical to imposing a fixed iteration limit: both translation **and** rotation tests must be met. Note that `0.01` means 1 cm for translation and 0.01 radians (~0.57°) for rotation.

## Full one-parameter results

Each row changes only the named parameter from the current default; baseline weights, gate, offset and all other settings remain the same. The baseline has 0.10 m voxels, 8 m local radius, 8 m max scan range, five points per voxel, 0.20 m initial sigma, 0.05 m minimum model deviation, 10 minimum pairs, 0.35 minimum inlier ratio, 0.20 m maximum inlier RMSE, 30 maximum iterations, and `1e-5` translation and rotation tolerances.

| Changed parameter | Stationary path | Spin path / end | Trial 8 path / end | Trial 9 path | Trial 10 path | Trial 11 end | Rejected: spin / 8 / 9 / 10 / 11 |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| voxel_size=0.03 | 0.069 | 0.856 / 0.080 | 6.295 / 0.142 | 2.543 | 2.496 | 0.266 | 0 / 0 / 0 / 0 / 0 |
| voxel_size=0.05 | 0.105 | 0.799 / 0.074 | 6.310 / 0.140 | 2.549 | 2.508 | 0.270 | 0 / 0 / 0 / 0 / 0 |
| voxel_size=0.15 | 0.252 | 0.770 / 0.059 | 6.237 / 0.153 | 2.548 | 2.532 | 0.287 | 0 / 0 / 0 / 0 / 0 |
| voxel_size=0.2 | 0.289 | 0.745 / 0.033 | 6.303 / 0.139 | 2.570 | 2.558 | 0.281 | 0 / 0 / 0 / 0 / 0 |
| local_radius=2 | 0.000 | 0.037 / 0.021 | 5.141 / 0.135 | 2.973 | 2.554 | 0.258 | 104 / 268 / 29 / 100 / 149 |
| local_radius=4 | 0.274 | 0.668 / 0.100 | 5.667 / 0.155 | 2.535 | 2.560 | 1.373 | 55 / 127 / 0 / 0 / 4 |
| local_radius=12 | 0.197 | 0.779 / 0.065 | 6.397 / 0.131 | 2.542 | 2.499 | 0.293 | 0 / 0 / 0 / 0 / 0 |
| max_scan_range=3 | 0.189 | 0.659 / 0.064 | 5.781 / 0.132 | 2.540 | 2.518 | 0.297 | 0 / 0 / 0 / 0 / 0 |
| max_scan_range=5 | 0.220 | 0.732 / 0.102 | 6.122 / 0.162 | 2.535 | 2.523 | 0.306 | 0 / 0 / 0 / 0 / 0 |
| max_scan_range=10 | 0.225 | 0.931 / 0.074 | 6.960 / 0.130 | 2.547 | 2.524 | 0.266 | 0 / 0 / 0 / 0 / 0 |
| max_points_per_voxel=1 | 0.259 | 0.693 / 0.052 | 6.330 / 0.141 | 2.546 | 2.536 | 0.286 | 0 / 0 / 0 / 0 / 0 |
| max_points_per_voxel=2 | 0.253 | 0.604 / 0.046 | 6.228 / 0.160 | 2.546 | 2.540 | 0.291 | 0 / 0 / 0 / 0 / 0 |
| max_points_per_voxel=10 | 0.157 | 0.842 / 0.083 | 6.484 / 0.143 | 2.545 | 2.521 | 0.268 | 0 / 0 / 0 / 0 / 0 |
| initial_sigma=0.05 | 0.194 | 0.666 / 0.056 | 6.283 / 0.146 | 2.537 | 2.498 | 0.274 | 0 / 0 / 0 / 0 / 0 |
| initial_sigma=0.1 | 0.193 | 0.839 / 0.070 | 6.320 / 0.136 | 2.533 | 2.516 | 0.282 | 0 / 0 / 0 / 0 / 0 |
| initial_sigma=0.4 | 0.189 | 0.801 / 0.064 | 6.287 / 0.133 | 2.564 | 2.542 | 0.292 | 0 / 0 / 0 / 1 / 0 |
| min_model_deviation=0.01 | 0.197 | 0.798 / 0.070 | 6.215 / 0.139 | 2.537 | 2.509 | 0.285 | 0 / 0 / 0 / 0 / 0 |
| min_model_deviation=0.1 | 0.197 | 0.764 / 0.066 | 6.446 / 0.134 | 2.543 | 2.543 | 0.279 | 0 / 0 / 0 / 0 / 0 |
| min_model_deviation=0.2 | 0.197 | 0.928 / 0.083 | 6.423 / 0.129 | 2.543 | 2.543 | 0.264 | 0 / 0 / 0 / 0 / 0 |
| min_pairs=5 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| min_pairs=20 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| min_pairs=40 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| min_inlier_ratio=0.2 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| min_inlier_ratio=0.6 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| min_inlier_ratio=0.8 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| max_inlier_rmse=0.03 | 0.197 | 0.037 / 0.021 | 5.141 / 0.135 | 2.525 | 2.554 | 0.258 | 104 / 268 / 93 / 100 / 149 |
| max_inlier_rmse=0.05 | 0.197 | 0.037 / 0.021 | 5.132 / 0.137 | 2.525 | 2.540 | 0.258 | 104 / 267 / 93 / 58 / 149 |
| max_inlier_rmse=0.08 | 0.197 | 0.037 / 0.021 | 6.245 / 0.144 | 2.525 | 2.499 | 0.258 | 104 / 1 / 93 / 0 / 149 |
| max_inlier_rmse=0.1 | 0.197 | 0.791 / 0.070 | 6.394 / 0.138 | 2.539 | 2.499 | 0.271 | 2 / 0 / 2 / 0 / 1 |
| max_inlier_rmse=0.15 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| max_inlier_rmse=0.3 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=1 | 0.135 | 0.258 / 0.029 | 5.424 / 0.134 | 2.524 | 2.501 | 0.273 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=2 | 0.174 | 0.465 / 0.046 | 5.640 / 0.081 | 2.531 | 2.499 | 0.279 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=3 | 0.184 | 0.656 / 0.064 | 5.921 / 0.065 | 2.535 | 2.498 | 0.284 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=5 | 0.191 | 0.915 / 0.111 | 6.227 / 0.064 | 2.545 | 2.502 | 0.270 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=10 | 0.197 | 1.018 / 0.133 | 6.438 / 0.138 | 2.545 | 2.510 | 0.278 | 0 / 0 / 0 / 0 / 0 |
| max_iterations=50 | 0.197 | 0.759 / 0.067 | 6.349 / 0.135 | 2.542 | 2.499 | 0.270 | 0 / 0 / 0 / 0 / 0 |
| translation_tolerance=0.0001 | 0.192 | 0.779 / 0.065 | 6.370 / 0.149 | 2.542 | 2.498 | 0.283 | 0 / 0 / 0 / 0 / 0 |
| translation_tolerance=0.001 | 0.185 | 0.760 / 0.068 | 6.313 / 0.131 | 2.545 | 2.527 | 0.266 | 0 / 0 / 0 / 0 / 0 |
| translation_tolerance=0.01 | 0.182 | 0.736 / 0.067 | 6.309 / 0.132 | 2.542 | 2.503 | 0.278 | 0 / 0 / 0 / 0 / 0 |
| rotation_tolerance=0.0001 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| rotation_tolerance=0.001 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |
| rotation_tolerance=0.01 | 0.197 | 0.779 / 0.065 | 6.394 / 0.138 | 2.542 | 2.499 | 0.292 | 0 / 0 / 0 / 0 / 0 |

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
