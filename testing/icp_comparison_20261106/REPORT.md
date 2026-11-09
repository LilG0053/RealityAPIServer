# TurtleBot ICP comparison

Recordings tested: November 6, 2026. All distances are meters. Each recording was replayed through both algorithms and four parameter variants, for 48 replays total. Each method received the same raw scans and odometry. The live server and its parameters were not changed.

**Main finding:** On these trials, normal ICP had less false movement while stationary and spinning, and returned closer to the start on the approximate return trips. For the 2.64 m straight run, the default KISS-inspired version's total distance was closer to the measured distance than normal ICP's. Wheel odometry was closer still. No single method won every test, and distance alone cannot establish pose accuracy.

The KISS-inspired method is the project's 2D adaptation, not the full upstream KISS-ICP. “Normal” means `map/localization.py` calling `map/icp.py`.

## How to read the results

- **Path** is the sum of movement between consecutive scan poses. Jitter makes it grow even when the robot is stationary.
- **End** is straight-line distance between the first and last estimated positions. For an approximate return to the start, a smaller number is better, subject to how accurately the robot actually returned.
- Both methods use the recorded odometry as an input. These are estimates, not independent measurements of truth.
- Scan fitness and RMSE use different references, point sampling, and thresholds across the two methods. Do not rank their absolute values as if they were measured identically.
- Timing is indicative only: the normal replay's measured interval includes coordinate conversion and extra alignment evaluation; the KISS-inspired interval measures its registration call. The two timings are therefore not directly equivalent.

## Baseline results

Numbers in each method column are **path / end**. “Measured” is what you described; `~0 end` means the robot returned approximately, not to a precisely surveyed mark.

| Trial | Description and measurement | Scans | Odometry path / end | Normal ICP path / end | KISS-inspired path / end |
|---|---|---:|---:|---:|---:|
| 3 | Actual distance between normal ICP and odometry, but no exact measurement supplied | 125 | 2.534 / 2.528 | 2.460 / 2.438 | 2.724 / 2.542 |
| 4 | Stationary: 0 m | 120 | 0.000 / 0.000 | 0.054 / 0.009 | 0.210 / 0.001 |
| 5 | Spin in place: robot center approximately stationary | 129 | 0.037 / 0.021 | 0.671 / 0.080 | 1.327 / 0.082 |
| 7 | Random movement; no measured path or final position | 744 | 14.597 / 1.470 | 15.523 / 1.764 | 20.380 / 1.355 |
| 8 | Approximately 2.9 m each way: ~5.8 m total, ~0 m end | 292 | 5.141 / 0.135 | 5.336 / 0.119 | 6.683 / 0.131 |
| 9 | Approximately 2.54 m, with a small sideways movement | 124 | 2.525 / 2.522 | 2.468 / 2.437 | 2.711 / 2.546 |
| 10 | Approximately 2.64 m | 120 | 2.554 / 2.548 | 2.430 / 2.393 | 2.741 / 2.523 |
| 11 | Circle; returned approximately to start | 170 | 3.779 / 0.258 | 3.794 / 0.127 | 4.201 / 0.294 |

### What those numbers mean

- **Stationary:** Normal ICP added 5.4 cm of apparent travel; KISS-inspired added 21.0 cm. KISS-inspired ended closer to its starting point (0.1 cm versus 0.9 cm), illustrating why path length and endpoint are separate measurements. Neither test proves global accuracy.
- **Spinning:** Odometry reported 3.7 cm travel; normal reported 67.1 cm and KISS-inspired 132.7 cm. Their final position errors relative to their own starting positions were both about 8 cm. This is a significant indication of pose jitter or scan-matching movement during the turn, not evidence the robot actually traveled those distances. A sensor offset, scan timing, scan distortion, odometry timing and geometry all remain possible causes.
- **Trial 8:** Against an approximate 5.8 m total, normal ICP was short by 0.464 m, KISS-inspired was high by 0.883 m, and odometry was short by 0.659 m. Approximate endpoint errors were 0.119, 0.131 and 0.135 m respectively, if the physical endpoint was exactly the start. These are not precision ground-truth results.
- **Trial 9:** Against the approximate 2.54 m *travel distance*, odometry was within 1.5 cm, normal ICP within 7.2 cm, and KISS-inspired within 17.1 cm. The sideways motion means actual endpoint displacement could be smaller than path length, so do not compare the 2.54 m measurement directly to endpoint distance unless it was measured start-to-finish.
- **Trial 10:** Against 2.64 m travel, odometry was short by 8.6 cm, normal ICP by 21.0 cm, and KISS-inspired was high by 10.1 cm. A close distance does not prove the estimated sideways position or heading is correct.
- **Trial 11:** Assuming the circle finished exactly where it began, endpoint discrepancies were 12.7 cm for normal ICP, 29.4 cm for KISS-inspired, and 25.8 cm for odometry. Since the return was approximate, treat these as indicators. The recorded start/end heading difference is about -48° in all three estimates; do not call this heading error without knowing the actual final orientation.
- **Trials 3 and 7:** The information supplied does not provide a numeric ground truth sufficient to rank both methods. Trial 7 shows large differences, including 15.523 m versus 20.380 m reported path, but neither number can be declared correct from the recording alone.

## Parameter experiments

Only one parameter was changed at a time, in the offline comparison script. No project defaults were modified:

- Normal default: robust kernel distance **0.10 m**. Alternatives: **0.05 m** and **0.20 m**. Smaller values reduce the influence of more distant matches more strongly.
- KISS-inspired default: voxel size **0.10 m**. Alternatives: **0.05 m** and **0.20 m**. A smaller voxel retains more map detail and usually costs more computation.

The values below are **path / end**, in meters. Lower is better for both values on trials 4 and 5; on the approximate returns (8 and 11), lower end is preferred, while trial 8 path should be compared with ~5.8 m. For trials 9 and 10, compare path with 2.54 and 2.64 m respectively. Other trials lack measured targets.

| Trial | Normal kernel .05 | Normal default .10 | Normal kernel .20 | KISS voxel .05 | KISS default .10 | KISS voxel .20 |
|---|---:|---:|---:|---:|---:|---:|
| 3 | 2.446 / 2.427 | 2.460 / 2.438 | 2.507 / 2.481 | 2.673 / 2.551 | 2.724 / 2.542 | 2.768 / 2.516 |
| 4 | 0.037 / 0.002 | 0.054 / 0.009 | 0.074 / 0.022 | 0.113 / 0.001 | 0.210 / 0.001 | 0.345 / 0.003 |
| 5 | 0.404 / 0.072 | 0.671 / 0.080 | 0.789 / 0.107 | 1.135 / 0.089 | 1.327 / 0.082 | 1.413 / 0.083 |
| 7 | 15.509 / 2.199 | 15.523 / 1.764 | 15.833 / 1.293 | 19.960 / 1.376 | 20.380 / 1.355 | 21.300 / 1.448 |
| 8 | 5.226 / 0.055 | 5.336 / 0.119 | 5.455 / 0.025 | 6.779 / 0.152 | 6.683 / 0.131 | 6.874 / 0.144 |
| 9 | 2.452 / 2.431 | 2.468 / 2.437 | 2.521 / 2.476 | 2.666 / 2.551 | 2.711 / 2.546 | 2.846 / 2.562 |
| 10 | 2.405 / 2.381 | 2.430 / 2.393 | 2.483 / 2.425 | 2.662 / 2.505 | 2.741 / 2.523 | 2.741 / 2.535 |
| 11 | 3.704 / 0.115 | 3.794 / 0.127 | 3.835 / 0.134 | 4.143 / 0.288 | 4.201 / 0.294 | 4.366 / 0.267 |

### Did the parameter changes help?

- **Normal kernel 0.05:** Improved stationary path (5.4 to 3.7 cm) and spin path (67.1 to 40.4 cm), and reduced approximate end displacement on trials 8 and 11. It made measured-path errors on trials 9 and 10 slightly worse. It is a promising candidate for follow-up, not a universal winner.
- **Normal kernel 0.20:** Made stationary/spin false travel worse, but brought measured-path estimates closer on trials 8, 9 and 10; trial 8's approximate endpoint also improved. Trial 11's endpoint worsened slightly.
- **KISS voxel 0.05:** Reduced stationary and spin false travel versus the default, and improved trial 10's path error from 10.1 cm to 2.2 cm. Its approximate return endpoint got worse on trial 8 and slightly better on trial 11. It retained more points and was slower in these replays.
- **KISS voxel 0.20:** Increased stationary and spin false travel. It slightly improved the approximate trial 11 endpoint but worsened measured-path errors on trials 8 and 9. It was generally faster in these replays.

This is a small, non-independent set of runs, and settings were evaluated on the same data used to select them. Do not assume a parameter will generalize until testing it on new recordings. “Updated” indicates an algorithm produced an update, not that its pose is correct or that its optimization converged. KISS rejected three scans on trial 7; no other replay reported a rejected/no-update scan.

## Why KISS-inspired can be worse here

It matches a voxel-downsampled scan against an accumulated local map; normal ICP matches against the previous scan. A map with small pose errors can pull later estimates, and downsampling can lose useful room detail. Each method also has different correspondence thresholds and weighting. In-place rotation may expose mounting offset or timestamp effects that neither algorithm currently corrects. These are hypotheses: the recordings alone do not identify which one caused the error.

## Next test

Use a marked robot-center position and orientation. Record (1) 30 seconds stationary, (2) slow and fast spins returning to the mark, (3) a measured straight trip, and (4) a loop returning to a marked pose. Record actual endpoint position and heading, plus total path where possible. Run each setting on those same new recordings and compare endpoint error and heading error as well as total path. Confirm the actual LiDAR-to-robot mounting offset and investigate scan/odometry timestamp synchronization before tuning heavily around spin errors.

## Reproduce and inspect

- `summary.json` holds one summary per run. Each `trial_<number>_<variant>.csv` has the per-scan values; the last row has the final totals.
- `trial_3.jsonl`, `trial_4.jsonl`, `trial_5.jsonl`, `trial_7.jsonl`, `trial_8.jsonl`, `trial_9.jsonl`, `trial_10.jsonl`, and `trial_11.jsonl` were the inputs; the original recordings were not modified.
- Run a fresh comparison into a new directory:

  ```bash
  .venv/bin/python -m testing.compare_recorded_trials --output-dir testing/icp_comparison_new
  ```

- Full per-scan metrics, including signed and total rotation, are in each CSV. Trial names and parameter settings are recorded in each filename. `reference_points` means the previous scan size for normal ICP and local map size for KISS-inspired ICP.
