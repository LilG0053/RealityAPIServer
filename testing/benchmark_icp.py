"""Average three room-scan ICP tests per case and append the results to CSV."""

import argparse
import csv
from datetime import date
from decimal import Decimal, ROUND_DOWN
import math
from pathlib import Path
import time

import numpy as np
import map.icp as icp_module
from testing.icp_room_cases import CASES, make_room_scans

# Noise severity ranges from 0 to 1; edit each room independently here.
CASE_NOISE = {
    "room": [0.1, 0.5, 0.7],
    "furnished_room": [0.1, 0.5, 0.7],
    "corridor": [0.1, 0.5, 0.7],
}
RUNS_PER_CASE = 3
ICP_SETTINGS = {
    "max_iterations": 100,
    "distance_threshold": 0.3,
    "convergence_translation_threshold": 1e-3,
    "convergence_rotation_threshold": 1e-4,
    "point_pairs_threshold": 10,
    "verbose": False,
}
FIELDS = ["date", "algorithm", "noise_level", "translation_error_m",
          "rotation_error_deg", "inlier_count", "source_count", "fitness",
          "inlier_rmse_m", "time_ms", "case", "label"]


def run_once(reference, moving, expected):
    start_time = time.perf_counter()

    history, aligned = icp_module.icp(reference, moving.copy(), **ICP_SETTINGS)

    elapsed_ms = (time.perf_counter() - start_time) * 1000

    quality = icp_module.evaluate_alignment(reference, aligned, ICP_SETTINGS["distance_threshold"])
    result = {"inlier_count": quality["inlier_count"], "source_count": quality["source_count"],
              "fitness": quality["fitness"], "inlier_rmse_m": quality["inlier_rmse"], "time_ms": elapsed_ms}
    if not history:
        return result
    estimated = np.eye(3)
    for step in history:
        correction = np.eye(3)
        correction[:2, :] = step
        estimated = correction @ estimated
    result["translation_error_m"] = float(np.linalg.norm(estimated[:2, 2] - expected[:2, 2]))
    actual_angle = math.atan2(estimated[1, 0], estimated[0, 0])
    expected_angle = math.atan2(expected[1, 0], expected[0, 0])
    delta = actual_angle - expected_angle
    result["rotation_error_deg"] = abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))
    return result


def truncate_str(number, decimals=0):
    """Truncate after averaging, including values in scientific notation."""
    precision = Decimal(1).scaleb(-decimals)
    return format(Decimal(str(number)).quantize(precision, rounding=ROUND_DOWN), "f")


def average_results(results):
    averaged = {}
    for key, decimals in [("translation_error_m", 5), ("rotation_error_deg", 5),
                          ("fitness", 4), ("inlier_rmse_m", 5), ("time_ms", 0)]:
        values = [result.get(key) for result in results]
        # An unavailable result makes the three-run average unavailable.
        averaged[key] = truncate_str(sum(values) / len(values), decimals) if all(
            value is not None for value in values) else ""
    mean_inliers = sum(result["inlier_count"] for result in results) / len(results)
    averaged["inlier_count"] = truncate_str(mean_inliers, 2)
    averaged["source_count"] = results[0]["source_count"]

    return averaged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="testing/icp_room_averages.csv", help="CSV file to append to")
    parser.add_argument("--label", default="baseline", help="Name this experiment")
    parser.add_argument("--noise", type=float, default=None, help="override noise for all selected cases (0 to 1)")
    parser.add_argument("--seed", type=int, default=42, help="first of three consecutive random seeds")
    parser.add_argument("--cases", nargs="+", choices=list(CASES), default=list(CASES))
    args = parser.parse_args()
    if args.noise is not None and not 0 <= args.noise <= 1:
        parser.error("--noise must be between 0 and 1")
    if args.seed < 0:
        parser.error("--seed must be nonnegative")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and output.stat().st_size:
        with output.open(newline="", encoding="utf-8") as existing:
            if next(csv.reader(existing), None) != FIELDS:
                parser.error("existing CSV has a different header; choose another --output")

    rows = []
    for case in args.cases:
        noise_levels = CASE_NOISE[case] if args.noise is None else [args.noise]
        for noise in noise_levels:
            if not 0 <= noise <= 1:
                parser.error(f"noise level for {case} must be between 0 and 1: {noise}")
            results = []
            for seed in range(args.seed, args.seed + RUNS_PER_CASE):
                reference, moving, expected = make_room_scans(case, noise, seed)
                if len(moving) != 500:
                    raise ValueError(f"{case}: expected 500 source points, got {len(moving)}")
                result = run_once(reference, moving, expected)
                if "translation_error_m" not in result:
                    print(f"{case}, noise {noise}, seed {seed}: ICP produced no transform; average pose errors left blank")
                results.append(result)
            rows.append({"date": date.today().isoformat(), "algorithm": "map.icp.icp",
                         "noise_level": noise, **average_results(results),
                         "case": case, "label": args.label})

    write_header = not output.exists() or output.stat().st_size == 0
    with output.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} rows (each averages {RUNS_PER_CASE} tests) to {output}")


if __name__ == "__main__":
    main()
