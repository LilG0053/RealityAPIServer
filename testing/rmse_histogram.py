"""Save a histogram of successive scan RMSE values.

The dashboard button writes a JSON list and runs this script. It can also be
run on a saved list:

    python testing/rmse_histogram.py saved_scans/rmse-histograms/<stamp>.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Vertical marks drawn on every saved histogram, as (legend name, percentile).
RMSE_MARKS = (
    ("Median", 50, "#1d4e89", "--"),
    ("p90", 90, "#1b7f4e", "-."),
    ("p95", 95, "#6d28d9", ":"),
)


def finite_rmses(values) -> list[float]:
    """Keep finite RMSE numbers and reject anything else."""
    kept = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            kept.append(number)
    return kept


def save_rmse_histogram(values, output_path: Path) -> Path:
    """Write a PNG histogram of scan RMSEs. Returns the path that was written."""
    samples = finite_rmses(values)
    if not samples:
        raise ValueError("No finite scan RMSEs to plot")

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # One bin when every scan landed on the same error; otherwise a short set of bins.
    bins = 1 if len(samples) < 2 or min(samples) == max(samples) else min(20, max(5, round(math.sqrt(len(samples)))))

    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.hist(samples, bins=bins, color="#ff7043", edgecolor="#1b2430")
    for name, percent, color, style in RMSE_MARKS:
        value = float(np.percentile(samples, percent))
        axis.axvline(value, color=color, linestyle=style, linewidth=1.6, label=f"{name} {value:.3f} m")
    axis.set_xlabel("Inlier RMSE (m)")
    axis.set_ylabel("Scans")
    axis.set_title(f"Scan RMSE ({len(samples)} scans)")
    axis.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(output_path, dpi=120)
    plt.close(figure)
    return output_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Save a histogram of scan RMSE values.")
    parser.add_argument("values", type=Path, help="JSON list of RMSE values in meters")
    parser.add_argument("-o", "--output", type=Path, help="PNG to write; defaults to the JSON path with a .png suffix")
    args = parser.parse_args(argv)

    samples = json.loads(args.values.read_text())
    if not isinstance(samples, list):
        print("RMSE file must contain a JSON list", file=sys.stderr)
        return 1
    output = args.output or args.values.with_suffix(".png")
    try:
        written = save_rmse_histogram(samples, output)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(written)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
