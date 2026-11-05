"""Compare two labeled ICP benchmark runs, case by case and overall."""

import argparse
from collections import Counter
import csv
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path

# Only pose error and runtime have an unambiguous direction in this report.
METRICS = [
    ("translation_error_m", "Translation error (m)", "lower"),
    ("rotation_error_deg", "Rotation error (deg)", "lower"),
    ("inlier_count", "Inlier count", None),
    ("source_count", "Source count", None),
    ("fitness", "Fitness", None),
    ("inlier_rmse_m", "Inlier RMSE (m)", None),
    ("time_ms", "ICP time (ms)", "lower"),
]


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def noise_key(value):
    try:
        result = Decimal(value)
        if not result.is_finite():
            raise InvalidOperation
        return result.normalize()
    except (InvalidOperation, TypeError):
        raise ValueError(f"Invalid noise level: {value!r}") from None


def index_rows(rows, label):
    selected = [r for r in rows if r['label'] == label]
    if not selected:
        raise ValueError(f"No rows found for label {label!r}. Use --list to see labels.")
    algorithms = {r['algorithm'] for r in selected}
    if len(algorithms) != 1:
        raise ValueError(f"Label {label!r} contains multiple algorithms; use a unique label for each run.")
    indexed = {}
    for row in selected:
        key = (row['case'], noise_key(row['noise_level']))
        if key in indexed:
            raise ValueError(
                f"Label {label!r} has duplicate results for {key[0]}, noise {key[1]}. "
                "Use a unique label for each benchmark run or choose a CSV with one row per case and noise."
            )
        indexed[key] = row
    return indexed, next(iter(algorithms))


def verdict(before, after, direction):
    if before is None or after is None:
        return "missing"
    if math.isclose(before, after, rel_tol=1e-9, abs_tol=1e-12):
        return "same"
    if direction is None:
        return "higher" if after > before else "lower"
    return "better" if after < before else "worse"


def fmt(value):
    return "n/a" if value is None else f"{value:.5g}"


def build_report(before_label, after_label, before, after, before_algorithm, after_algorithm):
    common = sorted(before.keys() & after.keys())
    if not common:
        raise ValueError("The labels have no case and noise combinations in common.")
    lines = [f"ICP COMPARISON: {before_label} -> {after_label}",
             f"Algorithms: {before_algorithm} -> {after_algorithm}",
             f"Matched case/noise combinations: {len(common)}",
             "Each input row is already an average of three benchmark tests.", ""]
    for title, missing in [(f"Only in {before_label}", before.keys() - after.keys()),
                           (f"Only in {after_label}", after.keys() - before.keys())]:
        if missing:
            lines.append(title + ": " + ", ".join(f"{c} noise {n}" for c, n in sorted(missing)))
    lines += ["", "OVERALL: mean across matched case/noise combinations",
              f"{'Metric':26} {'Before':>11} {'After':>11} {'Change':>11}  Result (case counts)"]
    for key, title, direction in METRICS:
        pairs = [(number(before[k].get(key)), number(after[k].get(key))) for k in common]
        usable = [(a, b) for a, b in pairs if a is not None and b is not None]
        if not usable:
            lines.append(f"{title:26} {'n/a':>11} {'n/a':>11} {'n/a':>11}  no comparable values")
            continue
        avg_before = sum(a for a, _ in usable) / len(usable)
        avg_after = sum(b for _, b in usable) / len(usable)
        counts = Counter(verdict(a, b, direction) for a, b in usable)
        details = ", ".join(f"{count} {status}" for status, count in sorted(counts.items()))
        lines.append(f"{title:26} {fmt(avg_before):>11} {fmt(avg_after):>11} {fmt(avg_after - avg_before):>11}  "
                     f"{verdict(avg_before, avg_after, direction)} ({details}; {len(usable)}/{len(common)} cases)")
    for case, noise in common:
        key_pair = (case, noise)
        lines += ["", f"{case} | noise {noise}",
                  f"{'Metric':26} {'Before':>11} {'After':>11} {'Change':>11}  Result"]
        for key, title, direction in METRICS:
            a, b = number(before[key_pair].get(key)), number(after[key_pair].get(key))
            diff = None if a is None or b is None else b - a
            lines.append(f"{title:26} {fmt(a):>11} {fmt(b):>11} {fmt(diff):>11}  {verdict(a, b, direction)}")
    lines += ["", "Change = after - before. Lower pose error and ICP time are favorable.",
              "Inliers, source count, fitness, and RMSE are descriptive: they alone don't prove accuracy.",
              "Use identical cases, seeds, settings, and hardware; the CSV does not record all of them.",
              "Timing varies by run; this benchmark truncates its averaged milliseconds to whole numbers."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="testing/icp_room_with_time.csv", help="benchmark CSV")
    parser.add_argument("--before", help="label of earlier benchmark")
    parser.add_argument("--after", help="label of later benchmark")
    parser.add_argument("--list", action="store_true", help="list labels and algorithms")
    parser.add_argument("--output", default="testing/icp_comparison.txt", help="report file to overwrite")
    args = parser.parse_args()
    try:
        with Path(args.input).open(newline="", encoding="utf-8-sig") as file:
            reader = csv.DictReader(file)
            required = {"label", "algorithm", "case", "noise_level"}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError("CSV must contain label, algorithm, case, and noise_level columns.")
            rows = list(reader)
        if args.list:
            counts = Counter((r['label'], r['algorithm']) for r in rows)
            for (label, algorithm), count in sorted(counts.items()):
                print(f"{label} | {algorithm} | {count} rows")
            return
        if not args.before or not args.after:
            parser.error("provide --before and --after, or use --list")
        if args.before == args.after:
            parser.error("--before and --after must be different labels")
        before, ba = index_rows(rows, args.before)
        after, aa = index_rows(rows, args.after)
        report = build_report(args.before, args.after, before, after, ba, aa)
        output = Path(args.output)
        if output.resolve() == Path(args.input).resolve():
            raise ValueError("Report output must differ from the input CSV.")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(report, encoding="utf-8")
        print(report)
        print(f"Saved report to {output}")
    except (OSError, ValueError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == '__main__':
    main()
