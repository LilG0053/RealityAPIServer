#!/usr/bin/env python

"""Replay simulated LiDAR scans, or measure how simplification affects post latency.

Run with no arguments to replay the recorded poses. Pass --latency to sweep
simplification epsilons and write testing/lidar_simplification_latency.png.
"""

import argparse
import math
import random
from pathlib import Path
from time import perf_counter

import numpy as np
from websockets.sync.client import connect

from client_test import URI, build_device_info, exchange
from map.lidar import simplify_lidar_2d
from realityapi_pb2 import Lidar, ServerRequest, Vector3


POINT_COUNT = 550

# Epsilon 0 sends the raw scan. Larger values run simplify_lidar_2d first.
LATENCY_EPSILONS = np.round(np.linspace(0.0, 0.1, 11), 2)
LATENCY_PLOT = Path(__file__).resolve().parent / "lidar_simplification_latency.png"


def build_lidar_scan(transform, scan_number: int, point_count: int = POINT_COUNT) -> Lidar:
    """Create points around the robot in the same coordinate frame as its pose."""
    points = []
    for index in range(point_count):
        angle = 2 * math.pi * index / point_count + transform.rotation.y
        # Vary the distance to make a recognizable, changing test scan.
        distance = 1.5 + 0.3 * math.sin(3 * angle + scan_number * 0.1) + random.uniform(-0.05, 0.05)
        points.append(Vector3(
            x=transform.position.x + distance * math.cos(angle),
            y=transform.position.y,
            z=transform.position.z + distance * math.sin(angle),
        ))
    return Lidar(points=points)


def replay() -> None:
    """Post one raw scan per recorded pose and print each server round trip."""
    devices = build_device_info()
    latencies = []
    with connect(URI) as websocket:
        for scan_number, device in enumerate(devices):
            scan = build_lidar_scan(device.transform, scan_number)
            started = perf_counter()
            device.lidar.CopyFrom(scan)
            reply = exchange(websocket, ServerRequest(post_device_info=device))
            latency_ms = (perf_counter() - started) * 1000
            latencies.append(latency_ms)
            print(
                f"<<< {reply.WhichOneof('data')} | {device.id}: "
                f"{len(device.lidar.points)} LiDAR points | {latency_ms:.1f} ms"
            )
    print(f"Average latency: {sum(latencies) / len(latencies):.1f} ms")


def measure_lidar_latency(
    point_count: int = POINT_COUNT,
    epsilons: np.ndarray | None = None,
    output_path: Path | None = None,
) -> Path:
    """Time simplification plus the server round trip for each epsilon.

    Scan generation is excluded. Epsilon 0 does not call simplify_lidar_2d.
    The saved plot shows the median, 95th, and 99th percentile of those times,
    with the number of points sent on a second y-axis.
    """
    if epsilons is None:
        epsilons = LATENCY_EPSILONS
    if output_path is None:
        output_path = LATENCY_PLOT

    devices = build_device_info()
    medians = []
    p95s = []
    p99s = []
    point_counts = []

    with connect(URI) as websocket:
        for epsilon in epsilons:
            samples = []
            sent_points = []
            for scan_number, device in enumerate(devices):
                scan = build_lidar_scan(device.transform, scan_number, point_count)
                started = perf_counter()
                if epsilon > 0:
                    scan = simplify_lidar_2d(scan, epsilon=float(epsilon))
                device.lidar.CopyFrom(scan)
                exchange(websocket, ServerRequest(post_device_info=device))
                samples.append((perf_counter() - started) * 1000)
                sent_points.append(len(scan.points))

            samples = np.array(samples)
            median, p95, p99 = np.percentile(samples, [50, 95, 99])
            medians.append(median)
            p95s.append(p95)
            p99s.append(p99)
            point_counts.append(int(np.median(sent_points)))
            print(
                f"epsilon {epsilon:.2f} m | {point_counts[-1]} points | "
                f"median {median:.1f} ms | p95 {p95:.1f} ms | p99 {p99:.1f} ms"
            )

    plot_lidar_latency(epsilons, medians, p95s, p99s, point_counts, point_count, output_path)
    print(f"Saved {output_path}")
    return output_path


def plot_lidar_latency(epsilons, medians, p95s, p99s, point_counts, point_count, output_path) -> None:
    """Plot latency percentiles and points sent, using a right-hand axis for points."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.5))
    latency_lines = [
        ax.plot(epsilons, medians, marker="o", label="Median")[0],
        ax.plot(epsilons, p95s, marker="o", label="p95")[0],
        ax.plot(epsilons, p99s, marker="o", label="p99")[0],
    ]
    ax.set_xlabel("Simplification epsilon (m)")
    ax.set_ylabel("Latency (ms)")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)

    points_ax = ax.twinx()
    points_line = points_ax.plot(
        epsilons,
        point_counts,
        color="0.35",
        linestyle="--",
        marker="s",
        label="Points sent",
    )[0]
    points_ax.set_ylabel("Points sent")
    points_ax.set_ylim(bottom=0)

    lines = latency_lines + [points_line]
    ax.legend(lines, [line.get_label() for line in lines], loc="upper right")
    ax.set_title(f"LiDAR post latency for {point_count}-point scans")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--latency",
        action="store_true",
        help="Sweep simplification epsilons and plot post latency",
    )
    args = parser.parse_args()
    if args.latency:
        measure_lidar_latency()
    else:
        replay()


if __name__ == "__main__":
    main()
