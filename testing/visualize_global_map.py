"""Build a synthetic global map over time for the dashboard demo.

Run from the project root: python -m testing.visualize_global_map
The demo writes dashboard/global_map_demo.json, not the live map file.
"""

import argparse
import math
from pathlib import Path
import time

import numpy as np

from map.localization import ScanLocalizer
from map.map import Map
from realityapi_pb2 import Lidar, Vector3


def simulated_room():
    """Create asymmetric walls and furniture inside the dashboard's room."""
    segments = [((1.0, 1.0), (4.0, 1.0)), ((4.0, 1.0), (4.0, 4.0)),
                ((4.0, 4.0), (1.0, 4.0)), ((1.0, 4.0), (1.0, 1.0)),
                ((1.7, 2.1), (2.7, 2.1)), ((2.7, 2.1), (2.7, 2.8))]
    points = []
    for start, end in segments:
        start, end = np.array(start), np.array(end)
        for fraction in np.linspace(0, 1, 36, endpoint=False):
            points.append(start + fraction * (end - start))
    return np.array(points)


def to_lidar(points):
    # Lift the synthetic points so they are visible above the dashboard floor.
    return Lidar(points=[Vector3(x=float(x), y=0.08, z=float(z)) for x, z in points])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval', type=float, default=2.0, help='seconds between stages')
    parser.add_argument('--output', default='dashboard/global_map_demo.json', help='demo JSON output')
    args = parser.parse_args()
    if args.interval < 0 or not math.isfinite(args.interval):
        parser.error('--interval must be a finite nonnegative number')

    target = Path(args.output)
    if target.resolve() == Path('dashboard/global_map.json').resolve():
        parser.error('use a demo output path, not the live global map')
    target.parent.mkdir(parents=True, exist_ok=True)
    reference = simulated_room()
    localizer = ScanLocalizer()
    point_map = Map()

    # Every stage retains earlier features and reveals more of the room.
    # Each scan has a known position error in the odometry-placed points.
    print(f'Writing demonstration map to {target}', flush=True)
    for stage in range(6):
        visible = reference[:36 * (stage + 1)]
        error = np.array([0.006 * stage, -0.004 * stage])
        scan = to_lidar(visible + error)
        status = localizer.update_correction(scan)
        if status not in ('first_scan', 'updated'):
            print(f'Stage {stage + 1}: {status}; scan not inserted', flush=True)
            continue
        corrected_scan = localizer.corrected_lidar(scan)
        actual = np.array([[p.x, p.z] for p in corrected_scan.points])
        mean_error = float(np.mean(np.linalg.norm(actual - visible, axis=1)))
        point_map.update_point_map(corrected_scan)
        point_map.export_global_points(str(target))
        count = sum(len(points) for points in point_map.point_map.values())
        print(f'Stage {stage + 1}/6: {status}; scan={len(visible)} points; '
              f'map={count} points; mean known-point error={mean_error:.4f} m', flush=True)
        if stage < 5:
            time.sleep(args.interval)
    print('Demo complete. This tests synthetic scans, not live robot behavior.', flush=True)


if __name__ == '__main__':
    main()
