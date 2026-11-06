"""Known-pose checks for odometry-seeded scan-to-map registration."""

import math
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

import server
from map.coordinate import convert_transform
from map.kiss_icp_2d import AdaptiveThreshold, ICPConfig, OdometryICP2D, voxel_downsample
from map.map import Map
from realityapi_pb2 import DeviceInfo, DeviceType, RawLidarScan, Transform, Vector3


ANGLES = np.linspace(-2.4, 2.4, 90)
RANGES = 2.5 + 0.7 * np.sin(1.7 * ANGLES) + 0.2 * np.cos(4 * ANGLES)


def room_scan(true_xy, odom_xy):
    """Ray-cast a rectangle with an off-center obstacle from a real viewpoint."""
    walls = [
        ((-3, -2.5), (3, -2.5)), ((3, -2.5), (3, 2.5)),
        ((3, 2.5), (-3, 2.5)), ((-3, 2.5), (-3, -2.5)),
        ((1.0, 0.5), (1.8, 0.5)), ((1.8, 0.5), (1.8, 1.3)),
        ((1.8, 1.3), (1.0, 1.3)), ((1.0, 1.3), (1.0, 0.5)),
    ]
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    ranges = []
    origin = np.asarray(true_xy)
    for sensor_angle in angles:
        direction = np.array([math.cos(sensor_angle + math.pi / 2),
                              math.sin(sensor_angle + math.pi / 2)])
        nearest = math.inf
        for start, end in walls:
            offset = np.asarray(start, dtype=float) - origin
            edge = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
            denominator = direction[0] * edge[1] - direction[1] * edge[0]
            if abs(denominator) < 1e-12:
                continue
            ray_distance = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator
            along_wall = (offset[0] * direction[1] - offset[1] * direction[0]) / denominator
            if ray_distance > 0 and 0 <= along_wall <= 1:
                nearest = min(nearest, ray_distance)
        ranges.append(nearest)

    return RawLidarScan(
        ranges=ranges, range_min=0.1, range_max=8.0,
        angle_min=float(angles[0]), angle_increment=float(angles[1] - angles[0]),
        transform=Transform(position=Vector3(x=odom_xy[0], y=odom_xy[1])),
    )


def scan(x=0.0, y=0.0, yaw=0.0, ranges=RANGES, device_id="robot"):
    return RawLidarScan(
        id=device_id,
        ranges=ranges,
        range_min=0.1,
        range_max=8.0,
        angle_min=float(ANGLES[0]),
        angle_increment=float(ANGLES[1] - ANGLES[0]),
        transform=Transform(position=Vector3(x=x, y=y), rotation=Vector3(z=yaw)),
    )


class OdometryICP2DTest(unittest.TestCase):
    def test_subsampling_keeps_measured_points_and_adapts_threshold(self):
        points = np.array([[0.02, 0.03], [0.04, 0.05], [0.16, 0.01]])
        np.testing.assert_array_equal(voxel_downsample(points, 0.1), points[[0, 2]])

        threshold = AdaptiveThreshold(ICPConfig())
        first_sigma = threshold.sigma
        correction = np.eye(3)
        correction[0, 2] = 0.3
        threshold.update(np.eye(3), correction)
        self.assertGreater(threshold.sigma, first_sigma)

    def test_known_odometry_error_is_corrected_against_local_map(self):
        localizer = OdometryICP2D()
        first = localizer.register_scan(scan())
        self.assertEqual(first.status, "first_scan")
        self.assertGreater(len(localizer.local_map.points()), 0)

        second = localizer.register_scan(scan(x=0.08, y=-0.05, yaw=0.02))
        self.assertEqual(second.status, "updated")
        np.testing.assert_allclose(second.pose[:2, 2], [0, 0], atol=0.01)
        self.assertAlmostEqual(math.atan2(second.pose[1, 0], second.pose[0, 0]), 0.0, places=2)
        self.assertGreaterEqual(second.inlier_ratio, 0.9)
        self.assertLess(second.inlier_rmse, 0.01)
        corrected_later_odom = localizer.corrected_transform(
            Transform(position=Vector3(x=0.08, y=-0.05), rotation=Vector3(z=0.02))
        )
        self.assertAlmostEqual(corrected_later_odom.position.x, 0.0, places=2)

    def test_no_overlap_keeps_correction_map_and_threshold(self):
        localizer = OdometryICP2D()
        localizer.register_scan(scan())
        before_map = localizer.local_map.points().copy()
        before_sigma = localizer.threshold.sigma

        rejected = localizer.register_scan(scan(x=20.0, y=20.0))
        self.assertEqual(rejected.status, "rejected")
        np.testing.assert_array_equal(localizer.map_from_odom, np.eye(3))
        np.testing.assert_array_equal(localizer.local_map.points(), before_map)
        self.assertEqual(localizer.threshold.sigma, before_sigma)

    def test_moderate_changed_returns_do_not_dominate_pose(self):
        localizer = OdometryICP2D()
        localizer.register_scan(scan())
        moved_returns = RANGES.copy()
        moved_returns[15:30] += 0.18

        result = localizer.register_scan(scan(x=0.08, y=-0.05, yaw=0.02, ranges=moved_returns))
        self.assertEqual(result.status, "updated")
        self.assertLess(np.linalg.norm(result.pose[:2, 2]), 0.03)
        self.assertGreater(result.inlier_ratio, 0.9)

    def test_moving_robot_with_changed_view_recovers_pose(self):
        localizer = OdometryICP2D()
        localizer.register_scan(room_scan((0.0, 0.0), (0.0, 0.0)))
        result = localizer.register_scan(room_scan((0.35, 0.10), (0.43, 0.05)))

        self.assertEqual(result.status, "updated")
        np.testing.assert_allclose(result.pose[:2, 2], [0.35, 0.10], atol=0.02)
        self.assertLess(result.inlier_rmse, 0.05)

    def test_too_few_points_do_not_bootstrap_map(self):
        localizer = OdometryICP2D()
        result = localizer.register_scan(scan(ranges=RANGES[:2]))
        self.assertEqual(result.status, "insufficient_scan")
        self.assertEqual(len(localizer.local_map.points()), 0)

    def test_server_uses_corrected_pose_once_and_preserves_it_on_odom_updates(self):
        previous_map = server.server_map
        previous_localizers = server.scan_localizers
        previous_directory = Path.cwd()
        with tempfile.TemporaryDirectory() as temporary:
            try:
                os.chdir(temporary)
                Path("dashboard").mkdir()
                server.server_map = Map()
                server.scan_localizers = {}
                identity = scan().transform
                first_server_pose = convert_transform(identity, server.TURTLEBOT_COORDS, server.SERVER_COORDS)
                server.post_device_info(DeviceInfo(
                    id="robot", device_type=DeviceType.TURTLE, transform=first_server_pose
                ))
                server.handle_raw_lidar_scan(scan())
                server.handle_raw_lidar_scan(scan(x=0.08, y=-0.05, yaw=0.02))

                corrected = server.server_map.get_device_info("robot").device_info.transform
                self.assertAlmostEqual(corrected.position.x, first_server_pose.position.x, places=2)
                self.assertAlmostEqual(corrected.position.z, first_server_pose.position.z, places=2)

                raw_server_pose = convert_transform(
                    scan(x=0.08, y=-0.05, yaw=0.02).transform,
                    server.TURTLEBOT_COORDS, server.SERVER_COORDS,
                )
                server.post_device_info(DeviceInfo(id="robot", transform=raw_server_pose))
                still_corrected = server.server_map.get_device_info("robot").device_info.transform
                self.assertAlmostEqual(still_corrected.position.x, first_server_pose.position.x, places=2)
                self.assertAlmostEqual(still_corrected.position.z, first_server_pose.position.z, places=2)
            finally:
                server.server_map = previous_map
                server.scan_localizers = previous_localizers
                os.chdir(previous_directory)


if __name__ == "__main__":
    unittest.main()
