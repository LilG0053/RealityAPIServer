"""Odometry-seeded, scan-to-local-map point-to-point ICP for planar LiDAR.

This is a 2D Python adaptation of KISS-ICP's registration pipeline. Odometry
replaces constant-velocity prediction; individual LiDAR beams are not deskewed.
All poses and local-map points use ROS odom XY coordinates, before the server's
display-coordinate conversion.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from realityapi_pb2 import RawLidarScan, Transform, Vector3


@dataclass(frozen=True)
class ICPConfig:
    voxel_size: float = 0.10
    local_radius: float = 8.0
    max_scan_range: float = 8.0
    max_points_per_voxel: int = 5
    initial_sigma: float = 0.20
    min_model_deviation: float = 0.05
    min_pairs: int = 10
    min_inlier_ratio: float = 0.35
    max_inlier_rmse: float = 0.20
    max_iterations: int = 30
    translation_tolerance: float = 1e-4
    rotation_tolerance: float = 1e-4

    def __post_init__(self) -> None:
        if (self.voxel_size <= 0 or self.local_radius <= 0
                or self.max_scan_range <= 0 or self.initial_sigma <= 0):
            raise ValueError("voxel_size, local_radius, max_scan_range, and initial_sigma must be positive")
        if self.max_points_per_voxel < 1 or self.min_pairs < 3 or self.max_iterations < 1:
            raise ValueError("map capacity, min_pairs, and max_iterations must be positive")
        if not 0 <= self.min_inlier_ratio <= 1 or self.max_inlier_rmse <= 0:
            raise ValueError("invalid registration quality threshold")


@dataclass(frozen=True)
class ICPResult:
    status: str
    pose: np.ndarray
    points: np.ndarray
    inlier_count: int
    inlier_ratio: float
    inlier_rmse: float
    max_correspondence_distance: float
    iterations: int

    @property
    def accepted(self) -> bool:
        return self.status in ("first_scan", "updated")


def pose_matrix(transform: Transform) -> np.ndarray:
    """Convert a ROS odom/base_link pose into a 3x3 planar rigid transform."""
    yaw = float(transform.rotation.z)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, transform.position.x],
        [s, c, transform.position.y],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """Map local row-vector points through a column-vector pose matrix."""
    return points @ pose[:2, :2].T + pose[:2, 2]


def scan_to_robot_points(raw_scan: RawLidarScan, max_scan_range: float | None = None) -> np.ndarray:
    """Return valid scan endpoints in base_link XY, without applying odometry."""
    ranges = np.asarray(raw_scan.ranges, dtype=float)
    beam_indices = np.arange(len(ranges))
    upper_range = min(raw_scan.range_max, max_scan_range) if max_scan_range is not None else raw_scan.range_max
    valid = np.isfinite(ranges) & (ranges > raw_scan.range_min) & (ranges < upper_range)
    angles = raw_scan.angle_min + beam_indices[valid] * raw_scan.angle_increment + math.pi / 2
    distances = ranges[valid]
    return np.column_stack((distances * np.cos(angles), distances * np.sin(angles)))


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep the first measured point in each occupied square, without rounding it."""
    if not len(points):
        return points.copy()
    cells = np.floor(points / voxel_size).astype(np.int64)
    _, first_indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first_indices)]


class LocalPointMap:
    """A bounded 2D voxel hash map; the room-wide map lives in map.Map."""

    def __init__(self, config: ICPConfig):
        self.config = config
        self.voxels: dict[tuple[int, int], list[np.ndarray]] = {}

    def points(self) -> np.ndarray:
        return np.asarray(
            [point for bucket in self.voxels.values() for point in bucket], dtype=float
        ).reshape(-1, 2)

    def update(self, local_points: np.ndarray, pose: np.ndarray) -> None:
        spacing = self.config.voxel_size / math.sqrt(self.config.max_points_per_voxel)
        for point in transform_points(local_points, pose):
            cell = tuple(np.floor(point / self.config.voxel_size).astype(int))
            bucket = self.voxels.setdefault(cell, [])
            if len(bucket) < self.config.max_points_per_voxel and all(
                np.linalg.norm(point - old) >= spacing for old in bucket
            ):
                bucket.append(point.copy())

        origin = pose[:2, 2]
        for cell, bucket in list(self.voxels.items()):
            if np.linalg.norm(bucket[0] - origin) >= self.config.local_radius:
                del self.voxels[cell]


class AdaptiveThreshold:
    """Estimate a correspondence cutoff from prior odometry/ICP disagreement."""

    def __init__(self, config: ICPConfig):
        self.config = config
        self.sum_squared_error = config.initial_sigma**2
        self.sample_count = 1

    @property
    def sigma(self) -> float:
        return math.sqrt(self.sum_squared_error / self.sample_count)

    def update(self, predicted_pose: np.ndarray, corrected_pose: np.ndarray) -> None:
        # Express the deviation in the predicted robot frame, as in KISS-ICP.
        deviation = np.linalg.inv(predicted_pose) @ corrected_pose
        angle = math.atan2(deviation[1, 0], deviation[0, 0])
        rotation_displacement = 2 * self.config.max_scan_range * abs(math.sin(angle / 2))
        error = np.linalg.norm(deviation[:2, 2]) + rotation_displacement
        if error > self.config.min_model_deviation:
            self.sum_squared_error += error**2
            self.sample_count += 1


def weighted_rigid_fit(source: np.ndarray, target: np.ndarray, weights: np.ndarray) -> np.ndarray | None:
    """Find the 2D rigid transform that minimizes weighted point-pair error."""
    weight_sum = weights.sum()
    if weight_sum <= 0:
        return None
    source_center = np.sum(source * weights[:, None], axis=0) / weight_sum
    target_center = np.sum(target * weights[:, None], axis=0) / weight_sum
    centered_source = source - source_center
    centered_target = target - target_center
    covariance = (centered_source * weights[:, None]).T @ centered_target
    if np.linalg.svd(covariance, compute_uv=False)[0] < 1e-10:
        return None

    u, _, vt = np.linalg.svd(covariance)
    orientation = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ np.diag([1.0, orientation]) @ u.T
    translation = target_center - rotation @ source_center
    correction = np.eye(3)
    correction[:2, :2] = rotation
    correction[:2, 2] = translation
    return correction


class OdometryICP2D:
    """Own one robot's local map and odom-to-corrected-map transform."""

    def __init__(self, config: ICPConfig | None = None):
        self.config = config or ICPConfig()
        self.local_map = LocalPointMap(self.config)
        self.threshold = AdaptiveThreshold(self.config)
        self.map_from_odom = np.eye(3)

    def corrected_transform(self, odom_transform: Transform) -> Transform:
        """Apply the latest accepted map/odom correction to any odometry pose."""
        pose = self.map_from_odom @ pose_matrix(odom_transform)
        return Transform(
            position=Vector3(x=float(pose[0, 2]), y=float(pose[1, 2]), z=odom_transform.position.z),
            rotation=Vector3(z=float(math.atan2(pose[1, 0], pose[0, 0]))),
        )

    def register_scan(self, raw_scan: RawLidarScan) -> ICPResult:
        local_points = scan_to_robot_points(raw_scan, self.config.max_scan_range)
        odom_pose = pose_matrix(raw_scan.transform)
        predicted_pose = self.map_from_odom @ odom_pose
        max_distance = 3 * self.threshold.sigma
        if not len(local_points):
            return ICPResult("empty_scan", predicted_pose, np.empty((0, 2)), 0, 0.0, math.inf, max_distance, 0)

        # The finer set is inserted into the map; the coarser set is used for ICP.
        merge_points = voxel_downsample(local_points, 0.5 * self.config.voxel_size)
        source_points = voxel_downsample(merge_points, 1.5 * self.config.voxel_size)
        if len(source_points) < self.config.min_pairs:
            return ICPResult("insufficient_scan", predicted_pose,
                             transform_points(local_points, predicted_pose),
                             0, 0.0, math.inf, max_distance, 0)
        map_points = self.local_map.points()
        if not len(map_points):
            self.local_map.update(merge_points, predicted_pose)
            return ICPResult("first_scan", predicted_pose, transform_points(local_points, predicted_pose),
                             0, 0.0, math.nan, max_distance, 0)

        tree = cKDTree(map_points)
        pose = predicted_pose.copy()
        placed = transform_points(source_points, pose)
        iterations = 0
        for iterations in range(1, self.config.max_iterations + 1):
            distances, indices = tree.query(placed)
            inliers = distances < max_distance
            if np.count_nonzero(inliers) < self.config.min_pairs:
                break

            source = placed[inliers]
            target = map_points[indices[inliers]]
            residual_squared = np.sum((source - target)**2, axis=1)
            # Geman-McClure weights reduce the pull of mismatched but accepted pairs.
            # Match the Geman-McClure IRLS weight used by upstream Registration.cpp.
            kernel_scale = self.threshold.sigma
            weights = (kernel_scale / (kernel_scale + residual_squared))**2
            correction = weighted_rigid_fit(source, target, weights)
            if correction is None:
                break
            placed = transform_points(placed, correction)
            pose = correction @ pose
            angle = math.atan2(correction[1, 0], correction[0, 0])
            if (np.linalg.norm(correction[:2, 2]) < self.config.translation_tolerance
                    and abs(angle) < self.config.rotation_tolerance):
                break

        # Recompute quality after the final correction, not from stale ICP pairs.
        distances, _ = tree.query(placed)
        inliers = distances < max_distance
        count = int(np.count_nonzero(inliers))
        ratio = count / len(source_points)
        rmse = float(np.sqrt(np.mean(distances[inliers]**2))) if count else math.inf
        accepted = (count >= self.config.min_pairs
                    and ratio >= self.config.min_inlier_ratio
                    and rmse <= self.config.max_inlier_rmse)
        if not accepted:
            return ICPResult("rejected", predicted_pose, transform_points(local_points, predicted_pose),
                             count, ratio, rmse, max_distance, iterations)

        self.map_from_odom = pose @ np.linalg.inv(odom_pose)
        self.threshold.update(predicted_pose, pose)
        self.local_map.update(merge_points, pose)
        return ICPResult("updated", pose, transform_points(local_points, pose),
                         count, ratio, rmse, max_distance, iterations)
