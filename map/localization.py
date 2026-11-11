from realityapi_pb2 import Lidar, Transform, Vector3
import math
from dataclasses import dataclass, field
from map.icp import icp
import numpy as np
from map.coordinate import convert_transform

@dataclass
class ScanLocalizer:
    prev_scan: np.ndarray | None = None

    # stores the accumulated correction from the devices odometry
    correction: np.ndarray = field(default_factory=lambda: np.eye(3))
    last_iterations: int = 0
    last_stop_reason: str = 'not_started'
    convergence_counts: dict[str, int] = field(default_factory=dict)
    last_icp_odom_xy: np.ndarray | None = None
    last_icp_odom_yaw: float | None = None

    def update_correction(self, points: np.ndarray) -> tuple[str, np.ndarray]:
        """
        Match an odometry-placed scan against the previous corrected scan.

        The scan must be in server coordinates, before OFFSET is applied.
        """
        self.last_iterations = 0
        if len(points) == 0:
            self.last_stop_reason = 'empty_scan'
            return "empty_scan", points

        # Apply the correction accumulated before this scan.
        predicted_points = self.corrected_lidar(points)

        # gets the x, z values for each point
        predicted_scan = predicted_points[:, [0, 2]]

        if self.prev_scan is None:
            self.last_stop_reason = 'first_scan'
            self.prev_scan = predicted_scan
            return "first_scan", predicted_points

        history, aligned_points, stop_reason = icp(
            reference_points=self.prev_scan,
            points=predicted_scan,
            max_iterations=100,
            distance_threshold=0.3,
            point_pairs_threshold=10,
            verbose=False,
            return_stop_reason=True,
        )
        self.last_iterations = len(history)
        self.last_stop_reason = stop_reason
        self.convergence_counts[stop_reason] = self.convergence_counts.get(stop_reason, 0) + 1

        if not history:
            # Keep the last corrected scan and correction matrix.
            return "no_update", predicted_points

        # Combines history from ICP into one matrix
        new_correction = np.eye(3)

        for step in history:
            step_3x3 = np.vstack((step, [0, 0, 1]))
            new_correction = step_3x3 @ new_correction

        # Updating correction
        self.correction = new_correction @ self.correction

        self.prev_scan = aligned_points

        # updates the x, z values of predicted points with aligned points
        predicted_points[:, [0, 2]] = aligned_points

        return "updated", predicted_points
    
    def corrected_lidar(self, points: np.ndarray) -> np.ndarray:
        """Apply the current correction to server-coordinate points."""
        # gets a copy of the lidar points so raw points array is not changed
        corrected = points.copy()

        # Extract x and z as an (N, 2) array.
        xz = points[:, [0, 2]]

        rotation = self.correction[:2, :2]
        translation = self.correction[:2, 2]

        corrected[:, [0, 2]] = xz @ rotation.T + translation

        # Column 1, the server's vertical y coordinate, stays unchanged.
        return corrected

    # take robot pose in sserver coordinates, applies correction
    def corrected_transform(self, server_transform: Transform, offset) -> Transform:
        corrected = Transform()
        corrected.CopyFrom(server_transform)

        # Apply the accumulated correction in the server x/z plane.
        position = np.array([
            server_transform.position.x,
            server_transform.position.z,
            1.0,
        ])

        corrected_position = self.correction @ position

        corrected.position.x = float(corrected_position[0]) + offset[0]
        corrected.position.y = server_transform.position.y + offset[1]
        corrected.position.z = float(corrected_position[1]) + offset[2]

        # ICP's x/z angle has the opposite sign from rotation about server +y.
        angle_xz = math.atan2(
            self.correction[1, 0],
            self.correction[0, 0],
        )
        corrected.rotation.y = server_transform.rotation.y - angle_xz

        return corrected

