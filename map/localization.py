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

    def align_to_prev_scan(self, lidar: Lidar) -> tuple[float, float, float]:
        """
        Wraps around icp for 2D, returning delta angle, x, and z in that order.
        The delta is the transform that aligns this scan onto the previous one.
        All zeros means there was no usable delta (first scan, empty scan, or too few matches).
        """
        if not lidar.points:
            return (0.0, 0.0, 0.0)
        # Only get X and Z columns
        scan = np.array([[point.x, point.z] for point in lidar.points])
        if self.prev_scan is None:
            self.prev_scan = scan
            return (0.0, 0.0, 0.0)
        transformation_history, points = icp(self.prev_scan, scan, max_iterations=100, distance_threshold=0.3, convergence_translation_threshold=1e-3,
        convergence_rotation_threshold=1e-4, point_pairs_threshold=10, verbose=False)
        self.prev_scan = points

        # Chain icp's per-iteration [R | T] steps into one transform
        total = np.eye(3)
        for step in transformation_history:
            total = np.vstack((step, [0, 0, 1])) @ total

        return (float(math.atan2(total[1, 0], total[0, 0])), float(total[0, 2]), float(total[1, 2]))

    def update_correction(self, lidar: Lidar) -> str:
        """
        Match an odometry-placed scan against the previous corrected scan.

        The scan must be in server coordinates, before OFFSET is applied.
        """
        if not lidar.points:
            return "empty_scan"

        # Apply the existing accumulated correction before matching.
        predicted_lidar = self.corrected_lidar(lidar)

        # gets the x, z values for each point
        predicted_scan = np.array([
            [point.x, point.z]
            for point in predicted_lidar.points
        ])

        if self.prev_scan is None:
            self.prev_scan = predicted_scan
            return "first_scan"

        history, aligned_points = icp(
            reference_points=self.prev_scan,
            points=predicted_scan,
            max_iterations=100,
            distance_threshold=0.3,
            point_pairs_threshold=10,
            verbose=False,
        )

        if not history:
            # Keep the last corrected scan and correction matrix.
            return "no_update"

        new_correction = np.eye(3)

        for step in history:
            step_3x3 = np.vstack((step, [0, 0, 1]))
            new_correction = step_3x3 @ new_correction

        # The new correction acts after the existing correction.
        self.correction = new_correction @ self.correction

        # ICP already applied the new correction to the predicted scan.
        self.prev_scan = aligned_points

        return "updated"
    
    def corrected_lidar(
        self,
        lidar: Lidar,
        offset=(0.0, 0.0, 0.0),
    ) -> Lidar:
        corrected_points = []

        for point in lidar.points:
            position = np.array([point.x, point.z, 1.0])
            corrected_position = self.correction @ position

            corrected_points.append(
                Vector3(
                    x=float(corrected_position[0]) + offset[0],
                    y=point.y + offset[1],
                    z=float(corrected_position[1]) + offset[2],
                )
            )

        return Lidar(points=corrected_points)

    def corrected_transform(
        self,
        server_transform: Transform,
        offset
    ) -> Transform:
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

