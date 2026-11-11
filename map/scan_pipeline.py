"""Convert raw scans and run the project's scan-to-scan localizer."""

from types import SimpleNamespace

import numpy as np
from scipy.spatial import cKDTree

from map.coordinate import convert_point_array, convert_transform
from map.icp_diagnostics import pose_matrix
from map.lidar import scan_to_world_points
from map.motion_gate import below_motion_threshold
from map.localization import ScanLocalizer


def register_scan(localizer: ScanLocalizer, raw_scan, robot_coords, server_coords, *, include_point_matches=False):
    """Return corrected server points and ROS-frame metrics for one raw scan."""
    # converts the raw robot transform into its predicted transform
    raw_server_pose = convert_transform(raw_scan.transform, robot_coords, server_coords)
    # predicted pose in server coordinates
    predicted_server_pose = localizer.corrected_transform(raw_server_pose, (0, 0, 0))

    # gets the predicted pose in robot coordinates
    predicted_ros_pose = convert_transform(predicted_server_pose, server_coords, robot_coords)

    # turns the raw scan into 2D NumPy array or LiDAR points
    # scan_to_world_points applies the robot's movement to each point
    rob_world_points = scan_to_world_points(raw_scan)
    server_points = convert_point_array(rob_world_points, robot_coords, server_coords)

    reference = localizer.prev_scan

    odom_xy = np.array([raw_scan.transform.position.x, raw_scan.transform.position.y])
    odom_yaw = float(raw_scan.transform.rotation.z)

    # motion gate, if the robot doesn't move do not run
    if below_motion_threshold(odom_xy, odom_yaw, localizer.last_icp_odom_xy, localizer.last_icp_odom_yaw):
        status, corrected_points = 'skipped_motion', localizer.corrected_lidar(server_points)
        localizer.last_iterations = 0
        localizer.last_stop_reason = 'skipped_motion'
    else:
        status, corrected_points = localizer.update_correction(server_points)
        if status in ('first_scan', 'updated'):
            localizer.last_icp_odom_xy = odom_xy.copy()
            localizer.last_icp_odom_yaw = odom_yaw
    corrected_server_pose = localizer.corrected_transform(raw_server_pose, (0, 0, 0))
    corrected_ros_pose = convert_transform(corrected_server_pose, server_coords, robot_coords)
    
    # comapres the previous scan to the new one, evaluates how well its alignment including rmse, fitness, etc.
    count, ratio, rmse = 0, 0.0, float('inf')
    point_matches = None
    if status != 'skipped_motion' and reference is not None and len(reference) and len(corrected_points):
        distances, indices = cKDTree(reference).query(corrected_points[:, [0, 2]])
        inliers = distances < 0.3
        count = int(np.count_nonzero(inliers))
        ratio = count / len(corrected_points)
        if count:
            rmse = float(np.sqrt(np.mean(distances[inliers] ** 2)))
        if include_point_matches:
            point_matches = (corrected_points[:, [0, 2]], reference, distances, indices)

    result = SimpleNamespace(
        status=status, accepted=status in ('first_scan', 'updated'),
        pose=pose_matrix(corrected_ros_pose), points=corrected_points,
        inlier_count=count, inlier_ratio=ratio, inlier_rmse=rmse,
        max_correspondence_distance=0.3, iterations=localizer.last_iterations,
        stop_reason=localizer.last_stop_reason,
        point_matches=point_matches,
    )
    return result, pose_matrix(predicted_ros_pose), corrected_server_pose, len(reference) if reference is not None else 0
