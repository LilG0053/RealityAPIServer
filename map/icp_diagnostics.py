"""Optional recording and diagnostics for the live 2D LiDAR localizer."""

import base64
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re

import numpy as np

def pose_matrix(transform):
    """Convert a ROS planar pose to a homogeneous matrix for reporting."""
    yaw = float(transform.rotation.z)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, transform.position.x],
                     [s, c, transform.position.y], [0.0, 0.0, 1.0]])


FIELDS = ['scan', 'received_utc', 'device_id', 'status', 'accepted', 'valid_points',
          'inlier_count', 'inlier_ratio', 'inlier_rmse_m', 'max_correspondence_distance_m',
          'iterations', 'stop_reason', 'converged', 'registration_time_ms', 'correction_translation_m',
          'correction_rotation_deg', 'corrected_x_m', 'corrected_y_m', 'corrected_yaw_deg',
          'odom_x_m', 'odom_y_m', 'odom_yaw_deg', 'pose_offset_x_m', 'pose_offset_y_m',
          'pose_offset_distance_m', 'pose_offset_yaw_deg', 'distance_from_first_m', 'heading_from_first_deg',
          'odom_distance_from_first_m', 'odom_path_length_m', 'corrected_path_length_m',
          'odom_step_m', 'corrected_step_m',
          'odom_heading_from_first_deg', 'odom_total_rotation_deg', 'corrected_total_rotation_deg',
          'odom_signed_rotation_deg', 'corrected_signed_rotation_deg', 'reference_points']


def finite_or_none(value):
    return float(value) if math.isfinite(value) else None


class ICPDiagnostics:
    """Keep a replayable recording and a readable per-scan CSV together."""

    def __init__(self, directory, name=None):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        if name is not None and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', name):
            raise ValueError('Trial name must contain only letters, numbers, underscores or hyphens.')
        trial = 1
        while True:
            trial_name = name if name is not None else f'trial_{trial}'
            self.recording_path = directory / (trial_name + '.jsonl')
            self.csv_path = directory / (trial_name + '.csv')
            if self.csv_path.exists():
                if name is not None:
                    raise FileExistsError(f'Trial already exists: {trial_name}')
                trial += 1
                continue
            try:
                self.recording = self.recording_path.open('x', encoding='utf-8')
                break
            except FileExistsError:
                if name is not None:
                    raise FileExistsError(f'Trial already exists: {trial_name}')
                trial += 1
        try:
            self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
        except Exception:
            self.recording.close()
            self.recording_path.unlink()
            raise
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()
        self.first_poses = {}
        self.first_offsets = {}
        self.latest_offsets = {}
        self.first_odometry = {}
        self.previous_poses = {}
        self.path_lengths = {}
        self.rotations = {}
        self.received_scans = 0
        self.unregistered_scans = 0
        self.counts = {}
        print(f'ICP recording: {self.recording_path}\nLive ICP metrics: {self.csv_path}', flush=True)

    def record_scan(self, scan, registered=True):
        """Record LiDAR scan arrivals with the scan information to be used for replay"""
        stamp = datetime.now(timezone.utc).isoformat()
        self.recording.write(json.dumps({
            'received_utc': stamp,
            'device_registered': registered,
            'scan_b64': base64.b64encode(scan.SerializeToString()).decode('ascii'),
        }) + '\n')
        self.recording.flush()
        self.received_scans += 1
        if not registered:
            self.unregistered_scans += 1
            print(f'ICP recorder: scan from {scan.id!r} received, but device is not registered.', flush=True)
        elif self.received_scans == 1:
            print(f'ICP recorder: first scan received from {scan.id!r}.', flush=True)
        return stamp

    def write(self, scan, result, predicted_pose, elapsed_ms, local_map_points, received_utc=None):
        stamp = received_utc if received_utc is not None else self.record_scan(scan)
        pose = result.pose
        odom = pose_matrix(scan.transform)
        # Compare pose origins; correction matrix translation alone depends on origin.
        displacement = pose[:2, 2] - predicted_pose[:2, 2]
        angle = math.atan2(pose[1, 0], pose[0, 0])
        odom_angle = math.atan2(odom[1, 0], odom[0, 0])
        offset_xy = pose[:2, 2] - odom[:2, 2]
        offset_angle = math.atan2(math.sin(angle - odom_angle), math.cos(angle - odom_angle))
        offset = (float(offset_xy[0]), float(offset_xy[1]),
                  float(np.linalg.norm(offset_xy)), math.degrees(offset_angle))
        self.first_offsets.setdefault(scan.id, offset)
        self.latest_offsets[scan.id] = offset
        predicted_angle = math.atan2(predicted_pose[1, 0], predicted_pose[0, 0])
        delta_angle = math.atan2(math.sin(angle - predicted_angle), math.cos(angle - predicted_angle))
        first = self.first_poses.setdefault(scan.id, pose.copy())
        first_odom = self.first_odometry.setdefault(scan.id, odom.copy())
        previous = self.previous_poses.get(scan.id)
        odom_step_m = corrected_step_m = None
        odom_path, corrected_path = self.path_lengths.get(scan.id, (0.0, 0.0))
        odom_signed, corrected_signed, odom_total, corrected_total = self.rotations.get(scan.id, (0.0, 0.0, 0.0, 0.0))
        if previous is not None:
            previous_odom, previous_corrected = previous
            odom_step_m = float(np.linalg.norm(odom[:2, 2] - previous_odom[:2, 2]))
            corrected_step_m = float(np.linalg.norm(pose[:2, 2] - previous_corrected[:2, 2]))
            odom_path += odom_step_m
            corrected_path += corrected_step_m
            previous_odom_angle = math.atan2(previous_odom[1, 0], previous_odom[0, 0])
            previous_corrected_angle = math.atan2(previous_corrected[1, 0], previous_corrected[0, 0])
            odom_step = math.atan2(math.sin(odom_angle - previous_odom_angle), math.cos(odom_angle - previous_odom_angle))
            corrected_step = math.atan2(math.sin(angle - previous_corrected_angle), math.cos(angle - previous_corrected_angle))
            odom_signed += odom_step
            corrected_signed += corrected_step
            odom_total += abs(odom_step)
            corrected_total += abs(corrected_step)
        self.path_lengths[scan.id] = (odom_path, corrected_path)
        self.rotations[scan.id] = (odom_signed, corrected_signed, odom_total, corrected_total)
        self.previous_poses[scan.id] = (odom.copy(), pose.copy())
        first_angle = math.atan2(first[1, 0], first[0, 0])
        heading_delta = math.atan2(math.sin(angle - first_angle), math.cos(angle - first_angle))
        first_odom_angle = math.atan2(first_odom[1, 0], first_odom[0, 0])
        odom_heading_delta = math.atan2(math.sin(odom_angle - first_odom_angle), math.cos(odom_angle - first_odom_angle))
        count = self.counts.get(scan.id, 0) + 1
        self.counts[scan.id] = count
        row = {
            'scan': count, 'received_utc': stamp, 'device_id': scan.id,
            'status': result.status, 'accepted': result.accepted,
            'valid_points': len(result.points), 'inlier_count': result.inlier_count,
            'inlier_ratio': result.inlier_ratio if result.status not in ('first_scan', 'empty_scan', 'insufficient_scan', 'skipped_motion') else None,
            'inlier_rmse_m': finite_or_none(result.inlier_rmse),
            'max_correspondence_distance_m': result.max_correspondence_distance,
            'iterations': result.iterations, 'registration_time_ms': elapsed_ms,
            'stop_reason': getattr(result, 'stop_reason', None),
            'converged': (getattr(result, 'stop_reason') == 'converged'
                          if getattr(result, 'stop_reason', None) not in (None, 'first_scan', 'empty_scan') else None),
            'correction_translation_m': float(np.linalg.norm(displacement)),
            'correction_rotation_deg': math.degrees(delta_angle),
            'corrected_x_m': pose[0, 2], 'corrected_y_m': pose[1, 2],
            'corrected_yaw_deg': math.degrees(angle), 'odom_x_m': odom[0, 2],
            'odom_y_m': odom[1, 2], 'odom_yaw_deg': math.degrees(odom_angle),
            'pose_offset_x_m': offset[0], 'pose_offset_y_m': offset[1],
            'pose_offset_distance_m': offset[2], 'pose_offset_yaw_deg': offset[3],
            'distance_from_first_m': float(np.linalg.norm(pose[:2, 2] - first[:2, 2])),
            'heading_from_first_deg': math.degrees(heading_delta),
            'odom_distance_from_first_m': float(np.linalg.norm(odom[:2, 2] - first_odom[:2, 2])),
            'odom_path_length_m': odom_path, 'corrected_path_length_m': corrected_path,
            'odom_step_m': odom_step_m, 'corrected_step_m': corrected_step_m,
            'odom_heading_from_first_deg': math.degrees(odom_heading_delta),
            'odom_total_rotation_deg': math.degrees(odom_total),
            'corrected_total_rotation_deg': math.degrees(corrected_total),
            'odom_signed_rotation_deg': math.degrees(odom_signed),
            'corrected_signed_rotation_deg': math.degrees(corrected_signed),
            'reference_points': local_map_points,
        }
        self.writer.writerow(row)
        self.csv_file.flush()
        if count == 1 or count % 10 == 0 or (not result.accepted and result.status != 'skipped_motion'):
            print(f"ICP {scan.id}: #{count} {result.status}, fitness={row['inlier_ratio']}, "
                  f"rmse={row['inlier_rmse_m']}, {elapsed_ms:.1f} ms, "
                  f"travel: odom={odom_path:.3f} m, corrected={corrected_path:.3f} m; "
                  f"corrected distance from start={row['distance_from_first_m']:.3f} m", flush=True)

    def close(self):
        self.recording.close()
        self.csv_file.close()
        if self.received_scans == 0:
            print('ICP recorder: no raw LiDAR scans reached this server while recording.', flush=True)
        elif not self.counts:
            print(f'ICP recorder: {self.received_scans} scans received, none processed '
                  f'({self.unregistered_scans} from unregistered devices).', flush=True)
        for device_id, initial in self.first_offsets.items():
            final = self.latest_offsets[device_id]
            odom_path, corrected_path = self.path_lengths[device_id]
            last_odom, last_pose = self.previous_poses[device_id]
            odom_displacement = np.linalg.norm(last_odom[:2, 2] - self.first_odometry[device_id][:2, 2])
            corrected_displacement = np.linalg.norm(last_pose[:2, 2] - self.first_poses[device_id][:2, 2])
            print(f'ICP {device_id} corrected pose relative to odometry (ROS x/y):\n'
                  f'  first: x={initial[0]:+.3f} m, y={initial[1]:+.3f} m, '
                  f'distance={initial[2]:.3f} m, yaw={initial[3]:+.2f} deg\n'
                  f'  last:  x={final[0]:+.3f} m, y={final[1]:+.3f} m, '
                  f'distance={final[2]:.3f} m, yaw={final[3]:+.2f} deg\n'
                  f'  estimated travel: odom={odom_path:.3f} m, corrected={corrected_path:.3f} m\n'
                  f'  straight-line distance from start: odom={odom_displacement:.3f} m, '
                  f'corrected={corrected_displacement:.3f} m', flush=True)
