import csv
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO

import numpy as np

from map.icp_diagnostics import ICPDiagnostics
from map.icp import icp
from map.kiss_icp_2d import ICPResult, pose_matrix
from realityapi_pb2 import RawLidarScan, Transform, Vector3


class ICPDiagnosticsTest(unittest.TestCase):
    def test_icp_stop_reasons_are_distinct(self):
        points = np.array([[0., 0.], [1., 0.], [0., 1.]])
        _, _, reason = icp(points, points.copy(), point_pairs_threshold=3, return_stop_reason=True)
        self.assertEqual(reason, 'converged')
        _, _, reason = icp(points, points.copy(), point_pairs_threshold=3, max_iterations=1,
                           convergence_translation_threshold=0, return_stop_reason=True)
        self.assertEqual(reason, 'max_iterations')
        _, _, reason = icp(points, points + 10, point_pairs_threshold=3, return_stop_reason=True)
        self.assertEqual(reason, 'insufficient_pairs')
        self.assertEqual(len(icp(points, points.copy(), point_pairs_threshold=3)), 2)

    def test_rotation_totals_handle_wrap_and_direction_changes(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(StringIO()):
            recorder = ICPDiagnostics(directory)
            for degrees in (170, -170, 170):
                transform = Transform(rotation=Vector3(z=math.radians(degrees)))
                scan = RawLidarScan(id='robot', transform=transform)
                pose = pose_matrix(transform)
                result = ICPResult('updated', pose, np.empty((0, 2)), 0, 0.0, 0.0, 1.0, 1)
                recorder.write(scan, result, pose, 1.0, 0)
            recorder.close()
            with recorder.csv_path.open(newline='') as file:
                row = list(csv.DictReader(file))[-1]
            self.assertAlmostEqual(float(row['odom_total_rotation_deg']), 40, places=3)
            self.assertAlmostEqual(float(row['corrected_total_rotation_deg']), 40, places=3)
            self.assertAlmostEqual(float(row['odom_signed_rotation_deg']), 0, places=3)
            self.assertAlmostEqual(float(row['odom_heading_from_first_deg']), 0, places=3)

    def test_trial_names_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(StringIO()):
            first = ICPDiagnostics(directory)
            first.close()
            second = ICPDiagnostics(directory)
            second.close()
            named = ICPDiagnostics(directory, name='corridor-test')
            named.close()
            self.assertEqual(first.csv_path.name, 'trial_1.csv')
            self.assertEqual(second.recording_path.name, 'trial_2.jsonl')
            self.assertEqual(named.csv_path.name, 'corridor-test.csv')
            with self.assertRaises(FileExistsError):
                ICPDiagnostics(directory, name='corridor-test')
            with self.assertRaises(ValueError):
                ICPDiagnostics(directory, name='../outside')

    def test_path_length_displacement_and_raw_recording(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(StringIO()) as output:
            recorder = ICPDiagnostics(directory)
            for (ox, oy), (cx, cy) in zip([(0, 0), (3, 0), (3, 4)],
                                           [(0, 0), (0, 2), (0, 5)]):
                scan = RawLidarScan(id='robot', transform=Transform(position=Vector3(x=ox, y=oy)))
                corrected = pose_matrix(Transform(position=Vector3(x=cx, y=cy)))
                result = ICPResult('updated', corrected, np.empty((0, 2)), 0, 0.0, 0.0, 1.0, 1)
                stamp = recorder.record_scan(scan)
                recorder.write(scan, result, pose_matrix(scan.transform), 1.0, 0, received_utc=stamp)
            recorder.close()
            with recorder.csv_path.open(newline='') as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[-1]['odom_path_length_m']), 7.0)
            self.assertEqual(rows[0]['odom_step_m'], '')
            self.assertAlmostEqual(float(rows[-1]['odom_step_m']), 4.0)
            self.assertAlmostEqual(float(rows[-1]['corrected_step_m']), 3.0)
            self.assertAlmostEqual(float(rows[-1]['corrected_path_length_m']), 5.0)
            self.assertAlmostEqual(float(rows[-1]['odom_distance_from_first_m']), 5.0)
            self.assertAlmostEqual(float(rows[-1]['pose_offset_distance_m']), np.sqrt(10))
            self.assertEqual(len(recorder.recording_path.read_text().splitlines()), 3)
            self.assertIn('odom=7.000 m, corrected=5.000 m', output.getvalue())

    def test_unregistered_scan_is_recorded_without_metrics(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(StringIO()) as output:
            recorder = ICPDiagnostics(directory)
            recorder.record_scan(RawLidarScan(id='unknown'), registered=False)
            recorder.close()
            item = json.loads(recorder.recording_path.read_text())
            self.assertFalse(item['device_registered'])
            self.assertIn('none processed', output.getvalue())
            with recorder.csv_path.open(newline='') as file:
                self.assertEqual(list(csv.DictReader(file)), [])


if __name__ == '__main__':
    unittest.main()
