import csv
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO

import numpy as np

from map.icp_diagnostics import ICPDiagnostics
from map.kiss_icp_2d import ICPResult, pose_matrix
from realityapi_pb2 import RawLidarScan, Transform, Vector3


class ICPDiagnosticsTest(unittest.TestCase):
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
