import unittest
import math
from unittest.mock import patch

from map.localization import ScanLocalizer
from map.kiss_icp_2d import OdometryICP2D
from map.scan_pipeline import register_scan
from server import SERVER_COORDS, TURTLEBOT_COORDS
from testing.test_odometry_icp_2d import scan


class MotionGateTest(unittest.TestCase):
    def test_normal_gate_accumulates_motion_since_accepted_scan(self):
        localizer = ScanLocalizer()
        def run(x, yaw=0):
            return register_scan(localizer, scan(x=x, yaw=yaw), TURTLEBOT_COORDS, SERVER_COORDS)[0]
        self.assertEqual(run(0).status, 'first_scan')
        with patch('map.localization.icp', wraps=__import__('map.icp', fromlist=['icp']).icp) as icp:
            self.assertEqual(run(0.01).status, 'skipped_motion')
            self.assertEqual(run(0.019, yaw=math.radians(4)).status, 'skipped_motion')
            icp.assert_not_called()
            run(0.021)
            icp.assert_called_once()

    def test_kiss_gate_accumulates_motion_since_accepted_scan(self):
        localizer = OdometryICP2D()
        self.assertEqual(localizer.register_scan(scan()).status, 'first_scan')
        with patch('map.kiss_icp_2d.cKDTree', wraps=__import__('scipy.spatial', fromlist=['cKDTree']).cKDTree) as tree:
            self.assertEqual(localizer.register_scan(scan(x=0.01)).status, 'skipped_motion')
            self.assertEqual(localizer.register_scan(scan(x=0.019, yaw=math.radians(4))).status, 'skipped_motion')
            tree.assert_not_called()
            localizer.register_scan(scan(x=0.021))
            tree.assert_called_once()

    def test_rotation_and_angle_wrap(self):
        for kind in ('normal', 'kiss'):
            localizer = ScanLocalizer() if kind == 'normal' else OdometryICP2D()
            def run(yaw):
                item = scan(yaw=math.radians(yaw))
                return (register_scan(localizer, item, TURTLEBOT_COORDS, SERVER_COORDS)[0]
                        if kind == 'normal' else localizer.register_scan(item))
            self.assertEqual(run(179).status, 'first_scan')
            self.assertEqual(run(-179).status, 'skipped_motion')
            self.assertNotEqual(run(-175).status, 'skipped_motion')


if __name__ == '__main__':
    unittest.main()
