"""Localization of a robot that drives around the room with drifting odometry.

MonteCarloTest exercises map/montecarlo.py on its own. TurtleBotHandshakeTest
runs the whole TurtleBot flow: the robot's on-device correction
(turtlebot/turtlebot/init_turtlebot.py) talking to the server's Map, as
coordinate_node.py does, with branch and bound and Monte Carlo on the server.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "turtlebot" / "turtlebot"))

import sim
from init_turtlebot import TurtleBotPose

from map.algorithms.montecarlo import MonteCarlo
from map.device.turtlebot import LIDAR_OFFSET
from map.map import Map
from map.wallgrid import WallGrid
from realityapi_pb2 import DeviceInfo, DeviceType, RawLidar2Scan

PATH = sim.true_path()
# Walls only, so the saved occupancy grid does not change the result.
WALLS = WallGrid(sim.ROOM)


class MonteCarloTest(unittest.TestCase):
    def test_tracks_loop_from_known_start_where_odometry_drifts(self):
        rng = np.random.default_rng(1)
        odom = sim.drifting_odometry(PATH, (0.0, 0.0, 0.0), rng)
        mc = MonteCarlo(WALLS, seed=2)
        mc.initialize(PATH[0])

        worst = 0.0
        for k in range(1, len(PATH)):
            mc.predict(odom[k - 1], odom[k])
            pose = mc.update(sim.body_scan(PATH[k]))
            worst = max(worst, math.dist(pose[:2], PATH[k][:2]))

        end = PATH[-1]
        # Dead reckoning: odometry started at (0, 0, 0), anchored at the true start.
        odom_end = sim.compose(PATH[0], odom[-1])
        self.assertGreater(math.dist(odom_end[:2], end[:2]), 0.3)
        self.assertLess(worst, 0.15)
        self.assertLess(math.dist(mc.pose[:2], end[:2]), 0.08)
        self.assertLess(sim.heading_error(mc.pose[2], end[2]), 3.0)
        self.assertGreater(mc.fit, 0.5)
        # Once confident, KLD sampling keeps far fewer than the maximum.
        self.assertLess(len(mc.particles), mc.max_particles)


class TurtleBotHandshakeTest(unittest.TestCase):
    """A simulated TurtleBot client and the server, as coordinate_node.py talks to it."""

    ID = "sim-turtlebot"

    def setUp(self):
        self.map = Map()
        self.map.wall_grids[DeviceType.TURTLE] = WALLS
        self.client = TurtleBotPose(self.ID)

    @property
    def bot(self):
        return self.map._devices[self.ID]

    def post(self, room_pose, odom, scan=True, raw=None):
        """One post_device_info: pose once placed, plus the scan the lidar sees at room_pose."""
        self.client.set_odometry(sim.ros_odometry(odom))
        info = DeviceInfo(id=self.ID, device_type=DeviceType.TURTLE)
        if self.client.localized:
            info.transform2.CopyFrom(self.client.server_pose())
        if scan:
            info.raw_lidar2_scan.CopyFrom(raw or sim.raw_scan(room_pose, LIDAR_OFFSET))
        self.client.posted_odom = self.client.odom
        return self.map.handle_device_info(info)

    def fetch(self):
        """fetch_single_device_info, and the client taking on the pose it returns."""
        reply = self.map.get_device_info(self.ID)
        if reply.HasField("device_info") and reply.device_info.HasField("transform2"):
            self.client.adopt(reply.device_info.transform2)
        return reply

    def assert_near(self, pose, truth, distance=0.1, degrees=2.0):
        self.assertLess(math.dist((pose.position.x, pose.position.y), truth[:2]), distance)
        self.assertLess(sim.heading_error(pose.rotation, truth[2]), degrees)

    def localize(self, odom):
        self.assertTrue(self.post(PATH[0], odom[0]).HasField("ok"))
        self.fetch()
        self.assertTrue(self.client.localized)

    def test_odometry_converts_to_server_axes(self):
        self.client.set_odometry(sim.ros_odometry((1.0, 2.0, 0.5)))
        odom = self.client.odom
        self.assertAlmostEqual(odom.position.x, 1.0, places=5)
        self.assertAlmostEqual(odom.position.y, 2.0, places=5)
        self.assertAlmostEqual(math.remainder(odom.rotation - 0.5, math.tau), 0.0, places=5)

    def test_placed_then_tracked_around_the_loop(self):
        # Odometry starts somewhere arbitrary relative to the room.
        odom = sim.drifting_odometry(PATH, (2.0, -1.0, 1.1), np.random.default_rng(3))

        # 1-3: a scan with no pose places the robot; the client fetches and adopts it.
        self.assertTrue(self.post(PATH[0], odom[0]).HasField("ok"))
        self.assertIsNotNone(self.bot.get_pose())
        self.fetch()
        self.assert_near(self.client.server_pose(), PATH[0])

        # 4: poses in server coordinates, with a pose-only post now and then and
        # a refresh every 20 posts, as the client does every couple of seconds.
        worst = 0.0
        for k in range(1, len(PATH)):
            if k % 5 == 0:
                self.post(PATH[k], odom[k], scan=False)
            self.assertTrue(self.post(PATH[k], odom[k]).HasField("ok"))
            pose = self.bot.get_pose()
            worst = max(worst, math.dist((pose.position.x, pose.position.y), PATH[k][:2]))
            if k % 20 == 0:
                self.fetch()

        self.assertLess(worst, 0.15)
        self.assert_near(self.bot.get_pose(), PATH[-1], distance=0.08, degrees=3.0)
        self.fetch()
        # After a refresh the robot's own server pose agrees with the truth too.
        self.assert_near(self.client.server_pose(), PATH[-1], distance=0.08, degrees=3.0)
        self.assertFalse(self.bot.needs_relocalization)
        self.assertIsNotNone(self.bot.accumulated_points())

    def test_odometry_reset_keeps_the_robot_in_place(self):
        odom = sim.drifting_odometry(PATH, (0.0, 0.0, 0.0), np.random.default_rng(4))
        self.localize(odom)
        for k in range(1, 10):
            self.post(PATH[k], odom[k])
        here = self.bot.get_pose()

        # /reset_pose: odometry jumps to zero while the robot stays put. Branch
        # and bound cannot run on this post, so the particles hold the pose.
        self.bot.LOCALIZE_RETRY = math.inf
        self.post(PATH[9], (0.0, 0.0, 0.0))
        self.assertTrue(self.bot.needs_relocalization)
        self.assert_near(self.bot.get_pose(), (here.position.x, here.position.y, here.rotation), distance=0.01, degrees=0.5)

        # The next scan reruns branch and bound, and a refresh brings the client back in line.
        self.bot.LOCALIZE_RETRY = 0.0
        self.post(PATH[9], (0.0, 0.0, 0.0))
        self.assertFalse(self.bot.needs_relocalization)
        self.fetch()
        self.assert_near(self.client.server_pose(), PATH[9])

    def test_unplaceable_scan_is_refused_and_hidden(self):
        blind = RawLidar2Scan(ranges=[float("inf")] * 720, range_min=0.15, range_max=12.0,
                              angle_min=-math.pi, angle_increment=2 * math.pi / 720)
        reply = self.post(PATH[0], (0.0, 0.0, 0.0), raw=blind)
        self.assertTrue(reply.HasField("err_cannot_execute"))
        self.assertIsNone(self.bot.get_pose())
        info = self.map.get_all_device_info().all_device_info.devices[0]
        self.assertFalse(info.HasField("transform2"))
        # A fetch gives the client nothing to adopt yet.
        self.fetch()
        self.assertFalse(self.client.localized)

    def test_poor_fit_requests_branch_and_bound(self):
        odom = sim.drifting_odometry(PATH, (0.0, 0.0, 0.0), np.random.default_rng(5))
        self.localize(odom)
        # Scans that match nothing in the room: a 0.5 m circle around the robot.
        circle = RawLidar2Scan(ranges=[0.5] * 360, range_min=0.15, range_max=12.0,
                               angle_min=-math.pi, angle_increment=2 * math.pi / 360)
        for k in range(1, self.bot.MC_POOR_FIT_UPDATES + 1):
            self.post(PATH[k], odom[k], raw=circle)
        self.assertTrue(self.bot.needs_relocalization)

    def test_parked_scans_stop_updating_the_map(self):
        odom = sim.drifting_odometry(PATH, (0.0, 0.0, 0.0), np.random.default_rng(6))
        self.localize(odom)
        for _ in range(self.bot.STATIONARY_SCAN_LIMIT + 3):
            self.post(PATH[0], odom[0])
        # The placement scan plus the parked ones, up to the limit, were merged.
        self.assertEqual(len(self.bot.scan_rmses), self.bot.STATIONARY_SCAN_LIMIT - 1)
        # Moving resets the budget.
        self.post(PATH[1], odom[1])
        self.assertEqual(self.bot.stationary_scans, 0)
        self.assertEqual(len(self.bot.scan_rmses), self.bot.STATIONARY_SCAN_LIMIT)


if __name__ == "__main__":
    unittest.main()
