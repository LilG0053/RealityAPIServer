"""Monte Carlo localization tracks a robot around the room with drifting odometry."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from map.blockout import WallGrid, load_blockout_polygons
from map.device import TurtleBot
from map.montecarlo import MonteCarlo, wrap_angle
import map.room_localization as room_localization
from realityapi_pb2 import DeviceType, Lidar, Transform3, Vector3

POLYGONS = load_blockout_polygons()  # server frame: room outline, then obstacles
SEGMENTS = [(p[i], p[(i + 1) % len(p)]) for p in POLYGONS for i in range(len(p))]
BEAM_ANGLES = np.linspace(-math.pi, math.pi, 720, endpoint=False)
MAX_RANGE = 12.0

# A loop through open floor, in the server frame (meters).
WAYPOINTS = [(5.5, 3.0), (5.5, 8.0), (1.5, 8.0), (1.5, 3.0), (5.5, 3.0)]
STEP = 0.1


def raycast(x, z, heading):
    """Robot-frame scan (robot faces +x) at a true room pose."""
    o = np.array([x, z])
    d = np.stack([np.cos(BEAM_ANGLES + heading), np.sin(BEAM_ANGLES + heading)], axis=1)
    ranges = np.full(len(d), np.inf)
    for a, b in SEGMENTS:
        e = b - a
        denom = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide="ignore", invalid="ignore"):
            w = a - o
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / denom
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t < ranges)
        ranges = np.where(hit, t, ranges)
    ok = ranges < MAX_RANGE
    r = ranges[ok]
    return np.column_stack((r * np.cos(BEAM_ANGLES[ok]), r * np.sin(BEAM_ANGLES[ok])))


def true_path():
    """Room poses along the loop: drive each leg, then turn in place."""
    poses = []
    for (x0, z0), (x1, z1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        heading = math.atan2(z1 - z0, x1 - x0)
        if poses:
            # Turn toward the next leg in 10 degree steps.
            current = poses[-1][2]
            turn = float(wrap_angle(heading - current))
            for k in range(1, int(abs(turn) / math.radians(10)) + 1):
                poses.append((x0, z0, current + math.copysign(math.radians(10) * k, turn)))
        steps = int(math.hypot(x1 - x0, z1 - z0) / STEP)
        for k in range(steps + 1):
            poses.append((x0 + (x1 - x0) * k / steps, z0 + (z1 - z0) * k / steps, heading))
    return poses


def compose(origin, pose):
    """Pose in a frame placed at origin (x, z, yaw)."""
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * pose[0] - s * pose[1] + origin[0], s * pose[0] + c * pose[1] + origin[1], pose[2] + origin[2])


def drifting_odometry(path, odom_origin, rng):
    """Odometry for the path: starts at odom_origin, drifts 3% in distance and 2 deg/m in heading."""
    odom = [compose(odom_origin, (0.0, 0.0, 0.0))]
    # Odometry frame: rotate/translate so the first true pose maps to odom[0].
    for before, after in zip(path, path[1:]):
        c, s = math.cos(-before[2]), math.sin(-before[2])
        dx, dz = after[0] - before[0], after[1] - before[1]
        forward = c * dx - s * dz
        side = s * dx + c * dz
        turn = float(wrap_angle(after[2] - before[2]))
        distance = math.hypot(forward, side)
        forward *= 1.03 + rng.normal(0, 0.01)
        turn += math.radians(2.0) * distance + rng.normal(0, math.radians(0.2))
        h = odom[-1][2]
        odom.append((
            odom[-1][0] + math.cos(h) * forward - math.sin(h) * side,
            odom[-1][1] + math.sin(h) * forward + math.cos(h) * side,
            h + turn,
        ))
    return odom


def heading_error(a, b):
    return math.degrees(abs(float(wrap_angle(a - b))))


class MonteCarloTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = true_path()
        # Walls only, so the saved occupancy grid does not change the result.
        cls.grid = WallGrid.from_blockout(occupancy_path=None)

    def test_tracks_loop_from_known_start_where_odometry_drifts(self):
        rng = np.random.default_rng(1)
        odom = drifting_odometry(self.path, (0.0, 0.0, 0.0), rng)
        mc = MonteCarlo(self.grid, seed=2)
        mc.initialize(self.path[0])

        worst = 0.0
        for k in range(1, len(self.path)):
            mc.predict(odom[k - 1], odom[k])
            pose = mc.update(raycast(*self.path[k]))
            worst = max(worst, math.dist(pose[:2], self.path[k][:2]))

        end = self.path[-1]
        # Dead reckoning: odometry started at (0, 0, 0), anchored at the true start.
        odom_end = compose(self.path[0], odom[-1])
        self.assertGreater(math.dist(odom_end[:2], end[:2]), 0.3)
        self.assertLess(worst, 0.15)
        self.assertLess(math.dist(mc.pose[:2], end[:2]), 0.08)
        self.assertLess(heading_error(mc.pose[2], end[2]), 3.0)
        self.assertGreater(mc.fit, 0.5)
        # Once confident, KLD sampling keeps far fewer than the maximum.
        self.assertLess(len(mc.particles), mc.max_particles)

    def _turtlebot_scan(self, odom_pose, room_pose):
        """What the server hands TurtleBot.observe_scan: odometry pose and odometry-frame scan."""
        body = raycast(*room_pose)
        c, s = math.cos(odom_pose[2]), math.sin(odom_pose[2])
        points = body @ np.array([[c, -s], [s, c]]).T + np.array(odom_pose[:2])
        transform = Transform3(
            position=Vector3(x=odom_pose[0], z=odom_pose[1]),
            rotation=Vector3(y=odom_pose[2]),
        )
        lidar = Lidar(points=[Vector3(x=float(x), z=float(z)) for x, z in points])
        return transform, lidar

    def _turtlebot(self):
        bot = TurtleBot(
            id="sim", device_type=DeviceType.TURTLE, timestamp=0.0,
            transform=Transform3(), lidar=Lidar(),
        )
        bot.LOCALIZE_RETRY = 0.0
        return bot

    def test_turtlebot_branch_and_bound_then_monte_carlo_track_the_loop(self):
        original_weight = WallGrid.OCCUPANCY_WEIGHT
        WallGrid.OCCUPANCY_WEIGHT = 0.0
        room_localization._cache = None
        try:
            rng = np.random.default_rng(3)
            # Odometry starts somewhere arbitrary relative to the room.
            odom = drifting_odometry(self.path, (2.0, -1.0, 1.1), rng)
            bot = self._turtlebot()

            self.assertTrue(bot.observe_scan(*self._turtlebot_scan(odom[0], self.path[0])))
            start = bot.room_pose(odom[0])
            self.assertLess(math.dist(start[:2], self.path[0][:2]), 0.1)
            self.assertLess(heading_error(start[2], self.path[0][2]), 2.0)

            worst = 0.0
            for k in range(1, len(self.path)):
                self.assertTrue(bot.observe_scan(*self._turtlebot_scan(odom[k], self.path[k])))
                worst = max(worst, math.dist(bot.room_pose(odom[k])[:2], self.path[k][:2]))
            self.assertLess(worst, 0.15)
            self.assertFalse(bot.needs_relocalization)

            # /reset_pose: odometry jumps to zero while the robot stays put.
            here = bot.room_pose(odom[-1])
            reset = (0.0, 0.0, 0.0)
            bot.LOCALIZE_RETRY = math.inf  # branch and bound cannot run on this scan
            self.assertTrue(bot.observe_scan(*self._turtlebot_scan(reset, self.path[-1])))
            self.assertTrue(bot.needs_relocalization)
            self.assertLess(math.dist(bot.room_pose(reset)[:2], here[:2]), 1e-6)
        finally:
            WallGrid.OCCUPANCY_WEIGHT = original_weight
            room_localization._cache = None

    def test_poor_fit_requests_branch_and_bound(self):
        bot = self._turtlebot()
        bot._set_origin(self.path[0], (0.0, 0.0, 0.0))
        bot.particles = MonteCarlo(self.grid, seed=4)
        bot.particles.initialize(self.path[0])
        bot._last_mc_odom = (0.0, 0.0, 0.0)
        # Scans of a room that is not this one: every point misses the walls.
        nowhere = Lidar(points=[Vector3(x=50.0 + k * 0.1, z=50.0) for k in range(100)])
        for k in range(1, TurtleBot.MC_POOR_FIT_UPDATES + 1):
            bot._track((0.1 * k, 0.0, 0.0), nowhere)
        self.assertTrue(bot.needs_relocalization)


if __name__ == "__main__":
    unittest.main()
