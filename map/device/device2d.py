"""A device on the ground plane that the server localizes from its 2D lidar.

The device posts its pose (Transform2) and raw scans (RawLidar2Scan) in server
coordinates. Every post that has enough to go on runs localization:
  - branch and bound over the whole room, until the device is placed and again
    whenever tracking is lost;
  - adaptive Monte Carlo after that, on each scan once the device has moved.
Both score scans against the device's WallGrid. Placed scans are merged into
the device's own accumulated map and, if the wall grid has one, into its
occupancy grid (which other devices may share).

The device reports poses through a correction it keeps on-device. The server
keeps its own correction on top of that (from the reported pose to the room
pose) and hands it over whenever the device fetches its own info; see
handover().
"""

from __future__ import annotations

import math
import weakref
from time import time
from typing import Any, ClassVar

import numpy as np
from google.protobuf.json_format import ParseDict

from common.transform2 import (
    apply_transform2,
    as_tuple,
    compose_transform2,
    invert_transform2,
    make_transform2,
    pose_change,
)
from map.algorithms.branch_and_bound import BranchAndBound
from map.algorithms.lidar import scan_to_device_points, simplify_points
from map.algorithms.localization import AccumulatedMap
from map.algorithms.montecarlo import MonteCarlo
from map.device.device import Device, _message_dict
from map.room import polygon_array
from map.tunable import TunableParameter
from map.wallgrid import WallGrid
from realityapi_pb2 import (
    DeviceInfo,
    Lidar2,
    RawLidar2Scan,
    Response,
    Transform2,
    Vector2,
)

# Scan RMSEs kept per device for the dashboard histogram, oldest dropped first.
SCAN_RMSE_LIMIT = 2000

# Branch and bound for each wall grid, with the grid version it was built from.
# Its search pyramid is large, so devices sharing a wall grid share one.
_matchers: "weakref.WeakKeyDictionary[WallGrid, tuple[int, BranchAndBound]]" = weakref.WeakKeyDictionary()


def _matcher(grid: WallGrid) -> BranchAndBound:
    """Branch and bound over a wall grid, rebuilt when the grid's likelihood changes."""
    cached = _matchers.get(grid)
    if cached is None or cached[0] != grid.version:
        cached = (grid.version, BranchAndBound(grid))
        _matchers[grid] = cached
    return cached[1]


class Device2D(Device):
    # Scans whose closest-point RMSE against the accumulated map exceeds this are not merged.
    RMSE_LIMIT = TunableParameter(
        0.2, minimum=0.0, maximum=1.0, step=0.1, label="2D scan RMSE limit", unit="m",
    )
    # While parked (not moved MC_MIN_TRANSLATION / MC_MIN_ROTATION since Monte
    # Carlo last ran), only this many scans update the map / occupancy / live scan.
    STATIONARY_SCAN_LIMIT: ClassVar[int] = 5

    # Seconds between branch-and-bound attempts while the device is not placed.
    LOCALIZE_RETRY: ClassVar[float] = 1.0
    # Lowest mean likelihood branch and bound accepts as a match. Likelihood is
    # (wall + w * occupancy) / (1 + w), so with w = 1 this is the walls-only
    # 0.5 wherever occupancy is unknown (0.5).
    BNB_MIN_SCORE: ClassVar[float] = 0.5
    # Monte Carlo runs on a scan once the device has moved this far since the
    # last run; until then the device counts as parked.
    MC_MIN_TRANSLATION: ClassVar[float] = 0.05
    MC_MIN_ROTATION: ClassVar[float] = math.radians(5.0)
    # Fit (mean grid likelihood of the scan at the estimated pose) below this
    # on MC_POOR_FIT_UPDATES updates in a row reruns branch and bound.
    MC_POOR_FIT = TunableParameter(
        0.4, minimum=0.0, maximum=1.0, step=0.05, label="Relocalize below Monte Carlo fit",
    )
    MC_POOR_FIT_UPDATES: ClassVar[int] = 5
    # The reported pose cannot move this far between two posts; odometry was reset or jumped.
    ODOM_JUMP_TRANSLATION: ClassVar[float] = 0.5
    ODOM_JUMP_ROTATION: ClassVar[float] = math.radians(60.0)
    # A relocalization that moves the pose further than this clears the
    # accumulated map, since scans merged at the old pose do not line up.
    RELOCALIZE_JUMP_TRANSLATION: ClassVar[float] = 0.5
    RELOCALIZE_JUMP_ROTATION: ClassVar[float] = math.radians(20.0)

    def __init__(self, id: str, wall_grid: WallGrid, timestamp: float = 0.0) -> None:
        super().__init__(id, timestamp)
        # What localization scores scans against. Its occupancy grid, if any,
        # is where placed scans go, and may be shared with other devices.
        self.wall_grid = wall_grid
        # This device's own map of placed scans.
        self.accumulated_map = AccumulatedMap()
        self.scan_rmses: list[float] = []

        # Room pose and latest live scan, server coordinates.
        self.transform2 = Transform2()
        self.lidar2 = Lidar2()

        # Room pose = compose_transform2(correction, reported pose). Branch and
        # bound sets it; Monte Carlo updates it on every scan after the device
        # moves; handover() resets it once the device has taken it on. None
        # until placed.
        self.correction: Transform2 | None = None
        self.last_localize_attempt = -math.inf
        # Monte Carlo particle map, seeded from the branch-and-bound pose.
        self.particles = None
        # Reported poses at the last Monte Carlo run and at the last post.
        self._last_mc_reported: Transform2 | None = None
        self._last_reported: Transform2 | None = None
        self._poor_fits = 0
        self.needs_relocalization = False
        # Scans since Monte Carlo last saw the device move.
        self.stationary_scans = 0

    @classmethod
    def from_dict(cls, data: dict[str, Any], **kwargs: Any) -> Device2D:
        # A reloaded device is not placed, so these stay hidden until its next
        # scan localizes it again.
        device = super().from_dict(data, **kwargs)
        device.transform2 = ParseDict(data.get("transform2", {}), Transform2())
        device.lidar2 = ParseDict(data.get("lidar2", {}), Lidar2())
        return device

    @property
    def localized(self) -> bool:
        return self.correction is not None

    def get_pose(self) -> Transform2 | None:
        """The device's room pose in server coordinates, or None until it is placed."""
        if not self.localized:
            return None
        pose = Transform2()
        pose.CopyFrom(self.transform2)
        return pose

    def scan_to_body_points(self, raw_scan: RawLidar2Scan) -> np.ndarray:
        """A raw scan as (N, 2) points in the device's own frame.

        This assumes the lidar sits at the device's center facing forward;
        devices with the sensor mounted elsewhere override it.
        """
        return scan_to_device_points(raw_scan, Transform2(position=Vector2()))

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        """Localize (branch and bound) or track (Monte Carlo) the device, then merge its scan."""
        super().handle_device_info(device_info)
        reported = None
        if device_info.HasField("transform2"):
            reported = Transform2()
            reported.CopyFrom(device_info.transform2)
        scan = None
        if device_info.HasField("raw_lidar2_scan"):
            scan = self.scan_to_body_points(device_info.raw_lidar2_scan)

        pose = self._observe(reported, scan)
        if pose is None:
            # A scan that did not place the device: it should send another.
            if scan is not None and not self.localized:
                return Response(err_cannot_execute={})
            return Response(ok={})

        self.transform2.CopyFrom(pose)
        if scan is not None:
            # While parked, only the first few scans update the map, occupancy
            # and live scan; the pose still updates.
            if len(scan) and self.stationary_scans < self.STATIONARY_SCAN_LIMIT:
                self._store_scan(self._merge_scan(pose, apply_transform2(pose, scan)))
        elif device_info.HasField("lidar2"):
            self.lidar2.CopyFrom(device_info.lidar2)
        return Response(ok={})

    def get_device_info(self) -> DeviceInfo:
        info = super().get_device_info()
        # No place in the room until branch and bound finds one.
        if self.localized:
            info.transform2.CopyFrom(self.transform2)
            info.lidar2.CopyFrom(self.lidar2)
        return info

    def to_dict(self) -> dict[str, Any]:
        ret = super().to_dict()
        ret["transform2"] = _message_dict(self.transform2)
        ret["lidar2"] = _message_dict(self.lidar2)
        return ret

    # Localization.

    def _observe(self, reported: Transform2 | None, scan: np.ndarray | None) -> Transform2 | None:
        """Take one post; return the room pose, or None if it is not known.

        reported is the pose the device sent (server coordinates through its
        on-device correction), or None before it has one. scan is the post's
        lidar as (N, 2) points in the device's own frame, or None.
        """
        if reported is not None:
            if self._last_reported is not None and self._jumped(
                self._last_reported, reported, self.ODOM_JUMP_TRANSLATION, self.ODOM_JUMP_ROTATION
            ):
                print(f"{self.id}: reported pose jumped between posts; rerunning branch and bound")
                self.needs_relocalization = True
                # The device did not move with it (e.g. /reset_pose): keep the
                # particles where they are and re-anchor the correction to them.
                if self.particles is not None and self.particles.pose is not None:
                    self.correction = compose_transform2(
                        make_transform2(*self.particles.pose), invert_transform2(reported)
                    )
                    self._last_mc_reported = reported
            self._last_reported = reported

        if scan is not None and (not self.localized or self.needs_relocalization):
            if self._localize(reported, scan):
                return self.get_pose()
        if not self.localized or reported is None:
            # No reported pose yet: the device has not taken on its first
            # placement, so there is nothing new to place.
            return None
        if scan is not None:
            self._track(reported, scan)
        return compose_transform2(self.correction, reported)

    def _localize(self, reported: Transform2 | None, scan: np.ndarray) -> bool:
        """Branch and bound over the whole room; seed the particle map on success."""
        now = time()
        if now - self.last_localize_attempt < self.LOCALIZE_RETRY:
            return False
        self.last_localize_attempt = now

        result = self._branch_and_bound(scan)
        if result is None:
            print(f"{self.id}: branch and bound found no match in the room; retrying")
            return False

        room, score = result
        if self.localized and self._jumped(
            self.transform2, room, self.RELOCALIZE_JUMP_TRANSLATION, self.RELOCALIZE_JUMP_ROTATION
        ):
            print(f"{self.id}: relocalized; clearing its accumulated map")
            self.accumulated_map.clear()
        # Without a reported pose the device will report exactly this room pose
        # once it takes it on, so the correction starts as the identity.
        base = reported if reported is not None else room
        self.correction = compose_transform2(room, invert_transform2(base))
        self._last_mc_reported = base
        self._last_reported = base
        self.transform2.CopyFrom(room)
        self.particles = MonteCarlo(self.wall_grid)
        self.particles.initialize(as_tuple(room))
        self.stationary_scans = 0
        self._poor_fits = 0
        self.needs_relocalization = False
        print(
            f"{self.id}: branch and bound placed it at ({room.position.x:.2f}, {room.position.y:.2f}), "
            f"heading {math.degrees(room.rotation):.1f} deg, score {score:.3f}"
        )
        return True

    def _branch_and_bound(self, scan: np.ndarray) -> tuple[Transform2, float] | None:
        """Find the device in the room from one scan in its own frame.

        Returns (pose, score): the device's server-frame pose (the scan is in
        its own frame, so the rotation branch and bound finds is its heading)
        and how well the scan fits there. None when no pose inside the room
        scores at least BNB_MIN_SCORE.
        """
        if len(scan) == 0:
            return None
        # Search every heading and every position in a square covering the
        # wall grid's room, scored against its latest occupancy blend.
        self.wall_grid.refresh()
        room = self.wall_grid.room
        corners = polygon_array(room.corners)
        low, high = corners.min(axis=0), corners.max(axis=0)
        center = (*((low + high) / 2), 0.0)
        window = float((high - low).max() / 2)
        result = _matcher(self.wall_grid).search(scan, center, window, math.pi, self.BNB_MIN_SCORE)
        if result is None:
            return None
        x, z, heading, score = result
        if not room.contains(x, z):
            return None
        return make_transform2(float(x), float(z), float(heading)), float(score)

    def _track(self, reported: Transform2, scan: np.ndarray) -> None:
        """Run Monte Carlo on this scan if the device moved since the last run; else count it as parked."""
        if self.particles is None:
            return
        distance, turn = pose_change(self._last_mc_reported, reported)
        if distance < self.MC_MIN_TRANSLATION and turn < self.MC_MIN_ROTATION:
            self.stationary_scans += 1
            return
        self.stationary_scans = 0
        # MonteCarlo works on plain (x, z, heading) tuples.
        self.wall_grid.refresh()
        self.particles.predict(as_tuple(self._last_mc_reported), as_tuple(reported))
        pose = self.particles.update(scan)
        self._last_mc_reported = reported
        if pose is None:
            return
        self.correction = compose_transform2(make_transform2(*pose), invert_transform2(reported))

        if self.particles.fit is not None and self.particles.fit < self.MC_POOR_FIT:
            self._poor_fits += 1
            if self._poor_fits >= self.MC_POOR_FIT_UPDATES and not self.needs_relocalization:
                print(f"{self.id}: Monte Carlo fit {self.particles.fit:.2f} stayed poor; rerunning branch and bound")
                self.needs_relocalization = True
        else:
            self._poor_fits = 0

    def handover(self) -> None:
        """The device has been sent its room pose and will report in that frame.

        It anchors the pose to its last post, so its next reports equal
        compose_transform2(correction, what it would have reported). Re-express
        the stored reported poses that way and reset the correction, so Monte
        Carlo does not mistake the change of frame for motion.
        """
        if not self.localized:
            return
        if self._last_mc_reported is not None:
            self._last_mc_reported = compose_transform2(self.correction, self._last_mc_reported)
        if self._last_reported is not None:
            self._last_reported = compose_transform2(self.correction, self._last_reported)
        self.correction = Transform2(position=Vector2())

    @staticmethod
    def _jumped(before: Transform2, now: Transform2, translation: float, rotation: float) -> bool:
        distance, turn = pose_change(before, now)
        return distance > translation or turn > rotation

    # Mapping.

    def _merge_scan(self, pose: Transform2, points: np.ndarray) -> np.ndarray:
        """Merge a server-frame (N, 2) X/Z scan into the accumulated map and occupancy grid (if any).

        Returns the simplified scan to show live.
        """
        had_map = self.accumulated_map.points is not None
        accepted = self.accumulated_map.update(points, rmse_limit=self.RMSE_LIMIT)
        if self.accumulated_map.last_rmse is not None:
            self._record_scan_rmse(self.accumulated_map.last_rmse)
        # The first scan is stored with no RMSE. Later scans enter only when
        # update() accepts them. A rejected scan must not change the grid.
        kept = accepted or (not had_map and self.accumulated_map.points is not None)
        occupancy = self.wall_grid.occupancy
        if kept and len(points) and occupancy is not None:
            occupancy.update(pose, points)
        return simplify_points(points)

    def _store_scan(self, points: np.ndarray) -> None:
        """Keep a server-frame (N, 2) X/Z scan as the live scan."""
        self.lidar2 = Lidar2(points=[Vector2(x=float(x), y=float(z)) for x, z in points])

    def _record_scan_rmse(self, rmse: float) -> None:
        """Keep at most SCAN_RMSE_LIMIT samples, dropping the oldest first."""
        self.scan_rmses.append(rmse)
        overflow = len(self.scan_rmses) - SCAN_RMSE_LIMIT
        if overflow > 0:
            del self.scan_rmses[:overflow]

    def accumulated_points(self) -> np.ndarray | None:
        """The accumulated map: server-frame (N, 2) X/Z points, or None before any scan."""
        return self.accumulated_map.points

    def clear_scan(self) -> None:
        """Forget the accumulated map, its RMSEs and the parked-scan budget.

        A shared occupancy grid is cleared by its owner, not per device.
        """
        self.accumulated_map.clear()
        self.scan_rmses.clear()
        self.stationary_scans = 0
