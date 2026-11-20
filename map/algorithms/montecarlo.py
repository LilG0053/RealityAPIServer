"""Adaptive Monte Carlo localization (AMCL) against a likelihood grid.

Standalone: needs only numpy and a grid object with
    likelihood  (nx, nz) array, 1.0 where a scan point is most expected
    origin      (x, z) of the corner of cell (0, 0)
    resolution  cell size in meters
such as map.wallgrid.WallGrid. Cell (i, j) covers
[origin_x + i*res, origin_x + (i+1)*res) x [origin_z + j*res, origin_z + (j+1)*res).

Poses are (x, z, heading) in the grid's frame. heading is measured from +x
toward +z. A scan is an (N, 2) array of points relative to the robot in its own
frame: the robot faces +x, so a scan point p sits at R(heading) @ p + (x, z).

Usage:
    filter = MonteCarlo(grid)
    filter.initialize(pose)                 # one known pose, e.g. from branch and bound
    filter.predict(odom_before, odom_now)   # each time the robot moves
    pose = filter.update(scan)              # with the scan taken at odom_now

Each update weights the particles by how well the scan fits the grid from
their pose (likelihood field model), takes the pose of the most likely
cluster of particles, and resamples. The number of particles adapts with
KLD sampling: many while the particles are spread out, few once they agree.
"""

from __future__ import annotations

import math

import numpy as np

Pose = tuple[float, float, float]


def wrap_angle(angle):
    """Wrap to [-pi, pi)."""
    return (np.asarray(angle) + math.pi) % (2 * math.pi) - math.pi


class MonteCarlo:
    def __init__(
        self,
        grid,
        *,
        min_particles: int = 100,
        max_particles: int = 5000,
        initial_particles: int = 1000,
        initial_std: tuple[float, float] = (0.05, math.radians(2.0)),
        # Odometry noise (Probabilistic Robotics, sample_motion_model_odometry):
        # rotation from rotation, rotation from translation,
        # translation from translation, translation from rotation.
        alphas: tuple[float, float, float, float] = (0.2, 0.2, 0.2, 0.2),
        max_beams: int = 60,
        z_hit: float = 0.95,
        z_rand: float = 0.05,
        # KLD sampling: with probability 1 - delta (z is its quantile), the
        # particle estimate is within kld_error of the true distribution.
        kld_error: float = 0.01,
        kld_z: float = 2.326,
        kld_bin: tuple[float, float] = (0.1, math.radians(10.0)),
        # Particles in neighboring bins of this size form one cluster.
        cluster_bin: tuple[float, float] = (0.5, math.radians(20.0)),
        seed: int | None = None,
    ):
        self.grid = grid
        self.min_particles = min_particles
        self.max_particles = max_particles
        self.initial_particles = initial_particles
        self.initial_std = initial_std
        self.alphas = alphas
        self.max_beams = max_beams
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.kld_error = kld_error
        self.kld_z = kld_z
        self.kld_bin = kld_bin
        self.cluster_bin = cluster_bin
        self.rng = np.random.default_rng(seed)

        self.particles = np.zeros((0, 3))   # rows of (x, z, heading)
        self.weights = np.zeros(0)
        self.pose: Pose | None = None
        # Mean grid likelihood of the last scan at the estimated pose (0..1).
        self.fit: float | None = None

    def initialize(self, pose: Pose, count: int | None = None) -> None:
        """Start the particle map as a small cloud around one pose."""
        count = count or self.initial_particles
        std_xy, std_heading = self.initial_std
        self.particles = np.column_stack((
            self.rng.normal(pose[0], std_xy, count),
            self.rng.normal(pose[1], std_xy, count),
            wrap_angle(self.rng.normal(pose[2], std_heading, count)),
        ))
        self.weights = np.full(count, 1.0 / count)
        self.pose = (float(pose[0]), float(pose[1]), float(wrap_angle(pose[2])))
        self.fit = None

    def predict(self, odom_before: Pose, odom_now: Pose) -> None:
        """Move every particle by the odometry change, with noise.

        The odometry poses may be in any frame; only their difference is used.
        """
        dx = odom_now[0] - odom_before[0]
        dz = odom_now[1] - odom_before[1]
        translation = math.hypot(dx, dz)
        # Below ~1 cm the direction of travel is noise; treat it as a turn in place.
        rot1 = float(wrap_angle(math.atan2(dz, dx) - odom_before[2])) if translation > 0.01 else 0.0
        rot2 = float(wrap_angle(odom_now[2] - odom_before[2] - rot1))

        # Driving backwards looks like a half turn; use the smaller angle for noise.
        rot1_noise = min(abs(rot1), abs(float(wrap_angle(rot1 - math.pi))))
        rot2_noise = min(abs(rot2), abs(float(wrap_angle(rot2 - math.pi))))
        a1, a2, a3, a4 = self.alphas
        n = len(self.particles)
        rot1_hat = rot1 - self.rng.normal(0.0, math.sqrt(a1 * rot1_noise ** 2 + a2 * translation ** 2), n)
        trans_hat = translation - self.rng.normal(
            0.0, math.sqrt(a3 * translation ** 2 + a4 * (rot1_noise ** 2 + rot2_noise ** 2)), n
        )
        rot2_hat = rot2 - self.rng.normal(0.0, math.sqrt(a1 * rot2_noise ** 2 + a2 * translation ** 2), n)

        heading = self.particles[:, 2] + rot1_hat
        self.particles[:, 0] += trans_hat * np.cos(heading)
        self.particles[:, 1] += trans_hat * np.sin(heading)
        self.particles[:, 2] = wrap_angle(heading + rot2_hat)

    def update(self, scan: np.ndarray) -> Pose | None:
        """Weight the particles by the scan, estimate the pose, and resample."""
        scan = np.asarray(scan, dtype=float).reshape(-1, 2)
        if len(scan) == 0 or len(self.particles) == 0:
            return self.pose

        beams = scan[np.linspace(0, len(scan) - 1, min(self.max_beams, len(scan))).astype(int)]
        log_weights = np.log(
            self.z_hit * self._likelihood(self._place(beams, self.particles)) + self.z_rand
        ).sum(axis=1)
        weights = self.weights * np.exp(log_weights - log_weights.max())
        total = weights.sum()
        self.weights = weights / total if total > 0 else np.full(len(weights), 1.0 / len(weights))

        self.pose = self._best_cluster_pose()
        self.fit = float(self._likelihood(self._place(scan, np.array([self.pose])))[0].mean())
        self._resample()
        return self.pose

    def _place(self, scan: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """Scan points at each pose: shape (poses, points, 2)."""
        cos, sin = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        x = cos * scan[:, 0] - sin * scan[:, 1] + poses[:, 0:1]
        z = sin * scan[:, 0] + cos * scan[:, 1] + poses[:, 1:2]
        return np.stack((x, z), axis=-1)

    def _likelihood(self, points: np.ndarray) -> np.ndarray:
        """Grid likelihood at each point; off the grid counts as 0."""
        likelihood = self.grid.likelihood
        index = np.floor((points - np.asarray(self.grid.origin)) / self.grid.resolution).astype(int)
        inside = (
            (index[..., 0] >= 0) & (index[..., 0] < likelihood.shape[0])
            & (index[..., 1] >= 0) & (index[..., 1] < likelihood.shape[1])
        )
        values = np.zeros(points.shape[:-1])
        values[inside] = likelihood[index[..., 0][inside], index[..., 1][inside]]
        return values

    def _bins(self, particles: np.ndarray, size: tuple[float, float]) -> np.ndarray:
        """Integer (x, z, heading) bin of each particle; heading bins wrap."""
        xy, heading = size
        heading_bins = max(1, round(2 * math.pi / heading))
        return np.column_stack((
            np.floor(particles[:, 0] / xy),
            np.floor(particles[:, 1] / xy),
            np.floor((particles[:, 2] + math.pi) / (2 * math.pi) * heading_bins) % heading_bins,
        )).astype(int)

    def _best_cluster_pose(self) -> Pose:
        """Weighted mean pose of the cluster of particles carrying the most weight."""
        bins = self._bins(self.particles, self.cluster_bin)
        heading_bins = max(1, round(2 * math.pi / self.cluster_bin[1]))
        unique, inverse = np.unique(bins, axis=0, return_inverse=True)
        inverse = inverse.reshape(-1)
        bin_weight = np.bincount(inverse, weights=self.weights, minlength=len(unique))

        # Flood-fill occupied bins into clusters: neighbors differ by at most 1
        # in every bin coordinate (heading wraps around).
        index_of = {tuple(b): i for i, b in enumerate(unique)}
        cluster = np.full(len(unique), -1)
        offsets = [(dx, dz, dh) for dx in (-1, 0, 1) for dz in (-1, 0, 1) for dh in (-1, 0, 1)]
        clusters = 0
        for start in range(len(unique)):
            if cluster[start] >= 0:
                continue
            cluster[start] = clusters
            stack = [start]
            while stack:
                bx, bz, bh = unique[stack.pop()]
                for dx, dz, dh in offsets:
                    neighbor = index_of.get((bx + dx, bz + dz, (bh + dh) % heading_bins))
                    if neighbor is not None and cluster[neighbor] < 0:
                        cluster[neighbor] = clusters
                        stack.append(neighbor)
            clusters += 1

        cluster_weight = np.bincount(cluster, weights=bin_weight, minlength=clusters)
        members = cluster[inverse] == np.argmax(cluster_weight)
        w = self.weights[members]
        p = self.particles[members]
        if w.sum() <= 0:
            w = np.ones(len(p))
        x, z = np.average(p[:, :2], axis=0, weights=w)
        heading = math.atan2(np.average(np.sin(p[:, 2]), weights=w), np.average(np.cos(p[:, 2]), weights=w))
        return float(x), float(z), float(heading)

    def _kld_count(self, bins_occupied: np.ndarray) -> np.ndarray:
        """Particles needed (KLD bound) for each count of occupied bins."""
        k = np.maximum(bins_occupied - 1, 1).astype(float)
        a = 2.0 / (9.0 * k)
        return k / (2.0 * self.kld_error) * (1.0 - a + np.sqrt(a) * self.kld_z) ** 3

    def _resample(self) -> None:
        """Draw particles by weight until the KLD bound for their spread is met."""
        draws = self.rng.choice(len(self.particles), size=self.max_particles, p=self.weights)
        drawn = self.particles[draws]
        # Draw order is random, so the k-th draw sees the bins filled so far.
        _, first = np.unique(self._bins(drawn, self.kld_bin), axis=0, return_index=True)
        new_bin = np.zeros(len(drawn), dtype=bool)
        new_bin[first] = True
        occupied = np.cumsum(new_bin)
        count = np.arange(1, len(drawn) + 1)
        enough = (count >= self.min_particles) & (count >= self._kld_count(occupied))
        keep = int(np.argmax(enough)) + 1 if enough.any() else len(drawn)
        self.particles = drawn[:keep].copy()
        self.weights = np.full(keep, 1.0 / keep)
