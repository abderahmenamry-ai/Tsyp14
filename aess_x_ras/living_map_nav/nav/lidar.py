"""Simulated 2D LiDAR built on PyBullet ray casting (p.rayTestBatch)."""
import numpy as np
import pybullet as p


class SimLidar:
    def __init__(self, world, cfg):
        self.world, self.cfg = world, cfg
        n = cfg.lidar_rays
        # BreezySLAM places sample i at -fov/2 + i*fov/(n-1) (first and last samples are the
        # extreme angles), so the simulated sensor must use exactly that spacing.
        self.step_deg = cfg.lidar_fov_deg / (n - 1)
        # sample i sits at (-fov/2 + i*step) degrees, CCW, relative to robot heading.
        # This is exactly the ordering BreezySLAM expects.
        self.rel_deg = -cfg.lidar_fov_deg / 2 + self.step_deg * np.arange(n)
        self.rel_rad = np.deg2rad(self.rel_deg)
        self.r0 = cfg.robot_radius + 0.02        # start rays just outside the robot body

    def index(self, angle_deg):
        """Index of the ray closest to a robot-relative angle (deg)."""
        return int(round((angle_deg - self.rel_deg[0]) / self.step_deg)) % self.cfg.lidar_rays

    def scan(self, pose=None):
        """Return (ranges_m[n], hit_points_xy[n,2], hit_mask[n]); misses are reported at max range."""
        c = self.cfg
        x, y, yaw = pose if pose is not None else self.world.true_pose()
        ang = yaw + self.rel_rad
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        start = np.column_stack([x + self.r0 * d[:, 0], y + self.r0 * d[:, 1],
                                 np.full(len(ang), c.lidar_height)])
        end = np.column_stack([x + c.lidar_range_m * d[:, 0], y + c.lidar_range_m * d[:, 1],
                               np.full(len(ang), c.lidar_height)])
        res = p.rayTestBatch(start.tolist(), end.tolist(), physicsClientId=self.world.client)
        frac = np.array([r[2] for r in res])
        hit = np.array([r[0] != -1 for r in res])
        rng_m = np.where(hit, self.r0 + frac * (c.lidar_range_m - self.r0), c.lidar_range_m)
        # sensor imperfections
        rng = self.world.rng
        rng_m = np.where(hit, rng_m + rng.normal(0, c.lidar_noise_std_m, len(rng_m)), rng_m)
        drop = rng.random(len(rng_m)) < c.lidar_dropout_prob
        rng_m = np.where(drop, c.lidar_range_m, rng_m)
        rng_m = np.clip(rng_m, 0.05, c.lidar_range_m)
        hit = hit & ~drop
        pts = np.column_stack([x + rng_m * d[:, 0], y + rng_m * d[:, 1]])
        return rng_m, pts, hit
