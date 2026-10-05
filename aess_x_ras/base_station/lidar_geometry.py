"""lidar_geometry.py - the little bit of SimLidar the controllers use (index of a bearing, bin spacing),
without needing a physics world. Layout matches the firmware's scan bins: bin i is at
-180 + i * 360/359 degrees (CCW from the robot's forward direction)."""
import numpy as np


class LidarGeometry:
    def __init__(self, cfg):
        n = cfg.lidar_rays
        self.cfg = cfg
        self.step_deg = cfg.lidar_fov_deg / (n - 1)
        self.rel_deg = -cfg.lidar_fov_deg / 2 + self.step_deg * np.arange(n)
        self.rel_rad = np.deg2rad(self.rel_deg)

    def index(self, angle_deg):
        return int(round((angle_deg - self.rel_deg[0]) / self.step_deg)) % self.cfg.lidar_rays
