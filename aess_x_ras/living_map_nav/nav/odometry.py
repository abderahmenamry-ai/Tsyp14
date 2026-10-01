"""Wheel-odometry model.

On the real robot this comes from encoders / IMU. In simulation we derive it
from the *commanded* motion and corrupt it with a systematic scale error plus
per-step noise, so dead-reckoning drifts and SLAM has something to correct.

This is the interface Person 4 (SLAM Support) can replace with a real
encoder/IMU model: `Odometry.measure(v_cmd, w_cmd, dt) -> (dxy_mm, dtheta_deg, dt)`.
"""
import math
import numpy as np


class Odometry:
    def __init__(self, cfg, rng=None):
        self.cfg = cfg
        self.rng = rng or np.random.default_rng(cfg.seed + 1)
        # dead-reckoning pose (x, y, yaw) in the start frame, for comparison plots
        self.x = self.y = self.yaw = 0.0

    def measure(self, v_cmd, w_cmd, dt, true_dyaw=None):
        """true_dyaw (rad): actual heading change since the last call, used to simulate the IMU
        gyro. If None, heading comes from the (biased) wheel model only."""
        c = self.cfg
        v = v_cmd * (1.0 + c.odom_scale_bias) * (1.0 + self.rng.normal(0, c.odom_noise_std))
        w = w_cmd * (1.0 + c.odom_yaw_bias) * (1.0 + self.rng.normal(0, c.odom_noise_std))
        if true_dyaw is not None:
            w = (true_dyaw + math.radians(c.gyro_bias_dps) * dt
                 + math.radians(self.rng.normal(0, c.gyro_noise_deg))) / dt
        # integrate raw dead reckoning (mid-point rule)
        self.yaw += w * dt / 2
        self.x += v * dt * math.cos(self.yaw)
        self.y += v * dt * math.sin(self.yaw)
        self.yaw += w * dt / 2
        return (v * dt * 1000.0, math.degrees(w * dt), dt)      # BreezySLAM pose_change format

    def dead_reckoning_pose(self):
        return self.x, self.y, self.yaw
