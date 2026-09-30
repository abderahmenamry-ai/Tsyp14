"""BreezySLAM wrapper: scan + odometry in, pose + occupancy map out.

Frames
------
* SLAM frame: origin = robot start pose, x forward at start, y left, metres, radians.
  (BreezySLAM natively puts the start at the map centre in mm; we shift/convert here.)
* Map image: uint8, 0 = occupied, ~127 = unknown, 255 = free, indexed [row=y, col=x],
  y increasing "up" -> display with origin="lower".
"""
import math
import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class FusedRMHC(RMHC_SLAM):
    """BreezySLAM RMHC scan matcher with a *damped* correction.

    Stock RMHC_SLAM jumps to whatever pose the random hill-climb finds. In long, nearly
    featureless corridors that pose has a small systematic heading bias that snowballs
    (we measured ~ -0.2 deg per metre). Here the odometry+gyro prediction stays the
    backbone and the scan match only nudges it: correction = clip(gain * (match - prediction)).
    The map is then built from the fused pose, so map and pose stay consistent.
    """
    fuse = dict(gxy=0.5, gth=0.25, cxy=40.0, cth=0.6)

    def _updateMapAndPointcloud(self, dxy_mm, dtheta_degrees, should_update_map):
        f = self.fuse
        pred = self.position.copy()
        pred.x_mm += dxy_mm * self._costheta()
        pred.y_mm += dxy_mm * self._sintheta()
        pred.theta_degrees += dtheta_degrees
        match = self._getNewPosition(pred)

        def clip(v, m):
            return max(-m, min(m, v))
        # While rotating fast, scan matching lags the true rotation and drags the estimate
        # back (measured: -36 deg of heading error over 18 s of turning), so trust the gyro.
        turning = abs(dtheta_degrees) > f.get("turn_thresh", 1.0)
        gxy = 0.0 if turning else f["gxy"]
        gth = 0.0 if turning else f["gth"]
        dx = clip(gxy * (match.x_mm - pred.x_mm), f["cxy"])
        dy = clip(gxy * (match.y_mm - pred.y_mm), f["cxy"])
        dth = clip(gth * (match.theta_degrees - pred.theta_degrees), f["cth"]) + getattr(self, "hint_deg", 0.0)
        pos = pred.copy()
        pos.x_mm += dx
        pos.y_mm += dy
        pos.theta_degrees += dth
        self.position = pos.copy()
        if should_update_map:
            self.map.update(self.scan_for_mapbuild, pos, self.map_quality, self.hole_width_mm)


def wall_heading_error(ranges_m, rel_rad, est_theta, rmax, min_segments=25, step=3):
    """Heading error (rad, in [-45, +45] deg) of the estimate w.r.t. an orthogonal wall grid.

    Builds short wall segments from LiDAR points `step` rays apart (same surface: close together),
    converts their direction to the SLAM frame with the current heading estimate, and takes the
    circular mean of 4*angle (so 0/90/180/270 deg walls all vote for the same correction).
    Returns None if there are not enough usable segments (open cavern, mostly dropouts)."""
    r = np.asarray(ranges_m)
    ok = r < rmax * 0.99
    x = r * np.cos(rel_rad)
    y = r * np.sin(rel_rad)
    n = len(r)
    i = np.arange(0, n - step)
    j = i + step
    good = ok[i] & ok[j]
    dx, dy = x[j] - x[i], y[j] - y[i]
    L = np.hypot(dx, dy)
    good &= (L > 0.05) & (L < 0.6)
    if good.sum() < min_segments:
        return None
    phi = np.arctan2(dy[good], dx[good]) + est_theta          # segment direction, SLAM frame
    w = L[good]
    z = np.sum(w * np.exp(4j * phi))
    if abs(z) < 0.3 * w.sum():                                # walls not aligned to any common grid
        return None
    return float(np.angle(z) / 4.0)                           # est - true-grid offset


class SlamNode:
    def __init__(self, cfg):
        self.cfg = cfg
        # BreezySLAM de-skews each sweep using the robot velocity and the laser's scan rate.
        # Our simulated scans are instantaneous snapshots (no motion during the sweep), so we
        # declare a very fast sweep to disable that correction. (On a real ~5-10 Hz LiDAR,
        # set this to the true scan rate instead.)
        laser = Laser(cfg.lidar_rays, 5000.0, cfg.lidar_fov_deg, int(cfg.lidar_range_m * 1000))
        self.slam = FusedRMHC(laser, cfg.map_size_px, cfg.map_size_m,
                              random_seed=cfg.seed,
                              sigma_xy_mm=cfg.slam_sigma_xy_mm,
                              sigma_theta_degrees=cfg.slam_sigma_theta_deg,
                              max_search_iter=cfg.slam_max_search_iter)
        self.slam.fuse = dict(gxy=cfg.fuse_gain_xy, gth=cfg.fuse_gain_theta,
                              cxy=cfg.fuse_clip_xy_mm, cth=cfg.fuse_clip_theta_deg)
        n = cfg.lidar_rays
        self.rel_rad = np.deg2rad(-cfg.lidar_fov_deg / 2 + (cfg.lidar_fov_deg / (n - 1)) * np.arange(n))
        self._buf = bytearray(cfg.map_size_px * cfg.map_size_px)
        self._centre_mm = cfg.map_size_m * 1000.0 / 2.0
        self.px_per_m = cfg.map_size_px / cfg.map_size_m
        self._pose = (0.0, 0.0, 0.0)

    def update(self, ranges_m, pose_change):
        """ranges_m: iterable of metres (SimLidar order); pose_change: (dxy_mm, dtheta_deg, dt)."""
        # BreezySLAM convention: 0 mm = "no return" (free space); ANY other value is an obstacle.
        # Rays that saw nothing within range must therefore be sent as 0, not as max range.
        rmax = self.cfg.lidar_range_m * 0.999
        scan_mm = [0 if r >= rmax else int(r * 1000) for r in ranges_m]
        # --- absolute heading hint from wall directions (removes gyro drift) ---
        c = self.cfg
        self.slam.hint_deg = 0.0
        if c.manhattan_gain > 0:
            pred_th = math.radians(self.slam.position.theta_degrees + pose_change[1])
            e = wall_heading_error(ranges_m, self.rel_rad, pred_th, c.lidar_range_m,
                                   c.manhattan_min_segments)
            if e is not None:
                corr = -c.manhattan_gain * math.degrees(e)
                self.slam.hint_deg = max(-c.manhattan_clip_deg, min(c.manhattan_clip_deg, corr))
        self.manhattan_used = self.slam.hint_deg != 0.0
        self.slam.update(scan_mm, pose_change)
        x_mm, y_mm, th_deg = self.slam.getpos()
        self._pose = ((x_mm - self._centre_mm) / 1000.0,
                      (y_mm - self._centre_mm) / 1000.0,
                      wrap(math.radians(th_deg)))
        return self._pose

    # --- exported interface used by the rest of the team -------------------
    def get_pose(self):
        """(x_m, y_m, theta_rad) in the SLAM (start) frame."""
        return self._pose

    def get_map(self):
        """uint8 array [size, size], 0 = wall, 127 = unknown, 255 = free."""
        self.slam.getmap(self._buf)
        return np.frombuffer(self._buf, dtype=np.uint8).reshape(self.cfg.map_size_px,
                                                                 self.cfg.map_size_px).copy()

    def world_to_pixel(self, x_m, y_m):
        """SLAM-frame metres -> (col, row) in the map image."""
        return ((x_m + self.cfg.map_size_m / 2) * self.px_per_m,
                (y_m + self.cfg.map_size_m / 2) * self.px_per_m)
