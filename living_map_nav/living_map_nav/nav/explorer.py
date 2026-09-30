"""Right-hand-rule wall-following / exploration controller.

Inputs each tick : LiDAR ranges (robot frame) + SLAM pose (+ SLAM map).
Outputs          : (v, w) command.

States
------
FOLLOW    hold `wall_target_m` from the right wall, aligned with it
TURN_LEFT wall ahead (inner corner / dead end): rotate left until the way is clear
SEEK      right wall vanished (side branch / outer corner): arc right to find it
RECOVER   SLAM pose says we are not moving: back off and turn away

SLAM data feeds motion decisions in three places:
  1. stuck detection (pose progress over a time window),
  2. map look-ahead guard (occupancy map checked ahead of the robot, catching
     obstacles that fall in a LiDAR gap/dropout),
  3. mission logic: distance travelled / return-to-start / map-growth stop.
"""
import math
import numpy as np


class WallFollower:
    def __init__(self, cfg, lidar):
        self.cfg, self.lidar = cfg, lidar
        self.state = "FOLLOW"
        self.recover_until = 0.0
        self._pose_hist = []         # (t, x, y, th) from SLAM
        self.distance_travelled = 0.0
        self.max_out = 0.0
        self._last_xy = None

    # --------------------------------------------------------------- helpers
    def _sector(self, ranges, centre_deg, half_deg, fn=np.min):
        i0 = self.lidar.index(centre_deg - half_deg)
        n = int(round(2 * half_deg / self.lidar.step_deg)) + 1
        idx = (i0 + np.arange(n)) % len(ranges)
        return float(fn(ranges[idx]))

    def _map_ahead_blocked(self, slam_map, slam_node, pose, look=0.55):
        """True if the SLAM occupancy map shows a wall in the cells just ahead."""
        if slam_map is None:
            return False
        x, y, th = pose
        for d in (0.35, look):
            for off in (-0.15, 0.0, 0.15):
                px = x + d * math.cos(th) - off * math.sin(th)
                py = y + d * math.sin(th) + off * math.cos(th)
                col, row = slam_node.world_to_pixel(px, py)
                col, row = int(col), int(row)
                if 0 <= col < slam_map.shape[1] and 0 <= row < slam_map.shape[0]:
                    if slam_map[row, col] < 60:          # dark = occupied
                        return True
        return False

    # --------------------------------------------------------------- mission
    def update_progress(self, pose):
        x, y, _ = pose
        if self._last_xy is not None:
            self.distance_travelled += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self.max_out = max(self.max_out, math.hypot(x, y))

    def mission_done(self, pose):
        """Back near the start after having explored away from it (SLAM frame origin = start)."""
        c = self.cfg
        return (self.max_out > c.min_out_distance_m
                and math.hypot(pose[0], pose[1]) < c.return_radius_m)

    def _stuck(self, t, pose, v_cmd):
        self._pose_hist.append((t, pose[0], pose[1], pose[2]))
        while self._pose_hist and t - self._pose_hist[0][0] > 6.0:
            self._pose_hist.pop(0)
        if len(self._pose_hist) < 40 or v_cmd < 0.15:
            return False
        t0, x0, y0, th0 = self._pose_hist[0]
        moved = math.hypot(pose[0] - x0, pose[1] - y0)
        return moved < 0.10

    # --------------------------------------------------------------- control
    def step(self, t, ranges, pose, slam_map=None, slam_node=None, last_v=0.0):
        c = self.cfg
        self.update_progress(pose)

        front = self._sector(ranges, 0, 22)
        a45 = self._sector(ranges, -45, 3, np.median)
        b90 = self._sector(ranges, -90, 3, np.median)
        fr = self._sector(ranges, -40, 20)           # front-right, anti-clipping
        fl = self._sector(ranges, 40, 20)

        # ---- recovery -----------------------------------------------------
        if t < self.recover_until:
            return -0.20, c.w_max * 0.6
        if self.state != "RECOVER" and self._stuck(t, pose, last_v):
            self.state, self.recover_until = "RECOVER", t + 1.5
            self._pose_hist.clear()
            return -0.20, c.w_max * 0.6
        if self.state == "RECOVER":
            self.state = "FOLLOW"

        # ---- state transitions --------------------------------------------
        map_block = self._map_ahead_blocked(slam_map, slam_node, pose) if slam_node else False
        if self.state == "TURN_LEFT":
            if front > c.front_clear_m and b90 < c.wall_lost_m and not map_block:
                self.state = "FOLLOW"
        elif front < c.front_stop_m or (map_block and front < 1.2):
            self.state = "TURN_LEFT"
        elif self.state == "SEEK":
            if b90 < c.wall_lost_m * 0.8:
                self.state = "FOLLOW"
        elif b90 > c.wall_lost_m and a45 > c.wall_lost_m * 1.2:
            self.state = "SEEK"

        # ---- commands -----------------------------------------------------
        if self.state == "TURN_LEFT":
            return 0.0, c.turn_w

        if self.state == "SEEK":
            return 0.30, -0.9

        # FOLLOW: wall angle + lateral error from two right-hand rays (-45deg, -90deg)
        th = math.radians(45)
        alpha = math.atan2(b90 - a45 * math.sin(th), a45 * math.cos(th))   # >0: heading into wall
        dist = b90 * math.cos(alpha)
        w = c.k_heading * alpha + c.k_dist * (c.wall_target_m - dist)
        if fr < 0.55:                       # corner about to be clipped on the right
            w += 0.6
        if fl < 0.45:                       # squeezed on the left: bias right
            w -= 0.4
        w = float(np.clip(w, -c.w_max, c.w_max))
        v = c.cruise_v * float(np.clip((front - 0.5) / 1.5, 0.35, 1.0))
        v *= 1.0 - 0.55 * abs(w) / c.w_max
        return float(np.clip(v, 0.08, c.v_max)), w
