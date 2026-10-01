"""
executor_controller.py
Goal-seeking controller for the Executor robot. Matches WallFollower's
interface exactly (step(), mission_done(), .state, .distance_travelled)
so it drops into NavStack via the new `controller=` parameter - no
other nav_stack.py changes needed.

Pulls targets one at a time from a comms.executor_mission.ExecutorMission
instance. Since the Writer and Executor calibrate to the same tunnel-
entrance origin and heading, a beacon's local x/y is already in the
Executor's own SLAM frame - no GPS back-conversion needed.
"""
import math
import numpy as np


class ExecutorController:
    def __init__(self, cfg, lidar, mission, arrival_radius_m=0.35):
        self.cfg, self.lidar = cfg, lidar
        self.mission = mission
        self.arrival_radius_m = arrival_radius_m

        self.state = "SEEKING_MISSION"
        self.distance_travelled = 0.0
        self._last_xy = None
        self.current_target = None  # (beacon, gps) tuple
        self._done = False

        self.mission.receive_mission()

    # --------------------------------------------------------------- helpers
    def _sector(self, ranges, centre_deg, half_deg, fn=np.min):
        i0 = self.lidar.index(centre_deg - half_deg)
        n = int(round(2 * half_deg / self.lidar.step_deg)) + 1
        idx = (i0 + np.arange(n)) % len(ranges)
        return float(fn(ranges[idx]))

    def _target_xy_m(self):
        beacon, _gps = self.current_target
        return beacon.x_coord_cm / 100.0, beacon.y_coord_cm / 100.0

    def update_progress(self, pose):
        x, y, _ = pose
        if self._last_xy is not None:
            self.distance_travelled += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)

    # --------------------------------------------------------------- mission
    def mission_done(self, pose):
        return self._done

    # --------------------------------------------------------------- control
    def step(self, t, ranges, pose, slam_map=None, slam_node=None, last_v=0.0):
        c = self.cfg
        self.update_progress(pose)

        if self.current_target is None:
            target = self.mission.next_target()
            if target is None:
                self.state = "DONE"
                self._done = True
                return 0.0, 0.0
            self.current_target = target
            self.state = "EN_ROUTE"

        x, y, th = pose
        tx, ty = self._target_xy_m()
        dist = math.hypot(tx - x, ty - y)

        if dist < self.arrival_radius_m:
            self.mission.reached_target()
            self.state = "ARRIVED"
            self.current_target = None
            return 0.0, 0.0  # one-tick pause at each beacon; next tick picks the next target

        # ---- obstacle check ahead, same sectoring approach as WallFollower
        front = self._sector(ranges, 0, 22)
        if front < c.front_stop_m:
            self.state = "AVOIDING"
            return 0.0, c.turn_w  # rotate in place until the way clears

        # ---- pure-pursuit heading control toward the target
        heading_to_target = math.atan2(ty - y, tx - x)
        heading_err = math.atan2(math.sin(heading_to_target - th), math.cos(heading_to_target - th))
        w = float(np.clip(c.k_heading * heading_err, -c.w_max, c.w_max))
        v = c.cruise_v * float(np.clip((front - 0.5) / 1.5, 0.35, 1.0))
        v *= 1.0 - 0.5 * abs(w) / c.w_max
        return float(np.clip(v, 0.08, c.v_max)), w
