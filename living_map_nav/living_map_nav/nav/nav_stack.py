"""NavStack: ties world + LiDAR + odometry + BreezySLAM + explorer together.

Public interface for the rest of the team (Writer/Executor, frame translation,
beacon dropping):

    nav = NavStack(cfg)
    nav.run(callbacks=[fn])          # fn(nav) called every tick
    nav.get_pose()                   # (x_m, y_m, theta_rad) SLAM frame, origin = start
    nav.get_map()                    # uint8 occupancy image (0 wall / 127 unknown / 255 free)
    nav.get_true_pose_slam_frame()   # ground truth in the same frame (evaluation only)
    nav.log                          # list of dict rows (also written to CSV by run_demo.py)
"""
import math
import time
from .config import Config
from .sim_world import MineWorld, START_POSE
from .lidar import SimLidar
from .odometry import Odometry
from .slam_node import SlamNode, wrap
from .explorer import WallFollower
from .events import EventSimulator


class NavStack:
    def __init__(self, cfg=None, gui=False, events=True):
        self.cfg = cfg or Config()
        self.world = MineWorld(self.cfg, gui=gui)
        self.lidar = SimLidar(self.world, self.cfg)
        self.odom = Odometry(self.cfg)
        self.slam = SlamNode(self.cfg)
        self.explorer = WallFollower(self.cfg, self.lidar)
        self.events = EventSimulator() if events else None
        self.t = 0.0
        self.v = self.w = 0.0
        self.log = []
        self.last_scan = None
        self._map_cache = None
        self._map_cache_t = -1e9
        self.stop_reason = None
        self._prev_yaw = self.world.true_pose()[2]

    # ------------------------------------------------------------ exported API
    def get_pose(self):
        return self.slam.get_pose()

    def get_map(self):
        return self.slam.get_map()

    def get_true_pose_slam_frame(self):
        x, y, yaw = self.world.true_pose()
        return x - START_POSE[0], y - START_POSE[1], wrap(yaw - START_POSE[2])

    # ------------------------------------------------------------ main loop
    def tick(self):
        c = self.cfg
        tp = self.get_true_pose_slam_frame()          # ground truth at the instant of the scan
        ranges, pts, hit = self.lidar.scan()
        yaw_now = self.world.true_pose()[2]
        dyaw = wrap(yaw_now - self._prev_yaw); self._prev_yaw = yaw_now      # simulated gyro input
        pose_change = self.odom.measure(self.v, self.w, c.dt, true_dyaw=dyaw)  # motion since last scan
        pose = self.slam.update(ranges, pose_change)

        # refresh the (expensive) map copy at 2 Hz for the look-ahead guard
        if self.t - self._map_cache_t >= 0.5:
            self._map_cache, self._map_cache_t = self.slam.get_map(), self.t

        self.v, self.w = self.explorer.step(self.t, ranges, pose, self._map_cache, self.slam, self.v)
        self.world.step(self.v, self.w)
        self.t += c.dt

        err = math.hypot(pose[0] - tp[0], pose[1] - tp[1])
        new_beacons = []
        if self.events:
            new_beacons = self.events.check(self.t, self.world.true_pose(), pose)
        self.last_scan = (ranges, pts, hit)
        row = dict(t=round(self.t, 2), state=self.explorer.state,
                   est_x=pose[0], est_y=pose[1], est_th=pose[2],
                   true_x=tp[0], true_y=tp[1], true_th=tp[2],
                   dr_x=self.odom.x, dr_y=self.odom.y, dr_th=self.odom.yaw,
                   pos_err=err, dist=self.explorer.distance_travelled,
                   v=self.v, w=self.w, collisions=self.world.collisions)
        self.log.append(row)
        return row, new_beacons

    def run(self, callbacks=(), verbose=True, print_every_s=30.0):
        c = self.cfg
        wall_t0 = time.time()
        next_print = 0.0
        while self.t < c.max_time_s:
            row, new_b = self.tick()
            for cb in callbacks:
                cb(self)
            if verbose:
                for b in new_b:
                    print(f"  [beacon] {b.kind:8s} t={b.t:6.1f}s  est=({b.est_xy[0]:.2f},{b.est_xy[1]:.2f}) "
                          f"true=({b.true_xy[0]:.2f},{b.true_xy[1]:.2f})  error={b.error_m:.2f} m")
                if self.t >= next_print:
                    print(f"t={self.t:6.1f}s state={row['state']:9s} dist={row['dist']:6.1f} m "
                          f"pos_err={row['pos_err']:.2f} m  (wall {time.time()-wall_t0:.0f}s)")
                    next_print += print_every_s
            if self.explorer.mission_done(self.get_pose()):
                self.stop_reason = "returned to start after full exploration"
                break
        else:
            self.stop_reason = "time limit"
        return self.stop_reason

    def close(self):
        self.world.close()
