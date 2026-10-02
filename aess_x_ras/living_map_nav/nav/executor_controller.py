"""
executor_controller.py
Goal-seeking controller for the Executor robot. Matches WallFollower's
interface exactly (step(), mission_done(), .state, .distance_travelled)
so it drops into NavStack via the `controller=` parameter - no
other nav_stack.py changes needed.

Pulls targets one at a time from a comms.executor_mission.ExecutorMission
instance. Since the Writer and Executor calibrate to the same tunnel-
entrance origin and heading, a beacon's local x/y is already in the
Executor's own SLAM frame - no GPS back-conversion needed.

Navigation: the Executor plans a path over its OWN SLAM occupancy map with
A* (walls inflated by a safety margin, unknown space treated as passable but
more expensive), follows it with pure pursuit, and re-plans every
`replan_period_s` or whenever it gets blocked. Unknown space is optimistic by
design: the Executor starts without a map, so it heads toward the target and
re-plans as the LiDAR reveals walls. If no path exists it falls back to
steering straight at the target (the previous behaviour).
"""
import heapq
import math
import numpy as np


class ExecutorController:
    DOWNSAMPLE = 2            # planning grid = 2 map px (10 cm at 5 cm/px)
    INFLATE_M = 0.35          # keep this far from walls when planning
    UNKNOWN_COST = 1.6        # cost of an unknown cell relative to a free cell
    LOOKAHEAD_M = 0.9         # pure-pursuit look-ahead along the path
    ROI_MARGIN_M = 8.0        # plan only inside the robot/target box + margin

    def __init__(self, cfg, lidar, mission, arrival_radius_m=0.35,
                 replan_period_s=1.5):
        self.cfg, self.lidar = cfg, lidar
        self.mission = mission
        self.arrival_radius_m = arrival_radius_m
        self.replan_period_s = replan_period_s

        self.state = "SEEKING_MISSION"
        self.distance_travelled = 0.0
        self._last_xy = None
        self.current_target = None  # (beacon, gps) tuple
        self._done = False

        self._path = []             # list of (x_m, y_m) waypoints, SLAM frame
        self._last_plan_t = -1e9
        self._avoid_since = None
        self.plans_made = 0
        self.plan_failures = 0

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

    # --------------------------------------------------------------- planning
    def _plan(self, pose, target_xy, slam_map, slam_node):
        """A* over the SLAM occupancy map. Returns a list of (x_m, y_m) or []."""
        d = self.DOWNSAMPLE
        H, W = slam_map.shape
        x, y, _ = pose
        # region of interest (bounding box of robot + target + margin), in map px
        c0, r0 = slam_node.world_to_pixel(x, y)
        c1, r1 = slam_node.world_to_pixel(*target_xy)
        m = int(self.ROI_MARGIN_M * slam_node.px_per_m)
        cmin = max(0, int(min(c0, c1)) - m); cmax = min(W, int(max(c0, c1)) + m)
        rmin = max(0, int(min(r0, r1)) - m); rmax = min(H, int(max(r0, r1)) + m)
        sub = slam_map[rmin:rmax, cmin:cmax]
        # downsample: a cell is a wall if ANY px is dark, free if all free, else unknown
        hh, ww = (sub.shape[0] // d) * d, (sub.shape[1] // d) * d
        blk = sub[:hh, :ww].reshape(hh // d, d, ww // d, d)
        wall = (blk < 60).any(axis=(1, 3))
        free = (blk > 200).all(axis=(1, 3))
        # inflate walls with a disk
        rad = int(math.ceil(self.INFLATE_M * slam_node.px_per_m / d))
        infl = wall.copy()
        for dy in range(-rad, rad + 1):
            for dx in range(-rad, rad + 1):
                if dx * dx + dy * dy <= rad * rad and (dx or dy):
                    sh = np.zeros_like(wall)
                    ys = slice(max(dy, 0), wall.shape[0] + min(dy, 0))
                    yd = slice(max(-dy, 0), wall.shape[0] + min(-dy, 0))
                    xs = slice(max(dx, 0), wall.shape[1] + min(dx, 0))
                    xd = slice(max(-dx, 0), wall.shape[1] + min(-dx, 0))
                    sh[ys, xs] = wall[yd, xd]
                    infl |= sh

        def to_cell(col, row):
            return int((row - rmin) // d), int((col - cmin) // d)

        start = to_cell(c0, r0)
        goal = to_cell(c1, r1)
        gh, gw = infl.shape
        if not (0 <= start[0] < gh and 0 <= start[1] < gw and 0 <= goal[0] < gh and 0 <= goal[1] < gw):
            return []
        # never block the cells the robot / target sit in
        sr = max(1, int(0.3 * slam_node.px_per_m / d))
        for (cy, cx) in (start, goal):
            infl[max(0, cy - sr):cy + sr + 1, max(0, cx - sr):cx + sr + 1] = False
        cost = np.where(free, 1.0, self.UNKNOWN_COST)

        INF = float("inf")
        g = {start: 0.0}
        came = {}
        pq = [(0.0, start)]
        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        found = False
        while pq:
            f, cur = heapq.heappop(pq)
            if cur == goal:
                found = True
                break
            gc = g.get(cur, INF)
            if f - self._h(cur, goal) > gc + 1e-9:
                continue
            for dy, dx, step in nbrs:
                ny, nx = cur[0] + dy, cur[1] + dx
                if not (0 <= ny < gh and 0 <= nx < gw) or infl[ny, nx]:
                    continue
                ng = gc + step * cost[ny, nx]
                if ng < g.get((ny, nx), INF):
                    g[(ny, nx)] = ng
                    came[(ny, nx)] = cur
                    heapq.heappush(pq, (ng + self._h((ny, nx), goal), (ny, nx)))
        if not found:
            return []
        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        cells.reverse()
        ppm = slam_node.px_per_m
        half = slam_node.cfg.map_size_m / 2
        path = []
        for cy, cx in cells:
            col = cmin + cx * d + d / 2
            row = rmin + cy * d + d / 2
            path.append((col / ppm - half, row / ppm - half))
        return path

    @staticmethod
    def _h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _carrot(self, x, y):
        """Pure-pursuit point: first path point at least LOOKAHEAD_M away,
        dropping points the robot has already passed."""
        while len(self._path) > 1 and math.hypot(self._path[0][0] - x, self._path[0][1] - y) < 0.35:
            self._path.pop(0)
        for px, py in self._path:
            if math.hypot(px - x, py - y) >= self.LOOKAHEAD_M:
                return px, py
        return self._path[-1] if self._path else None

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
            self._path = []
            self._last_plan_t = -1e9
            self.state = "EN_ROUTE"

        x, y, th = pose
        tx, ty = self._target_xy_m()
        dist = math.hypot(tx - x, ty - y)

        if dist < self.arrival_radius_m:
            self.mission.reached_target()
            self.state = "ARRIVED"
            self.current_target = None
            self._path = []
            return 0.0, 0.0  # one-tick pause at each beacon; next tick picks the next target

        # ---- (re)plan on the SLAM map
        front = self._sector(ranges, 0, 22)
        avoiding_long = (self._avoid_since is not None and t - self._avoid_since > 4.0)
        if slam_map is not None and slam_node is not None and \
                (not self._path or t - self._last_plan_t > self.replan_period_s or avoiding_long):
            self._last_plan_t = t
            path = self._plan(pose, (tx, ty), slam_map, slam_node)
            self.plans_made += 1
            if path:
                self._path = path
            else:
                self.plan_failures += 1
                self._path = []
            if avoiding_long:
                self._avoid_since = t   # restart the avoid timer after re-planning

        goal = self._carrot(x, y) if self._path else (tx, ty)
        gx, gy = goal
        heading_to_goal = math.atan2(gy - y, gx - x)
        heading_err = math.atan2(math.sin(heading_to_goal - th), math.cos(heading_to_goal - th))

        # ---- obstacle ahead: rotate in place, toward the side the path wants to go
        if front < c.front_stop_m:
            if self._avoid_since is None:
                self._avoid_since = t
            self.state = "AVOIDING"
            sign = 1.0 if heading_err >= 0 else -1.0
            return 0.0, sign * c.turn_w
        self._avoid_since = None

        # ---- pure-pursuit heading control toward the carrot
        self.state = "EN_ROUTE" if self._path else "EN_ROUTE_DIRECT"
        w = float(np.clip(c.k_heading * heading_err, -c.w_max, c.w_max))
        v = c.cruise_v * float(np.clip((front - 0.5) / 1.5, 0.35, 1.0))
        v *= 1.0 - 0.5 * abs(w) / c.w_max
        return float(np.clip(v, 0.08, c.v_max)), w
