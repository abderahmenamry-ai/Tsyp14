"""PyBullet mine-tunnel world + differential-drive robot.

The world is a solid block of "rock" with tunnels carved out of it (like a real
mine), so every wall is connected to the outer boundary. That matters: the
right-hand rule only visits every gallery if all walls are one connected body.
"""
import math
import numpy as np
import pybullet as p

CELL = 0.5  # m, rasterisation size of the wall boxes

# Rock block (x0, y0, x1, y1) in metres
ROCK = (0.0, 0.0, 40.0, 24.0)

# Carved galleries (x0, y0, x1, y1). Gafsa-style room-and-pillar flavour:
# one main gallery + side branches, a dead-end chamber, and a bend.
TUNNELS = [
    (2.0, 10.0, 37.0, 12.0),    # main gallery (robot starts at its west end)
    (8.0, 12.0, 10.0, 20.0),    # branch A  (north, dead end)
    (14.0, 5.0, 16.0, 10.0),    # branch B  (south) ...
    (11.0, 2.0, 19.0, 5.0),     #   ... ending in a chamber
    (22.0, 12.0, 24.0, 18.0),   # branch C  (north) ...
    (24.0, 16.0, 32.0, 18.0),   #   ... with a bend to the east
    (28.0, 4.0, 30.0, 10.0),    # branch D  (south) - collapses (see RUBBLE)
    (33.0, 8.0, 38.0, 14.0),    # end chamber
]

# Shallow recesses (0.5 m) in the gallery walls: real room-and-pillar mines have irregular
# rock faces, and these give scan matching along-track features (a perfectly straight,
# featureless tunnel is a known SLAM failure case - see the report).
NICHES = [
    (5.0, 9.5, 6.0, 10.0), (11.0, 9.5, 12.2, 10.0), (18.0, 9.5, 19.0, 10.0),
    (25.5, 9.5, 26.5, 10.0), (31.0, 9.5, 32.0, 10.0),
    (4.5, 12.0, 5.7, 12.5), (13.0, 12.0, 14.0, 12.5), (20.0, 12.0, 21.2, 12.5),
    (27.0, 12.0, 28.0, 12.5),
    (10.0, 14.0, 10.5, 15.2), (7.5, 17.0, 8.0, 18.0),          # branch A
    (16.0, 6.5, 16.5, 7.5), (13.5, 8.0, 14.0, 9.0),            # branch B
    (24.0, 13.0, 24.5, 14.0), (21.5, 15.0, 22.0, 16.0),        # branch C
    (26.0, 18.0, 27.0, 18.5), (29.5, 18.0, 30.5, 18.5), (27.5, 15.5, 28.5, 16.0),
    (27.5, 5.0, 28.0, 6.0),                                    # branch D
]
TUNNELS = TUNNELS + NICHES

# Collapse: rock fall that fully blocks branch D
RUBBLE = [(28.0, 7.0, 30.0, 8.0)]

# Ground-truth hazard locations (only used by the *simulated* event sensors
# and by the plots; navigation never reads them).
EVENT_ZONES = [
    {"kind": "gas",      "x": 9.0,  "y": 17.0, "r": 1.2},   # end of branch A
    {"kind": "collapse", "x": 29.0, "y": 8.8,  "r": 1.2},   # in front of rubble
    {"kind": "worker",   "x": 15.0, "y": 3.5,  "r": 1.2},   # chamber of branch B
]

START_POSE = (3.0, 11.0, 0.0)  # x, y, yaw (yaw must stay 0 => world frame == SLAM frame)


def _in_rects(x, y, rects):
    return any(r[0] <= x < r[2] and r[1] <= y < r[3] for r in rects)


def build_solid_grid():
    """Boolean grid [row(y), col(x)] of solid cells at CELL resolution."""
    nx = int(round((ROCK[2] - ROCK[0]) / CELL))
    ny = int(round((ROCK[3] - ROCK[1]) / CELL))
    grid = np.ones((ny, nx), dtype=bool)
    for j in range(ny):
        for i in range(nx):
            cx, cy = ROCK[0] + (i + 0.5) * CELL, ROCK[1] + (j + 0.5) * CELL
            if _in_rects(cx, cy, TUNNELS) and not _in_rects(cx, cy, RUBBLE):
                grid[j, i] = False
    return grid


class MineWorld:
    def __init__(self, cfg, gui=False):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.resetSimulation(physicsClientId=self.client)
        p.setGravity(0, 0, 0, physicsClientId=self.client)
        p.setTimeStep(1.0 / cfg.physics_hz, physicsClientId=self.client)
        self.solid = build_solid_grid()
        self.wall_ids = []
        self._build_walls()
        self.robot = self._build_robot()
        self.collisions = 0
        self.reset()

    # ------------------------------------------------------------------ build
    def _build_walls(self):
        H = 1.0
        ny, nx = self.solid.shape
        for j in range(ny):
            i = 0
            while i < nx:                       # merge horizontal runs of solid cells
                if self.solid[j, i]:
                    k = i
                    while k < nx and self.solid[j, k]:
                        k += 1
                    w, d = (k - i) * CELL, CELL
                    cx = ROCK[0] + i * CELL + w / 2
                    cy = ROCK[1] + j * CELL + d / 2
                    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, H / 2],
                                                 physicsClientId=self.client)
                    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, H / 2],
                                              rgbaColor=[0.55, 0.45, 0.35, 1],
                                              physicsClientId=self.client)
                    bid = p.createMultiBody(0, col, vis, [cx, cy, H / 2], physicsClientId=self.client)
                    p.changeDynamics(bid, -1, lateralFriction=0.0, physicsClientId=self.client)
                    self.wall_ids.append(bid)
                    i = k
                else:
                    i += 1

    def _build_robot(self):
        c = self.cfg
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=c.robot_radius, height=c.robot_height,
                                     physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=c.robot_radius, length=c.robot_height,
                                  rgbaColor=[0.1, 0.4, 0.9, 1], physicsClientId=self.client)
        rid = p.createMultiBody(c.robot_mass, col, vis, [0, 0, c.robot_height / 2],
                                physicsClientId=self.client)
        p.changeDynamics(rid, -1, lateralFriction=0.0, linearDamping=0.0, angularDamping=0.0,
                         physicsClientId=self.client)
        return rid

    def reset(self):
        x, y, yaw = START_POSE
        p.resetBasePositionAndOrientation(
            self.robot, [x, y, self.cfg.robot_height / 2],
            p.getQuaternionFromEuler([0, 0, yaw]), physicsClientId=self.client)
        p.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)

    # ------------------------------------------------------------------ state
    def true_pose(self):
        pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return pos[0], pos[1], yaw

    # ------------------------------------------------------------------ motion
    def step(self, v_cmd, w_cmd):
        """Advance one control period. Real motion = command * random wheel slip."""
        c = self.cfg
        slip = 1.0 + self.rng.normal(0.0, c.true_slip_std)
        v, w = v_cmd * slip, w_cmd * (1.0 + self.rng.normal(0.0, c.true_slip_std))
        n = int(round(c.dt * c.physics_hz))
        for _ in range(n):
            x, y, yaw = self.true_pose()
            p.resetBasePositionAndOrientation(              # keep the body upright & planar
                self.robot, [x, y, c.robot_height / 2], p.getQuaternionFromEuler([0, 0, yaw]),
                physicsClientId=self.client)
            p.resetBaseVelocity(self.robot, [v * math.cos(yaw), v * math.sin(yaw), 0], [0, 0, w],
                                physicsClientId=self.client)
            p.stepSimulation(physicsClientId=self.client)
        if any(cp[2] in self.wall_ids for cp in p.getContactPoints(self.robot, physicsClientId=self.client)):
            self.collisions += 1

    def close(self):
        p.disconnect(self.client)
