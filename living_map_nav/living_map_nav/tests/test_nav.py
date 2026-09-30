"""Smoke tests:  python -m pytest -q   (or just: python tests/test_nav.py)"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from nav.config import Config
from nav.sim_world import MineWorld, START_POSE
from nav.lidar import SimLidar
from nav.nav_stack import NavStack


def test_lidar_distances_match_geometry():
    cfg = Config(lidar_noise_std_m=0.0, lidar_dropout_prob=0.0)
    w = MineWorld(cfg)
    lid = SimLidar(w, cfg)
    r, _, _ = lid.scan()
    # start (3, 11) facing +x in a gallery spanning y in [10, 12]: walls 1 m to each side
    assert abs(r[lid.index(90)] - 1.0) < 0.02       # left wall
    assert abs(r[lid.index(-90)] - 1.0) < 0.02      # right wall
    assert r[lid.index(0)] > 30 * 0 + 7.9 or r[lid.index(0)] > 7.0   # long gallery ahead (max range 8 m)
    assert abs(r[lid.index(180)] - 1.0) < 0.05      # west end of gallery at x = 2
    w.close()


def test_slam_tracks_a_short_run():
    cfg = Config(max_time_s=25.0)
    nav = NavStack(cfg, events=False)
    nav.run(verbose=False)
    err = max(r["pos_err"] for r in nav.log)
    assert err < 0.5, f"SLAM error too large: {err:.2f} m"
    assert nav.world.collisions < 5
    nav.close()


if __name__ == "__main__":
    test_lidar_distances_match_geometry(); print("lidar ok")
    test_slam_tracks_a_short_run(); print("slam ok")
