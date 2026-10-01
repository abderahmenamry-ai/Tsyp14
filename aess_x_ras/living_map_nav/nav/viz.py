"""Plots, GIF frames and CSV export for the Phase 1 report/demo."""
import csv
import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image

from .sim_world import ROCK, START_POSE, EVENT_ZONES

COLORS = {"gas": "#d4a017", "collapse": "#c0392b", "worker": "#27ae60"}


def _extent_gt():
    return [ROCK[0], ROCK[2], ROCK[1], ROCK[3]]


def _extent_slam(cfg):
    h = cfg.map_size_m / 2
    return [START_POSE[0] - h, START_POSE[0] + h, START_POSE[1] - h, START_POSE[1] + h]


def _to_world(xs, ys):
    return np.asarray(xs) + START_POSE[0], np.asarray(ys) + START_POSE[1]


def _draw_zones(ax):
    for z in EVENT_ZONES:
        ax.add_patch(Circle((z["x"], z["y"]), z["r"], fill=False, ec=COLORS[z["kind"]], lw=1.5, ls="--"))
        ax.text(z["x"], z["y"] + z["r"] + 0.3, z["kind"], color=COLORS[z["kind"]], ha="center", fontsize=8)


def _style(ax, title):
    ax.set_xlim(ROCK[0], ROCK[2]); ax.set_ylim(ROCK[1], ROCK[3])
    ax.set_aspect("equal"); ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")


# --------------------------------------------------------------------- GIF
class FrameRecorder:
    def __init__(self, nav, every_s=4.0):
        self.nav, self.every_s, self.next_t = nav, every_s, 0.0
        self.frames = []
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(13, 4.6), dpi=70)

    def __call__(self, nav):
        if nav.t >= self.next_t:
            self.next_t += self.every_s
            self.frames.append(self._render())

    def _render(self):
        nav = self.nav
        ax1, ax2 = self.ax1, self.ax2
        ax1.clear(); ax2.clear()
        # left: ground truth world, LiDAR rays, true path
        ax1.imshow(nav.world.solid, origin="lower", extent=_extent_gt(), cmap="Greys", vmin=0, vmax=1.6)
        _draw_zones(ax1)
        tx = [r["true_x"] + START_POSE[0] for r in nav.log]
        ty = [r["true_y"] + START_POSE[1] for r in nav.log]
        ax1.plot(tx, ty, "b-", lw=1)
        if nav.last_scan is not None:
            _, pts, hit = nav.last_scan
            x, y, _ = nav.world.true_pose()
            for q, h in list(zip(pts, hit))[::6]:
                ax1.plot([x, q[0]], [y, q[1]], color="#e67e22" if h else "#f5cba7", lw=0.4)
        x, y, _ = nav.world.true_pose()
        ax1.add_patch(Circle((x, y), nav.cfg.robot_radius, color="b"))
        _style(ax1, f"PyBullet world + simulated LiDAR   t={nav.t:5.1f}s  [{nav.explorer.state}]")
        # right: SLAM map + estimated path
        ax2.imshow(nav.slam.get_map(), origin="lower", extent=_extent_slam(nav.cfg), cmap="gray", vmin=0, vmax=255)
        ex, ey = _to_world([r["est_x"] for r in nav.log], [r["est_y"] for r in nav.log])
        ax2.plot(ex, ey, "r-", lw=1)
        if nav.events:
            for b in nav.events.beacons:
                bx, by = _to_world(*b.est_xy)
                ax2.plot(bx, by, "*", ms=12, color=COLORS[b.kind], mec="k")
        px, py, _ = nav.get_pose()
        px, py = _to_world(px, py)
        ax2.add_patch(Circle((px, py), nav.cfg.robot_radius, color="r"))
        _style(ax2, f"BreezySLAM map + estimated path   err={nav.log[-1]['pos_err']:.2f} m")
        self.fig.tight_layout()
        self.fig.canvas.draw()
        return Image.fromarray(np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy())

    def save(self, path, ms=90):
        if not self.frames:
            return
        self.frames.append(self._render())
        self.frames[0].save(path, save_all=True, append_images=self.frames[1:], duration=ms, loop=0, optimize=True)
        plt.close(self.fig)


# --------------------------------------------------------------- final figs
def save_csv(nav, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(nav.log[0].keys()))
        w.writeheader(); w.writerows(nav.log)


def save_final_figures(nav, outdir):
    os.makedirs(outdir, exist_ok=True)
    L = nav.log
    est = np.array([[r["est_x"], r["est_y"]] for r in L])
    tru = np.array([[r["true_x"], r["true_y"]] for r in L])
    dr = np.array([[r["dr_x"], r["dr_y"]] for r in L])
    dist = np.array([r["dist"] for r in L])

    # 1. SLAM map alone
    fig, ax = plt.subplots(figsize=(9, 5.6), dpi=110)
    ax.imshow(nav.get_map(), origin="lower", extent=_extent_slam(nav.cfg), cmap="gray", vmin=0, vmax=255)
    ex, ey = _to_world(est[:, 0], est[:, 1]); ax.plot(ex, ey, "r-", lw=1, label="SLAM path")
    if nav.events:
        for b in nav.events.beacons:
            bx, by = _to_world(*b.est_xy)
            ax.plot(bx, by, "*", ms=14, color=COLORS[b.kind], mec="k", label=f"beacon: {b.kind}")
    ax.legend(loc="upper right", fontsize=8)
    _style(ax, "BreezySLAM occupancy map (final)")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "map_final.png")); plt.close(fig)

    # 2. truth vs estimate vs dead reckoning on the ground-truth layout
    fig, ax = plt.subplots(figsize=(9, 5.6), dpi=110)
    ax.imshow(nav.world.solid, origin="lower", extent=_extent_gt(), cmap="Greys", vmin=0, vmax=1.6)
    tx, ty = _to_world(tru[:, 0], tru[:, 1]); dx, dy = _to_world(dr[:, 0], dr[:, 1])
    ax.plot(dx, dy, color="#8e44ad", lw=1, alpha=0.8, label="odometry only (dead reckoning)")
    ax.plot(tx, ty, "b-", lw=1.4, label="ground truth")
    ax.plot(ex, ey, "r--", lw=1.2, label="SLAM estimate")
    _draw_zones(ax); ax.legend(loc="upper right", fontsize=8)
    _style(ax, "Trajectory: truth vs SLAM vs odometry-only")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "trajectory.png")); plt.close(fig)

    # 3. drift plot
    fig, ax = plt.subplots(figsize=(8, 4), dpi=110)
    dr_err = np.hypot(dr[:, 0] - tru[:, 0], dr[:, 1] - tru[:, 1])
    ax.plot(dist, dr_err, color="#8e44ad", label="odometry only")
    ax.plot(dist, [r["pos_err"] for r in L], "r", label="SLAM")
    ax.set_xlabel("distance travelled [m]"); ax.set_ylabel("position error [m]")
    ax.set_title("Localisation error vs distance travelled"); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "drift.png")); plt.close(fig)
    return dr_err
