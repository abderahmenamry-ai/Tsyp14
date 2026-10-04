"""
live_map.py
Graphical "live map" for the command post (the challenge asks for a live map
at the command post), plus a recorder that builds the integrated demo GIF
(Writer exploring -> beacons arriving -> Executor retrieving them).

Everything is drawn in the robots' shared local frame (origin = tunnel
entrance), which is the frame the beacons are written in. GPS coordinates for
each beacon are printed next to it, exactly as the command post receives them.
Needs matplotlib + pillow only (no PyBullet imports at module level).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image

from beacon import EventType, ttl_remaining

EVENT_STYLE = {
    EventType.GAS:      ("#d4a017", "gas"),
    EventType.COLLAPSE: ("#c0392b", "collapse"),
    EventType.TRAPPED:  ("#27ae60", "trapped worker"),
    EventType.EXPLORED: ("#7f8c8d", "explored"),
}
# default plot window = the simulated mine, in the local frame (world - start pose)
DEFAULT_BOUNDS = (-3.0, 37.0, -11.0, 13.0)   # xmin, xmax, ymin, ymax [m]


def plot_live_map(ax, command_post, occupancy=None, map_size_m=80.0, bounds=DEFAULT_BOUNDS,
                  path_xy=None, robot_xy=None, robot_label="", title="Command post - live map"):
    """Draw beacons (coloured by event type, styled by status) over the SLAM map."""
    ax.clear()
    if occupancy is not None:
        h = map_size_m / 2
        ax.imshow(occupancy, origin="lower", extent=[-h, h, -h, h], cmap="gray", vmin=0, vmax=255)
    else:
        ax.set_facecolor("#2c2c2c")
    if path_xy is not None and len(path_xy):
        p = np.asarray(path_xy)
        ax.plot(p[:, 0], p[:, 1], "-", color="#3498db", lw=1.0, alpha=0.9)
    if robot_xy is not None:
        ax.plot(*robot_xy, "o", ms=8, color="#3498db", mec="w", zorder=5)
        if robot_label:
            ax.annotate(robot_label, robot_xy, xytext=(6, 8), textcoords="offset points",
                        color="#3498db", fontsize=8, weight="bold")
    for beacon, gps in command_post.received:
        status = command_post.status_of(beacon)
        colour, _ = EVENT_STYLE[beacon.event_type]
        x, y = beacon.x_coord_cm / 100.0, beacon.y_coord_cm / 100.0
        if status == "pending":
            ax.plot(x, y, "*", ms=17, color=colour, mec="w", mew=1.2, zorder=6)
        elif status == "visited":
            ax.plot(x, y, "o", ms=13, mfc="none", mec=colour, mew=2.5, zorder=6)
            ax.plot(x, y, "+", ms=9, color=colour, mew=2, zorder=6)
        else:  # expired
            ax.plot(x, y, "x", ms=12, color="#bdc3c7", mew=2.5, zorder=6)
        tag = f"#{beacon.beacon_id} {EVENT_STYLE[beacon.event_type][1]}"
        if command_post.now_ms is not None and status != "expired":
            tag += f"  ttl {ttl_remaining(beacon, command_post.now_ms)}/{beacon.ttl}"
        ax.annotate(tag, (x, y), xytext=(8, -14), textcoords="offset points", fontsize=7,
                    color="w", bbox=dict(boxstyle="round,pad=0.2", fc="#000000", ec="none", alpha=0.55))
    ax.set_xlim(bounds[0], bounds[1]); ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect("equal"); ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m] (local frame)"); ax.set_ylabel("y [m]")
    legend = [Line2D([0], [0], marker="*", color="w", mfc="#888", mec="w", ms=12, ls="", label="pending"),
              Line2D([0], [0], marker="o", color="w", mfc="none", mec="#888", mew=2, ms=9, ls="", label="visited"),
              Line2D([0], [0], marker="x", color="#bdc3c7", mew=2, ms=8, ls="", label="expired (TTL out)")]
    ax.legend(handles=legend, loc="upper right", fontsize=7, framealpha=0.8)


def save_live_map(path, command_post, occupancy=None, **kw):
    fig, ax = plt.subplots(figsize=(9, 5.4), dpi=110)
    plot_live_map(ax, command_post, occupancy, **kw)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


class IntegratedRecorder:
    """Records frames of the full demo: left = what the robot is doing in the
    (ground-truth) mine, right = what the command post sees."""

    def __init__(self, command_post, phase_getter, every_s=5.0):
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "living_map_nav"))
        from nav import viz
        from nav.sim_world import START_POSE
        self.viz, self.START = viz, START_POSE
        self.cp, self.phase, self.every_s = command_post, phase_getter, every_s
        self.next_t = 0.0
        self.frames = []
        self.map_snapshot = None      # Writer's SLAM map; frozen after the Writer finishes
        self.freeze_map = False
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(13, 4.6), dpi=70)

    def reset_clock(self):
        self.next_t = 0.0

    def __call__(self, nav):
        if not self.freeze_map and (self.map_snapshot is None or nav.t >= self.next_t):
            self.map_snapshot = nav.get_map()
        if nav.t >= self.next_t:
            self.next_t += self.every_s
            self.frames.append(self._render(nav))

    def _render(self, nav):
        v, S = self.viz, self.START
        ax1, ax2 = self.ax1, self.ax2
        ax1.clear()
        ax1.imshow(nav.world.solid, origin="lower", extent=v._extent_gt(), cmap="Greys", vmin=0, vmax=1.6)
        v._draw_zones(ax1)
        tx = [r["true_x"] + S[0] for r in nav.log]; ty = [r["true_y"] + S[1] for r in nav.log]
        ax1.plot(tx, ty, "b-", lw=1)
        x, y, _ = nav.world.true_pose()
        ax1.add_patch(plt.Circle((x, y), nav.cfg.robot_radius * 1.6, color="b"))
        v._style(ax1, f"{self.phase()}   t={nav.t:5.1f}s  [{nav.explorer.state}]")
        path = [(r["est_x"], r["est_y"]) for r in nav.log]
        pose = nav.get_pose()
        plot_live_map(ax2, self.cp, self.map_snapshot, nav.cfg.map_size_m, path_xy=path,
                      robot_xy=(pose[0], pose[1]), robot_label=self.phase().split()[0])
        self.fig.tight_layout(); self.fig.canvas.draw()
        return Image.fromarray(np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy())

    def save(self, path, ms=90):
        if self.frames:
            self.frames[0].save(path, save_all=True, append_images=self.frames[1:],
                                duration=ms, loop=0, optimize=True)
        plt.close(self.fig)
