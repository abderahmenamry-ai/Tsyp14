"""
sil_sim.py - software-in-the-loop: the REAL firmware (src/apps.cpp, compiled for the PC) drives a simulated
robot through the PyBullet mine, with the REAL SLAM + controllers + command post on the host side.

      PyBullet mine  --LiDAR/encoders/gyro/sensors-->  firmware  --host link-->  base_station.RobotHost (SLAM, controller)
            ^                                              |  ^                          |
            |                                          nRF24 radio medium                 v
            +--- (v, w) from wheel model <-- motor PWM     v  |                  base_station.HubHost -> CommandPost
                                                         hub firmware
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "comms"))
sys.path.insert(0, os.path.join(ROOT, "living_map_nav"))

from firmware.sil import fwlib as F                      # noqa: E402
from firmware.sil.plant import DrivePlant, ld06_sweep     # noqa: E402
from firmware.sil.bench import CHASSIS                    # noqa: E402
from base_station import link as L                        # noqa: E402
from base_station.robot_host import RobotHost             # noqa: E402
from base_station.hub_host import HubHost                 # noqa: E402
from nav.config import Config                             # noqa: E402
from nav.sim_world import MineWorld, EVENT_ZONES, START_POSE   # noqa: E402
from nav.lidar import SimLidar                            # noqa: E402
from nav.slam_node import wrap                            # noqa: E402
from command_post import CommandPost                      # noqa: E402

SUB_US = 10_000


class SimRobot:
    """A firmware instance + the physical robot in the mine."""

    def __init__(self, role, cfg, seed_offset=0, gyro_seed=3):
        self.role = role
        self.cfg = Config(); self.cfg.__dict__.update(cfg.__dict__)
        self.cfg.seed = cfg.seed + seed_offset
        self.cfg.true_slip_std = 0.0               # slip is applied here, so the gyro can see the real motion
        self.world = MineWorld(self.cfg)
        self.lidar = SimLidar(self.world, self.cfg)
        self.fw = F.Fw(role, dict(CHASSIS))
        self.plant = DrivePlant(seed=gyro_seed)
        import random
        self.rng = random.Random(self.cfg.seed + 11)
        self.prev_pose = self.world.true_pose()
        self.sweep = []
        self.gas_level, self.zone_time = 600.0, {z["kind"]: 0.0 for z in EVENT_ZONES}
        self.t_s = 0.0
        self.truth_log = []
        self.fw.begin()

    def true_pose_slam(self):
        x, y, yaw = self.world.true_pose()
        return x - START_POSE[0], y - START_POSE[1], wrap(yaw - START_POSE[2])

    def _ld06_point_fn(self, ranges, hit):
        def fn(a_cw):
            rel = -a_cw                                # LD06 is clockwise, the SLAM frame is counter-clockwise
            rel = ((rel + 180.0) % 360.0) - 180.0
            i = self.lidar.index(rel)
            return (int(round(ranges[i] * 1000)), 100) if hit[i] else (0, 0)
        return fn

    def _sensors(self, dt):
        x, y, _ = self.world.true_pose()
        gas = tilt = pir = 0
        for z in EVENT_ZONES:
            inside = math.hypot(x - z["x"], y - z["y"]) < z["r"]
            self.zone_time[z["kind"]] = self.zone_time[z["kind"]] + dt if inside else 0.0
            if z["kind"] == "gas":
                target = 1300.0 if inside else 600.0
                self.gas_level += (target - self.gas_level) * min(1.0, dt / 0.6)
            elif z["kind"] == "collapse" and inside:
                tilt = 1 if (self.zone_time["collapse"] * 4.0) % 1.0 < 0.4 else 0
            elif z["kind"] == "worker" and inside:
                pir = 1
        return int(self.gas_level + self.rng.gauss(0, 6)), tilt, pir

    def tick(self, t_us0):
        """One 0.1 s simulation period = 10 firmware substeps. Returns after the world advanced."""
        cfg = self.cfg
        ranges, _, hit = self.lidar.scan()
        self.sweep = ld06_sweep(self._ld06_point_fn(ranges, hit))
        n = len(self.sweep)
        vsum = wsum = 0.0
        for k in range(10):
            t_us = t_us0 + (k + 1) * SUB_US
            for pkt in self.sweep[(k * n) // 10:((k + 1) * n) // 10]:
                self.fw.lidar_push(pkt)
            dl, dr = self.fw.duty()
            v, w = self.plant.step(dl, dr, SUB_US / 1e6)
            vsum += v; wsum += w
            self.fw.set_encoders(*self.plant.enc)
            self._slip_w = w
            self.fw.set_gyro(self.plant.gyro_dps(w), True)
            gas, tilt, pir = self._sensors(SUB_US / 1e6)
            self.fw.set_sensors(gas, tilt, pir)
            yield t_us                                  # caller polls firmware instances + hub, then host
        v, w = vsum / 10, wsum / 10
        slip_v = 1.0 + self.rng.gauss(0, 0.02); slip_w = 1.0 + self.rng.gauss(0, 0.02)
        self.world.step(v * slip_v, w * slip_w)
        self.truth_log.append((t_us0 / 1e6, self.true_pose_slam()))

    def close(self):
        self.fw.close(); self.world.close()


def err_series(host, robot, run_start_us):
    """Position error (host SLAM pose vs ground truth) for every scan. The sweep that produced a pose was
    captured one tick before the firmware reported it, so compare with the truth ~0.11 s earlier."""
    times = np.array([t for t, _ in robot.truth_log]); poses = [p for _, p in robot.truth_log]
    out = []
    for h in host.log:
        tt = run_start_us / 1e6 + h["t"] - 0.11
        i = int(np.argmin(np.abs(times - tt)))
        out.append(math.hypot(h["x"] - poses[i][0], h["y"] - poses[i][1]))
    return out or [0.0]


def run_phase(role, cp, medium, hub, hub_host, cfg, max_s, clock_offset_ms, t_us, seed_offset=0, quiet=False, on_tick=None):
    """Run one robot until its host reports mission complete (or max_s). Returns (RobotHost, SimRobot, t_us)."""
    robot = SimRobot(role, cfg, seed_offset=seed_offset)
    node = L.NODE_WRITER if role == F.ROLE_WRITER else L.NODE_EXECUTOR
    medium.attach(node, robot.fw)
    host = RobotHost("writer" if role == F.ROLE_WRITER else "executor", F.FwTransport(robot.fw), cfg=robot.cfg)
    run_start = None
    n_ticks = int(max_s / cfg.dt)
    for tick in range(n_ticks):
        for t in robot.tick(t_us):
            t_us = t
            robot.fw.set_time_us(t); robot.fw.poll()
            hub.set_time_us(t); hub.poll()
            host.poll()
            hub_host.poll()
            if run_start is None and robot.fw.state in (3, 4):
                run_start = t
            if run_start is not None:
                cp.now_ms = clock_offset_ms + (t - run_start) // 1000
        if on_tick:
            on_tick(robot, host, tick)
        if not quiet and tick % 300 == 0:
            tp = robot.true_pose_slam()
            err = math.hypot(host.pose[0] - tp[0], host.pose[1] - tp[1]) if host.log else 0.0
            print(f"  [{role==F.ROLE_WRITER and 'writer' or 'executor'}] t={tick*cfg.dt:6.1f}s fw_state={robot.fw.state} "
                  f"host={host.state:9s} dist={robot.plant.dist:6.1f} m pos_err={err:.2f} m", flush=True)
        if host.state == "DONE":
            # let the last frames (mode IDLE, final radio traffic) flow for a moment
            for _ in range(30):
                for t in robot.tick(t_us):
                    t_us = t
                    robot.fw.set_time_us(t); robot.fw.poll(); hub.set_time_us(t); hub.poll(); host.poll(); hub_host.poll()
            break
    medium.detach(node)
    robot.run_start_us = run_start or 0
    return host, robot, t_us


def run_full(p_ok=1.0, seed=7, quiet=False, writer_max_s=700.0, executor_max_s=400.0):
    """Writer mission, then Executor mission, through firmware + radio + hub + command post."""
    cfg = Config(); cfg.seed = seed
    cp = CommandPost()
    medium = F.RadioMedium(p_ok=p_ok, seed=seed)
    hub = F.Fw(F.ROLE_HUB, dict(heartbeat_ms=2000)); hub.begin()
    medium.attach(L.NODE_HUB, hub)
    hub_host = HubHost(F.FwTransport(hub), cp, 34.4250, 8.7842, 90.0)
    t_us = 0
    if not quiet:
        print("=== Writer (firmware in the loop) ===", flush=True)
    wh, wr, t_us = run_phase(F.ROLE_WRITER, cp, medium, hub, hub_host, cfg, writer_max_s, 0, t_us, quiet=quiet)
    w_err = err_series(wh, wr, wr.run_start_us)
    writer_end_ms = int(wh.log[-1]["t"] * 1000) if wh.log else 0
    result = dict(
        writer_state=wh.state, writer_time_s=wh.log[-1]["t"] if wh.log else 0, writer_dist_m=wr.plant.dist,
        writer_err_mean=float(np.mean(w_err)), writer_err_max=float(np.max(w_err)),
        beacons_composed=len(wh.beacons_sent), events_suppressed=wh.events_suppressed, beacons_acked=wr.fw.stat("acked"),
        beacons_at_command_post=len(cp.received), hub_forwarded=hub.stat("hub_fwd"), hub_duplicates=hub.stat("hub_dup"),
        radio_sent=medium.sent, radio_delivered=medium.delivered, writer_radio_fail=wr.fw.stat("radio_fail"),
        pir_false_alarms=wr.fw.stat("pir_false"), collisions=wr.world.collisions,
        host_bad_frames=wh.parser.bad_frames + wr.fw.stat("bad_frames"),
        beacon_errors_m=[], writer_map=wh.slam.get_map(),
    )
    tp_slam = wr.true_pose_slam
    wr.close()
    if not quiet:
        print(cp.live_map_summary(), flush=True)
        print("=== Executor (firmware in the loop) ===", flush=True)
    eh, er, t_us = run_phase(F.ROLE_EXECUTOR, cp, medium, hub, hub_host, cfg, executor_max_s, writer_end_ms, t_us,
                             seed_offset=100, quiet=quiet)
    e_err = err_series(eh, er, er.run_start_us)
    visited = sum(1 for b, _ in cp.received if cp.status_of(b) == "visited")
    result.update(
        executor_state=eh.state, executor_time_s=eh.log[-1]["t"] if eh.log else 0, executor_dist_m=er.plant.dist,
        executor_err_max=float(np.max(e_err)), targets_received_by_executor=er.fw.stat("targets_rx"),
        visited=visited, total_beacons=len(cp.received), executor_collisions=er.world.collisions,
        hub_dup_total=hub.stat("hub_dup"), radio_sent=medium.sent, radio_delivered=medium.delivered,
        statuses=[(b.beacon_id, b.event_type.name, cp.status_of(b)) for b, _ in cp.received],
    )
    er.close(); hub.close()
    result["command_post"] = cp
    return result
