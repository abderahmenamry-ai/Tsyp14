"""bench.py - a firmware instance on a bench: motors/encoders/gyro (DrivePlant), a synthetic LiDAR scene,
event-sensor inputs and a host-link parser. No PyBullet needed, so the behaviour tests run in seconds."""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "comms"))
sys.path.insert(0, os.path.join(ROOT, "living_map_nav"))

from firmware.sil import fwlib as F          # noqa: E402
from firmware.sil.plant import DrivePlant, ld06_sweep   # noqa: E402
from base_station import link as L           # noqa: E402

CHASSIS = dict(wheel_diameter_m=0.065, wheel_base_m=0.30, encoder_ticks_rev=374, max_wheel_speed_ms=0.9,
               max_v_ms=0.6, max_w_rads=1.6, obstacle_stop_mm=330, event_cooldown_ms=3000, heartbeat_ms=1000, mission_req_ms=1000)
DT_US = 10_000                                  # 100 Hz substep


def open_scene(angle_cw):
    return 3000, 100                            # 3 m of free space in every direction


def wall_ahead(mm):
    """Obstacle `mm` in front of the robot (LD06 clockwise angle within +-30 deg of 0)."""
    def scene(a):
        a = ((a + 180) % 360) - 180
        return (mm, 100) if abs(a) <= 30 else (3000, 100)
    return scene


class Bench:
    def __init__(self, role=F.ROLE_WRITER, params=None, scene=open_scene, seed=3):
        p = dict(CHASSIS); p.update(params or {})
        self.fw = F.Fw(role, p)
        self.role, self.scene = role, scene
        self.plant = DrivePlant(seed=seed)
        self.parser = L.FrameParser()
        self.frames = []                         # (type, payload) received from the firmware
        self.sweep, self.sweep_phase = [], 0
        self.lidar_on, self.t_us = True, 0
        self.gas, self.tilt, self.pir = 600, False, False
        self.v = self.w = 0.0
        self.yaw = 0.0                           # true heading of the simulated robot (rad)
        self.fw.begin()

    # --- host side helpers
    def send(self, frame): self.fw.host_push(frame)
    def cmd(self, v, w): self.send(L.cmd_vel(v, w))
    def mode(self, m): self.send(L.set_mode(m))
    def last(self, mtype):
        for t, p in reversed(self.frames):
            if t == mtype:
                return p
        return None
    def count(self, mtype): return sum(1 for t, _ in self.frames if t == mtype)
    def clear(self): self.frames.clear()
    def telemetry(self):
        p = self.last(L.UP_TELEMETRY)
        return L.parse_telemetry(p) if p else {}

    # --- one 10 ms step of the whole bench
    def substep(self, t_us):
        self.t_us = t_us
        fw = self.fw
        if self.lidar_on:                        # one revolution per 100 ms, packets spread over the 10 substeps
            phase = (t_us // DT_US) % 10
            if phase == 0:
                self.sweep = ld06_sweep(self.scene)
            n = len(self.sweep)
            lo, hi = (phase * n) // 10, ((phase + 1) * n) // 10
            for pkt in self.sweep[lo:hi]:
                fw.lidar_push(pkt)
        dl, dr = fw.duty()
        self.v, self.w = self.plant.step(dl, dr, DT_US / 1e6)
        self.yaw += self.w * DT_US / 1e6
        fw.set_encoders(*self.plant.enc)
        fw.set_gyro(self.plant.gyro_dps(self.w), True)
        fw.set_sensors(self.gas, self.tilt, self.pir)
        fw.set_time_us(t_us)
        fw.poll()
        self.frames += self.parser.feed(fw.host_pop())

    def run(self, seconds, others=(), each=None):
        steps = int(seconds * 1e6 / DT_US)
        for _ in range(steps):
            t = self.t_us + DT_US
            self.substep(t)
            for o in others:
                o.substep(t) if isinstance(o, Bench) else (o.set_time_us(t), o.poll())
            if each:
                each(t)

    def boot_to_run(self, others=()):
        self.run(2.0, others)
        assert self.fw.state == 2, "firmware did not reach READY (state=%d)" % self.fw.state
        self.mode(L.MODE_RUN)
        self.run(0.1, others)
        assert self.fw.state == 3, "firmware did not enter RUN (state=%d)" % self.fw.state
