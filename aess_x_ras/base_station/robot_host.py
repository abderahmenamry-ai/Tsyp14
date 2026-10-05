"""
robot_host.py - the SLAM host for ONE robot (Writer or Executor).

Receives the firmware's UP_SCAN / UP_EVENT / UP_TELEMETRY frames, runs the existing SlamNode
(BreezySLAM) and the existing controller (WallFollower for the Writer, ExecutorController for the
Executor) and answers with DN_CMD_VEL. For the Writer it also turns UP_EVENT into a beacon (it needs
the SLAM pose, which only the host has) and sends it back as DN_BEACON_TX for the firmware to radio.

Everything is driven by `poll()`, so the same class runs on the real robot (serial transport, called
in a loop) and inside the software-in-the-loop simulation (called once per simulated tick).
"""
import math
import struct
import numpy as np

from . import paths  # noqa: F401  (side effect: sys.path)
from . import link as L
from .lidar_geometry import LidarGeometry
from nav.config import Config
from nav.slam_node import SlamNode
from nav.explorer import WallFollower
from nav.executor_controller import ExecutorController
from beacon import make_beacon, EventType
from outside_network import GPSCoord   # noqa: F401

EVENT_BY_CODE = {1: EventType.GAS, 2: EventType.COLLAPSE, 3: EventType.TRAPPED}


class RadioMission:
    """Same interface as comms.executor_mission.ExecutorMission, but the target list arrives over the
    radio (hub -> Executor firmware -> UP_MISSION_TARGET) instead of from the command post directly."""

    def __init__(self, send):
        self.send = send
        self.targets = []                # list of (beacon, None)
        self._incoming = {}
        self._count = 0
        self.ready = False               # True once a full list (possibly empty) has arrived
        self.current_target = None
        self.reached_ids = []

    def on_target_frame(self, index, count, beacon):
        if count == 0:
            self.targets, self._incoming, self.ready = [], {}, True
            return
        if count != self._count:                      # a new/changed list: start collecting again
            self._incoming, self._count = {}, count
        self._incoming[index] = beacon                # packets can arrive in any order, or twice
        if len(self._incoming) == count:
            self.targets = [(self._incoming[i], None) for i in sorted(self._incoming)]
            self._incoming, self.ready = {}, True

    # --- ExecutorMission interface
    def receive_mission(self):
        pass                              # the list is pushed to us; nothing to pull

    def next_target(self):
        if not self.targets:
            return None
        self.current_target = self.targets[0]
        return self.current_target

    def reached_target(self):
        if self.current_target:
            beacon, _ = self.current_target
            self.reached_ids.append(beacon.beacon_id)
            self.send(L.encode(L.DN_TARGET_REACHED, struct.pack("<B", beacon.beacon_id)))
            self.targets = [t for t in self.targets if t[0].beacon_id != beacon.beacon_id]
            self.current_target = None


class RobotHost:
    def __init__(self, role, transport, cfg=None, writer_id=1, dedupe_radius_m=3.0):
        assert role in ("writer", "executor")
        self.role, self.tp = role, transport
        self.cfg = cfg or Config()
        self.parser = L.FrameParser()
        self.slam = SlamNode(self.cfg)
        self.geom = LidarGeometry(self.cfg)
        if role == "writer":
            self.controller = WallFollower(self.cfg, self.geom)
            self.mission = None
        else:
            self.mission = RadioMission(self._send)
            self.controller = ExecutorController(self.cfg, self.geom, self.mission)
        self.writer_id = writer_id
        self.dedupe_radius_m = dedupe_radius_m
        self.events_suppressed = 0
        self.state = "WAIT_READY"        # WAIT_READY -> RUNNING -> DONE
        self.fw_state = L.ST_BOOT
        self.telemetry = {}
        self.pose = (0.0, 0.0, 0.0)
        self.v = self.w = 0.0
        self._map, self._map_t = None, -1e9
        self.beacons_sent = []           # BeaconMessage objects composed here
        self.acked = set()
        self.log = []                    # one dict per scan (for plots / tests)
        self.stop_reason = None
        self._next_id = 1
        self._seq = 0
        self.mission_asked = False

    # ---------------------------------------------------------------- plumbing
    def _send(self, frame: bytes):
        self.tp.write(frame)

    def poll(self):
        """Process everything the firmware has sent since the last call."""
        for mtype, payload in self.parser.feed(self.tp.read()):
            self._on_frame(mtype, payload)

    def _on_frame(self, mtype, p):
        if mtype == L.UP_TELEMETRY:
            self.telemetry = L.parse_telemetry(p)
            self.fw_state = self.telemetry["state"]
            if self.state == "WAIT_READY" and self.fw_state == L.ST_READY:
                self._send(L.set_mode(L.MODE_RUN))           # firmware is warmed up: start the mission
        elif mtype == L.UP_SCAN:
            self._on_scan(L.parse_scan(p))
        elif mtype == L.UP_EVENT and self.role == "writer":
            et, value, t_ms = struct.unpack(L.EVENT_FMT, p[:struct.calcsize(L.EVENT_FMT)])
            self._on_event(et, value, t_ms)
        elif mtype == L.UP_BEACON_STATUS:
            bid, ok = struct.unpack("<BB", p[:2])
            if ok:
                self.acked.add(bid)
        elif mtype == L.UP_MISSION_TARGET and self.mission is not None:
            idx, cnt = p[0], p[1]
            self.mission.on_target_frame(idx, cnt, L.unpack_beacon(p[2:18]))
        # UP_HELLO / UP_LOG / others: nothing to do

    # ---------------------------------------------------------------- SLAM + control
    def _on_scan(self, s: L.Scan):
        if self.fw_state not in (L.ST_RUN, L.ST_VERIFY):
            return                                          # not started yet / already stopped
        if self.state == "WAIT_READY":
            self.state = "RUNNING"
        if self.state != "RUNNING":
            return
        c = self.cfg
        raw = np.asarray(s.ranges_mm, dtype=float)
        ranges = np.where(raw <= 0, c.lidar_range_m, raw / 1000.0)      # 0 = no return -> max range
        dt = (s.dt_ms / 1000.0) if s.dt_ms > 0 else c.dt
        self.pose = self.slam.update(ranges, (s.odom_dxy_mm, s.odom_dth_deg, dt))
        t = s.t_ms / 1000.0
        if t - self._map_t >= 0.5:
            self._map, self._map_t = self.slam.get_map(), t

        if self.mission is not None and not self.mission.ready:
            v, w = 0.0, 0.0                                  # Executor: wait for the mission to arrive by radio
        else:
            v, w = self.controller.step(t, ranges, self.pose, self._map, self.slam, self.v)
        self.v, self.w = v, w
        self._seq += 1
        self._send(L.cmd_vel(v, w, self._seq))
        self.log.append(dict(t=t, x=self.pose[0], y=self.pose[1], th=self.pose[2], v=v, w=w,
                             state=self.controller.state, fw_state=self.fw_state))

        if self.controller.mission_done(self.pose):
            self.state, self.stop_reason = "DONE", "mission complete"
            self._send(L.cmd_vel(0, 0, self._seq + 1))
            self._send(L.set_mode(L.MODE_IDLE))

    # ---------------------------------------------------------------- events -> beacons
    def _on_event(self, event_code, value, t_ms):
        et = EVENT_BY_CODE.get(event_code)
        if et is None:
            return
        x_m, y_m, th = self.pose
        for old in self.beacons_sent:      # the same incident keeps firing while the robot stays near it: one beacon per incident
            if old.event_type == et and math.hypot(old.x_coord_cm / 100.0 - x_m, old.y_coord_cm / 100.0 - y_m) < self.dedupe_radius_m:
                self.events_suppressed += 1
                return
        heading = int(((math.degrees(th) % 360.0) / 360.0) * 256) & 0xFF
        beacon = make_beacon(self._next_id, self.writer_id, et, x_m * 100, y_m * 100, heading, value, t_ms)
        self._next_id += 1
        self.beacons_sent.append(beacon)
        self._send(L.encode(L.DN_BEACON_TX, L.pack_beacon(beacon)))
