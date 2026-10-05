"""fwlib.py - ctypes wrapper around build/libfw_sim.so: complete firmware instances (the same
src/apps.cpp that runs on the ESP32) driven from Python, with a virtual clock."""
import ctypes as C
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.abspath(os.path.join(HERE, ".."))
LIB_PATH = os.path.join(FW_DIR, "build", "libfw_sim.so")

ROLE_WRITER, ROLE_EXECUTOR, ROLE_HUB = 1, 2, 3
RADIO_HOOK = C.CFUNCTYPE(C.c_bool, C.c_void_p, C.c_int, C.c_int, C.POINTER(C.c_uint8), C.c_int)


def _load():
    if not os.path.exists(LIB_PATH):
        subprocess.check_call(["make", "-s", "all"], cwd=FW_DIR)
    lib = C.CDLL(LIB_PATH)
    P = C.c_void_p
    lib.fw_create.restype = P; lib.fw_create.argtypes = [C.c_int]
    lib.fw_set_param.argtypes = [P, C.c_char_p, C.c_double]
    lib.fw_begin.argtypes = [P, C.c_int]
    lib.fw_destroy.argtypes = [P]
    lib.fw_set_time_us.argtypes = [P, C.c_uint64]
    lib.fw_poll.argtypes = [P]
    lib.fw_set_encoders.argtypes = [P, C.c_int32, C.c_int32]
    lib.fw_set_gyro.argtypes = [P, C.c_float, C.c_int]
    lib.fw_lidar_push.argtypes = [P, C.c_char_p, C.c_int]
    lib.fw_host_push.argtypes = [P, C.c_char_p, C.c_int]
    lib.fw_set_sensors.argtypes = [P, C.c_int, C.c_int, C.c_int]
    lib.fw_radio_push.argtypes = [P, C.c_char_p, C.c_int]
    lib.fw_set_radio_hook.argtypes = [P, RADIO_HOOK, P]
    lib.fw_get_duty.argtypes = [P, C.POINTER(C.c_float), C.POINTER(C.c_float)]
    lib.fw_host_pop.argtypes = [P, C.c_char_p, C.c_int]; lib.fw_host_pop.restype = C.c_int
    lib.fw_state.argtypes = [P]; lib.fw_state.restype = C.c_int
    lib.fw_faults.argtypes = [P]; lib.fw_faults.restype = C.c_int
    lib.fw_odo.argtypes = [P, C.c_int]; lib.fw_odo.restype = C.c_float
    lib.fw_stat.argtypes = [P, C.c_int]; lib.fw_stat.restype = C.c_int
    lib.fw_sizes.argtypes = [C.POINTER(C.c_int)]
    return lib


LIB = _load()
STAT = dict(acked=0, pending=1, pir_false=2, targets_rx=3, hub_fwd=4, hub_dup=5, radio_tx=6, radio_fail=7,
            bad_frames=8, scans=9, ev_gas=10, ev_collapse=11, ev_trapped=12)


def wire_sizes():
    out = (C.c_int * 11)()
    LIB.fw_sizes(out)
    keys = ["beacon", "scan", "event", "telemetry", "cmdvel", "calib", "beacon_rx", "node_status",
            "mission_target", "rf_heartbeat", "rf_target"]
    return dict(zip(keys, list(out)))


class Fw:
    """One firmware instance (Writer, Executor or Hub)."""

    def __init__(self, role, params=None):
        self.role = role
        self.h = LIB.fw_create(role)
        for k, v in (params or {}).items():
            LIB.fw_set_param(self.h, k.encode(), float(v))
        self._hook = None
        self.began = False

    def begin(self):
        LIB.fw_begin(self.h, self.role); self.began = True

    def set_time_us(self, t): LIB.fw_set_time_us(self.h, int(t))
    def poll(self): LIB.fw_poll(self.h)
    def set_encoders(self, l, r): LIB.fw_set_encoders(self.h, int(l), int(r))
    def set_gyro(self, dps, ok=True): LIB.fw_set_gyro(self.h, dps, 1 if ok else 0)
    def lidar_push(self, b): LIB.fw_lidar_push(self.h, bytes(b), len(b))
    def host_push(self, b): LIB.fw_host_push(self.h, bytes(b), len(b))
    def set_sensors(self, gas, tilt, pir): LIB.fw_set_sensors(self.h, int(gas), int(bool(tilt)), int(bool(pir)))
    def radio_push(self, b): LIB.fw_radio_push(self.h, bytes(b), len(b))

    def set_radio_hook(self, fn):
        self._hook = RADIO_HOOK(lambda user, s, d, buf, n: fn(s, d, bytes(buf[:n])))
        LIB.fw_set_radio_hook(self.h, self._hook, None)

    def duty(self):
        l, r = C.c_float(), C.c_float()
        LIB.fw_get_duty(self.h, C.byref(l), C.byref(r))
        return l.value, r.value

    def host_pop(self):
        buf = C.create_string_buffer(65536)
        n = LIB.fw_host_pop(self.h, buf, 65536)
        return buf.raw[:n]

    @property
    def state(self): return LIB.fw_state(self.h)
    @property
    def faults(self): return LIB.fw_faults(self.h)
    def odo(self, which): return LIB.fw_odo(self.h, which)
    def stat(self, name): return LIB.fw_stat(self.h, STAT[name])

    def close(self):
        if self.h:
            LIB.fw_destroy(self.h); self.h = None


class FwTransport:
    """Transport (see base_station/transport.py) that talks to a simulated firmware's host UART."""

    def __init__(self, fw):
        self.fw = fw

    def read(self): return self.fw.host_pop()
    def write(self, data): self.fw.host_push(data)
    def close(self): pass


class RadioMedium:
    """Simulated 2.4 GHz channel between firmware instances. A send succeeds (auto-ACK) with
    probability p_ok; on success the packet lands in the destination's receive queue."""

    def __init__(self, p_ok=1.0, seed=1):
        import random
        self.nodes, self.p_ok, self.rng = {}, p_ok, random.Random(seed)
        self.sent = self.delivered = 0
        self.by_type = {}

    def attach(self, node_id, fw):
        self.nodes[node_id] = fw
        fw.set_radio_hook(self._send)

    def detach(self, node_id):
        self.nodes.pop(node_id, None)

    def _send(self, src, dst, data):
        self.sent += 1
        if dst not in self.nodes or self.rng.random() > self.p_ok:
            return False
        self.nodes[dst].radio_push(data)
        self.delivered += 1
        self.by_type[data[0]] = self.by_type.get(data[0], 0) + 1
        return True
