"""
link.py - Python side of the host-link protocol (mirror of firmware/include/protocol.h + link.h).

Frame:  0xAA 0x55 | type u8 | len u16 LE | payload | crc16 u16 LE   (CRC-16/CCITT-FALSE over type,len,payload)
Every message layout here is checked against the C++ sizeof() in firmware/sil/test_sil.py.
"""
import struct
from dataclasses import dataclass

SOF = b"\xAA\x55"
MAX_PAYLOAD = 1024
SCAN_BINS = 360

# --- message types (keep in sync with protocol.h)
UP_HELLO, UP_SCAN, UP_EVENT, UP_TELEMETRY, UP_BEACON_STATUS = 0x01, 0x10, 0x12, 0x13, 0x15
UP_MISSION_TARGET, UP_LOG = 0x16, 0x17
UP_BEACON_RX, UP_TARGET_REACHED, UP_NODE_STATUS, UP_MISSION_REQ = 0x20, 0x21, 0x22, 0x23
DN_CMD_VEL, DN_SET_MODE, DN_BEACON_TX, DN_MISSION_REQ, DN_TARGET_REACHED = 0x80, 0x81, 0x82, 0x83, 0x84
DN_SET_CALIB, DN_MISSION_PUSH = 0x87, 0x88

MODE_IDLE, MODE_RUN, MODE_ESTOP = 0, 1, 2
ST_BOOT, ST_WARMUP, ST_READY, ST_RUN, ST_VERIFY, ST_ESTOP, ST_FAULT = range(7)
FL_LIDAR_LOST, FL_IMU_LOST, FL_CMD_TIMEOUT, FL_OBSTACLE_STOP, FL_RADIO_LOST, FL_ESTOP = 1, 2, 4, 8, 16, 32
ROLE_WRITER, ROLE_EXECUTOR, ROLE_HUB = 1, 2, 3
NODE_HUB, NODE_WRITER, NODE_EXECUTOR = 0, 1, 2

BEACON_FMT = "<BBBBhhBHIB"          # 16 bytes: id, writer, type, priority, x_cm, y_cm, heading, sensor, t_ms, ttl
SCAN_HDR_FMT = "<HIffHH"            # scan_id, t_ms, odom_dxy_mm, odom_dth_deg, dt_ms, n
EVENT_FMT = "<BHI"
TELEMETRY_FMT = "<IBBHhhHHHBB"
CMD_VEL_FMT = "<hhH"
CALIB_FMT = "<iif"
BEACON_RX_FMT = "<BB" + BEACON_FMT[1:] + "ii"
NODE_STATUS_FMT = "<BBhhHHB"


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BH", msg_type, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """Incremental parser: feed raw bytes, get (type, payload) tuples; resynchronises after noise."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                del self.buf[:-1]            # keep a possible lone 0xAA at the end
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            mtype, length = struct.unpack_from("<BH", self.buf, 2)
            if length > MAX_PAYLOAD:
                del self.buf[:2]; self.bad_frames += 1; continue
            total = 7 + length
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            (crc,) = struct.unpack_from("<H", self.buf, 5 + length)
            if crc16_ccitt(body) == crc:
                out.append((mtype, bytes(self.buf[5:5 + length])))
                del self.buf[:total]
            else:
                self.bad_frames += 1
                del self.buf[:2]             # skip this SOF and resync
        return out


# ------------------------------------------------------------------ message helpers
def pack_beacon(b) -> bytes:
    """b: comms.beacon.BeaconMessage"""
    return struct.pack(BEACON_FMT, b.beacon_id, b.writer_id, int(b.event_type), int(b.priority),
                       b.x_coord_cm, b.y_coord_cm, b.heading, b.sensor_value, b.timestamp_ms, b.ttl)


def unpack_beacon(raw: bytes):
    from comms.beacon import BeaconMessage, EventType, Priority
    (bid, wid, et, pr, x, y, hd, sv, ts, ttl) = struct.unpack(BEACON_FMT, raw)
    return BeaconMessage(bid, wid, EventType(et), Priority(pr), x, y, hd, sv, ts, ttl)


@dataclass
class Scan:
    scan_id: int
    t_ms: int
    odom_dxy_mm: float
    odom_dth_deg: float
    dt_ms: int
    ranges_mm: list


def parse_scan(p: bytes) -> Scan:
    scan_id, t_ms, dxy, dth, dt_ms, n = struct.unpack_from(SCAN_HDR_FMT, p, 0)
    hdr = struct.calcsize(SCAN_HDR_FMT)
    ranges = list(struct.unpack_from("<%dH" % n, p, hdr))
    return Scan(scan_id, t_ms, dxy, dth, dt_ms, ranges)


def parse_telemetry(p: bytes) -> dict:
    t, st, mode, faults, vl, vr, batt, front, gas, pend, rok = struct.unpack(TELEMETRY_FMT, p)
    return dict(t_ms=t, state=st, mode=mode, faults=faults, vl=vl, vr=vr, battery_mv=batt,
                front_min_mm=front, gas_raw=gas, beacons_pending=pend, radio_ok=rok)


def cmd_vel(v_ms: float, w_rads: float, seq: int = 0) -> bytes:
    v = max(-32768, min(32767, int(round(v_ms * 1000))))
    w = max(-32768, min(32767, int(round(w_rads * 1000))))
    return encode(DN_CMD_VEL, struct.pack(CMD_VEL_FMT, v, w, seq & 0xFFFF))


def set_mode(mode: int) -> bytes:
    return encode(DN_SET_MODE, struct.pack("<B", mode))
