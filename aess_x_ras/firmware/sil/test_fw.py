"""test_fw.py - behaviour tests for the real firmware code (compiled for the host) on a simulated bench.
Run:  python3 firmware/sil/test_fw.py      (from aess_x_ras/)  or  make -C firmware test"""
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from firmware.sil.bench import Bench, wall_ahead, F, L, DT_US        # noqa: E402
from firmware.sil import fwlib                                        # noqa: E402
from beacon import make_beacon, EventType                              # noqa: E402
from outside_network import translate_to_gps, CalibrationData          # noqa: E402

checks = fails = 0


def check(cond, msg):
    global checks, fails
    checks += 1
    if not cond:
        fails += 1
        print("  FAIL:", msg)


def section(name): print(name)


def test_wire_sizes():
    section("wire format: C++ sizeof == Python struct sizes")
    s = fwlib.wire_sizes()
    import struct as st
    check(s["beacon"] == st.calcsize(L.BEACON_FMT) == 16, "beacon %s" % s["beacon"])
    check(s["scan"] == st.calcsize(L.SCAN_HDR_FMT) + 2 * 360, "scan %s" % s["scan"])
    check(s["event"] == st.calcsize(L.EVENT_FMT), "event")
    check(s["telemetry"] == st.calcsize(L.TELEMETRY_FMT), "telemetry %s vs %s" % (s["telemetry"], st.calcsize(L.TELEMETRY_FMT)))
    check(s["cmdvel"] == st.calcsize(L.CMD_VEL_FMT), "cmdvel")
    check(s["calib"] == st.calcsize(L.CALIB_FMT), "calib")
    check(s["beacon_rx"] == st.calcsize(L.BEACON_RX_FMT), "beacon_rx %s vs %s" % (s["beacon_rx"], st.calcsize(L.BEACON_RX_FMT)))
    check(s["node_status"] == st.calcsize(L.NODE_STATUS_FMT), "node_status")
    check(s["mission_target"] == 2 + 16, "mission_target")
    check(s["rf_heartbeat"] + 4 <= 32 and s["rf_target"] + 4 <= 32, "radio packets fit 32 bytes")


def test_boot_and_scan_stream():
    section("boot: gyro calibration + LiDAR lock -> READY, scans stream to the host")
    b = Bench()
    b.run(0.5)
    check(b.fw.state == 1, "WARMUP at 0.5 s (state %d)" % b.fw.state)
    b.run(1.5)
    check(b.fw.state == 2, "READY after 2 s (state %d)" % b.fw.state)
    check(b.count(L.UP_HELLO) == 1, "HELLO sent once")
    b.mode(L.MODE_RUN); b.run(1.0)
    check(b.fw.state == 3, "RUN after SET_MODE")
    scans = [L.parse_scan(p) for t, p in b.frames if t == L.UP_SCAN]
    check(len(scans) >= 8, "about 10 scans/s reach the host (%d in 3 s window)" % len(scans))
    s = scans[-1]
    check(len(s.ranges_mm) == 360 and all(2900 <= r <= 3100 for r in s.ranges_mm if r), "scan content: 3 m open space")
    check(sum(1 for r in s.ranges_mm if r == 0) <= 2, "no empty bins in a clean sweep")
    check(b.fw.stat("bad_frames") == 0 and b.parser.bad_frames == 0, "no corrupt frames")


def test_drive_and_odometry():
    section("closed loop: (v, w) command -> PID -> motors -> encoders -> odometry")
    b = Bench(); b.boot_to_run(); b.clear()
    t0 = b.plant.dist
    for _ in range(30):                           # 3 s straight at 0.4 m/s, command refreshed every 100 ms
        b.cmd(0.4, 0.0); b.run(0.1)
    steady = abs(b.v)
    check(abs(steady - 0.4) < 0.03, "wheel speed settles at the set-point (%.3f m/s)" % steady)
    true_d = b.plant.dist - t0
    scans = [L.parse_scan(p) for t, p in b.frames if t == L.UP_SCAN]
    odo_mm = sum(s.odom_dxy_mm for s in scans)
    check(abs(odo_mm / 1000 - true_d) / true_d < 0.04, "odometry distance within 4 %% (%.2f m vs %.2f m)" % (odo_mm / 1000, true_d))
    b.clear(); yaw0 = b.yaw
    for _ in range(20):                           # spin in place at 1 rad/s for 2 s
        b.cmd(0.0, 1.0); b.run(0.1)
    scans = [L.parse_scan(p) for t, p in b.frames if t == L.UP_SCAN]
    dth = sum(s.odom_dth_deg for s in scans)
    true_dth = math.degrees(b.yaw - yaw0)
    check(true_dth > 80 and abs(dth - true_dth) < 3.0, "turn measured by gyro+encoders within 3 deg (%.1f vs true %.1f)" % (dth, true_dth))


def test_safety_layer():
    section("safety layer (independent of the host)")
    b = Bench(); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    check(abs(b.v) > 0.3, "driving")
    b.run(1.0)                                    # host goes silent
    check(abs(b.v) < 0.02, "command timeout stops the robot (v=%.3f)" % b.v)
    check(b.fw.faults & L.FL_CMD_TIMEOUT, "CMD_TIMEOUT flag set")

    b = Bench(scene=wall_ahead(150)); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    check(abs(b.v) < 0.02 and b.fw.faults & L.FL_OBSTACLE_STOP, "obstacle 15 cm ahead blocks forward motion")
    for _ in range(10):
        b.cmd(0.0, 0.8); b.run(0.1)
    check(abs(b.w) > 0.4, "...but turning in place is still allowed (w=%.2f)" % b.w)

    b = Bench(); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    b.lidar_on = False; b.run(1.5)
    check(b.fw.faults & L.FL_LIDAR_LOST and abs(b.v) < 0.02, "LiDAR dropout stops the robot")
    b.lidar_on = True; b.run(1.0)
    for _ in range(5):
        b.cmd(0.4, 0.0); b.run(0.1)
    check(not (b.fw.faults & L.FL_LIDAR_LOST) and abs(b.v) > 0.1, "...and it recovers when the LiDAR returns")

    b = Bench(); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    b.mode(L.MODE_ESTOP); b.run(0.05)
    check(b.fw.duty() == (0.0, 0.0) and b.fw.state == 5, "E-STOP: motors off immediately")
    b.cmd(0.5, 0.0); b.mode(L.MODE_RUN); b.run(0.5)
    check(b.fw.state == 5 and abs(b.v) < 0.02, "E-STOP latches: RUN is refused until cleared with IDLE")
    b.mode(L.MODE_IDLE); b.run(0.2)
    check(b.fw.state == 2, "IDLE clears the E-STOP")

    b = Bench(); b.boot_to_run()
    garbage = bytes(range(256)) * 3
    bad = bytearray(L.cmd_vel(0.5, 0.0)); bad[6] ^= 0xFF
    b.send(garbage); b.send(bytes(bad)); b.run(0.3)
    check(abs(b.v) < 0.02, "line noise and a corrupt frame do not move the robot")
    for _ in range(5):
        b.cmd(0.3, 0.0); b.run(0.1)
    check(abs(b.v) > 0.1, "valid frames still work after noise")


def test_event_sensors():
    section("event sensors -> UP_EVENT")
    b = Bench(); b.boot_to_run(); b.clear()
    b.run(2.0); b.gas = 1200; b.run(1.0)
    check(b.count(L.UP_EVENT) == 0, "gas spike shorter than the hold time is ignored")
    b.gas = 600; b.run(3.0); b.gas = 1200; b.run(3.0)
    ev = [struct.unpack(L.EVENT_FMT, p) for t, p in b.frames if t == L.UP_EVENT]
    check(len(ev) == 1 and ev[0][0] == 1 and ev[0][1] >= 1100, "sustained gas -> exactly one GAS event with the sensor value %s" % ev)

    b = Bench(); b.boot_to_run(); b.clear()
    for _ in range(2):                            # two bumps: not a collapse
        b.tilt = True; b.run(0.1); b.tilt = False; b.run(0.1)
    check(b.count(L.UP_EVENT) == 0, "two bumps ignored")
    b.run(2.0)
    for _ in range(3):
        b.tilt = True; b.run(0.1); b.tilt = False; b.run(0.1)
    ev = [struct.unpack(L.EVENT_FMT, p)[0] for t, p in b.frames if t == L.UP_EVENT]
    check(ev == [2], "three trips in 1 s -> COLLAPSE event")

    b = Bench(); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    b.clear(); b.pir = True; b.run(0.05); b.pir = False
    for _ in range(3):
        b.cmd(0.5, 0.0); b.run(0.1)
    check(b.fw.state == 4 and abs(b.v) < 0.1 or b.fw.state == 4, "PIR candidate while driving -> VERIFY (robot stops to look again)")
    b.run(4.0)
    check(b.count(L.UP_EVENT) == 0 and b.fw.stat("pir_false") == 1, "single PIR blip is a false alarm: no event")
    check(b.fw.state == 3, "back to RUN after the check")

    b = Bench(); b.boot_to_run()
    for _ in range(10):
        b.cmd(0.5, 0.0); b.run(0.1)
    b.clear(); b.pir = True
    for _ in range(6):
        b.cmd(0.5, 0.0); b.run(0.1)
    check(abs(b.v) < 0.1, "the Writer holds still while verifying (host command overridden)")
    b.run(2.0)
    ev = [struct.unpack(L.EVENT_FMT, p)[0] for t, p in b.frames if t == L.UP_EVENT]
    check(ev == [3], "PIR still active after settling -> TRAPPED event")


def _beacon_pkt(src, dst, btype, payload, seq=0):
    return bytes([btype, src, dst, seq]) + payload


def test_hub():
    section("hub: radio -> command post, GPS translation, de-duplication, mission hand-over")
    hub = F.Fw(F.ROLE_HUB, fwlib_params()); hub.begin()
    sent = []
    hub.set_radio_hook(lambda s, d, data: sent.append((d, data)) or True)
    parser = L.FrameParser(); frames = []
    cal = CalibrationData(34.4250, 8.7842, 215)
    hub.host_push(L.encode(L.DN_SET_CALIB, struct.pack(L.CALIB_FMT, int(34.4250 * 1e7), int(8.7842 * 1e7), 215.0)))
    t = 0

    def run(ms):
        nonlocal t
        for _ in range(ms // 10):
            t += 10_000; hub.set_time_us(t); hub.poll(); frames.extend(parser.feed(hub.host_pop()))

    run(50)
    b = make_beacon(7, 1, EventType.TRAPPED, 1140, -840, 100, 1, 73600)
    pkt = _beacon_pkt(F.ROLE_WRITER, 0, 0x01, L.pack_beacon(b))
    hub.radio_push(pkt); run(30)
    rx = [p for ty, p in frames if ty == L.UP_BEACON_RX]
    check(len(rx) == 1, "beacon forwarded to the command post")
    lat, lon = struct.unpack("<ii", rx[0][18:26])
    g = translate_to_gps(b, cal)
    check(abs(lat / 1e7 - g.lat) < 2e-7 and abs(lon / 1e7 - g.lon) < 2e-7, "firmware GPS == Python GPS (%.7f, %.7f)" % (lat / 1e7, lon / 1e7))
    check(any(d == 1 and data[0] == 0x02 and data[4] == 0x01 and data[5] == 7 for d, data in sent), "hub acknowledges the beacon (RF_ACK)")
    hub.radio_push(pkt); run(30)
    check(len([1 for ty, p in frames if ty == L.UP_BEACON_RX]) == 1 and hub.stat("hub_dup") == 1, "retransmission acked again but not forwarded twice")
    # mission hand-over
    b2 = make_beacon(8, 1, EventType.GAS, 300, 50, 0, 900, 90000)
    tgt = struct.pack("<B", 2) + L.pack_beacon(b) + L.pack_beacon(b2)
    sent.clear()
    hub.host_push(L.encode(L.DN_MISSION_PUSH, tgt)); run(30)
    hub.radio_push(_beacon_pkt(F.ROLE_EXECUTOR, 0, 0x12, b"")); run(30)
    tg = [data for d, data in sent if d == 2 and data[0] == 0x10]
    check(len(tg) >= 2 and tg[0][4] == 0 and tg[0][5] == 2, "Executor's mission request answered with the target list")
    hub.radio_push(_beacon_pkt(F.ROLE_EXECUTOR, 0, 0x11, bytes([7]))); run(30)
    check(any(ty == L.UP_TARGET_REACHED and p[0] == 7 for ty, p in frames), "REACHED forwarded to the command post")
    sent.clear(); hub.radio_push(_beacon_pkt(F.ROLE_EXECUTOR, 0, 0x12, b"")); run(30)
    tg = [data for d, data in sent if d == 2 and data[0] == 0x10]
    check(len(tg) == 1 and tg[0][5] == 1, "a reached target is dropped from the stored mission")


def fwlib_params():
    return {}


def test_writer_beacon_delivery():
    section("Writer + hub over a LOSSY radio: retries until acked; TTL aging drops stale beacons")
    for p_ok in (1.0, 0.5, 0.25):
        medium = fwlib.RadioMedium(p_ok=p_ok, seed=5)
        w = Bench(); hub = F.Fw(F.ROLE_HUB, fwlib_params()); hub.begin()
        medium.attach(F.NODE_WRITER if hasattr(F, "NODE_WRITER") else 1, w.fw); medium.attach(0, hub)
        hp = L.FrameParser(); hub_frames = []

        def each(t):
            hub_frames.extend(hp.feed(hub.host_pop()))
        w.boot_to_run(others=(hub,))
        hub.host_push(L.encode(L.DN_SET_CALIB, struct.pack(L.CALIB_FMT, int(34.4250e7), int(8.7842e7), 90.0)))
        for i, (et, x, y) in enumerate([(EventType.GAS, 900, 1700), (EventType.COLLAPSE, 2900, 880), (EventType.TRAPPED, 1500, 350)]):
            b = make_beacon(i + 1, 1, et, x, y, 0, 500, w.fw and 0)
            b.timestamp_ms = 0
            w.send(L.encode(L.DN_BEACON_TX, L.pack_beacon(b)))
        w.run(40.0, others=(hub,), each=each)
        got = sorted(p[2:18][0] for t, p in [(ty, pl) for ty, pl in hub_frames] if t == L.UP_BEACON_RX)
        check(got == [1, 2, 3], "p_ok=%.2f: all 3 beacons reach the command post exactly once (got %s)" % (p_ok, got))
        check(w.fw.stat("acked") == 3 and w.fw.stat("pending") == 0, "p_ok=%.2f: Writer saw all 3 acks" % p_ok)
        if p_ok < 1:
            check(w.fw.stat("radio_fail") > 0, "p_ok=%.2f: link really was lossy (%d failed sends)" % (p_ok, w.fw.stat("radio_fail")))
        w.fw.close(); hub.close()

    # no hub in range: the beacon is retried, then dies of old age (TTL), never blocking the robot
    w = Bench(); medium = fwlib.RadioMedium(p_ok=1.0); medium.attach(1, w.fw)    # hub absent
    w.boot_to_run()
    b = make_beacon(1, 1, EventType.EXPLORED, 0, 0, 0, 0, 0)                     # LOW: lives 150 s
    w.send(L.encode(L.DN_BEACON_TX, L.pack_beacon(b)))
    w.run(20.0)
    check(w.fw.stat("pending") == 1 and w.fw.stat("radio_fail") > 3, "unacked beacon keeps being retried")
    check(w.fw.faults & L.FL_RADIO_LOST, "RADIO_LOST fault raised after repeated failures")
    for _ in range(10):
        w.cmd(0.3, 0.0); w.run(0.1)
    check(abs(w.v) > 0.15, "radio loss does not stop the robot (v=%.2f)" % w.v)
    w.fw.close()


def test_executor_mission_radio():
    section("Executor: asks for the mission over the radio, forwards targets, reports REACHED reliably")
    medium = fwlib.RadioMedium(p_ok=0.6, seed=2)
    ex = Bench(role=F.ROLE_EXECUTOR); hub = F.Fw(F.ROLE_HUB, fwlib_params()); hub.begin()
    medium.attach(2, ex.fw); medium.attach(0, hub)
    hp = L.FrameParser(); hub_frames = []
    b1 = make_beacon(1, 1, EventType.TRAPPED, 1500, 350, 0, 1, 70000)
    b2 = make_beacon(2, 1, EventType.GAS, 900, 1700, 0, 900, 390000)
    hub.host_push(L.encode(L.DN_MISSION_PUSH, struct.pack("<B", 2) + L.pack_beacon(b1) + L.pack_beacon(b2)))
    ex.boot_to_run(others=(hub,))
    ex.run(8.0, others=(hub,), each=lambda t: hub_frames.extend(hp.feed(hub.host_pop())))
    got = [(p[0], p[1], p[2]) for ty, p in ex.frames if ty == L.UP_MISSION_TARGET]
    check(any(g[2] == 1 for g in got) and any(g[2] == 2 for g in got), "both targets reach the Executor host over a 60 %% link (%s)" % got)
    ex.send(L.encode(L.DN_TARGET_REACHED, bytes([1])))
    ex.run(15.0, others=(hub,), each=lambda t: hub_frames.extend(hp.feed(hub.host_pop())))
    check(any(ty == L.UP_TARGET_REACHED and p[0] == 1 for ty, p in hub_frames), "REACHED got through the lossy link (retried until acked)")
    ex.fw.close(); hub.close()


if __name__ == "__main__":
    for fn in (test_wire_sizes, test_boot_and_scan_stream, test_drive_and_odometry, test_safety_layer,
               test_event_sensors, test_hub, test_writer_beacon_delivery, test_executor_mission_radio):
        fn()
    print("\n%d checks, %d failures" % (checks, fails))
    sys.exit(1 if fails else 0)
