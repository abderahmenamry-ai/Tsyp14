// test_core.cpp - unit tests for the portable firmware modules. Build & run: make test (see Makefile)
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "protocol.h"
#include "link.h"
#include "ld06.h"
#include "odometry.h"
#include "motor_control.h"
#include "detectors.h"
#include "beacon_sender.h"

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

static void test_layout_and_crc() {
  printf("layout + crc\n");
  CHECK(sizeof(BeaconMessage) == 16);
  CHECK(sizeof(ScanMsg) == 18 + 2 * LM_SCAN_BINS);
  CHECK(sizeof(RfTarget) + RF_HDR <= RF_MAX_PACKET);
  CHECK(sizeof(BeaconMessage) + RF_HDR <= RF_MAX_PACKET);
  CHECK(sizeof(RfHeartbeat) + RF_HDR <= RF_MAX_PACKET);
  const char *s = "123456789";
  CHECK(crc16_ccitt((const uint8_t *)s, 9) == 0x29B1);          // CRC-16/CCITT-FALSE check value
}

static void test_framing() {
  printf("framing\n");
  uint8_t out[LM_MAX_PAYLOAD + 8];
  CmdVelMsg c = { 300, -500, 7 };
  size_t n = linkEncode(DN_CMD_VEL, &c, sizeof c, out);
  CHECK(n == sizeof c + 7);
  FrameParser p; bool got = false;
  // line noise (no false start-of-frame), then the frame: parser must lock on
  const uint8_t junk[] = { 0x00, 0xAA, 0x13, 0xFF, 0x7E };
  for (uint8_t b : junk) got |= p.feed(b);
  CHECK(!got);
  for (size_t i = 0; i < n; i++) got = p.feed(out[i]);
  CHECK(got && p.type() == DN_CMD_VEL && p.length() == sizeof c);
  // a false start-of-frame may swallow the next frame, but the stream must recover on later frames
  FrameParser q; const uint8_t junk2[] = { 0xAA, 0x55, 0x33 };
  for (uint8_t b : junk2) q.feed(b);
  int okFrames = 0;
  for (int rep = 0; rep < 6; rep++) for (size_t i = 0; i < n; i++) okFrames += q.feed(out[i]);
  CHECK(okFrames >= 4);
  CmdVelMsg d; memcpy(&d, p.payload(), sizeof d);
  CHECK(d.v_mm_s == 300 && d.w_mrad_s == -500 && d.seq == 7);
  // corrupt one payload byte -> rejected, then the next good frame still parses
  uint8_t bad[LM_MAX_PAYLOAD + 8]; memcpy(bad, out, n); bad[6] ^= 0x40;
  bool any = false; for (size_t i = 0; i < n; i++) any |= p.feed(bad[i]);
  CHECK(!any && p.badFrames() >= 1);
  got = false; for (size_t i = 0; i < n; i++) got = p.feed(out[i]);
  CHECK(got);
  // empty payload and a max-size scan frame
  n = linkEncode(DN_MISSION_REQ, nullptr, 0, out); got = false; for (size_t i = 0; i < n; i++) got = p.feed(out[i]);
  CHECK(got && p.length() == 0);
  static ScanMsg sm; memset(&sm, 0, sizeof sm); sm.n = LM_SCAN_BINS; sm.range_mm[180] = 1234;
  static uint8_t big[LM_MAX_PAYLOAD + 8]; n = linkEncode(UP_SCAN, &sm, sizeof sm, big);
  got = false; for (size_t i = 0; i < n; i++) got = p.feed(big[i]);
  CHECK(got && p.length() == sizeof sm);
  ScanMsg back; memcpy(&back, p.payload(), sizeof back); CHECK(back.range_mm[180] == 1234);
}

// ---- LD06: build packets the way the sensor does
static size_t ld06_packet(uint8_t *o, float start_deg, float step_deg, const uint16_t *d) {
  o[0] = 0x54; o[1] = 0x2C; o[2] = 0x10; o[3] = 0x0E;               // 3600 deg/s
  uint16_t sa = (uint16_t)lroundf(start_deg * 100), ea = (uint16_t)lroundf(fmodf(start_deg + 11 * step_deg, 360.0f) * 100);
  o[4] = sa & 0xFF; o[5] = sa >> 8;
  for (int i = 0; i < 12; i++) { o[6 + 3 * i] = d[i] & 0xFF; o[7 + 3 * i] = d[i] >> 8; o[8 + 3 * i] = 100; }
  o[42] = ea & 0xFF; o[43] = ea >> 8; o[44] = 0; o[45] = 0;
  o[46] = crc8_ld06(o, 46);
  return 47;
}

static void test_ld06() {
  printf("ld06 parser + scan assembly\n");
  RobotParams P; Ld06Parser ps; ScanAssembler as(&P);
  // full revolution, 12 points per packet, 0.8 deg per point -> 450 points = 37.5 packets (use 38)
  int sweeps = 0;
  for (int rev = 0; rev < 3; rev++) {
    for (int k = 0; k < 38; k++) {
      float start = k * 12 * 0.8f; if (start >= 360.0f) break;
      uint16_t d[12];
      for (int i = 0; i < 12; i++) {
        float a = start + i * 0.8f;                                   // clockwise LD06 angle
        d[i] = (a >= 89.0f && a <= 91.0f) ? 1500 : 3000;              // wall at LD06 90 deg (right side)
      }
      uint8_t buf[47]; ld06_packet(buf, start, 0.8f, d);
      for (int i = 0; i < 47; i++) if (ps.feed(buf[i])) { if (as.add(ps.packet())) sweeps++; }
    }
  }
  CHECK(sweeps == 2);                                                // sweep completes when the angle wraps
  CHECK(ps.badCrc() == 0 && ps.good() > 100);
  const uint16_t *b = as.bins();
  // LD06 90 deg clockwise == robot right == -90 deg CCW == bin 90
  NEAR(b[90], 1500, 0); CHECK(b[180] == 3000);
  // a corrupted byte must be rejected
  uint8_t buf[47]; uint16_t d[12] = {1000}; ld06_packet(buf, 10, 0.8f, d); buf[20] ^= 0x01;
  bool ok = false; for (int i = 0; i < 47; i++) ok |= ps.feed(buf[i]);
  CHECK(!ok && ps.badCrc() == 1);
}

static void test_odometry() {
  printf("odometry\n");
  RobotParams P; Odometry o(&P);
  float mpt = (float)M_PI * P.wheel_diameter_m / P.encoder_ticks_rev;
  int ticks = ((int)lroundf(1.0f / mpt) / 100) * 100;                // ~1 m, multiple of 100 so the loop is exact
  float metres = ticks * mpt;
  for (int i = 0; i < 100; i++) o.update(ticks / 100, ticks / 100, true, 0.0f, 0.01f);
  NEAR(o.x(), metres, 0.001); NEAR(o.y(), 0.0, 0.001);
  float dxy, dth, dt; o.takeDelta(&dxy, &dth, &dt);
  NEAR(dxy, metres * 1000.0, 1.0); NEAR(dth, 0.0, 0.1); NEAR(dt, 1.0, 0.001);
  // in-place turn of 90 deg (gyro says 90 deg/s for 1 s; wheels agree)
  Odometry t(&P);
  float arc = (float)M_PI / 2 * P.wheel_base_m / 2;                  // each wheel travels this far
  int tk = ((int)lroundf(arc / mpt) / 100) * 100;
  for (int i = 0; i < 100; i++) t.update(-tk / 100, tk / 100, true, 90.0f, 0.01f);
  NEAR(t.yaw() * 180 / M_PI, 90.0, 2.0);
  t.takeDelta(&dxy, &dth, &dt); NEAR(dth, 90.0, 2.0); NEAR(dxy, 0.0, 5.0);
  // gyro lost -> encoder-only heading still works
  Odometry e(&P);
  int tk1 = (int)lroundf(arc / mpt);
  for (int i = 0; i < tk1; i++) e.update(-1, 1, false, 0.0f, 0.005f);
  NEAR(e.yaw() * 180 / M_PI, 90.0, 1.0);
}

static void test_pid() {
  printf("wheel pid + drive controller\n");
  RobotParams P; DriveController d(&P);
  // first-order motor + wheel model: speed follows duty * max_speed with tau = 80 ms
  float vl = 0, vr = 0; float dl = 0, dr = 0;
  d.setCommand(0.4f, 0.0f);
  for (int i = 0; i < 300; i++) {                                    // 3 s at 100 Hz
    d.update(vl, vr, 0.01f, &dl, &dr);
    vl += (dl * P.max_wheel_speed_ms * 0.93f - vl) * (0.01f / 0.08f);   // plant is 7 % weaker than nominal
    vr += (dr * P.max_wheel_speed_ms * 0.93f - vr) * (0.01f / 0.08f);
  }
  NEAR(vl, 0.4, 0.02); NEAR(vr, 0.4, 0.02);                          // integral term removes the 7 % error
  d.setCommand(0.0f, 1.0f);                                          // spin in place, w = 1 rad/s
  for (int i = 0; i < 300; i++) {
    d.update(vl, vr, 0.01f, &dl, &dr);
    vl += (dl * P.max_wheel_speed_ms * 0.93f - vl) * (0.01f / 0.08f);
    vr += (dr * P.max_wheel_speed_ms * 0.93f - vr) * (0.01f / 0.08f);
  }
  NEAR((vr - vl) / P.wheel_base_m, 1.0, 0.05);
  d.setCommand(5.0f, 9.0f); CHECK(d.v() <= P.max_v_ms + 1e-6 && d.w() <= P.max_w_rads + 1e-6);   // command limits
}

static void test_detectors() {
  printf("event detectors\n");
  RobotParams P; P.warmup_enabled = false;
  GasDetector g(&P); g.start(0);
  uint32_t t = 0; bool fired = false;
  for (t = 0; t < 5000; t += 20) fired |= g.update(t, 600);          // clean air baseline
  CHECK(!fired);
  for (; t < 5500; t += 20) fired |= g.update(t, 1200);              // 0.5 s spike: below the hold time
  CHECK(!fired);
  for (; t < 8000; t += 20) fired |= g.update(t, 600);
  int n = 0; for (; t < 11000; t += 20) n += g.update(t, 1200);      // 3 s sustained
  CHECK(n == 1);                                                     // fires exactly once, then latches
  for (; t < 16000; t += 20) g.update(t, 600);                       // back to clean air releases the latch
  n = 0; for (; t < 19000; t += 20) n += g.update(t, 1200); CHECK(n == 1);

  RobotParams W; W.warmup_enabled = true; W.mq2_warmup_ms = 10000;
  GasDetector gw(&W); gw.start(0); bool early = false;
  for (t = 0; t < 9000; t += 20) early |= gw.update(t, 3000);        // reading is huge, but still warming up
  CHECK(!early);

  TiltDetector td(&P); uint32_t k = 0; bool f = false;
  for (int rep = 0; rep < 2; rep++) { f |= td.update(k, true); k += 100; f |= td.update(k, false); k += 100; }
  CHECK(!f);                                                         // two bumps are not a collapse
  f = false; k += 3000;
  for (int rep = 0; rep < 3; rep++) { f |= td.update(k, true); k += 100; f |= td.update(k, false); k += 100; }
  CHECK(f);                                                          // three trips in < 1 s are
  f = false; k += 3000;
  for (int rep = 0; rep < 3; rep++) { f |= td.update(k, true); k += 1500; f |= td.update(k, false); k += 100; }
  CHECK(!f);                                                         // trips spread over seconds are not

  RobotParams Q; Q.warmup_enabled = true; Q.pir_warmup_ms = 5000;
  PirDetector pd(&Q); pd.start(0);
  CHECK(!pd.candidate(1000, true));  pd.candidate(1100, false);
  CHECK(pd.candidate(6000, true));
}

static void test_sender() {
  printf("beacon sender (retries + aging)\n");
  BeaconSender s;
  BeaconMessage hi = makeBeacon(1, 1, EVENT_TRAPPED, 100, 200, 0, 1, 1000);
  BeaconMessage lo = makeBeacon(2, 1, EVENT_EXPLORED, 0, 0, 0, 0, 1000);
  CHECK(s.add(hi, 1000)); CHECK(s.add(lo, 1000));
  const BeaconMessage *b = s.due(1000); CHECK(b && b->beacon_id == 1);
  s.sent(b, 1000);
  b = s.due(1000); CHECK(b && b->beacon_id == 2); s.sent(b, 1000);
  CHECK(s.due(1500) == nullptr);                                     // nothing due before the rebroadcast interval
  b = s.due(3000); CHECK(b && b->beacon_id == 1);                    // HIGH: every 2 s
  s.sent(b, 3000);
  CHECK(s.due(4000) == nullptr);
  b = s.due(11000); CHECK(b != nullptr);                             // LOW: every 10 s
  s.onAck(1); CHECK(s.pending() == 1);
  uint32_t tries = s.tries(2);
  CHECK(tries >= 1);
  // aging: LOW (TTL 5 ticks = 150 s) expires without an ack, HIGH would have lived 600 s
  BeaconSender s2; s2.add(lo, 1000); s2.add(hi, 1000);
  CHECK(s2.due(1000 + 149000) != nullptr);
  const BeaconMessage *x = s2.due(1000 + 151000);
  CHECK(x && x->beacon_id == 1);                                     // only the HIGH one is left
  CHECK(s2.expiredCount() == 1);
}

int main() {
  test_layout_and_crc(); test_framing(); test_ld06(); test_odometry(); test_pid(); test_detectors(); test_sender();
  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
