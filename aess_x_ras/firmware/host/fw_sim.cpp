// fw_sim.cpp - C ABI around the firmware so Python (ctypes) can drive complete firmware
// instances inside the PyBullet simulation: the SAME apps.cpp that runs on the ESP32.
#include "sim_hal.h"
#include "apps.h"
#include <string.h>

struct Instance {
  SimHal hal;
  AppBase *app = nullptr;
  RobotBase *robot = nullptr;
  WriterApp *writer = nullptr;
  ExecutorApp *executor = nullptr;
  HubApp *hub = nullptr;
  RobotParams params;
};

extern "C" {
#define EXPORT __attribute__((visibility("default")))

EXPORT Instance *fw_create(int role) {
  Instance *i = new Instance();
  i->params.warmup_enabled = false;                       // the simulation does not wait for sensor preheat
  i->hal.self_node = role == ROLE_WRITER ? NODE_WRITER : (role == ROLE_EXECUTOR ? NODE_EXECUTOR : NODE_HUB);
  return i;
}
// Override the chassis geometry etc. before fw_begin().
EXPORT void fw_set_param(Instance *i, const char *name, double v) {
  RobotParams &p = i->params;
#define P(n) if (!strcmp(name, #n)) { p.n = (decltype(p.n))v; return; }
  P(wheel_diameter_m) P(wheel_base_m) P(encoder_ticks_rev) P(max_wheel_speed_ms) P(max_v_ms) P(max_w_rads)
  P(kp) P(ki) P(kd) P(max_accel_ms2) P(min_duty) P(gyro_weight) P(gyro_scale) P(lidar_sign) P(lidar_offset_deg)
  P(obstacle_stop_mm) P(obstacle_cone_deg) P(cmd_timeout_ms) P(lidar_timeout_ms) P(warmup_enabled)
  P(gas_rise_counts) P(gas_hold_ms) P(tilt_min_edges) P(tilt_window_ms) P(pir_verify_ms) P(pir_settle_ms)
  P(event_cooldown_ms) P(heartbeat_ms) P(mission_req_ms) P(mq2_warmup_ms) P(pir_warmup_ms) P(lidar_max_mm)
}
EXPORT void fw_begin(Instance *i, int role) {
  if (role == ROLE_WRITER)        { i->writer = new WriterApp(&i->hal, i->params); i->robot = i->writer; i->app = i->writer; }
  else if (role == ROLE_EXECUTOR) { i->executor = new ExecutorApp(&i->hal, i->params); i->robot = i->executor; i->app = i->executor; }
  else                            { i->hub = new HubApp(&i->hal, i->params); i->app = i->hub; }
  i->app->begin();
}
EXPORT void fw_destroy(Instance *i) { delete i->app; delete i; }

EXPORT void fw_set_time_us(Instance *i, uint64_t us) { i->hal.now_us = us; }
EXPORT void fw_poll(Instance *i) { i->app->poll(); }

// ---- inputs
EXPORT void fw_set_encoders(Instance *i, int32_t l, int32_t r) { i->hal.enc[0] = l; i->hal.enc[1] = r; }
EXPORT void fw_set_gyro(Instance *i, float dps, int ok) { i->hal.gyro_dps = dps; i->hal.gyro_ok = ok != 0; }
EXPORT void fw_lidar_push(Instance *i, const uint8_t *b, int n) { i->hal.lidar_rx.insert(i->hal.lidar_rx.end(), b, b + n); }
EXPORT void fw_host_push(Instance *i, const uint8_t *b, int n) { i->hal.host_rx.insert(i->hal.host_rx.end(), b, b + n); }
EXPORT void fw_set_sensors(Instance *i, int gas, int tilt, int pir) { i->hal.gas_raw = gas; i->hal.tilt = tilt != 0; i->hal.pir = pir != 0; }
EXPORT void fw_radio_push(Instance *i, const uint8_t *b, int n) { i->hal.radio_rx.push_back(std::vector<uint8_t>(b, b + n)); }
EXPORT void fw_set_radio_hook(Instance *i, RadioSendHook h, void *user) { i->hal.hook = h; i->hal.hook_user = user; }

// ---- outputs
EXPORT void fw_get_duty(Instance *i, float *l, float *r) { *l = i->hal.duty[0]; *r = i->hal.duty[1]; }
EXPORT int fw_host_pop(Instance *i, uint8_t *out, int max) {
  int n = (int)i->hal.host_tx.size(); if (n > max) n = max;
  memcpy(out, i->hal.host_tx.data(), n);
  i->hal.host_tx.erase(i->hal.host_tx.begin(), i->hal.host_tx.begin() + n);
  return n;
}
EXPORT void fw_sizes(int *out) {                         // sizeof() of every wire struct, checked by the Python tests
  out[0] = sizeof(BeaconMessage); out[1] = sizeof(ScanMsg); out[2] = sizeof(EventMsg); out[3] = sizeof(TelemetryMsg);
  out[4] = sizeof(CmdVelMsg); out[5] = sizeof(SetCalibMsg); out[6] = sizeof(BeaconRxMsg); out[7] = sizeof(NodeStatusMsg);
  out[8] = sizeof(MissionTargetMsg); out[9] = sizeof(RfHeartbeat); out[10] = sizeof(RfTarget);
}
EXPORT int fw_state(Instance *i) { return i->app->state(); }
EXPORT int fw_faults(Instance *i) { return i->robot ? i->robot->faults() : 0; }
EXPORT float fw_odo(Instance *i, int which) {            // 0 x, 1 y, 2 yaw, 3 distance
  if (!i->robot) return 0;
  const Odometry &o = i->robot->odometry();
  return which == 0 ? o.x() : which == 1 ? o.y() : which == 2 ? o.yaw() : o.distance();
}
EXPORT int fw_stat(Instance *i, int which) {             // misc counters for the tests
  switch (which) {
    case 0: return i->writer ? (int)i->writer->beaconsAcked() : 0;
    case 1: return i->writer ? i->writer->beaconsPending() : 0;
    case 2: return i->writer ? (int)i->writer->pirFalseAlarms() : 0;
    case 3: return i->executor ? (int)i->executor->targetsReceived() : 0;
    case 4: return i->hub ? (int)i->hub->beaconsForwarded() : 0;
    case 5: return i->hub ? (int)i->hub->duplicates() : 0;
    case 6: return (int)i->hal.radio_tx_count;
    case 7: return (int)i->hal.radio_tx_fail;
    case 8: return (int)i->app->bad_frames();
    case 9: return i->robot ? (int)i->robot->scansSent() : 0;
    case 10: return i->writer ? (int)i->writer->eventsFired(EVENT_GAS) : 0;
    case 11: return i->writer ? (int)i->writer->eventsFired(EVENT_COLLAPSE) : 0;
    case 12: return i->writer ? (int)i->writer->eventsFired(EVENT_TRAPPED) : 0;
    default: return 0;
  }
}
}
