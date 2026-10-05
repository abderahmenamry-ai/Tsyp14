// sim_hal.h - Hal implementation for the host (unit tests + software-in-the-loop simulation).
// Time is virtual: the test/simulation advances it explicitly, so runs are deterministic.
#ifndef LM_SIM_HAL_H
#define LM_SIM_HAL_H
#include <deque>
#include <vector>
#include <stdint.h>
#include "hal.h"

typedef bool (*RadioSendHook)(void *user, int src_node, int dst_node, const uint8_t *buf, int len);

class SimHal : public Hal {
 public:
  uint64_t now_us = 0;
  float duty[2] = {0, 0};
  int32_t enc[2] = {0, 0};
  float gyro_dps = 0; bool gyro_ok = true;
  std::deque<uint8_t> lidar_rx, host_rx;
  std::vector<uint8_t> host_tx;
  std::deque<std::vector<uint8_t>> radio_rx;
  int gas_raw = 600; bool tilt = false, pir = false;
  uint16_t battery = 7400; bool led = false;
  uint8_t last_retries = 0;
  int self_node = 0;
  RadioSendHook hook = nullptr; void *hook_user = nullptr;
  uint32_t radio_tx_count = 0, radio_tx_fail = 0;

  uint32_t millis() override { return (uint32_t)(now_us / 1000); }
  uint32_t micros() override { return (uint32_t)now_us; }
  void setMotor(Side s, float d) override { duty[s] = d; }
  int32_t encoderCount(Side s) override { return enc[s]; }
  bool readGyroZ(float *dps) override { if (!gyro_ok) return false; *dps = gyro_dps; return true; }
  int lidarAvailable() override { return (int)lidar_rx.size(); }
  int lidarRead() override { if (lidar_rx.empty()) return -1; int b = lidar_rx.front(); lidar_rx.pop_front(); return b; }
  int hostAvailable() override { return (int)host_rx.size(); }
  int hostRead() override { if (host_rx.empty()) return -1; int b = host_rx.front(); host_rx.pop_front(); return b; }
  void hostWrite(const uint8_t *buf, size_t len) override { host_tx.insert(host_tx.end(), buf, buf + len); }
  bool radioSend(uint8_t dst, const uint8_t *buf, uint8_t len) override {
    radio_tx_count++;
    bool ok = hook ? hook(hook_user, self_node, dst, buf, len) : false;
    last_retries = ok ? 0 : 15; if (!ok) radio_tx_fail++;
    return ok;
  }
  int radioRead(uint8_t *buf) override {
    if (radio_rx.empty()) return 0;
    auto &p = radio_rx.front(); int n = (int)p.size();
    for (int i = 0; i < n; i++) buf[i] = p[i];
    radio_rx.pop_front(); return n;
  }
  uint8_t radioLastRetries() override { return last_retries; }
  int readGasRaw() override { return gas_raw; }
  bool readTilt() override { return tilt; }
  bool readPir() override { return pir; }
  uint16_t batteryMv() override { return battery; }
  void setLed(bool on) override { led = on; }
};
#endif
