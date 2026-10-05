#ifndef LM_HAL_ESP32_H
#define LM_HAL_ESP32_H
#include "hal.h"
#include "protocol.h"

class Esp32Hal : public Hal {
 public:
  void begin(Role role);                          // configure only the peripherals this role needs
  bool radioPresent() const { return radioOk_; }

  uint32_t millis() override;
  uint32_t micros() override;
  void setMotor(Side s, float duty) override;
  int32_t encoderCount(Side s) override;
  bool readGyroZ(float *dps) override;
  int lidarAvailable() override;
  int lidarRead() override;
  int hostAvailable() override;
  int hostRead() override;
  void hostWrite(const uint8_t *buf, size_t len) override;
  bool radioSend(uint8_t dst_node, const uint8_t *buf, uint8_t len) override;
  int radioRead(uint8_t *buf) override;
  uint8_t radioLastRetries() override { return lastRetries_; }
  int readGasRaw() override;
  bool readTilt() override;
  bool readPir() override;
  uint16_t batteryMv() override;
  void setLed(bool on) override;
 private:
  Role role_ = ROLE_HUB;
  bool radioOk_ = false;
  uint8_t self_ = 0, lastRetries_ = 0;
  bool imuOk_ = false;
};
#endif
