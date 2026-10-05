// hal.h - hardware abstraction layer.
// Everything the robot logic needs from the outside world goes through this
// interface. src/hal_esp32.cpp implements it on real hardware (Arduino-ESP32);
// host/sim_hal.cpp implements it for unit tests and the software-in-the-loop
// simulation. The application code (apps.cpp etc.) never touches Arduino APIs.
#ifndef LM_HAL_H
#define LM_HAL_H

#include <stdint.h>
#include <stddef.h>

enum Side { LEFT = 0, RIGHT = 1 };

class Hal {
 public:
  virtual ~Hal() {}
  // --- time
  virtual uint32_t millis() = 0;
  virtual uint32_t micros() = 0;

  // --- drive (duty in [-1, 1], positive = forward)
  virtual void setMotor(Side s, float duty) = 0;
  virtual int32_t encoderCount(Side s) = 0;       // signed, +forward

  // --- IMU
  virtual bool readGyroZ(float *dps) = 0;         // false = IMU not responding

  // --- LiDAR UART (LD06, 230400 8N1)
  virtual int lidarAvailable() = 0;
  virtual int lidarRead() = 0;                    // -1 if empty

  // --- host link (USB serial / TCP)
  virtual int hostAvailable() = 0;
  virtual int hostRead() = 0;
  virtual void hostWrite(const uint8_t *buf, size_t len) = 0;

  // --- nRF24 radio. Destinations are logical node ids; the HAL maps them to pipe addresses.
  virtual bool radioSend(uint8_t dst_node, const uint8_t *buf, uint8_t len) = 0;  // true = auto-ACKed
  virtual int radioRead(uint8_t *buf) = 0;        // returns length, 0 if nothing waiting
  virtual uint8_t radioLastRetries() = 0;         // ARC_CNT of the last send (link quality)

  // --- event sensors (Writer)
  virtual int readGasRaw() = 0;                   // 12-bit ADC counts (0-4095) at the divider output
  virtual bool readTilt() = 0;                    // true = switch tripped
  virtual bool readPir() = 0;                     // true = motion

  // --- misc
  virtual uint16_t batteryMv() = 0;
  virtual void setLed(bool on) = 0;
};

#endif
