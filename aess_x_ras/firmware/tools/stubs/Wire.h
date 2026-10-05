#pragma once
#include <stdint.h>
class TwoWire {
 public:
  bool begin(int, int); void setClock(uint32_t); void beginTransmission(int); int write(uint8_t);
  uint8_t endTransmission(bool = true); uint8_t requestFrom(int, int); int available(); int read();
};
extern TwoWire Wire;
