// crc.h - checksums used by the host link (CRC-16/CCITT-FALSE) and the LD06 LiDAR (CRC-8, poly 0x4D).
#ifndef LM_CRC_H
#define LM_CRC_H
#include <stdint.h>
#include <stddef.h>

inline uint16_t crc16_ccitt(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// LD06 packets end with a CRC-8 (polynomial 0x4D, init 0, MSB first) over the first 46 bytes.
inline uint8_t crc8_ld06(const uint8_t *d, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
  }
  return crc;
}
#endif
