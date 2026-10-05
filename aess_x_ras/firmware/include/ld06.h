// ld06.h - LDROBOT LD06 360-degree LiDAR parser and scan assembler.
//
// Packet (47 bytes, 230400 baud): 0x54 | 0x2C | speed u16 (deg/s) | start angle u16 (0.01 deg)
//   | 12 x { distance u16 (mm), intensity u8 } | end angle u16 (0.01 deg) | timestamp u16 (ms) | crc8
// Angles increase CLOCKWISE seen from above. We convert to the SLAM convention (CCW from +x, the
// robot's forward direction) and bin one sweep into LM_SCAN_BINS bins spaced 360/359 degrees apart
// starting at -180 deg - the same layout as the simulated LiDAR, so the host feeds BreezySLAM directly.
// NOTE: sign/zero offset and the CRC variant must be verified on the real unit at bring-up.
#ifndef LM_LD06_H
#define LM_LD06_H
#include <stdint.h>
#include <math.h>
#include "crc.h"
#include "params.h"
#include "protocol.h"

struct Ld06Packet {
  uint16_t speed;
  float start_deg, end_deg;
  uint16_t dist[12];
  uint8_t inten[12];
};

class Ld06Parser {
 public:
  // Returns true when a CRC-valid packet has just been completed (read it with packet()).
  bool feed(uint8_t b) {
    if (n_ == 0) { if (b == 0x54) buf_[n_++] = b; return false; }
    if (n_ == 1) { if (b == 0x2C) { buf_[n_++] = b; } else n_ = (b == 0x54) ? (buf_[0] = 0x54, 1) : 0; return false; }
    buf_[n_++] = b;
    if (n_ < 47) return false;
    n_ = 0;
    if (crc8_ld06(buf_, 46) != buf_[46]) { badCrc_++; return false; }
    pkt_.speed = buf_[2] | (buf_[3] << 8);
    pkt_.start_deg = (buf_[4] | (buf_[5] << 8)) / 100.0f;
    for (int i = 0; i < 12; i++) {
      pkt_.dist[i]  = buf_[6 + 3 * i] | (buf_[7 + 3 * i] << 8);
      pkt_.inten[i] = buf_[8 + 3 * i];
    }
    pkt_.end_deg = (buf_[42] | (buf_[43] << 8)) / 100.0f;
    good_++;
    return true;
  }
  const Ld06Packet &packet() const { return pkt_; }
  uint32_t good() const { return good_; }
  uint32_t badCrc() const { return badCrc_; }
 private:
  uint8_t buf_[47];
  uint8_t n_ = 0;
  Ld06Packet pkt_;
  uint32_t good_ = 0, badCrc_ = 0;
};

// Collects packets into one sweep; a sweep completes when the LD06 angle wraps past 360 -> 0.
class ScanAssembler {
 public:
  explicit ScanAssembler(const RobotParams *p) : p_(p) { clear(); }
  void clear() { for (int i = 0; i < LM_SCAN_BINS; i++) bins_[i] = 0; count_ = 0; haveLast_ = false; }

  // Returns true when a full sweep is ready in bins().
  bool add(const Ld06Packet &pk) {
    bool done = false;
    if (haveLast_ && pk.start_deg < lastStart_ - 180.0f) {      // wrapped: previous sweep is complete
      for (int i = 0; i < LM_SCAN_BINS; i++) out_[i] = bins_[i];
      outCount_ = count_;
      for (int i = 0; i < LM_SCAN_BINS; i++) bins_[i] = 0;
      count_ = 0; done = true;
    }
    lastStart_ = pk.start_deg; haveLast_ = true;
    float span = pk.end_deg - pk.start_deg; if (span < 0) span += 360.0f;
    for (int i = 0; i < 12; i++) {
      uint16_t d = pk.dist[i];
      if (d < p_->lidar_min_mm || d > p_->lidar_max_mm || pk.inten[i] < p_->lidar_min_intensity) continue;
      float a_cw = pk.start_deg + span * i / 11.0f;
      float a = p_->lidar_sign * a_cw + p_->lidar_offset_deg;   // robot frame, CCW
      a = fmodf(a, 360.0f); if (a >= 180.0f) a -= 360.0f; if (a < -180.0f) a += 360.0f;
      int idx = (int)lroundf((a + 180.0f) * (LM_SCAN_BINS - 1) / 360.0f);
      if (idx < 0) idx = 0;
      if (idx >= LM_SCAN_BINS) idx = LM_SCAN_BINS - 1;
      if (bins_[idx] == 0 || d < bins_[idx]) bins_[idx] = d;     // keep the nearest return per bin
      count_++;
    }
    return done;
  }
  const uint16_t *bins() const { return out_; }
  int pointsInSweep() const { return outCount_; }
 private:
  const RobotParams *p_;
  uint16_t bins_[LM_SCAN_BINS], out_[LM_SCAN_BINS];
  int count_, outCount_ = 0;
  float lastStart_ = 0; bool haveLast_;
};
#endif
