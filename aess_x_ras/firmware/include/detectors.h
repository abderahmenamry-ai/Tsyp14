// detectors.h - event-sensor logic for the Writer: turns raw MQ-2 / SW-520D / HC-SR501 readings
// into robust events. Thresholds and the false-positive handling come from
// architecture/sensor_research.pdf and architecture/failure_cases.pdf.
#ifndef LM_DETECTORS_H
#define LM_DETECTORS_H
#include <stdint.h>
#include <math.h>
#include "params.h"

// MQ-2: gas/smoke anomaly = ADC counts held above a slowly learned clean-air baseline.
class GasDetector {
 public:
  explicit GasDetector(const RobotParams *p) : p_(p) {}
  void start(uint32_t now) { t0_ = now; }
  // returns true once when a sustained rise is confirmed
  bool update(uint32_t now, int raw) {
    last_ = raw;
    if (p_->warmup_enabled && now - t0_ < p_->mq2_warmup_ms) { baseline_ = raw; return false; }
    if (!init_) { baseline_ = raw; init_ = true; }
    bool above = (raw - baseline_) > p_->gas_rise_counts;
    if (!above) {                                            // learn the baseline only in clean air
      baseline_ += 0.002f * (raw - baseline_);
      aboveSince_ = 0; if (latched_ && (raw - baseline_) < 0.5f * p_->gas_rise_counts) latched_ = false;
      return false;
    }
    if (aboveSince_ == 0) aboveSince_ = now ? now : 1;
    if (!latched_ && now - aboveSince_ >= p_->gas_hold_ms) { latched_ = true; return true; }
    return false;
  }
  int lastRaw() const { return last_; }
  float baseline() const { return baseline_; }
 private:
  const RobotParams *p_;
  uint32_t t0_ = 0, aboveSince_ = 0; float baseline_ = 0; bool init_ = false, latched_ = false; int last_ = 0;
};

// SW-520D: the ball bounces and also trips on the robot's own bumps, so a collapse needs several
// trips inside a short window.
class TiltDetector {
 public:
  explicit TiltDetector(const RobotParams *p) : p_(p) {}
  bool update(uint32_t now, bool tripped) {
    bool edge = tripped && !prev_; prev_ = tripped;
    if (edge) {
      if (n_ == 0 || now - first_ > p_->tilt_window_ms) { first_ = now; n_ = 1; }
      else n_++;
      if (n_ >= p_->tilt_min_edges) { n_ = 0; return true; }
    }
    return false;
  }
 private:
  const RobotParams *p_; bool prev_ = false; uint8_t n_ = 0; uint32_t first_ = 0;
};

// HC-SR501: needs warm-up, and the robot's own motion changes what it sees. A rising edge while
// driving makes the Writer stop (VERIFY); the worker is only confirmed if the PIR is still active
// after the robot has settled.
class PirDetector {
 public:
  explicit PirDetector(const RobotParams *p) : p_(p) {}
  void start(uint32_t now) { t0_ = now; }
  bool warmedUp(uint32_t now) const { return !p_->warmup_enabled || now - t0_ >= p_->pir_warmup_ms; }
  // Rising edge of PIR output (candidate) - only after warm-up.
  bool candidate(uint32_t now, bool pir) {
    bool edge = pir && !prev_; prev_ = pir;
    return edge && warmedUp(now);
  }
 private:
  const RobotParams *p_; bool prev_ = false; uint32_t t0_ = 0;
};
#endif
