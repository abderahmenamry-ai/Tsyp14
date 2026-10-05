// motor_control.h - (v, w) command -> per-wheel PID speed control -> PWM duty.
#ifndef LM_MOTOR_CONTROL_H
#define LM_MOTOR_CONTROL_H
#include <math.h>
#include "params.h"

class WheelPid {
 public:
  void configure(const RobotParams *p) { p_ = p; }
  void reset() { i_ = 0; prevErr_ = 0; }
  // target and measured in m/s. Returns duty in [-1, 1].
  float step(float target, float measured, float dt) {
    float ff = target / p_->max_wheel_speed_ms;                       // feed-forward
    float err = target - measured;
    i_ += err * dt;
    float iLim = 0.4f / (p_->ki > 1e-6f ? p_->ki : 1.0f);
    if (i_ > iLim) i_ = iLim;
    if (i_ < -iLim) i_ = -iLim;             // anti-windup
    float d = (dt > 1e-6f) ? (err - prevErr_) / dt : 0; prevErr_ = err;
    float u = ff + p_->kp * err + p_->ki * i_ + p_->kd * d;
    if (fabsf(target) < 0.02f) { u = 0; i_ = 0; }                     // standing still: no creep
    else if (fabsf(u) < p_->min_duty) u = (u >= 0 ? 1 : -1) * p_->min_duty;   // overcome static friction
    if (u > 1) u = 1;
    if (u < -1) u = -1;
    return u;
  }
 private:
  const RobotParams *p_ = nullptr;
  float i_ = 0, prevErr_ = 0;
};

class DriveController {
 public:
  explicit DriveController(const RobotParams *p) : p_(p) { l_.configure(p); r_.configure(p); }
  void setCommand(float v, float w) {
    if (v > p_->max_v_ms) v = p_->max_v_ms;
    if (v < -p_->max_v_ms) v = -p_->max_v_ms;
    if (w > p_->max_w_rads) w = p_->max_w_rads;
    if (w < -p_->max_w_rads) w = -p_->max_w_rads;
    v_ = v; w_ = w;
  }
  void stop() { v_ = w_ = 0; }
  void hardStop() { v_ = w_ = sl_ = sr_ = 0; l_.reset(); r_.reset(); }
  // Call at control_hz with measured wheel speeds. Outputs duty for each side.
  void update(float vl_meas, float vr_meas, float dt, float *duty_l, float *duty_r) {
    float tl = v_ - w_ * p_->wheel_base_m * 0.5f;
    float tr = v_ + w_ * p_->wheel_base_m * 0.5f;
    // slew-limit the set-points so the PID is never asked for a step
    float maxStep = p_->max_accel_ms2 * dt;
    sl_ += clampf(tl - sl_, -maxStep, maxStep);
    sr_ += clampf(tr - sr_, -maxStep, maxStep);
    *duty_l = l_.step(sl_, vl_meas, dt);
    *duty_r = r_.step(sr_, vr_meas, dt);
  }
  float v() const { return v_; } float w() const { return w_; }
  bool commandedStop() const { return fabsf(v_) < 1e-3f && fabsf(w_) < 1e-3f; }
 private:
  static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
  const RobotParams *p_;
  WheelPid l_, r_;
  float v_ = 0, w_ = 0, sl_ = 0, sr_ = 0;
};
#endif
