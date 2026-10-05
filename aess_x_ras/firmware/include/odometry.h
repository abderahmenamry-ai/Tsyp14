// odometry.h - differential-drive dead reckoning from wheel encoders, heading fused with the gyro.
// Produces (a) a running pose for telemetry and (b) the per-scan delta in BreezySLAM's
// pose_change format (distance in mm, heading change in degrees, dt) that the SLAM host consumes.
#ifndef LM_ODOMETRY_H
#define LM_ODOMETRY_H
#include <stdint.h>
#include <math.h>
#include "params.h"

class Odometry {
 public:
  explicit Odometry(const RobotParams *p) : p_(p) {}

  // dl, dr: encoder ticks since the last call (signed); gyro_dps valid if gyro_ok; dt in seconds.
  void update(int32_t dl, int32_t dr, bool gyro_ok, float gyro_dps, float dt) {
    const float m_per_tick = (float)M_PI * p_->wheel_diameter_m / p_->encoder_ticks_rev;
    float sl = dl * m_per_tick, sr = dr * m_per_tick;
    float ds = 0.5f * (sl + sr);
    float dth_enc = (sr - sl) / p_->wheel_base_m;                      // rad, CCW positive
    float dth = dth_enc;
    if (gyro_ok) {
      float dth_gyro = gyro_dps * p_->gyro_scale * (float)M_PI / 180.0f * dt;
      dth = p_->gyro_weight * dth_gyro + (1.0f - p_->gyro_weight) * dth_enc;
    }
    yaw_ += 0.5f * dth;                                                // mid-point integration
    x_ += ds * cosf(yaw_); y_ += ds * sinf(yaw_);
    yaw_ += 0.5f * dth;
    accDist_ += ds; accTh_ += dth; accT_ += dt;
    if (dt > 1e-6f) { vl_ = sl / dt; vr_ = sr / dt; }
    dist_ += fabsf(ds);
  }

  // Take (and reset) the motion accumulated since the previous call: BreezySLAM pose_change.
  void takeDelta(float *dxy_mm, float *dth_deg, float *dt_s) {
    *dxy_mm = accDist_ * 1000.0f; *dth_deg = accTh_ * 180.0f / (float)M_PI; *dt_s = accT_;
    accDist_ = accTh_ = accT_ = 0;
  }
  float x() const { return x_; } float y() const { return y_; } float yaw() const { return yaw_; }
  float vLeft() const { return vl_; } float vRight() const { return vr_; }
  float distance() const { return dist_; }
 private:
  const RobotParams *p_;
  float x_ = 0, y_ = 0, yaw_ = 0, vl_ = 0, vr_ = 0, dist_ = 0;
  float accDist_ = 0, accTh_ = 0, accT_ = 0;
};
#endif
