// params.h - robot parameters. Defaults describe the reference chassis; the software-in-the-loop
// simulation overrides them at runtime (the simulated robot is bigger than a hobby chassis).
#ifndef LM_PARAMS_H
#define LM_PARAMS_H
#include <stdint.h>

struct RobotParams {
  // --- geometry / drivetrain
  float wheel_diameter_m   = 0.065f;
  float wheel_base_m       = 0.150f;      // distance between wheel contact patches
  float encoder_ticks_rev  = 374.0f;      // ticks per WHEEL revolution (quadrature x4, after gearbox)
  float max_wheel_speed_ms = 0.60f;       // full-duty wheel speed, used for feed-forward
  float max_v_ms           = 0.60f;       // command limits
  float max_w_rads         = 1.60f;

  // --- wheel speed PID (runs at control_hz)
  float kp = 0.8f, ki = 2.5f, kd = 0.0f;
  float max_accel_ms2      = 1.5f;        // slew limit on the wheel speed set-point
  float min_duty           = 0.12f;       // static-friction dead-band compensation
  uint16_t control_hz      = 100;

  // --- heading estimate: complementary filter between encoder yaw-rate and gyro
  float gyro_weight        = 0.98f;       // 1.0 = gyro only
  float gyro_scale         = 1.0f;

  // --- LiDAR (LD06) mounting: LD06 angles increase CLOCKWISE; the SLAM host wants CCW from +x
  float lidar_sign         = -1.0f;
  float lidar_offset_deg   = 0.0f;        // rotate if the LD06 zero mark is not pointing forward
  uint16_t lidar_min_mm    = 50;
  uint16_t lidar_max_mm    = 8000;        // beyond this = no return (matches the SLAM config)
  uint8_t  lidar_min_intensity = 8;

  // --- safety layer (runs in firmware, independent of the host)
  uint32_t cmd_timeout_ms  = 500;         // no CMD_VEL for this long -> stop
  uint32_t lidar_timeout_ms= 1000;
  uint16_t obstacle_stop_mm= 220;         // forward motion blocked if anything this close ahead
  float obstacle_cone_deg  = 30.0f;       // half-angle of the forward cone

  // --- event sensors (see architecture/sensor_research.pdf)
  uint32_t mq2_warmup_ms   = 60000;       // demo preheat; real calibration needs a long first-use preheat
  uint32_t pir_warmup_ms   = 60000;       // HC-SR501 needs 30-60 s after power-on
  float    gas_rise_counts = 350.0f;      // ADC counts above the learned baseline
  uint32_t gas_hold_ms     = 1500;        // must stay above the threshold this long
  uint8_t  tilt_min_edges  = 3;           // >= 3 trips ...
  uint32_t tilt_window_ms  = 1000;        // ... inside this window
  uint32_t pir_verify_ms   = 3000;        // stop and re-check before declaring a trapped worker
  uint32_t pir_settle_ms   = 800;         // ignore PIR right after stopping (own motion)
  uint32_t event_cooldown_ms = 8000;      // minimum gap between two events of the same type
  bool     warmup_enabled  = true;        // false only for tests/simulation

  // --- radio
  uint32_t heartbeat_ms    = 2000;
  uint32_t node_lost_ms    = 10000;
  uint32_t mission_req_ms  = 3000;
};

#endif
