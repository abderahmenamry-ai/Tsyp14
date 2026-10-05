// apps.h - the three firmware roles. All three run on an ESP32 and share the AppBase plumbing:
//   WriterApp   : drives, scans, detects events, radios beacons          (robot #1)
//   ExecutorApp : drives, scans, fetches the mission over the radio      (robot #2)
//   HubApp      : outside-network node: radio <-> command post, GPS translation
//
// Division of labour (see architecture/robot_architecture.md):
//   ESP32 (real time, 100 Hz) : motor PID, encoder + gyro odometry, LD06 parsing, sensor logic,
//                               safety layer, nRF24 reliability, host framing
//   SLAM host (laptop / Pi)   : BreezySLAM, wall-following / goal-seeking controllers, beacon
//                               composition (needs the SLAM pose), command post
#ifndef LM_APPS_H
#define LM_APPS_H

#include "hal.h"
#include "params.h"
#include "protocol.h"
#include "link.h"
#include "ld06.h"
#include "odometry.h"
#include "motor_control.h"
#include "detectors.h"
#include "beacon_sender.h"
#include "frame_translation.h"

#define FW_MAJOR 1
#define FW_MINOR 0

class AppBase {
 public:
  AppBase(Hal *hal, const RobotParams &p, Role role) : hal_(hal), p_(p), role_(role) {}
  virtual ~AppBase() {}
  virtual void begin();
  virtual void poll() = 0;
  uint8_t state() const { return state_; }
  uint32_t bad_frames() const { return parser_.badFrames(); }
 protected:
  void sendHost(uint8_t type, const void *payload, uint16_t len);
  void pollHost();                                        // parse incoming bytes -> onHostFrame
  virtual void onHostFrame(uint8_t type, const uint8_t *pl, uint16_t len) = 0;
  void sendRadio(uint8_t dst, uint8_t type, const void *payload, uint8_t len, bool *acked = nullptr);

  Hal *hal_;
  RobotParams p_;
  Role role_;
  uint8_t state_ = ST_BOOT;
  FrameParser parser_;
  uint8_t txbuf_[LM_MAX_PAYLOAD + 8];
  uint8_t rfSeq_ = 0;
};

// ---- common robot base: drive + LiDAR + odometry + safety ----------------------------------
class RobotBase : public AppBase {
 public:
  RobotBase(Hal *hal, const RobotParams &p, Role role)
      : AppBase(hal, p, role), odo_(&p_), drive_(&p_), asm_(&p_) {}
  void begin() override;
  void poll() override;
  // introspection for tests / telemetry
  uint16_t faults() const { return faults_; }
  uint8_t mode() const { return mode_; }
  uint32_t missionMs() const { return runStarted_ ? hal_->millis() - t0Run_ : 0; }
  const Odometry &odometry() const { return odo_; }
  uint16_t frontMinMm() const { return frontMin_; }
  uint16_t scansSent() const { return scanId_; }
 protected:
  void onHostFrame(uint8_t type, const uint8_t *pl, uint16_t len) override;
  virtual bool handleRoleFrame(uint8_t type, const uint8_t *pl, uint16_t len) { (void)type; (void)pl; (void)len; return false; }
  virtual void roleTick(uint32_t now) = 0;                // events / radio, once per poll
  virtual void roleTelemetry(TelemetryMsg *t) { (void)t; }
  virtual bool roleHoldsDrive() { return false; }         // Writer VERIFY state overrides cmd_vel
  virtual void onEnterRun() {}

  void pollLidar(uint32_t now);
  void controlTick(uint32_t now);
  void sendTelemetry(uint32_t now);
  void setState(uint8_t s) { state_ = s; }
  void calibrateGyro(uint32_t now);

  Odometry odo_;
  DriveController drive_;
  Ld06Parser ld_;
  ScanAssembler asm_;
  uint16_t lastBins_[LM_SCAN_BINS] = {};
  uint16_t frontMin_ = 0xFFFF;

  uint8_t mode_ = MODE_IDLE;
  bool runRequested_ = false, runStarted_ = false;
  uint32_t t0Run_ = 0;
  float cmdV_ = 0, cmdW_ = 0;
  uint32_t lastCmd_ = 0, lastLidar_ = 0, lastCtrlUs_ = 0, lastTele_ = 0, lastGyroOk_ = 0, bootMs_ = 0;
  int32_t prevEncL_ = 0, prevEncR_ = 0;
  uint16_t faults_ = 0, scanId_ = 0, lastScanMs_ = 0;
  uint32_t lastScanT_ = 0;
  // gyro bias calibration (robot stationary during WARMUP)
  float gyroBias_ = 0, gyroSum_ = 0; int gyroN_ = 0; bool gyroCalDone_ = false;
  bool lidarAlive_ = false;
};

class WriterApp : public RobotBase {
 public:
  WriterApp(Hal *hal, const RobotParams &p) : RobotBase(hal, p, ROLE_WRITER), gas_(&p_), tilt_(&p_), pir_(&p_) {}
  void begin() override;
  uint32_t eventsFired(uint8_t type) const { return type < 4 ? evCount_[type] : 0; }
  uint32_t beaconsAcked() const { return sender_.ackedCount(); }
  int beaconsPending() const { return sender_.pending(); }
  uint32_t pirFalseAlarms() const { return pirFalse_; }
  const BeaconSender &sender() const { return sender_; }
 protected:
  bool handleRoleFrame(uint8_t type, const uint8_t *pl, uint16_t len) override;
  void roleTick(uint32_t now) override;
  void roleTelemetry(TelemetryMsg *t) override;
  bool roleHoldsDrive() override { return verifying_; }
  void onEnterRun() override;
 private:
  void emitEvent(uint8_t type, uint16_t value, uint32_t now);
  void pollRadio(uint32_t now);
  GasDetector gas_; TiltDetector tilt_; PirDetector pir_;
  BeaconSender sender_;
  bool verifying_ = false; uint32_t verifyStart_ = 0; bool verifyConfirmed_ = false;
  uint32_t lastEvent_[4] = {}; uint32_t evCount_[4] = {}; uint32_t pirFalse_ = 0;
  uint32_t lastHeartbeat_ = 0; uint8_t radioFails_ = 0; bool radioOk_ = true;
};

class ExecutorApp : public RobotBase {
 public:
  ExecutorApp(Hal *hal, const RobotParams &p) : RobotBase(hal, p, ROLE_EXECUTOR) {}
  uint32_t targetsReceived() const { return targetsRx_; }
 protected:
  bool handleRoleFrame(uint8_t type, const uint8_t *pl, uint16_t len) override;
  void roleTick(uint32_t now) override;
  void roleTelemetry(TelemetryMsg *t) override;
 private:
  void pollRadio(uint32_t now);
  bool wantMission_ = true; uint32_t lastReq_ = 0, lastHeartbeat_ = 0, lastReach_ = 0;
  uint8_t reachQ_[8] = {}; uint8_t nReach_ = 0;               // REACHED reports not yet acknowledged by the hub
  uint32_t targetsRx_ = 0; bool radioOk_ = true; uint8_t radioFails_ = 0;
  uint8_t ids_[8] = {}; uint8_t nIds_ = 0, expected_ = 0;      // targets already forwarded to the host
};

class HubApp : public AppBase {
 public:
  HubApp(Hal *hal, const RobotParams &p) : AppBase(hal, p, ROLE_HUB) {}
  void begin() override;
  void poll() override;
  uint32_t beaconsForwarded() const { return fwd_; }
  uint32_t duplicates() const { return dup_; }
 protected:
  void onHostFrame(uint8_t type, const uint8_t *pl, uint16_t len) override;
 private:
  void pollRadio(uint32_t now);
  void sendTargets(uint32_t now);
  struct Node { uint32_t lastSeen = 0; bool seen = false; };
  Node nodes_[3];
  CalibrationData cal_ = {0.0, 0.0, 0.0};
  bool calibrated_ = false;
  BeaconMessage targets_[8]; uint8_t nTargets_ = 0;
  struct Seen { uint8_t writer; uint8_t id; uint32_t ts; bool used; };
  Seen seen_[32] = {}; uint8_t seenPos_ = 0;
  uint8_t reachedSeen_[16] = {}; uint8_t reachedPos_ = 0;
  uint32_t fwd_ = 0, dup_ = 0;
};

#endif
