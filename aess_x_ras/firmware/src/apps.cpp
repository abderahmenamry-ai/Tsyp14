// apps.cpp - Writer / Executor / Hub application logic (portable, no Arduino includes).
#include "apps.h"
#include <string.h>
#include <math.h>

static inline uint8_t nodeOf(Role r) { return r == ROLE_WRITER ? NODE_WRITER : (r == ROLE_EXECUTOR ? NODE_EXECUTOR : NODE_HUB); }

// =============================================================== AppBase
void AppBase::begin() {
  HelloMsg h = { (uint8_t)role_, LM_PROTO_VERSION, FW_MAJOR, FW_MINOR };
  sendHost(UP_HELLO, &h, sizeof h);
}

void AppBase::sendHost(uint8_t type, const void *payload, uint16_t len) {
  size_t n = linkEncode(type, payload, len, txbuf_);
  hal_->hostWrite(txbuf_, n);
}

void AppBase::pollHost() {
  int budget = 4096;                                           // bound the work per poll
  while (budget-- > 0 && hal_->hostAvailable() > 0) {
    int b = hal_->hostRead();
    if (b < 0) break;
    if (parser_.feed((uint8_t)b)) onHostFrame(parser_.type(), parser_.payload(), parser_.length());
  }
}

void AppBase::sendRadio(uint8_t dst, uint8_t type, const void *payload, uint8_t len, bool *acked) {
  uint8_t pkt[RF_MAX_PACKET];
  if (len > RF_MAX_PACKET - RF_HDR) len = RF_MAX_PACKET - RF_HDR;
  pkt[0] = type; pkt[1] = nodeOf(role_); pkt[2] = dst; pkt[3] = rfSeq_++;
  if (len) memcpy(pkt + RF_HDR, payload, len);
  bool ok = hal_->radioSend(dst, pkt, (uint8_t)(RF_HDR + len));
  if (acked) *acked = ok;
}

// =============================================================== RobotBase
void RobotBase::begin() {
  AppBase::begin();
  state_ = ST_WARMUP;
  bootMs_ = hal_->millis();
  lastCtrlUs_ = hal_->micros();
  lastGyroOk_ = bootMs_;
  prevEncL_ = hal_->encoderCount(LEFT);
  prevEncR_ = hal_->encoderCount(RIGHT);
  hal_->setLed(true);
}

void RobotBase::poll() {
  uint32_t now = hal_->millis();
  pollHost();
  pollLidar(now);
  controlTick(now);

  if (state_ == ST_WARMUP && lidarAlive_ && gyroCalDone_) { state_ = ST_READY; hal_->setLed(false); }
  if (runRequested_ && state_ == ST_READY) {
    mode_ = MODE_RUN; state_ = ST_RUN; runStarted_ = true; t0Run_ = now; lastCmd_ = now;
    runRequested_ = false; onEnterRun();
  }
  roleTick(now);
  sendTelemetry(now);
}

void RobotBase::onHostFrame(uint8_t type, const uint8_t *pl, uint16_t len) {
  uint32_t now = hal_->millis();
  switch (type) {
    case DN_CMD_VEL:
      if (len >= sizeof(CmdVelMsg)) {
        CmdVelMsg m; memcpy(&m, pl, sizeof m);
        cmdV_ = m.v_mm_s / 1000.0f; cmdW_ = m.w_mrad_s / 1000.0f; lastCmd_ = now;
      }
      break;
    case DN_SET_MODE:
      if (len >= 1) {
        uint8_t m = pl[0];
        if (m == MODE_RUN) {
          if (state_ == ST_ESTOP) break;                      // must be cleared with IDLE first
          runRequested_ = true;
        } else if (m == MODE_IDLE) {
          runRequested_ = false; mode_ = MODE_IDLE; cmdV_ = cmdW_ = 0; faults_ &= ~FL_ESTOP;
          if (state_ == ST_RUN || state_ == ST_VERIFY || state_ == ST_ESTOP) state_ = ST_READY;
        } else if (m == MODE_ESTOP) {
          mode_ = MODE_ESTOP; state_ = ST_ESTOP; faults_ |= FL_ESTOP; cmdV_ = cmdW_ = 0;
          drive_.hardStop(); hal_->setMotor(LEFT, 0); hal_->setMotor(RIGHT, 0);
        }
      }
      break;
    default:
      handleRoleFrame(type, pl, len);
  }
}

void RobotBase::calibrateGyro(uint32_t now) {
  if (gyroCalDone_) return;
  float g;
  if (hal_->readGyroZ(&g)) {
    gyroSum_ += g; gyroN_++;
    if (gyroN_ >= 100) { gyroBias_ = gyroSum_ / gyroN_; gyroCalDone_ = true; }
  } else if (now - bootMs_ > 1500) {                          // no IMU: continue on encoders only
    gyroCalDone_ = true; faults_ |= FL_IMU_LOST;
  }
}

void RobotBase::pollLidar(uint32_t now) {
  int budget = 1024;
  while (budget-- > 0 && hal_->lidarAvailable() > 0) {
    int b = hal_->lidarRead();
    if (b < 0) break;
    if (!ld_.feed((uint8_t)b)) continue;
    lastLidar_ = now; lidarAlive_ = true; faults_ &= ~FL_LIDAR_LOST;
    if (!asm_.add(ld_.packet())) continue;

    // a full sweep: remember it for the safety layer and ship it to the SLAM host
    memcpy(lastBins_, asm_.bins(), sizeof lastBins_);
    uint16_t m = 0xFFFF;
    int half = (int)(p_.obstacle_cone_deg * (LM_SCAN_BINS - 1) / 360.0f);
    int centre = LM_SCAN_BINS / 2;                            // bin of 0 degrees (straight ahead)
    for (int i = centre - half; i <= centre + half; i++) {
      uint16_t d = lastBins_[i];
      if (d != 0 && d < m) m = d;
    }
    frontMin_ = m;

    static ScanMsg sm;
    float dxy, dth, dts; odo_.takeDelta(&dxy, &dth, &dts);
    sm.scan_id = scanId_++;
    sm.t_ms = missionMs();
    sm.odom_dxy_mm = dxy; sm.odom_dth_deg = dth;
    sm.dt_ms = (uint16_t)(dts * 1000.0f + 0.5f);
    sm.n = LM_SCAN_BINS;
    memcpy(sm.range_mm, lastBins_, sizeof lastBins_);
    sendHost(UP_SCAN, &sm, sizeof sm);
  }
  if (lidarAlive_ && now - lastLidar_ > p_.lidar_timeout_ms) faults_ |= FL_LIDAR_LOST;
}

void RobotBase::controlTick(uint32_t now) {
  uint32_t us = hal_->micros();
  uint32_t period = 1000000u / p_.control_hz;
  uint32_t dtUs = us - lastCtrlUs_;
  if (dtUs < period) return;
  float dt = dtUs * 1e-6f; lastCtrlUs_ = us;

  // ---- sensing
  int32_t el = hal_->encoderCount(LEFT), er = hal_->encoderCount(RIGHT);
  int32_t dl = el - prevEncL_, dr = er - prevEncR_; prevEncL_ = el; prevEncR_ = er;
  float g = 0; bool gyroOk = hal_->readGyroZ(&g);
  if (state_ == ST_WARMUP) calibrateGyro(now);
  if (gyroOk) { lastGyroOk_ = now; faults_ &= ~FL_IMU_LOST; }
  else if (gyroCalDone_ && now - lastGyroOk_ > 300) faults_ |= FL_IMU_LOST;
  odo_.update(dl, dr, gyroOk && gyroCalDone_, g - gyroBias_, dt);

  // ---- safety layer: the host proposes (v, w); the firmware decides what is allowed
  float v = cmdV_, w = cmdW_;
  bool allowed = (mode_ == MODE_RUN) && (state_ == ST_RUN) && !roleHoldsDrive();
  if (!allowed) { v = w = 0; }
  if (allowed) {
    if (now - lastCmd_ > p_.cmd_timeout_ms) { faults_ |= FL_CMD_TIMEOUT; v = w = 0; } else faults_ &= ~FL_CMD_TIMEOUT;
    if (faults_ & FL_LIDAR_LOST) v = w = 0;
    if (v > 0 && frontMin_ < p_.obstacle_stop_mm) { v = 0; faults_ |= FL_OBSTACLE_STOP; } else faults_ &= ~FL_OBSTACLE_STOP;
  }

  // ---- actuation
  float dutyL, dutyR;
  drive_.setCommand(v, w);
  drive_.update(odo_.vLeft(), odo_.vRight(), dt, &dutyL, &dutyR);
  if (state_ == ST_ESTOP) { dutyL = dutyR = 0; }
  hal_->setMotor(LEFT, dutyL); hal_->setMotor(RIGHT, dutyR);
}

void RobotBase::sendTelemetry(uint32_t now) {
  if (now - lastTele_ < 200) return;
  lastTele_ = now;
  TelemetryMsg t; memset(&t, 0, sizeof t);
  t.t_ms = missionMs(); t.state = state_; t.mode = mode_; t.faults = faults_;
  t.vl_mm_s = (int16_t)(odo_.vLeft() * 1000.0f); t.vr_mm_s = (int16_t)(odo_.vRight() * 1000.0f);
  t.battery_mv = hal_->batteryMv(); t.front_min_mm = frontMin_;
  roleTelemetry(&t);
  sendHost(UP_TELEMETRY, &t, sizeof t);
}

// =============================================================== WriterApp
void WriterApp::begin() {
  RobotBase::begin();
  uint32_t now = hal_->millis();
  gas_.start(now); pir_.start(now);
}

void WriterApp::onEnterRun() {}

bool WriterApp::handleRoleFrame(uint8_t type, const uint8_t *pl, uint16_t len) {
  if (type == DN_BEACON_TX && len >= sizeof(BeaconMessage)) {
    BeaconMessage b; memcpy(&b, pl, sizeof b);
    sender_.add(b, missionMs());
    return true;
  }
  return false;
}

void WriterApp::emitEvent(uint8_t type, uint16_t value, uint32_t now) {
  if (lastEvent_[type] != 0 && now - lastEvent_[type] < p_.event_cooldown_ms) return;
  lastEvent_[type] = now ? now : 1; evCount_[type]++;
  EventMsg e = { type, value, missionMs() };
  sendHost(UP_EVENT, &e, sizeof e);
}

void WriterApp::roleTick(uint32_t now) {
  // gas keeps learning its baseline from boot; the other detectors only act while running
  if (gas_.update(now, hal_->readGasRaw()) && state_ == ST_RUN) emitEvent(EVENT_GAS, (uint16_t)gas_.lastRaw(), now);

  if (state_ == ST_RUN || state_ == ST_VERIFY) {
    if (tilt_.update(now, hal_->readTilt())) emitEvent(EVENT_COLLAPSE, p_.tilt_min_edges, now);
    bool pir = hal_->readPir();
    if (state_ == ST_RUN && pir_.candidate(now, pir)) {          // possible worker: stop and look again
      state_ = ST_VERIFY; verifying_ = true; verifyStart_ = now; verifyConfirmed_ = false;
    } else if (state_ == ST_VERIFY) {
      uint32_t el = now - verifyStart_;
      if (el >= p_.pir_settle_ms && pir) verifyConfirmed_ = true;
      if (verifyConfirmed_ || el >= p_.pir_verify_ms) {
        if (verifyConfirmed_) emitEvent(EVENT_TRAPPED, 1, now); else pirFalse_++;
        verifying_ = false; state_ = ST_RUN; lastCmd_ = now;      // give the host a fresh command window
      }
    }
  }
  pollRadio(now);
}

void WriterApp::pollRadio(uint32_t now) {
  uint8_t buf[RF_MAX_PACKET]; int n;
  int guard = 8;
  while (guard-- > 0 && (n = hal_->radioRead(buf)) >= RF_HDR) {
    if (buf[2] != NODE_WRITER) continue;
    if (buf[0] == RF_ACK && n >= RF_HDR + (int)sizeof(RfAck)) {
      RfAck a; memcpy(&a, buf + RF_HDR, sizeof a);
      if (a.acked_type == RF_BEACON) {
        sender_.onAck(a.acked_id);
        BeaconStatusMsg s = { a.acked_id, 1 };
        sendHost(UP_BEACON_STATUS, &s, sizeof s);
      }
    }
  }
  bool ok;
  uint32_t mm = missionMs();
  if (runStarted_) {
    const BeaconMessage *b = sender_.due(mm);
    if (b) {
      sendRadio(NODE_HUB, RF_BEACON, b, sizeof(BeaconMessage), &ok);
      sender_.sent(b, mm);
      radioFails_ = ok ? 0 : (uint8_t)(radioFails_ < 255 ? radioFails_ + 1 : 255);
    }
  }
  if (now - lastHeartbeat_ >= p_.heartbeat_ms) {
    lastHeartbeat_ = now;
    RfHeartbeat h = { state_, (int16_t)(odo_.x() * 100.0f), (int16_t)(odo_.y() * 100.0f), hal_->batteryMv(), (uint8_t)(faults_ & 0xFF) };
    sendRadio(NODE_HUB, RF_HEARTBEAT, &h, sizeof h, &ok);
    radioFails_ = ok ? 0 : (uint8_t)(radioFails_ < 255 ? radioFails_ + 1 : 255);
  }
  if (radioFails_ >= 5) faults_ |= FL_RADIO_LOST; else faults_ &= ~FL_RADIO_LOST;
}

void WriterApp::roleTelemetry(TelemetryMsg *t) {
  t->gas_raw = (uint16_t)gas_.lastRaw();
  t->beacons_pending = (uint8_t)sender_.pending();
  t->radio_ok = (faults_ & FL_RADIO_LOST) ? 0 : 1;
}

// =============================================================== ExecutorApp
bool ExecutorApp::handleRoleFrame(uint8_t type, const uint8_t *pl, uint16_t len) {
  if (type == DN_MISSION_REQ) { wantMission_ = true; lastReq_ = 0; return true; }
  if (type == DN_TARGET_REACHED && len >= 1) {
    uint8_t id = pl[0];
    bool queued = false;
    for (uint8_t i = 0; i < nReach_; i++) if (reachQ_[i] == id) queued = true;
    if (!queued) {
      if (nReach_ == 8) { for (uint8_t j = 0; j + 1 < 8; j++) reachQ_[j] = reachQ_[j + 1]; nReach_ = 7; }
      reachQ_[nReach_++] = id; lastReach_ = 0;                                  // several can be pending at once
    }
    for (uint8_t i = 0; i < nIds_; i++) if (ids_[i] == id) {                    // forget it locally
      for (uint8_t j = i; j + 1 < nIds_; j++) ids_[j] = ids_[j + 1];
      nIds_--; if (expected_ > 0) expected_--; break;
    }
    return true;
  }
  return false;
}

void ExecutorApp::roleTick(uint32_t now) { pollRadio(now); }

void ExecutorApp::pollRadio(uint32_t now) {
  uint8_t buf[RF_MAX_PACKET]; int n; int guard = 8;
  while (guard-- > 0 && (n = hal_->radioRead(buf)) >= RF_HDR) {
    if (buf[2] != NODE_EXECUTOR) continue;
    if (buf[0] == RF_TARGET && n >= RF_HDR + (int)sizeof(RfTarget)) {
      RfTarget t; memcpy(&t, buf + RF_HDR, sizeof t);
      if (t.count == 0) {                                          // hub says: nothing to do
        MissionTargetMsg m = { 0, 0, t.beacon };
        sendHost(UP_MISSION_TARGET, &m, sizeof m);
        nIds_ = 0; expected_ = 0; wantMission_ = false;
      } else {
        bool known = false;
        for (uint8_t i = 0; i < nIds_; i++) if (ids_[i] == t.beacon.beacon_id) known = true;
        if (!known && nIds_ < 8) {                                 // the mission list arrives packet by packet over a lossy link
          ids_[nIds_++] = t.beacon.beacon_id;
          MissionTargetMsg m = { t.index, t.count, t.beacon };
          sendHost(UP_MISSION_TARGET, &m, sizeof m);
          targetsRx_++;
        }
        expected_ = t.count;
        wantMission_ = (nIds_ < expected_);                       // keep asking until the whole list is in
      }
    } else if (buf[0] == RF_ACK && n >= RF_HDR + (int)sizeof(RfAck)) {
      RfAck a; memcpy(&a, buf + RF_HDR, sizeof a);
      if (a.acked_type == RF_REACHED)
        for (uint8_t i = 0; i < nReach_; i++) if (reachQ_[i] == a.acked_id) {
          for (uint8_t j = i; j + 1 < nReach_; j++) reachQ_[j] = reachQ_[j + 1];
          nReach_--; break;
        }
    }
  }
  bool ok;
  if (wantMission_ && (lastReq_ == 0 || now - lastReq_ >= p_.mission_req_ms)) {
    lastReq_ = now ? now : 1;
    sendRadio(NODE_HUB, RF_MISSION_REQ, nullptr, 0, &ok);
    radioFails_ = ok ? 0 : (uint8_t)(radioFails_ < 255 ? radioFails_ + 1 : 255);
  }
  if (nReach_ > 0 && (lastReach_ == 0 || now - lastReach_ >= 1000)) {
    lastReach_ = now ? now : 1;
    for (uint8_t i = 0; i < nReach_; i++) {                                    // retry every unacknowledged report
      sendRadio(NODE_HUB, RF_REACHED, &reachQ_[i], 1, &ok);
      radioFails_ = ok ? 0 : (uint8_t)(radioFails_ < 255 ? radioFails_ + 1 : 255);
    }
  }
  if (now - lastHeartbeat_ >= p_.heartbeat_ms) {
    lastHeartbeat_ = now;
    RfHeartbeat h = { state_, (int16_t)(odo_.x() * 100.0f), (int16_t)(odo_.y() * 100.0f), hal_->batteryMv(), (uint8_t)(faults_ & 0xFF) };
    sendRadio(NODE_HUB, RF_HEARTBEAT, &h, sizeof h, &ok);
    radioFails_ = ok ? 0 : (uint8_t)(radioFails_ < 255 ? radioFails_ + 1 : 255);
  }
  if (radioFails_ >= 5) faults_ |= FL_RADIO_LOST; else faults_ &= ~FL_RADIO_LOST;
}

void ExecutorApp::roleTelemetry(TelemetryMsg *t) {
  t->radio_ok = (faults_ & FL_RADIO_LOST) ? 0 : 1;
  t->beacons_pending = nReach_;
}

// =============================================================== HubApp
void HubApp::begin() { AppBase::begin(); state_ = ST_READY; }

void HubApp::poll() {
  uint32_t now = hal_->millis();
  pollHost();
  pollRadio(now);
}

void HubApp::onHostFrame(uint8_t type, const uint8_t *pl, uint16_t len) {
  uint32_t now = hal_->millis();
  if (type == DN_SET_CALIB && len >= sizeof(SetCalibMsg)) {
    SetCalibMsg c; memcpy(&c, pl, sizeof c);
    cal_.entryLat = c.lat_e7 / 1e7; cal_.entryLon = c.lon_e7 / 1e7; cal_.headingOffsetDeg = c.heading_deg;
    calibrated_ = true;
  } else if (type == DN_MISSION_PUSH && len >= 1) {
    uint8_t n = pl[0]; if (n > 8) n = 8;
    if (len >= 1 + n * sizeof(BeaconMessage)) {
      nTargets_ = n;
      for (uint8_t i = 0; i < n; i++) memcpy(&targets_[i], pl + 1 + i * sizeof(BeaconMessage), sizeof(BeaconMessage));
      if (nodes_[NODE_EXECUTOR].seen) sendTargets(now);        // push straight away if the Executor is listening
    }
  }
}

void HubApp::sendTargets(uint32_t now) {
  (void)now;
  bool ok;
  if (nTargets_ == 0) {
    RfTarget t; memset(&t, 0, sizeof t);
    sendRadio(NODE_EXECUTOR, RF_TARGET, &t, sizeof t, &ok);
    return;
  }
  for (uint8_t i = 0; i < nTargets_; i++) {
    RfTarget t = { i, nTargets_, targets_[i] };
    sendRadio(NODE_EXECUTOR, RF_TARGET, &t, sizeof t, &ok);
  }
}

void HubApp::pollRadio(uint32_t now) {
  uint8_t buf[RF_MAX_PACKET]; int n; int guard = 8;
  while (guard-- > 0 && (n = hal_->radioRead(buf)) >= RF_HDR) {
    if (buf[2] != NODE_HUB) continue;
    uint8_t type = buf[0], src = buf[1];
    if (src > NODE_EXECUTOR) continue;
    nodes_[src].lastSeen = now; nodes_[src].seen = true;
    uint8_t linkq = (uint8_t)(255 - 16 * (hal_->radioLastRetries() > 15 ? 15 : hal_->radioLastRetries()));
    bool ok;
    switch (type) {
      case RF_BEACON:
        if (n >= RF_HDR + (int)sizeof(BeaconMessage)) {
          BeaconMessage b; memcpy(&b, buf + RF_HDR, sizeof b);
          RfAck a = { RF_BEACON, b.beacon_id };
          sendRadio(src, RF_ACK, &a, sizeof a, &ok);                 // always (re-)acknowledge
          bool dup = false;
          for (auto &s : seen_) if (s.used && s.writer == b.writer_id && s.id == b.beacon_id && s.ts == b.timestamp_ms) dup = true;
          if (dup) { dup_++; break; }
          seen_[seenPos_ % 32] = { b.writer_id, b.beacon_id, b.timestamp_ms, true }; seenPos_++;
          BeaconRxMsg r; memset(&r, 0, sizeof r);
          r.src = src; r.link_q = linkq; r.beacon = b;
          if (calibrated_) {
            GPSCoord g = translateToGPS(b, cal_);
            r.lat_e7 = (int32_t)llround(g.lat * 1e7); r.lon_e7 = (int32_t)llround(g.lon * 1e7);
          }
          sendHost(UP_BEACON_RX, &r, sizeof r); fwd_++;
        }
        break;
      case RF_HEARTBEAT:
        if (n >= RF_HDR + (int)sizeof(RfHeartbeat)) {
          RfHeartbeat h; memcpy(&h, buf + RF_HDR, sizeof h);
          NodeStatusMsg s = { src, h.state, h.x_cm, h.y_cm, h.battery_mv, 0, linkq };
          sendHost(UP_NODE_STATUS, &s, sizeof s);
        }
        break;
      case RF_MISSION_REQ: {
        uint8_t who = src; sendHost(UP_MISSION_REQ, &who, 1);
        sendTargets(now);
        break;
      }
      case RF_REACHED:
        if (n >= RF_HDR + 1) {
          uint8_t id = buf[RF_HDR];
          RfAck a = { RF_REACHED, id };
          sendRadio(src, RF_ACK, &a, sizeof a, &ok);
          bool dup = false;
          for (uint8_t v : reachedSeen_) if (v == id) dup = true;
          if (dup) { dup_++; break; }
          reachedSeen_[reachedPos_++ % 16] = id;
          TargetReachedMsg m = { id };
          sendHost(UP_TARGET_REACHED, &m, sizeof m);
          for (uint8_t i = 0; i < nTargets_; i++) if (targets_[i].beacon_id == id) {   // drop it from the stored mission
            for (uint8_t j = i; j + 1 < nTargets_; j++) targets_[j] = targets_[j + 1];
            nTargets_--; break;
          }
        }
        break;
      default: break;
    }
  }
}
