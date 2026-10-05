// beacon_sender.h - reliable beacon delivery with message aging.
// A beacon is re-sent every REBROADCAST_MS[priority] until the hub acknowledges it (RF_ACK) or its
// TTL runs out (beacon_protocol.h: ttlRemaining). High priority = faster retries and longer life.
#ifndef LM_BEACON_SENDER_H
#define LM_BEACON_SENDER_H
#include <stdint.h>
#include "protocol.h"

class BeaconSender {
 public:
  static const int CAP = 16;
  bool add(const BeaconMessage &b, uint32_t now) {
    for (int i = 0; i < CAP; i++) if (!used_[i]) {
      used_[i] = true; msg_[i] = b; acked_[i] = false; next_[i] = now; tries_[i] = 0; return true;
    }
    return false;
  }
  // Returns a beacon that is due for (re)transmission, or nullptr. Drops expired/acked entries.
  const BeaconMessage *due(uint32_t now) {
    for (int i = 0; i < CAP; i++) {
      if (!used_[i]) continue;
      if (acked_[i] || isExpired(msg_[i], now)) { used_[i] = false; expired_ += !acked_[i]; continue; }
      if ((int32_t)(now - next_[i]) >= 0) return &msg_[i];
    }
    return nullptr;
  }
  // Call after a transmission attempt of `b`. `radio_ok` = nRF24 hardware auto-ACK; the application
  // level RF_ACK from the hub (onAck) is what actually ends the retries.
  void sent(const BeaconMessage *b, uint32_t now) {
    int i = (int)(b - msg_);
    tries_[i]++;
    next_[i] = now + REBROADCAST_MS[b->priority];
  }
  void onAck(uint8_t beacon_id) {
    for (int i = 0; i < CAP; i++) if (used_[i] && msg_[i].beacon_id == beacon_id) { acked_[i] = true; ackedCount_++; }
  }
  int pending() const { int n = 0; for (int i = 0; i < CAP; i++) n += (used_[i] && !acked_[i]); return n; }
  uint32_t expiredCount() const { return expired_; }
  uint32_t ackedCount() const { return ackedCount_; }
  uint16_t tries(uint8_t beacon_id) const { for (int i = 0; i < CAP; i++) if (used_[i] && msg_[i].beacon_id == beacon_id) return tries_[i]; return 0; }
 private:
  // mission-clock expiry (mirrors comms/beacon.py::is_expired)
  static bool isExpired(const BeaconMessage &b, uint32_t now) { return ttlRemaining(b, now) == 0; }
  bool used_[CAP] = {}, acked_[CAP] = {};
  BeaconMessage msg_[CAP];
  uint32_t next_[CAP] = {};
  uint16_t tries_[CAP] = {};
  uint32_t expired_ = 0, ackedCount_ = 0;
};
#endif
