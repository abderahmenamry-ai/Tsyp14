// beacon_protocol.h
// Beacon message format for the Mine/Tunnel Rescue "Living Map" project
// Fits in a 16-byte payload, well within nRF24L01's 32-byte limit.

#ifndef BEACON_PROTOCOL_H
#define BEACON_PROTOCOL_H

#include <stdint.h>

// Event types the Writer robot can detect and report
enum EventType : uint8_t {
  EVENT_EXPLORED   = 0,  // routine marker, area checked, nothing found
  EVENT_GAS        = 1,  // toxic gas detected
  EVENT_COLLAPSE   = 2,  // structural collapse / instability detected
  EVENT_TRAPPED    = 3   // trapped worker detected
};

// Priority drives beacon aging: higher priority = longer TTL + faster rebroadcast
enum PriorityLevel : uint8_t {
  PRIORITY_LOW     = 0,
  PRIORITY_MEDIUM  = 1,
  PRIORITY_HIGH    = 2
};

// Starting TTL and rebroadcast interval (ms) per priority level
static const uint8_t  TTL_BY_PRIORITY[3]       = { 5, 12, 20 };
static const uint32_t REBROADCAST_MS[3]        = { 10000, 5000, 2000 };

#pragma pack(push, 1)
struct BeaconMessage {
  uint8_t  beacon_id;     // unique ID for this beacon
  uint8_t  writer_id;     // which Writer robot dropped it
  uint8_t  event_type;    // see EventType
  uint8_t  priority;      // see PriorityLevel
  int16_t  x_coord_cm;    // local X position, cm
  int16_t  y_coord_cm;    // local Y position, cm
  uint8_t  heading;       // 0-255 mapped to 0-360 degrees
  uint16_t sensor_value;  // raw sensor reading (gas ppm, vibration magnitude, etc.)
  uint32_t timestamp_ms;  // ms since mission start
  uint8_t  ttl;           // aging counter, decremented each rebroadcast
};
#pragma pack(pop)  // 16 bytes total

// Returns the default priority for a given event type.
// Adjust these mappings if your team wants different urgency rules.
inline PriorityLevel priorityForEvent(EventType type) {
  switch (type) {
    case EVENT_TRAPPED:  return PRIORITY_HIGH;
    case EVENT_COLLAPSE: return PRIORITY_HIGH;
    case EVENT_GAS:      return PRIORITY_MEDIUM;
    case EVENT_EXPLORED:
    default:             return PRIORITY_LOW;
  }
}

// Builds a beacon message. Called by the Writer robot at the moment
// an event is detected (or periodically for routine "explored" markers).
inline BeaconMessage makeBeacon(uint8_t beacon_id, uint8_t writer_id,
                                 EventType type, int16_t x_cm, int16_t y_cm,
                                 uint8_t heading, uint16_t sensor_value,
                                 uint32_t timestamp_ms) {
  BeaconMessage msg;
  msg.beacon_id    = beacon_id;
  msg.writer_id    = writer_id;
  msg.event_type   = type;
  msg.priority     = priorityForEvent(type);
  msg.x_coord_cm   = x_cm;
  msg.y_coord_cm   = y_cm;
  msg.heading      = heading;
  msg.sensor_value = sensor_value;
  msg.timestamp_ms = timestamp_ms;
  msg.ttl          = TTL_BY_PRIORITY[msg.priority];
  return msg;
}

#endif // BEACON_PROTOCOL_H
