// protocol.h - robot <-> SLAM-host link ("host link") and nRF24 radio packets.
// Portable C++11 (no Arduino includes) so the same file is used by the ESP32
// build, the host unit tests and the software-in-the-loop simulation.
//
// ---------------------------------------------------------------- host link
// Transport: USB serial 921600 8N1 (Phase 2) or a TCP socket (simulation).
// Frame:  0xAA 0x55 | type u8 | len u16 LE | payload[len] | crc16 u16 LE
// crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len, payload.
// All multi-byte fields are little-endian (ESP32 and x86/ARM hosts are LE).
#ifndef LM_PROTOCOL_H
#define LM_PROTOCOL_H

#include <stdint.h>
#include "beacon_protocol.h"   // BeaconMessage (comms/firmware), 16 bytes

#define LM_PROTO_VERSION 1
#define LM_FRAME_SOF0 0xAA
#define LM_FRAME_SOF1 0x55
#define LM_MAX_PAYLOAD 1024
#define LM_SCAN_BINS 360       // 1 bin per degree, same spacing as the simulated LiDAR

enum NodeId : uint8_t { NODE_HUB = 0, NODE_WRITER = 1, NODE_EXECUTOR = 2 };
enum Role   : uint8_t { ROLE_WRITER = 1, ROLE_EXECUTOR = 2, ROLE_HUB = 3 };

// ---- device -> host
enum HostMsgUp : uint8_t {
  UP_HELLO          = 0x01,
  UP_SCAN           = 0x10,   // LiDAR scan + odometry since the previous scan
  UP_EVENT          = 0x12,   // an event sensor fired (Writer)
  UP_TELEMETRY      = 0x13,
  UP_BEACON_STATUS  = 0x15,   // beacon_id + acked flag (Writer)
  UP_MISSION_TARGET = 0x16,   // a target arrived over the radio (Executor)
  UP_LOG            = 0x17,
  UP_BEACON_RX      = 0x20,   // hub: beacon received over the radio, with GPS
  UP_TARGET_REACHED = 0x21,   // hub: the Executor reports a target reached
  UP_NODE_STATUS    = 0x22,   // hub: heartbeat of a robot
  UP_MISSION_REQ    = 0x23    // hub: the Executor asked for the mission
};
// ---- host -> device
enum HostMsgDown : uint8_t {
  DN_CMD_VEL        = 0x80,
  DN_SET_MODE       = 0x81,
  DN_BEACON_TX      = 0x82,   // host composed a beacon (needs the SLAM pose); send it by radio
  DN_MISSION_REQ    = 0x83,   // Executor: ask the hub for the mission
  DN_TARGET_REACHED = 0x84,   // Executor: report a target reached over the radio
  DN_SET_CALIB      = 0x87,   // hub: entrance GPS + compass bearing
  DN_MISSION_PUSH   = 0x88    // hub: the command post's current target list
};

enum RunMode : uint8_t { MODE_IDLE = 0, MODE_RUN = 1, MODE_ESTOP = 2 };
enum RobotState : uint8_t { ST_BOOT = 0, ST_WARMUP = 1, ST_READY = 2, ST_RUN = 3,
                            ST_VERIFY = 4, ST_ESTOP = 5, ST_FAULT = 6 };
enum FaultFlags : uint16_t { FL_LIDAR_LOST = 1, FL_IMU_LOST = 2, FL_CMD_TIMEOUT = 4,
                             FL_OBSTACLE_STOP = 8, FL_RADIO_LOST = 16, FL_ESTOP = 32 };

#pragma pack(push, 1)
struct HelloMsg { uint8_t role; uint8_t proto; uint8_t fw_major; uint8_t fw_minor; };
struct ScanMsg {                      // 18 + 2*360 = 738 bytes
  uint16_t scan_id;
  uint32_t t_ms;                      // mission time at the end of the sweep
  float    odom_dxy_mm;               // distance travelled since the previous scan (BreezySLAM format)
  float    odom_dth_deg;              // heading change since the previous scan (gyro-fused)
  uint16_t dt_ms;                     // time since the previous scan
  uint16_t n;                         // = LM_SCAN_BINS
  uint16_t range_mm[LM_SCAN_BINS];    // 0 = no return
};
struct EventMsg { uint8_t event_type; uint16_t sensor_value; uint32_t t_ms; };
struct TelemetryMsg {
  uint32_t t_ms; uint8_t state; uint8_t mode; uint16_t faults;
  int16_t vl_mm_s; int16_t vr_mm_s; uint16_t battery_mv; uint16_t front_min_mm;
  uint16_t gas_raw; uint8_t beacons_pending; uint8_t radio_ok;
};
struct BeaconStatusMsg { uint8_t beacon_id; uint8_t acked; };
struct MissionTargetMsg { uint8_t index; uint8_t count; BeaconMessage beacon; };
struct CmdVelMsg { int16_t v_mm_s; int16_t w_mrad_s; uint16_t seq; };
struct SetModeMsg { uint8_t mode; };
struct TargetReachedMsg { uint8_t beacon_id; };
struct SetCalibMsg { int32_t lat_e7; int32_t lon_e7; float heading_deg; };
struct MissionPushMsg { uint8_t count; BeaconMessage targets[8]; };   // first `count` are valid
struct BeaconRxMsg { uint8_t src; uint8_t link_q; BeaconMessage beacon; int32_t lat_e7; int32_t lon_e7; };
struct NodeStatusMsg { uint8_t node; uint8_t state; int16_t x_cm; int16_t y_cm; uint16_t battery_mv;
                       uint16_t age_ms; uint8_t link_q; };
#pragma pack(pop)

// ---------------------------------------------------------------- radio link
// nRF24L01+: channel 76, 250 kbps, dynamic payloads, auto-ack, 15 retries / 1500 us.
// Star topology: the hub (outside network node) is the only node both robots talk to.
// Packet (<= 32 bytes):  type u8 | src u8 | dst u8 | seq u8 | payload
enum RadioType : uint8_t {
  RF_BEACON      = 0x01,   // Writer -> hub    : BeaconMessage (16 B)
  RF_ACK         = 0x02,   // any -> any       : {acked_type, acked_id}
  RF_HEARTBEAT   = 0x03,   // robot -> hub     : RfHeartbeat
  RF_TARGET      = 0x10,   // hub -> Executor  : {index, count, BeaconMessage}
  RF_REACHED     = 0x11,   // Executor -> hub  : {beacon_id}
  RF_MISSION_REQ = 0x12    // Executor -> hub  : (no payload)
};
#define RF_HDR 4
#define RF_MAX_PACKET 32
#pragma pack(push, 1)
struct RfHeartbeat { uint8_t state; int16_t x_cm; int16_t y_cm; uint16_t battery_mv; uint8_t faults8; };
struct RfAck { uint8_t acked_type; uint8_t acked_id; };
struct RfTarget { uint8_t index; uint8_t count; BeaconMessage beacon; };
#pragma pack(pop)

#endif
