// exxeta-arm wire protocol, version 1 (firmware side).
//
// Role: encode and decode the frames exchanged with the PC. This file and
// protocol.cpp contain NO Arduino code, so the same source is compiled into the
// ESP32 firmware and into the host unit tests (`pio test -e native`).
// The Python twin is dora/exxeta_arm/protocol.py; both are checked against
// tests/protocol_vectors.json. Description: docs/software/protocol.md.
//
// Frame:  ver:u8=1 | type:u8 | seq:u16 | payload | crc16     (little-endian)
// Serial: 0x00, COBS(frame), 0x00        UDP: one frame per datagram.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace proto {

const uint8_t VERSION = 1;
const int N_JOINTS = 6;
const int N_POTS = 4;
const size_t MAX_FRAME = 200;  // largest frame we ever send or accept

enum Msg : uint8_t {
  MSG_HELLO = 0x01, MSG_HEARTBEAT = 0x02, MSG_ENABLE = 0x03, MSG_DISABLE = 0x04, MSG_SET_JOINT_TARGETS = 0x05,
  MSG_STOP = 0x06, MSG_CLEAR = 0x07, MSG_RELEASE = 0x08, MSG_GET_CONFIG = 0x10, MSG_SET_CONFIG = 0x11, MSG_CALIBRATION = 0x12,
  MSG_ACK = 0x80, MSG_HELLO_ACK = 0x81, MSG_STATE = 0x82, MSG_LOG = 0x83, MSG_CONFIG = 0x90
};
enum Mode : uint8_t { MODE_DISABLED = 0, MODE_IDLE = 1, MODE_MOVING = 2, MODE_STOPPED = 3, MODE_FAULT = 4 };
enum Fault : uint16_t {
  FLT_LINK_LOST = 1, FLT_BATTERY_WARN = 2, FLT_BATTERY_CRITICAL = 4, FLT_SERVO_POWER_OFF = 8,
  FLT_STOP_LATCHED = 16, FLT_LIMIT_CLAMPED = 32, FLT_CHARGER_PRESENT = 64
};
enum Status : uint8_t { ST_OK = 0, ST_BUSY = 1, ST_BAD_TOKEN = 2, ST_BAD_ARG = 3, ST_REFUSED = 4 };
enum Cfg : uint8_t {
  CFG_SERVO_CAL = 1, CFG_JOINT_LIMITS = 2, CFG_POT_CAL = 3, CFG_DEVICE_NAME = 4, CFG_AUTH_TOKEN = 5,
  CFG_WIFI = 6, CFG_WATCHDOG_MS = 7, CFG_STATE_RATE_HZ = 8, CFG_REST_POSE = 9
};
enum Cal : uint8_t { CAL_JOG_PULSE = 1, CAL_SAVE = 2, CAL_FACTORY_RESET = 3 };

uint16_t crc16(const uint8_t* data, size_t n);

// COBS. `out` must hold n + n/254 + 1 bytes. Decode returns 0 on malformed input.
size_t cobsEncode(const uint8_t* in, size_t n, uint8_t* out);
size_t cobsDecode(const uint8_t* in, size_t n, uint8_t* out);

struct Frame {
  uint8_t type = 0;
  uint16_t seq = 0;
  const uint8_t* payload = nullptr;  // points into the buffer given to decodeFrame
  size_t len = 0;
};

// Returns the frame length written to `out` (needs len + 6 bytes).
size_t encodeFrame(uint8_t type, uint16_t seq, const uint8_t* payload, size_t len, uint8_t* out);
bool decodeFrame(const uint8_t* raw, size_t n, Frame& f);

// little-endian helpers
inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
inline void put32(uint8_t* p, uint32_t v) { put16(p, v & 0xFFFF); put16(p + 2, v >> 16); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline int16_t get16s(const uint8_t* p) { return (int16_t)get16(p); }
inline uint32_t get32(const uint8_t* p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }

// Angles travel as centi-degrees.
int16_t toCdeg(float deg);

struct State {
  uint8_t mode = 0;
  uint16_t faults = 0;
  float targetDeg[N_JOINTS] = {0};
  float positionDeg[N_JOINTS] = {0};
  float potDeg[N_POTS] = {0};
  uint8_t potValid = 0;
  uint16_t batteryMv = 0;
  uint8_t batteryPct = 0;
  uint32_t uptimeMs = 0;
  uint16_t rxOk = 0, rxBad = 0, lastSeq = 0;
};
const size_t STATE_SIZE = 3 + 12 + 12 + 8 + 1 + 2 + 1 + 4 + 6;  // 49 bytes
size_t packState(const State& s, uint8_t* out);

}  // namespace proto
