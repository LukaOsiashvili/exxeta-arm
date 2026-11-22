// See protocol.h. Pure C++ (no Arduino), shared with the host unit tests.
#include "protocol.h"

#include <math.h>
#include <string.h>

namespace proto {

uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;  // CRC-16/CCITT-FALSE
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

size_t cobsEncode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t codeIdx = 0, o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < n; i++) {
    if (in[i] == 0) {
      out[codeIdx] = code;
      codeIdx = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFF) {
        out[codeIdx] = code;
        codeIdx = o++;
        code = 1;
      }
    }
  }
  out[codeIdx] = code;
  return o;
}

size_t cobsDecode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t i = 0, o = 0;
  while (i < n) {
    uint8_t code = in[i];
    if (code == 0 || i + code > n) return 0;
    for (uint8_t k = 1; k < code; k++) out[o++] = in[i + k];
    i += code;
    if (code < 0xFF && i < n) out[o++] = 0;
  }
  return o;
}

size_t encodeFrame(uint8_t type, uint16_t seq, const uint8_t* payload, size_t len, uint8_t* out) {
  out[0] = VERSION;
  out[1] = type;
  put16(out + 2, seq);
  if (len) memcpy(out + 4, payload, len);
  put16(out + 4 + len, crc16(out, 4 + len));
  return len + 6;
}

bool decodeFrame(const uint8_t* raw, size_t n, Frame& f) {
  if (n < 6) return false;
  if (crc16(raw, n - 2) != get16(raw + n - 2)) return false;
  if (raw[0] != VERSION) return false;
  f.type = raw[1];
  f.seq = get16(raw + 2);
  f.payload = raw + 4;
  f.len = n - 6;
  return true;
}

int16_t toCdeg(float deg) {
  float v = roundf(deg * 100.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

size_t packState(const State& s, uint8_t* out) {
  size_t o = 0;
  out[o++] = s.mode;
  put16(out + o, s.faults); o += 2;
  for (int i = 0; i < N_JOINTS; i++) { put16(out + o, (uint16_t)toCdeg(s.targetDeg[i])); o += 2; }
  for (int i = 0; i < N_JOINTS; i++) { put16(out + o, (uint16_t)toCdeg(s.positionDeg[i])); o += 2; }
  for (int i = 0; i < N_POTS; i++) { put16(out + o, (uint16_t)toCdeg(s.potDeg[i])); o += 2; }
  out[o++] = s.potValid;
  put16(out + o, s.batteryMv); o += 2;
  out[o++] = s.batteryPct;
  put32(out + o, s.uptimeMs); o += 4;
  put16(out + o, s.rxOk); o += 2;
  put16(out + o, s.rxBad); o += 2;
  put16(out + o, s.lastSeq); o += 2;
  return o;
}

}  // namespace proto
