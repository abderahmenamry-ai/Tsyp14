// link.h - framing for the host link (see protocol.h).
#ifndef LM_LINK_H
#define LM_LINK_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "crc.h"
#include "protocol.h"

// Build one frame into `out` (size >= len + 7). Returns total bytes written.
inline size_t linkEncode(uint8_t type, const void *payload, uint16_t len, uint8_t *out) {
  out[0] = LM_FRAME_SOF0; out[1] = LM_FRAME_SOF1;
  out[2] = type; out[3] = (uint8_t)(len & 0xFF); out[4] = (uint8_t)(len >> 8);
  if (len) memcpy(out + 5, payload, len);
  uint16_t crc = crc16_ccitt(out + 2, 3 + len);
  out[5 + len] = (uint8_t)(crc & 0xFF); out[6 + len] = (uint8_t)(crc >> 8);
  return (size_t)len + 7;
}

// Byte-stream parser. Feed bytes; returns true when a complete, CRC-valid frame is ready.
// Resynchronises automatically after noise or a corrupt frame.
class FrameParser {
 public:
  FrameParser() { reset(); }
  bool feed(uint8_t b) {
    switch (st_) {
      case 0: if (b == LM_FRAME_SOF0) st_ = 1; break;
      case 1: st_ = (b == LM_FRAME_SOF1) ? 2 : (b == LM_FRAME_SOF0 ? 1 : 0); break;
      case 2: type_ = b; st_ = 3; break;
      case 3: len_ = b; st_ = 4; break;
      case 4:
        len_ |= (uint16_t)b << 8;
        if (len_ > LM_MAX_PAYLOAD) { bad_++; reset(); break; }
        pos_ = 0; st_ = len_ ? 5 : 6; break;
      case 5: buf_[pos_++] = b; if (pos_ == len_) st_ = 6; break;
      case 6: crcLo_ = b; st_ = 7; break;
      case 7: {
        uint16_t rx = crcLo_ | ((uint16_t)b << 8);
        uint8_t hdr[3] = { type_, (uint8_t)(len_ & 0xFF), (uint8_t)(len_ >> 8) };
        uint16_t c = crc16_ccitt(hdr, 3);
        c = crc16_ccitt(buf_, len_, c);
        st_ = 0;
        if (c == rx) return true;
        bad_++; return false;
      }
    }
    return false;
  }
  uint8_t type() const { return type_; }
  uint16_t length() const { return len_; }
  const uint8_t *payload() const { return buf_; }
  uint32_t badFrames() const { return bad_; }
  void reset() { st_ = 0; len_ = 0; pos_ = 0; }
 private:
  uint8_t st_ = 0, type_ = 0, crcLo_ = 0;
  uint16_t len_ = 0, pos_ = 0;
  uint8_t buf_[LM_MAX_PAYLOAD];
  uint32_t bad_ = 0;
};
#endif
