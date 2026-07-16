#pragma once
#include <stdint.h>
#include <string.h>

// ============================================================================
//  monMon application packet — rides *inside* one XBee 802.15.4 API payload.
//  Mirror this layout in the Python base (base/.../core/packets.py).
//
//    byte 0  TYPE   see PacketType
//    byte 1  SEQ    rolling 0..255, for loss/reorder detection
//    byte 2  FLAGS  bit0 = MORE fragments follow (reassembly)
//    byte 3+ PAYLOAD (<= MAX_DATA bytes)
// ============================================================================

namespace monmon {

enum PacketType : uint8_t {
  PKT_RTCM    = 0x01,  // base -> rover: chunk of the RTCM3 byte stream
  PKT_NMEA    = 0x02,  // rover -> base: one NMEA sentence (or fragment)
  PKT_CONTROL = 0x03,  // either way: stakeout MARK, config, etc.
  PKT_TELEM   = 0x04,  // reserved: structured telemetry
};

enum PacketFlag : uint8_t {
  FLAG_MORE = 0x01,    // more fragments follow this one
};

static const uint8_t HEADER_LEN = 3;
static const uint8_t MAX_FRAME  = 100;                 // 802.15.4 payload cap
static const uint8_t MAX_DATA   = MAX_FRAME - HEADER_LEN;  // 97 usable bytes

// Encode a frame into out[] (capacity >= HEADER_LEN + dataLen). Returns length.
inline uint8_t encode(uint8_t* out, uint8_t type, uint8_t seq, uint8_t flags,
                      const uint8_t* data, uint8_t dataLen) {
  out[0] = type;
  out[1] = seq;
  out[2] = flags;
  if (dataLen) memcpy(out + HEADER_LEN, data, dataLen);
  return HEADER_LEN + dataLen;
}

// A decoded view over a received frame (data points into the source buffer).
struct Frame {
  uint8_t        type;
  uint8_t        seq;
  uint8_t        flags;
  const uint8_t* data;
  uint8_t        len;
};

inline bool decode(const uint8_t* in, uint8_t inLen, Frame& f) {
  if (inLen < HEADER_LEN) return false;
  f.type  = in[0];
  f.seq   = in[1];
  f.flags = in[2];
  f.data  = in + HEADER_LEN;
  f.len   = inLen - HEADER_LEN;
  return true;
}

}  // namespace monmon
