"""monMon application packet codec — Python mirror of firmware src/monmon_packet.h.

Layout inside one XBee payload:
    byte 0  TYPE
    byte 1  SEQ    rolling 0..255
    byte 2  FLAGS  bit0 = MORE fragments follow
    byte 3+ PAYLOAD
"""

from __future__ import annotations

from dataclasses import dataclass

# packet types
RTCM = 0x01
NMEA = 0x02
CONTROL = 0x03
TELEM = 0x04

TYPE_NAME = {RTCM: "RTCM", NMEA: "NMEA", CONTROL: "CTRL", TELEM: "TELEM"}

# flags
FLAG_MORE = 0x01

HEADER_LEN = 3
MAX_FRAME = 100
MAX_DATA = MAX_FRAME - HEADER_LEN  # 97


def encode(ptype: int, seq: int, flags: int, data: bytes) -> bytes:
    return bytes([ptype & 0xFF, seq & 0xFF, flags & 0xFF]) + bytes(data)


@dataclass
class Packet:
    type: int
    seq: int
    flags: int
    data: bytes

    @property
    def type_name(self) -> str:
        return TYPE_NAME.get(self.type, f"0x{self.type:02X}")


def decode(buf: bytes) -> Packet | None:
    if len(buf) < HEADER_LEN:
        return None
    return Packet(buf[0], buf[1], buf[2], bytes(buf[HEADER_LEN:]))


def chunk_rtcm(data: bytes, seq_start: int = 0):
    """Split an RTCM byte stream into ordered RTCM packets (frames)."""
    frames = []
    seq = seq_start
    for off in range(0, len(data), MAX_DATA):
        chunk = data[off:off + MAX_DATA]
        more = FLAG_MORE if off + MAX_DATA < len(data) else 0
        frames.append(encode(RTCM, seq, more, chunk))
        seq = (seq + 1) & 0xFF
    return frames, seq
