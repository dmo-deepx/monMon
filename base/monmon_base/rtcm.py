"""Tiny RTCM3 stream inspector — counts message types (CRC-validated).

Used to see what a caster is actually sending. RTCM3 frame:
  0xD3 | 6b reserved + 10b length | payload[length] | CRC-24Q[3]
message type = first 12 bits of the payload (DF002).
"""

from __future__ import annotations


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


# What common types mean (for the dashboard / diagnosis).
KNOWN = {
    1005: "ARP", 1006: "ARP+H", 1007: "ant", 1008: "ant", 1033: "rcv",
    1074: "GPS", 1075: "GPS", 1077: "GPS",
    1084: "GLO", 1085: "GLO", 1087: "GLO",
    1094: "GAL", 1095: "GAL", 1097: "GAL",
    1124: "BDS", 1125: "BDS", 1127: "BDS",
    1230: "GLObias",
}


class RtcmScanner:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.counts: dict[int, int] = {}
        self.total = 0
        self.bad_crc = 0
        self.head = b""          # first bytes ever seen (for identifying non-RTCM streams)

    def feed(self, data: bytes) -> None:
        if len(self.head) < 32 and data:
            self.head = (self.head + data)[:32]
        b = self.buf
        b += data
        while len(b) >= 6:
            if b[0] != 0xD3:
                nxt = b.find(0xD3, 1)
                if nxt < 0:
                    b.clear()
                    return
                del b[:nxt]
                continue
            length = ((b[1] & 0x03) << 8) | b[2]
            need = 3 + length + 3
            if len(b) < need:
                return                      # wait for the rest of the frame
            crc = (b[3 + length] << 16) | (b[3 + length + 1] << 8) | b[3 + length + 2]
            if crc24q(bytes(b[: 3 + length])) == crc:
                if length >= 2:
                    mt = (b[3] << 4) | (b[4] >> 4)
                    self.counts[mt] = self.counts.get(mt, 0) + 1
                    self.total += 1
                del b[:need]
            else:
                self.bad_crc += 1
                del b[:1]                   # resync on the next preamble

    def has_base_position(self) -> bool:
        return bool({1005, 1006} & self.counts.keys())

    def summary(self, limit: int = 8) -> str:
        if not self.counts:
            return "(none yet)"
        items = sorted(self.counts.items())
        return "  ".join(f"{t}×{n}" for t, n in items[:limit])
