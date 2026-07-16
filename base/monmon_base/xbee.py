"""Minimal XBee 3 (802.15.4) API-mode driver over pyserial.

Mirrors the rover firmware: brings a module to API mode @ 115200 from any
starting state, then talks 16-bit-addressed TX16 (0x01) / RX16 (0x81) frames.
API mode 1 (non-escaped) to match the firmware's AP=1.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial

BAUDS = [115200, 9600, 38400, 57600, 19200, 230400]
BROADCAST = 0xFFFF


@dataclass
class RxPacket:
    src: int              # 16-bit source address (low 16 bits if 64-bit)
    rssi: int | None      # positive magnitude; dBm = -rssi (None if frame carries none)
    options: int
    payload: bytes
    kind: str = "RX16(0x81)"


class XBee802:
    def __init__(self, port: str, baud: int = 115200, debug: bool = False):
        self.port = port
        self.debug = debug
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._buf = bytearray()
        # throughput counters (cumulative)
        self.tx_wire = 0      # bytes written to the serial link (full escaped frames)
        self.tx_frames = 0
        self.tx_app = 0       # application payload bytes sent (over the air)
        self.rx_wire = 0      # bytes read from the serial link
        self.rx_frames = 0
        self.rx_app = 0       # application payload bytes received

    # ---- low-level framing --------------------------------------------------
    def _reopen(self, baud: int) -> None:
        self.ser.close()
        time.sleep(0.2)
        self.ser = serial.Serial(self.port, baud, timeout=0.05)
        self._buf.clear()

    # ---- API mode 2 (escaped) framing --------------------------------------
    # xbee-arduino always escapes, so the whole system runs AP=2. Bytes
    # 0x7E/0x7D/0x11/0x13 after the start delimiter are escaped as
    # 0x7D followed by (byte XOR 0x20). Required for binary RTCM.
    _ESCAPE = 0x7D
    _SPECIAL = (0x7E, 0x7D, 0x11, 0x13)

    @classmethod
    def _encode_frame(cls, data: bytes) -> bytes:
        n = len(data)
        chk = 0xFF - (sum(data) & 0xFF)
        out = bytearray([0x7E])
        for b in list(((n >> 8) & 0xFF, n & 0xFF)) + list(data) + [chk]:
            if b in cls._SPECIAL:
                out.append(cls._ESCAPE)
                out.append(b ^ 0x20)
            else:
                out.append(b)
        return bytes(out)

    def _write_frame(self, data: bytes) -> None:
        enc = self._encode_frame(data)
        self.ser.write(enc)
        self.tx_wire += len(enc)
        self.tx_frames += 1

    @classmethod
    def _extract_frames(cls, buf: bytearray) -> list[bytes]:
        """Pull complete, unescaped frames out of buf, consuming what it uses."""
        out = []
        while True:
            i = buf.find(0x7E)
            if i < 0:
                buf.clear()
                break
            if i > 0:
                del buf[:i]
            # Unescape from just after the start delimiter until we have a full frame.
            unesc = bytearray()
            j = 1
            need = None
            complete = False
            while j < len(buf):
                b = buf[j]
                if b == 0x7E:            # unexpected delimiter -> current frame is junk
                    break
                if b == cls._ESCAPE:
                    if j + 1 >= len(buf):
                        j = len(buf)     # incomplete escape; wait for more
                        break
                    unesc.append(buf[j + 1] ^ 0x20)
                    j += 2
                else:
                    unesc.append(b)
                    j += 1
                if need is None and len(unesc) >= 2:
                    need = 2 + ((unesc[0] << 8) | unesc[1]) + 1
                if need is not None and len(unesc) >= need:
                    complete = True
                    break
            if complete:
                n = (unesc[0] << 8) | unesc[1]
                frame = bytes(unesc[2:2 + n])
                chk = unesc[2 + n]
                del buf[:j]
                if (sum(frame) + chk) & 0xFF == 0xFF:
                    out.append(frame)
            elif j < len(buf) and buf[j] == 0x7E:
                del buf[:j]              # drop the junk partial, resync on next delimiter
            else:
                break                    # incomplete; keep bytes and wait for more
        return out

    def _read_frames(self) -> list[bytes]:
        d = self.ser.read(512)
        if d:
            self._buf += d
            self.rx_wire += len(d)
        return self._extract_frames(self._buf)

    # ---- AT commands --------------------------------------------------------
    def send_at(self, cmd: str, value: bytes = b"", frame_id: int = 1) -> None:
        data = bytes([0x08, frame_id, ord(cmd[0]), ord(cmd[1])]) + bytes(value)
        self._write_frame(data)

    def _wait_at(self, cmd: str, timeout: float):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for f in self._read_frames():
                if len(f) >= 5 and f[0] == 0x88 and f[2] == ord(cmd[0]) and f[3] == ord(cmd[1]):
                    return f[4], bytes(f[5:])   # (status, value)
            time.sleep(0.01)
        return None, None

    def at_query(self, cmd: str, timeout: float = 2.0):
        self._read_frames()  # drain
        self.send_at(cmd)
        return self._wait_at(cmd, timeout)

    def at_set(self, cmd: str, value: bytes, timeout: float = 2.0) -> bool:
        self._read_frames()
        self.send_at(cmd, value)
        status, _ = self._wait_at(cmd, timeout)
        return status == 0

    # ---- transparent-mode fallback -----------------------------------------
    def _expect_ok(self, timeout: float) -> bool:
        t0 = time.time()
        s = b""
        while time.time() - t0 < timeout:
            s += self.ser.read(16)
            if s.endswith(b"OK\r"):
                return True
        return False

    def _enter_cmd_mode(self) -> bool:
        time.sleep(1.1)
        self.ser.reset_input_buffer()
        self.ser.write(b"+++")
        return self._expect_ok(1.5)

    def _cmd(self, line: str) -> bool:
        self.ser.write(line.encode() + b"\r")
        return self._expect_ok(1.5)

    # ---- bootstrap + config -------------------------------------------------
    def bootstrap(self) -> bool:
        """Bring the module to API mode @ 115200 from any state."""
        for b in BAUDS:                                   # pass 1: already API?
            self._reopen(b)
            status, _ = self.at_query("AP", timeout=0.7)
            if status is not None:
                if self.debug:
                    print(f"  [xbee] API mode at {b} baud")
                if b != 115200:
                    self.at_set("BD", bytes([7]))
                    self.at_set("WR", b"")
                    self.at_set("AC", b"")
                    self._reopen(115200)
                return True
        for b in BAUDS:                                   # pass 2: transparent?
            self._reopen(b)
            if self._enter_cmd_mode():
                if self.debug:
                    print(f"  [xbee] transparent at {b} baud; converting")
                self._cmd("ATAP2")
                self._cmd("ATBD7")
                self._cmd("ATWR")
                self._cmd("ATCN")
                self._reopen(115200)
                status, _ = self.at_query("AP", timeout=0.7)
                if status is not None:
                    return True
        return False

    def configure(self, pan: int, channel: int, my: int) -> dict:
        """Force network params; returns which set calls succeeded."""
        # AP=2 first and applied: escaped framing must match before sending any
        # parameter whose frame might contain 0x7E/0x7D/0x11/0x13.
        ap = self.at_set("AP", bytes([2]))
        self.at_set("AC", b"")
        time.sleep(0.1)
        res = {
            "AP": ap,
            "MM": self.at_set("MM", bytes([0])),   # Digi Mode: clean framing + ACKs/retries
            "AO": self.at_set("AO", bytes([2])),   # legacy 0x80/0x81 RX frames (carry RSSI)
            "ID": self.at_set("ID", bytes([(pan >> 8) & 0xFF, pan & 0xFF])),
            "CH": self.at_set("CH", bytes([channel])),
            "MY": self.at_set("MY", bytes([(my >> 8) & 0xFF, my & 0xFF])),
            "AC": self.at_set("AC", b""),
        }
        return res

    # ---- data path ----------------------------------------------------------
    def tx16(self, dest: int, payload: bytes, options: int = 0, frame_id: int = 0) -> None:
        data = bytes([0x01, frame_id, (dest >> 8) & 0xFF, dest & 0xFF, options]) + bytes(payload)
        self.tx_app += len(payload)
        self._write_frame(data)

    def broadcast(self, payload: bytes) -> None:
        self.tx16(BROADCAST, payload)

    def rx(self) -> list[RxPacket]:
        """Return any receive packets since the last call (handles legacy + modern)."""
        out = []
        for f in self._read_frames():
            if not f:
                continue
            api = f[0]
            if api == 0x81 and len(f) >= 5:            # RX16 (legacy)
                out.append(RxPacket((f[1] << 8) | f[2], f[3], f[4], bytes(f[5:]), "RX16(0x81)"))
            elif api == 0x80 and len(f) >= 11:          # RX64 (legacy)
                out.append(RxPacket((f[7] << 8) | f[8], f[9], f[10], bytes(f[11:]), "RX64(0x80)"))
            elif api == 0x90 and len(f) >= 12:          # Receive Packet (modern)
                out.append(RxPacket((f[9] << 8) | f[10], None, f[11], bytes(f[12:]), "RX(0x90)"))
            elif api == 0x89:                           # TX status
                continue
            else:
                continue
        for rp in out:
            self.rx_frames += 1
            self.rx_app += len(rp.payload)
        return out

    def close(self) -> None:
        self.ser.close()
