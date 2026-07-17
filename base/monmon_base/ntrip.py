"""Minimal NTRIP client (Rev 1 & 2).

Runs a reader thread that connects to the caster, streams RTCM3 into a queue,
and reconnects on failure. GGA is pushed upstream via send_gga() from any thread
(sockets are full-duplex). Kept hand-rolled + pure-Python so it fits our
"GGA arrives over the radio" topology and bundles cleanly for offline use.
"""

from __future__ import annotations

import base64
import queue
import socket
import threading
import time


class _Dechunker:
    """Incremental HTTP/1.1 chunked-transfer decoder (NTRIP v2 casters use it)."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.remaining = 0        # bytes left in the current chunk body

    def feed(self, data: bytes) -> bytes:
        self.buf += data
        out = bytearray()
        while True:
            if self.remaining > 0:
                take = min(self.remaining, len(self.buf))
                if take == 0:
                    break
                out += self.buf[:take]
                del self.buf[:take]
                self.remaining -= take
            else:
                idx = self.buf.find(b"\r\n")
                if idx < 0:
                    break
                line = bytes(self.buf[:idx]).strip()
                del self.buf[: idx + 2]
                if not line:            # the CRLF that trails a chunk body
                    continue
                try:
                    self.remaining = int(line.split(b";")[0], 16)
                except ValueError:
                    break               # not chunked after all; stop trying
                if self.remaining == 0:
                    break               # final chunk
        return bytes(out)


class NtripClient(threading.Thread):
    def __init__(self, cfg):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.rtcm: queue.Queue[bytes] = queue.Queue()
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        # status (read by the dashboard)
        self.connected = False
        self.bytes_in = 0
        self.last_error = ""
        self.gga_sent = 0

    # ---- connection ---------------------------------------------------------
    def _open(self):
        c = self.cfg
        sock = socket.create_connection((c.host, c.port), timeout=10)
        if c.https:
            import ssl
            sock = ssl.create_default_context().wrap_socket(sock, server_hostname=c.host)
        auth = base64.b64encode(f"{c.username}:{c.password}".encode()).decode()
        if c.version >= 2:
            req = (
                f"GET /{c.mountpoint} HTTP/1.1\r\n"
                f"Host: {c.host}:{c.port}\r\n"
                f"Ntrip-Version: Ntrip/2.0\r\n"
                f"User-Agent: NTRIP monMon/0.1\r\n"
                f"Authorization: Basic {auth}\r\n"
                f"Connection: close\r\n\r\n"
            )
        else:
            req = (
                f"GET /{c.mountpoint} HTTP/1.0\r\n"
                f"User-Agent: NTRIP monMon/0.1\r\n"
                f"Authorization: Basic {auth}\r\n\r\n"
            )
        sock.sendall(req.encode())

        # Read the handshake: v1 => "ICY 200 OK\r\n" then RTCM; v2 => HTTP headers
        # ending in \r\n\r\n then RTCM. Preserve any RTCM bytes that trail it.
        sock.settimeout(15)
        data = b""
        initial = b""
        chunked = False
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("caster closed during handshake")
            data += chunk
            if data[:3] == b"ICY" and b"\r\n" in data:
                hdr, _, initial = data.partition(b"\r\n")   # v1: raw stream, never chunked
                if b"200" not in hdr:
                    raise ConnectionError(f"caster refused: {hdr!r}")
                break
            if b"\r\n\r\n" in data:
                hdr, _, initial = data.partition(b"\r\n\r\n")
                first = hdr.split(b"\r\n", 1)[0]
                if b"SOURCETABLE" in first:
                    raise ConnectionError("got sourcetable — check mountpoint name")
                if b"200" not in first:
                    raise ConnectionError(f"caster refused: {first!r}")
                chunked = b"chunked" in hdr.lower()          # NTRIP v2 may use HTTP chunked
                break
            if len(data) > 8192:
                raise ConnectionError("no valid handshake from caster")
        return sock, initial, chunked

    def run(self):
        while not self._stop.is_set():
            try:
                sock, initial, chunked = self._open()
                with self._lock:
                    self._sock = sock
                self.connected = True
                self.last_error = ""
                dechunk = _Dechunker() if chunked else None

                def emit(raw: bytes) -> None:
                    data = dechunk.feed(raw) if dechunk else raw
                    if data:
                        self.rtcm.put(data)
                        self.bytes_in += len(data)

                if initial:
                    emit(initial)
                sock.settimeout(1.0)
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("stream closed by caster")
                    emit(chunk)
            except Exception as e:  # noqa: BLE001 — surface any failure, then retry
                self.connected = False
                self.last_error = str(e)
                self._close_sock()
                if self._stop.wait(3.0):
                    break

    # ---- io -----------------------------------------------------------------
    def send_gga(self, gga: str) -> None:
        with self._lock:
            s = self._sock
        if s is None or not self.connected:
            return
        try:
            s.sendall((gga.strip() + "\r\n").encode("ascii", "replace"))
            self.gga_sent += 1
        except OSError as e:
            self.last_error = f"gga send: {e}"

    def _close_sock(self) -> None:
        with self._lock:
            s, self._sock = self._sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass

    def stop(self) -> None:
        self._stop.set()
        self._close_sock()


def make_gga(lat: float, lon: float, alt: float) -> str:
    """Build a minimal $GPGGA (fix quality 1) for a static reference position."""
    hhmmss = time.strftime("%H%M%S", time.gmtime()) + ".00"
    la = abs(lat)
    lo = abs(lon)
    latd, lond = int(la), int(lo)
    body = (
        f"GPGGA,{hhmmss},"
        f"{latd:02d}{(la - latd) * 60:09.6f},{'N' if lat >= 0 else 'S'},"
        f"{lond:03d}{(lo - lond) * 60:09.6f},{'E' if lon >= 0 else 'W'},"
        f"1,10,1.0,{alt:.1f},M,0.0,M,,"
    )
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"
