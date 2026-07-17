"""Verify what a caster actually streams — no XBee/rover needed.

Connects to a caster, de-chunks + CRC-validates the stream, and reports the RTCM
message-type breakdown (or an ASCII/hex preview if it isn't RTCM). Handy for
debugging a base station after each config change.

Usage:
    uv run monmon-caster-check [caster] [-c config.yaml] [-s seconds]
"""

from __future__ import annotations

import argparse
import time

from . import config, rtcm
from .ntrip import NtripClient, make_gga


def main() -> None:
    ap = argparse.ArgumentParser(
        prog="monmon-caster-check",
        description="Verify what RTCM a caster streams (no XBee needed).",
    )
    ap.add_argument("caster", nargs="?", help="caster name (default: config's 'default')")
    ap.add_argument("-c", "--config", default="config.yaml")
    ap.add_argument("-s", "--seconds", type=float, default=12.0)
    args = ap.parse_args()

    cfg = config.load(args.config, caster=args.caster)
    n = cfg.ntrip
    print(f"[{cfg.caster}] connecting to {n.host}:{n.port}/{n.mountpoint} …")
    nt = NtripClient(n)
    nt.start()

    sc = rtcm.RtcmScanner()
    preview = bytearray()
    t0 = time.time()
    nudged = False
    while time.time() - t0 < args.seconds:
        while not nt.rtcm.empty():
            chunk = nt.rtcm.get_nowait()
            if len(preview) < 240:
                preview += chunk
            sc.feed(chunk)
        if not nudged and time.time() - t0 > 4 and sc.total == 0 and nt.connected:
            nt.send_gga(make_gga(n.static_lat or 35.0, n.static_lon or 139.0, n.static_alt or 40.0))
            nudged = True
            print("  (no RTCM yet — sent a placeholder GGA to nudge a VRS caster)")
        time.sleep(0.1)
    while not nt.rtcm.empty():
        sc.feed(nt.rtcm.get_nowait())
    nt.stop()

    err = f"  error={nt.last_error!r}" if nt.last_error else ""
    print(f"connected={nt.connected}  bytes_in={nt.bytes_in}{err}")

    if sc.total:
        base = "yes" if sc.has_base_position() else "NO — need 1005/1006"
        print(f"RTCM OK: {sc.total} msgs  bad_crc={sc.bad_crc}  base_pos={base}")
        for t in sorted(sc.counts):
            print(f"  {t:5d} x{sc.counts[t]:<5} {rtcm.KNOWN.get(t, '')}")
        if not sc.has_base_position():
            print("  WARNING: no base position (1005/1006) — rover cannot reach RTK fix")
    else:
        h = bytes(preview)
        proto = ("UBX" if h[:2] == b"\xb5\x62"
                 else "RTCM3?" if h[:1] == b"\xd3"
                 else "TEXT/NMEA" if h[:1].isalpha() or h[:1] in (b"$", b"!")
                 else "unknown")
        print(f"NO valid RTCM (looks like {proto}, bad_crc={sc.bad_crc}).")
        print(f"  hex:   {h[:32].hex()}")
        print(f"  ascii: {h[:120].decode('ascii', 'replace')}")


if __name__ == "__main__":
    main()
