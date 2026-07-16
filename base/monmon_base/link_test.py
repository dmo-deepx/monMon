"""Interactive XBee link test for the monMon base.

Bootstraps the base XBee, prints identity, runs an MY read/write self-test,
then listens for rover packets while broadcasting a dummy RTCM packet at 1 Hz.

Usage:
    uv run monmon-link-test [PORT]           (default /dev/cu.usbserial-4)
"""

from __future__ import annotations

import sys
import time

from . import packet
from .xbee import XBee802

PAN = 0x3332
CHANNEL = 0x0C
BASE_MY = 0x0000


def _proto(vr: bytes) -> str:
    if not vr:
        return "unknown"
    n = {0x1: "Zigbee", 0x2: "802.15.4", 0x3: "DigiMesh"}.get(vr[0] >> 4, "unknown")
    return n


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-4"
    print(f"[base] opening {port}")
    xb = XBee802(port, debug=True)

    print("[base] bootstrapping XBee (any state -> API @ 115200)...")
    if not xb.bootstrap():
        print("[base] BOOTSTRAP FAILED — check the adapter/port and try again.")
        return

    _, vr = xb.at_query("VR")
    _, hv = xb.at_query("HV")
    print(f"[base] VR={vr.hex().upper() if vr else '?'} ({_proto(vr or b'')})  "
          f"HV={hv.hex().upper() if hv else '?'}")
    for p in ("MM", "AO", "CE", "A2"):
        _, v = xb.at_query(p)
        print(f"[base] {p}={v.hex() if v else '?'}")

    # --- MY write self-test: does this module accept an MY write? ---
    _, before = xb.at_query("MY")
    ok = xb.at_set("MY", b"\xB0\x07")
    _, after = xb.at_query("MY")
    print(f"[base] MY self-test: set_ok={ok} before={before.hex() if before else '?'} "
          f"after={after.hex() if after else '?'} (expected B007)")

    _, ap_before = xb.at_query("AP")
    res = xb.configure(PAN, CHANNEL, BASE_MY)
    _, ap_after = xb.at_query("AP")
    _, my = xb.at_query("MY")
    print(f"[base] AP {ap_before.hex() if ap_before else '?'} -> "
          f"{ap_after.hex() if ap_after else '?'}  (1 = non-escaped, required)")
    print(f"[base] configured {res}  MY now={my.hex() if my else '?'} "
          f"(PAN={PAN:04X} CH={CHANNEL:02X})")

    print("\n[base] LISTENING. Press a rover button to send a MARK; GGA arrives at 1 Hz "
          "if the F9P is streaming.")
    print("[base] Broadcasting a dummy RTCM packet every second — the rover's "
          "'RTCM' age on the TFT should reset.\n")

    seq = 0
    last_tx = 0.0
    last_rover_rssi = None
    try:
        while True:
            for rp in xb.rx():
                if rp.rssi is not None:
                    last_rover_rssi = rp.rssi
                rssi_s = f"-{rp.rssi}dBm" if rp.rssi is not None else "rssi?"
                pkt = packet.decode(rp.payload)
                if pkt:
                    text = pkt.data.decode("ascii", "replace").strip()
                    print(f"RX {rp.kind} src=0x{rp.src:04X} {rssi_s} "
                          f"{pkt.type_name} seq={pkt.seq} len={len(pkt.data)}: {text}  "
                          f"raw={rp.payload.hex()}")
                else:
                    print(f"RX {rp.kind} src=0x{rp.src:04X} {rssi_s} raw={rp.payload.hex()}")

            now = time.time()
            if now - last_tx >= 1.0:
                last_tx = now
                # RTCM path test: dummy correction bytes (F9P ignores garbage)
                xb.broadcast(packet.encode(packet.RTCM, seq, 0, b"MONMON-RTCM-TEST"))
                # TELEM: useful info for the rover to display
                up = f"-{last_rover_rssi}" if last_rover_rssi is not None else "??"
                hhmmss = time.strftime("%H:%M:%SZ", time.gmtime())
                info = f"up{up} NTRIP:off {hhmmss}"
                xb.broadcast(packet.encode(packet.TELEM, seq, 0, info.encode("ascii")))
                seq = (seq + 1) & 0xFF
                print(f"TX broadcast RTCM+TELEM seq={seq} info='{info}'")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[base] stopping.")
    finally:
        xb.close()


if __name__ == "__main__":
    main()
