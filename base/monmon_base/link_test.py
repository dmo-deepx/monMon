"""Live XBee link dashboard for the monMon base.

Bootstraps the base XBee, then shows a per-rover link-budget panel while
broadcasting a dummy RTCM + TELEM packet each second (until real NTRIP lands).

Usage:
    uv run monmon-link-test [PORT]           (default /dev/cu.usbserial-3)
"""

from __future__ import annotations

import sys
import time

from . import packet
from .xbee import XBee802

PAN = 0x3332
CHANNEL = 0x0C
BASE_MY = 0x0000

# Link budget: XBee 3 802.15.4 RX sensitivity is ~-103 dBm; use -100 as a
# conservative floor. margin = received_dBm - floor.
NOISE_FLOOR = -100
MARGIN_FULL = 60          # dB that fills the bar
BAR_W = 14

# Bandwidth budget
SERIAL_BPS = 11520        # 115200 baud, 8N1 -> 11520 usable bytes/s (host<->XBee UART)
BYTE_US = 32              # 802.15.4 over-air: 250 kbps -> 32 us/byte
OA_OVERHEAD = 20          # ~PHY+MAC+Digi header+FCS bytes added per frame over air
TX_FIXED_US = 1200        # broadcast: CCA + backoff + preamble
RX_FIXED_US = 2200        # unicast: rover TX + ACK turnaround

# ANSI
CLEAR = "\033[2J\033[H"
HIDE, SHOW = "\033[?25l", "\033[?25h"
RESET, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
GREEN, YELLOW, RED, CYAN = "\033[92m", "\033[93m", "\033[91m", "\033[96m"


def _utc() -> str:
    return time.strftime("%H:%M:%SZ", time.gmtime())


def _margin_color(margin: int) -> str:
    if margin >= 40:
        return GREEN
    if margin >= 22:
        return YELLOW
    return RED


def _bar(margin: int) -> str:
    filled = max(0, min(BAR_W, round(margin / MARGIN_FULL * BAR_W)))
    return "█" * filled + "░" * (BAR_W - filled)


def _pct_bar(pct: float, width: int = BAR_W) -> str:
    filled = max(0, min(width, round(pct / 100 * width)))
    return "█" * filled + "░" * (width - filled)


def _pct_color(pct: float) -> str:
    if pct >= 80:
        return RED
    if pct >= 50:
        return YELLOW
    return GREEN


def _rating(margin: int) -> str:
    if margin >= 40:
        return "excellent"
    if margin >= 30:
        return "good"
    if margin >= 22:
        return "fair"
    if margin >= 12:
        return "marginal"
    return "poor"


def _render(vr: bytes | None, tx_count: int, rovers: dict, bw: dict) -> None:
    now = time.time()
    out = [
        f"{BOLD}{CYAN}monMon base{RESET}  "
        f"PAN {PAN:04X}  CH {CHANNEL:02X}  MY {BASE_MY:04X}   {_utc()}",
        f"{DIM}NTRIP: off   ·   broadcasting test RTCM+TELEM   ·   tx={tx_count}{RESET}",
        "",
        f"{BOLD}{'ROVER':<8}{'RSSI':>8}{'MARGIN':>8}  {'LINK':<{BAR_W}}  "
        f"{'RATING':<10}{'PKTS':>7}{'AGE':>6}  LAST{RESET}",
    ]
    if not rovers:
        out.append(f"{DIM}  waiting for rovers…{RESET}")
    for src in sorted(rovers):
        r = rovers[src]
        rssi = r.get("rssi")
        age = now - r.get("last", now)
        age_s = f"{age:4.1f}s"
        if age > 5:
            age_s = f"{RED}{age_s}{RESET}"
        if rssi is None:
            row = (f"0x{src:04X}{'--':>8}{'--':>8}  {' ' * BAR_W}  "
                   f"{'--':<10}{r.get('count', 0):>7}{age_s:>6}")
        else:
            dbm = -rssi
            margin = dbm - NOISE_FLOOR
            c = _margin_color(margin)
            last = r.get("text", "")[:22]
            row = (f"0x{src:04X}{dbm:>6}dBm{margin:>6}dB  {c}{_bar(margin)}{RESET}  "
                   f"{c}{_rating(margin):<10}{RESET}{r.get('count', 0):>7} {age_s}  "
                   f"{DIM}{last}{RESET}")
        out.append(row)
    out.append("")
    ac = _pct_color(bw["air"])
    sc_tx = _pct_color(bw["ser_tx"])
    out.append(f"{BOLD}BANDWIDTH{RESET}  "
               f"TX {bw['tx_bps']:>4.0f}B/s {bw['tx_fps']:>2.0f}fr/s   "
               f"RX {bw['rx_bps']:>4.0f}B/s {bw['rx_fps']:>2.0f}fr/s")
    out.append(f"           serial TX {sc_tx}{bw['ser_tx']:>2.0f}%{RESET} "
               f"RX {bw['ser_rx']:>2.0f}%   "
               f"channel ~{ac}{bw['air']:>2.0f}%{RESET} {ac}{_pct_bar(bw['air'])}{RESET}")
    out.append("")
    out.append(f"{DIM}Ctrl-C to quit{RESET}")
    sys.stdout.write(CLEAR + "\r\n".join(out) + "\r\n")
    sys.stdout.flush()


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-3"
    print(f"[base] opening {port} …")
    xb = XBee802(port)

    print("[base] bootstrapping XBee (any state → API @ 115200)…")
    if not xb.bootstrap():
        print("[base] BOOTSTRAP FAILED — check the adapter/port and retry.")
        return
    _, vr = xb.at_query("VR")
    res = xb.configure(PAN, CHANNEL, BASE_MY)
    proto = "802.15.4" if (vr and vr[0] >> 4 == 2) else "?"
    print(f"[base] XBee ready: VR={vr.hex().upper() if vr else '?'} ({proto})  "
          f"config={'OK' if all(res.values()) else res}")
    time.sleep(0.6)

    rovers: dict[int, dict] = {}
    seq = 0
    tx_count = 0
    last_tx = 0.0
    last_render = 0.0
    prev = (time.time(), 0, 0, 0, 0, 0, 0)   # (t, tx_wire, tx_fr, tx_app, rx_wire, rx_fr, rx_app)
    sys.stdout.write(HIDE)
    try:
        while True:
            for rp in xb.rx():
                r = rovers.setdefault(rp.src, {"count": 0})
                if rp.rssi is not None:
                    r["rssi"] = rp.rssi
                r["last"] = time.time()
                r["count"] += 1
                pkt = packet.decode(rp.payload)
                if pkt:
                    r["text"] = f"{pkt.type_name}: {pkt.data.decode('ascii', 'replace').strip()}"

            now = time.time()
            if now - last_tx >= 1.0:
                last_tx = now
                xb.broadcast(packet.encode(packet.RTCM, seq, 0, b"MONMON-RTCM-TEST"))
                info = f"NTRIP:off {_utc()}"
                xb.broadcast(packet.encode(packet.TELEM, seq, 0, info.encode("ascii")))
                seq = (seq + 1) & 0xFF
                tx_count += 1

            if now - last_render >= 0.5:
                last_render = now
                pt, ptw, ptf, pta, prw, prf, pra = prev
                dt = max(1e-3, now - pt)
                d_tw, d_tf, d_ta = xb.tx_wire - ptw, xb.tx_frames - ptf, xb.tx_app - pta
                d_rw, d_rf, d_ra = xb.rx_wire - prw, xb.rx_frames - prf, xb.rx_app - pra
                air_us = (d_tf * TX_FIXED_US + (d_ta + d_tf * OA_OVERHEAD) * BYTE_US
                          + d_rf * RX_FIXED_US + (d_ra + d_rf * OA_OVERHEAD) * BYTE_US)
                bw = {
                    "tx_bps": d_tw / dt, "tx_fps": d_tf / dt,
                    "rx_bps": d_rw / dt, "rx_fps": d_rf / dt,
                    "ser_tx": (d_tw / dt) / SERIAL_BPS * 100,
                    "ser_rx": (d_rw / dt) / SERIAL_BPS * 100,
                    "air": min(100.0, air_us / (dt * 1e6) * 100),
                }
                prev = (now, xb.tx_wire, xb.tx_frames, xb.tx_app,
                        xb.rx_wire, xb.rx_frames, xb.rx_app)
                _render(vr, tx_count, rovers, bw)

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW + RESET + "\n")
        sys.stdout.flush()
        xb.close()
        print("[base] stopped.")


if __name__ == "__main__":
    main()
