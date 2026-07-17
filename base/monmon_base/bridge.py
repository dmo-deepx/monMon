"""monMon base station — the real bridge.

Rover GGA (over the radio) -> NTRIP caster;  caster RTCM3 -> chunked -> broadcast
to all rovers.  Live dashboard shows NTRIP status, per-rover link budget, and
bandwidth budget.

Usage:
    uv run monmon-base [config.yaml] [--caster NAME] [--list]
"""

from __future__ import annotations

import argparse
import glob
import sys
import time

from . import config, packet, rtcm, ui
from .ntrip import NtripClient, make_gga
from .xbee import XBee802

# link budget
NOISE_FLOOR = -100
MARGIN_FULL = 60
# bandwidth budget
SERIAL_BPS = 11520
BYTE_US = 32
OA_OVERHEAD = 20
TX_FIXED_US = 1200
RX_FIXED_US = 2200
# cap RTCM broadcast burst per loop tick (avoid flooding on reconnect backlog)
MAX_FLUSH = 40 * packet.MAX_DATA


def _resolve_port(p: str) -> str:
    if p != "auto":
        return p
    m = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyUSB*")
    if len(m) == 1:
        return m[0]
    raise SystemExit(f"[xbee].port='auto' matched {m or 'nothing'} — set it explicitly")


def _gga_quality(gga: str) -> int:
    parts = gga.split(",")
    try:
        return int(parts[6])
    except (IndexError, ValueError):
        return 0


def _render(cfg, xb, nt, rovers, bw, rtcm_bps, fwd_fps, ref_src, gga_age, scanner):
    now = time.time()
    out = [
        f"{ui.BOLD}{ui.CYAN}monMon base{ui.RESET}  "
        f"PAN {cfg.xbee.pan_id:04X}  CH {cfg.xbee.channel:02X}  "
        f"MY {cfg.xbee.base_addr:04X}   {ui.utc()}",
    ]
    n = cfg.ntrip
    label = f"{ui.BOLD}NTRIP{ui.RESET} {ui.CYAN}[{cfg.caster}]{ui.RESET} {n.host}:{n.port}/{n.mountpoint}"
    if nt.connected:
        out.append(f"{label}  {ui.GREEN}CONNECTED{ui.RESET}  "
                   f"in {rtcm_bps:.0f} B/s  fwd {fwd_fps:.0f} fr/s")
    else:
        err = f" ({nt.last_error})" if nt.last_error else ""
        out.append(f"{label}  {ui.RED}DISCONNECTED{ui.RESET}{ui.DIM}{err}{ui.RESET}")
    src = (f"{n.gga_source} 0x{ref_src:04X}" if n.gga_source == "rover" and ref_src
           else n.gga_source)
    age = f"{gga_age:.0f}s ago" if gga_age is not None else "never"
    out.append(f"{ui.DIM}GGA: src {src}  ·  sent {nt.gga_sent} ({age}){ui.RESET}")
    if scanner.total == 0 and nt.bytes_in > 0:
        head = scanner.head
        hexs = head[:24].hex()
        asc = "".join(chr(c) if 32 <= c < 127 else "." for c in head[:24])
        proto = ("UBX" if head[:2] == b"\xb5\x62" else "RTCM3" if head[:1] == b"\xd3"
                 else "TEXT" if head[:1].isalpha() else "?")
        out.append(f"{ui.RED}RTCM: 0 parsed{ui.RESET} {ui.DIM}(looks like {proto}, "
                   f"bad_crc={scanner.bad_crc}) head={hexs} [{asc}]{ui.RESET}")
    elif scanner.total or nt.connected:
        warn = "" if scanner.has_base_position() else f"  {ui.RED}NO base pos (1005/1006)!{ui.RESET}"
        out.append(f"{ui.DIM}RTCM: {scanner.summary()}  bad_crc={scanner.bad_crc}{ui.RESET}{warn}")
    out.append("")
    out.append(
        f"{ui.BOLD}{'ROVER':<8}{'RSSI':>8}{'MARGIN':>8}  {'LINK':<14}  "
        f"{'RATING':<10}{'FIX':>4}{'PKTS':>7}{'AGE':>6}{ui.RESET}"
    )
    if not rovers:
        out.append(f"{ui.DIM}  waiting for rovers…{ui.RESET}")
    for addr in sorted(rovers):
        r = rovers[addr]
        rssi = r.get("rssi")
        age = now - r.get("last", now)
        age_s = f"{age:4.1f}s"
        if age > 5:
            age_s = f"{ui.RED}{age_s}{ui.RESET}"
        fix = r.get("q", 0)
        if rssi is None:
            out.append(f"0x{addr:04X}{'--':>8}{'--':>8}  {' ' * 14}  "
                       f"{'--':<10}{fix:>4}{r.get('count', 0):>7}{age_s:>6}")
        else:
            dbm = -rssi
            margin = dbm - NOISE_FLOOR
            c = ui.margin_color(margin)
            out.append(
                f"0x{addr:04X}{dbm:>6}dBm{margin:>6}dB  {c}{ui.bar(margin, MARGIN_FULL)}{ui.RESET}  "
                f"{c}{ui.rating(margin):<10}{ui.RESET}{fix:>4}{r.get('count', 0):>7} {age_s}"
            )
    out.append("")
    ac = ui.pct_color(bw["air"])
    sc = ui.pct_color(bw["ser_tx"])
    out.append(
        f"{ui.BOLD}BANDWIDTH{ui.RESET}  "
        f"TX {bw['tx_bps']:>4.0f}B/s {bw['tx_fps']:>2.0f}fr/s   "
        f"RX {bw['rx_bps']:>4.0f}B/s {bw['rx_fps']:>2.0f}fr/s"
    )
    out.append(
        f"           serial TX {sc}{bw['ser_tx']:>2.0f}%{ui.RESET} RX {bw['ser_rx']:>2.0f}%   "
        f"channel ~{ac}{bw['air']:>2.0f}%{ui.RESET} {ac}{ui.pct_bar(bw['air'])}{ui.RESET}"
    )
    out.append("")
    out.append(f"{ui.DIM}Ctrl-C to quit{ui.RESET}")
    sys.stdout.write(ui.CLEAR + "\r\n".join(out) + "\r\n")
    sys.stdout.flush()


def main() -> None:
    ap = argparse.ArgumentParser(prog="monmon-base", description="monMon base station bridge")
    ap.add_argument("config", nargs="?", default="config.yaml", help="YAML config path")
    ap.add_argument("-C", "--caster", help="caster name from config (overrides 'default')")
    ap.add_argument("-l", "--list", action="store_true", help="list configured casters and exit")
    args = ap.parse_args()

    if args.list:
        names, default = config.available_casters(args.config)
        print("casters:" if names else "no casters configured")
        for n in names:
            print(f"  {n}{'   (default)' if n == default else ''}")
        return

    cfg = config.load(args.config, caster=args.caster)
    print(f"[base] caster '{cfg.caster}' → {cfg.ntrip.host}:{cfg.ntrip.port}/{cfg.ntrip.mountpoint}")
    port = _resolve_port(cfg.xbee.port)

    print(f"[base] opening XBee on {port} …")
    xb = XBee802(port)
    if not xb.bootstrap():
        print("[base] XBee bootstrap FAILED — check the adapter/port.")
        return
    res = xb.configure(cfg.xbee.pan_id, cfg.xbee.channel, cfg.xbee.base_addr)
    print(f"[base] XBee configured: {'OK' if all(res.values()) else res}")

    nt = NtripClient(cfg.ntrip)
    nt.start()
    print(f"[base] NTRIP client started → {cfg.ntrip.host}:{cfg.ntrip.port}/{cfg.ntrip.mountpoint}")
    time.sleep(0.5)

    rovers: dict[int, dict] = {}
    ref_gga: str | None = None
    ref_src = 0
    scanner = rtcm.RtcmScanner()
    rtcm_buf = bytearray()
    rtcm_seq = 0
    fwd_frames = 0
    last_gga_sent = 0.0
    gga_sent_at: float | None = None
    last_render = 0.0
    prev = (time.time(), 0, 0, 0, 0, 0, 0)
    prev_rtcm_in = 0
    prev_fwd = 0

    sys.stdout.write(ui.HIDE)
    try:
        while True:
            # ---- inbound from rovers ----
            for rp in xb.rx():
                r = rovers.setdefault(rp.src, {"count": 0})
                if rp.rssi is not None:
                    r["rssi"] = rp.rssi
                r["last"] = time.time()
                r["count"] += 1
                pkt = packet.decode(rp.payload)
                if pkt and pkt.type == packet.NMEA:
                    line = pkt.data.decode("ascii", "replace").strip()
                    if "GGA" in line:
                        r["gga"] = line
                        r["q"] = _gga_quality(line)
                        if cfg.ntrip.primary_rover in (0, rp.src):
                            ref_gga = line
                            ref_src = rp.src

            # ---- RTCM from caster -> broadcast to rovers ----
            while not nt.rtcm.empty():
                chunk = nt.rtcm.get_nowait()
                rtcm_buf += chunk
                scanner.feed(chunk)
            if rtcm_buf:
                take = bytes(rtcm_buf[:MAX_FLUSH])
                del rtcm_buf[:MAX_FLUSH]
                frames, rtcm_seq = packet.chunk_rtcm(take, rtcm_seq)
                for fr in frames:
                    xb.broadcast(fr)
                fwd_frames += len(frames)

            # ---- GGA upstream ----
            now = time.time()
            if now - last_gga_sent >= cfg.ntrip.gga_interval:
                gga = None
                if cfg.ntrip.gga_source == "static":
                    gga = make_gga(cfg.ntrip.static_lat, cfg.ntrip.static_lon, cfg.ntrip.static_alt)
                elif ref_gga:
                    gga = ref_gga
                if gga:
                    nt.send_gga(gga)
                    last_gga_sent = now
                    gga_sent_at = now

            # ---- dashboard ----
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
                rtcm_bps = (nt.bytes_in - prev_rtcm_in) / dt
                fwd_fps = (fwd_frames - prev_fwd) / dt
                prev = (now, xb.tx_wire, xb.tx_frames, xb.tx_app, xb.rx_wire, xb.rx_frames, xb.rx_app)
                prev_rtcm_in = nt.bytes_in
                prev_fwd = fwd_frames
                gga_age = (now - gga_sent_at) if gga_sent_at else None
                _render(cfg, xb, nt, rovers, bw, rtcm_bps, fwd_fps, ref_src, gga_age, scanner)

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(ui.SHOW + ui.RESET + "\n")
        sys.stdout.flush()
        nt.stop()
        xb.close()
        print("[base] stopped.")


if __name__ == "__main__":
    main()
