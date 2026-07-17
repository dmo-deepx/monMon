"""Load the base station config from a YAML file (matches the survey tooling)."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path

import yaml


@dataclass
class XBeeCfg:
    port: str = "auto"
    pan_id: int = 0x3332
    channel: int = 0x0C
    base_addr: int = 0x0000


@dataclass
class NtripCfg:
    host: str = ""
    port: int = 2101
    mountpoint: str = ""
    username: str = ""
    password: str = ""
    version: int = 2
    https: bool = False
    gga_source: str = "rover"      # "rover" | "static"
    gga_interval: int = 10
    primary_rover: int = 0x0000
    static_lat: float = 0.0
    static_lon: float = 0.0
    static_alt: float = 0.0


@dataclass
class Config:
    xbee: XBeeCfg
    ntrip: NtripCfg


def _coerce_int(v):
    # allow "0x3332" as a quoted string too (PyYAML also parses bare 0x.. as int)
    if isinstance(v, str) and v.lower().startswith("0x"):
        return int(v, 16)
    return v


def _build(cls, d: dict | None):
    known = {f.name for f in fields(cls)}
    kw = {k: _coerce_int(v) for k, v in (d or {}).items() if k in known}
    return cls(**kw)


def load(path: str = "config.yaml") -> Config:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"{path} not found — copy config.example.yaml to {path} and fill in your caster"
        )
    with p.open() as f:
        d = yaml.safe_load(f) or {}
    return Config(_build(XBeeCfg, d.get("xbee")), _build(NtripCfg, d.get("ntrip")))
