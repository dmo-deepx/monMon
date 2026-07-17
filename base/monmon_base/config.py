"""Load the base station config from a YAML file (matches the survey tooling).

The `ntrip` section holds common fields plus a `casters:` map of named profiles
that override them; `default:` picks one, and `--caster NAME` overrides that.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
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
    caster: str = ""                       # selected caster name
    casters: list[str] = field(default_factory=list)


def _coerce_int(v):
    # allow "0x3332" as a quoted string too (PyYAML also parses bare 0x.. as int)
    if isinstance(v, str) and v.lower().startswith("0x"):
        return int(v, 16)
    return v


def _build(cls, d: dict | None):
    known = {f.name for f in fields(cls)}
    kw = {k: _coerce_int(v) for k, v in (d or {}).items() if k in known}
    return cls(**kw)


def _read(path: str) -> dict:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"{path} not found — copy config.example.yaml to {path} and fill in your caster"
        )
    with p.open() as f:
        return yaml.safe_load(f) or {}


def available_casters(path: str = "config.yaml") -> tuple[list[str], str | None]:
    nt = _read(path).get("ntrip", {}) or {}
    return list((nt.get("casters") or {}).keys()), nt.get("default")


def load(path: str = "config.yaml", caster: str | None = None) -> Config:
    d = _read(path)
    nt = d.get("ntrip", {}) or {}
    casters = nt.get("casters") or {}
    common = {k: v for k, v in nt.items() if k not in ("casters", "default")}

    if casters:
        name = caster or nt.get("default")
        if name is None:
            if len(casters) == 1:
                name = next(iter(casters))
            else:
                raise SystemExit(
                    f"multiple casters configured — choose one with --caster "
                    f"(available: {', '.join(casters)})"
                )
        if name not in casters:
            raise SystemExit(
                f"unknown caster '{name}' (available: {', '.join(casters)})"
            )
        merged = {**common, **(casters[name] or {})}
    else:
        # flat ntrip section (no casters map)
        name = "ntrip"
        merged = common

    return Config(
        xbee=_build(XBeeCfg, d.get("xbee")),
        ntrip=_build(NtripCfg, merged),
        caster=name,
        casters=list(casters),
    )
