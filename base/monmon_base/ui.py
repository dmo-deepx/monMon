"""Shared terminal-dashboard helpers (ANSI colours + bars)."""

from __future__ import annotations

import time

CLEAR = "\033[2J\033[H"
HIDE, SHOW = "\033[?25l", "\033[?25h"
RESET, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
GREEN, YELLOW, RED, CYAN = "\033[92m", "\033[93m", "\033[91m", "\033[96m"


def utc() -> str:
    return time.strftime("%H:%M:%SZ", time.gmtime())


def bar(value: float, full: float, width: int = 14) -> str:
    filled = max(0, min(width, round(value / full * width)))
    return "█" * filled + "░" * (width - filled)


def pct_bar(pct: float, width: int = 14) -> str:
    return bar(pct, 100, width)


def margin_color(m: float) -> str:
    return GREEN if m >= 40 else YELLOW if m >= 22 else RED


def pct_color(p: float) -> str:
    return RED if p >= 80 else YELLOW if p >= 50 else GREEN


def rating(m: float) -> str:
    if m >= 40:
        return "excellent"
    if m >= 30:
        return "good"
    if m >= 22:
        return "fair"
    if m >= 12:
        return "marginal"
    return "poor"
