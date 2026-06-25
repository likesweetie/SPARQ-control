from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque


def estimate_classical_can_bits(
    dlc: int,
    *,
    is_eff: bool = False,
    is_rtr: bool = False,
    stuff_factor: float = 1.15,
    include_intermission: bool = True,
) -> int:
    dlc = max(0, min(int(dlc), 8))
    data_bits = 0 if is_rtr else 8 * dlc
    base_bits = (67 if is_eff else 47) + data_bits
    if not include_intermission:
        base_bits -= 3
    return max(0, int(math.ceil(base_bits * stuff_factor)))


@dataclass
class BusLoadWindow:
    window_s: float = 1.0
    bitrate: float = 1_000_000.0
    events: Deque[tuple[float, int, str]] = field(default_factory=deque)

    def add(self, timestamp: float, bits: int, direction: str) -> None:
        self.events.append((timestamp, max(0, int(bits)), direction))
        self.prune(timestamp)

    def prune(self, now: float) -> None:
        cutoff = now - self.window_s
        while self.events and self.events[0][0] < cutoff:
            self.events.popleft()

    def snapshot(self, now: float) -> dict:
        self.prune(now)
        rx_frames = tx_frames = rx_bits = tx_bits = 0

        for _t, bits, direction in self.events:
            if direction == "rx":
                rx_frames += 1
                rx_bits += bits
            elif direction == "tx":
                tx_frames += 1
                tx_bits += bits

        duration = max(self.window_s, 1e-9)
        estimated_bps = (rx_bits + tx_bits) / duration
        return {
            "rx_frames": rx_frames,
            "tx_frames": tx_frames,
            "rx_rate": rx_frames / duration,
            "tx_rate": tx_frames / duration,
            "rx_bits": rx_bits,
            "tx_bits": tx_bits,
            "estimated_bps": estimated_bps,
            "estimated_kbps": estimated_bps / 1000.0,
            "load_percent": (estimated_bps / max(self.bitrate, 1e-9)) * 100.0,
            "bitrate": self.bitrate,
            "window_s": self.window_s,
        }
