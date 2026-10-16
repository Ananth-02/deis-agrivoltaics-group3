"""Flight and battery log: one CSV row per state sample, host epoch time.

The position block (20 Hz) drives the rows; attitude and battery come from a
second, slower block and are carried forward to the next position row, so every
row is complete and the battery log has no separate timeline to align.
"""
from __future__ import annotations

import csv
import threading
from typing import Any, Dict, List, Optional

COLS = ["t", "t_cf_ms", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
        "vbat", "battery_level", "pm_state", "sup_info", "zrange_mm", "lh_status", "lh_active"]

# lighthouse.status: 0 no base station received, 1 received but geometry or calibration missing,
# 2 base station data goes to the state estimator
LH_TO_ESTIMATOR = 2

# pm.state values in the Crazyflie firmware
PM_BATTERY, PM_CHARGING, PM_CHARGED, PM_LOW_POWER, PM_SHUTDOWN = 0, 1, 2, 3, 4

# supervisor.info bits (crazyflie-firmware, src/modules/src/supervisor.c)
SUP_CAN_BE_ARMED = 1 << 0
SUP_IS_ARMED = 1 << 1
SUP_CAN_FLY = 1 << 3
SUP_IS_FLYING = 1 << 4
SUP_IS_TUMBLED = 1 << 5
SUP_IS_LOCKED = 1 << 6
SUP_IS_CRASHED = 1 << 7


class FlightLogger:
    def __init__(self, path: str) -> None:
        self.path = path
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(COLS)
        self._lock = threading.Lock()
        self._slow: Dict[str, Any] = {}
        self.latest: Dict[str, Any] = {}
        self.rows = 0

    def update_slow(self, values: Dict[str, Any]) -> None:
        with self._lock:
            self._slow.update(values)

    def add_fast(self, t: float, t_cf_ms: Optional[int], values: Dict[str, Any]) -> None:
        with self._lock:
            row = dict(self._slow)
            row.update(values)
            row["t"] = t
            row["t_cf_ms"] = t_cf_ms if t_cf_ms is not None else ""
            self.latest = row
            self._w.writerow([_fmt(row.get(c, "")) for c in COLS])
            self.rows += 1
            if self.rows % 20 == 0:
                self._f.flush()

    def close(self) -> None:
        with self._lock:
            self._f.flush()
            self._f.close()


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.6f}" if abs(v) > 1e6 else f"{v:.4f}"
    return str(v)


def load_flight_log(path: str) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = {c: [] for c in COLS}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            for c in COLS:
                v = r.get(c, "")
                out[c].append(float(v) if v not in ("", None) else float("nan"))
    return out
