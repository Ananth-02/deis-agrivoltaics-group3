"""Crazyflie 2.1 Brushless over Crazyradio, wrapped in the small interface the mission uses.

The same interface is implemented by `sim.SimVehicle`, so the mission, logging
and analysis are exercised without hardware. Works with cflib 0.1.17 (in the
venv today) and newer: arming uses the cflib call when it exists and otherwise
sends the legacy platform-channel request, which the firmware still accepts.
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Any, Dict, List, Optional

from .flightlog import FlightLogger

FAST_VARS = ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
             "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"]
SLOW_VARS = ["stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw", "pm.vbat",
             "pm.batteryLevel", "pm.state", "supervisor.info", "range.zrange"]
_KEY = {"stateEstimate.x": "x", "stateEstimate.y": "y", "stateEstimate.z": "z",
        "stateEstimate.vx": "vx", "stateEstimate.vy": "vy", "stateEstimate.vz": "vz",
        "stabilizer.roll": "roll", "stabilizer.pitch": "pitch", "stabilizer.yaw": "yaw",
        "pm.vbat": "vbat", "pm.batteryLevel": "battery_level", "pm.state": "pm_state",
        "supervisor.info": "sup_info", "range.zrange": "zrange_mm"}

PLATFORM_PORT = 0x0D
PLATFORM_COMMAND = 0
PLATFORM_REQUEST_ARMING = 1


def _toc_has(toc: Any, name: str) -> bool:
    group, var = name.split(".")
    try:
        return toc.toc[group][var] is not None
    except KeyError:
        return False


class CrazyflieVehicle:
    def __init__(self, uri: str, logger: Optional[FlightLogger] = None,
                 fast_ms: int = 50, slow_ms: int = 100) -> None:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self.uri = uri
        self.logger = logger
        self.fast_ms, self.slow_ms = fast_ms, slow_ms
        self.cf = Crazyflie(rw_cache="./cache")
        self.scf = SyncCrazyflie(uri, cf=self.cf)
        self._link_lost = threading.Event()
        self.link_error = ""
        self.missing_vars: List[str] = []
        self._latest: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._confs: List[Any] = []

    # ---- connection ----
    def open(self) -> "CrazyflieVehicle":
        self.cf.connection_lost.add_callback(self._on_lost)
        self.scf.open_link()
        return self

    def close(self) -> None:
        for c in self._confs:
            try:
                c.stop()
            except Exception:  # noqa: BLE001
                pass
        self.scf.close_link()

    def _on_lost(self, uri: str, msg: str) -> None:
        self.link_error = msg
        self._link_lost.set()

    def link_ok(self) -> bool:
        return not self._link_lost.is_set()

    # ---- information ----
    def param(self, name: str) -> Optional[str]:
        try:
            return self.cf.param.get_value(name, timeout=5)
        except Exception:  # noqa: BLE001 - missing parameter means "not present"
            return None

    def decks(self) -> Dict[str, bool]:
        names = ["bcFlow2", "bcFlow", "bcAI", "bcLighthouse4", "bcLoco", "bcMultiranger"]
        return {n: self.param("deck." + n) == "1" for n in names}

    def start_logging(self) -> None:
        from cflib.crazyflie.log import LogConfig

        toc = self.cf.log.toc
        fast = [v for v in FAST_VARS if _toc_has(toc, v)]
        slow = [v for v in SLOW_VARS if _toc_has(toc, v)]
        self.missing_vars = [v for v in FAST_VARS + SLOW_VARS if v not in fast + slow]
        cf_fast = LogConfig(name="c1_fast", period_in_ms=self.fast_ms)
        for v in fast:
            cf_fast.add_variable(v, "float")
        cf_slow = LogConfig(name="c1_slow", period_in_ms=self.slow_ms)
        for v in slow:
            cf_slow.add_variable(v)
        for c, cb in ((cf_slow, self._on_slow), (cf_fast, self._on_fast)):
            self.cf.log.add_config(c)
            c.data_received_cb.add_callback(cb)
            c.start()
            self._confs.append(c)

    def _on_fast(self, ts: int, data: Dict[str, Any], _conf: Any) -> None:
        vals = {_KEY[k]: v for k, v in data.items()}
        t = time.time()
        with self._lock:
            self._latest.update(vals)
            self._latest["t"] = t
        if self.logger:
            self.logger.add_fast(t, ts, vals)

    def _on_slow(self, ts: int, data: Dict[str, Any], _conf: Any) -> None:
        vals = {_KEY[k]: v for k, v in data.items()}
        with self._lock:
            self._latest.update(vals)
        if self.logger:
            self.logger.update_slow(vals)

    def latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    # ---- commands ----
    def arm(self, on: bool) -> None:
        for owner in (getattr(self.cf, "supervisor", None), self.cf.platform):
            fn = getattr(owner, "send_arming_request", None)
            if fn is not None:
                fn(on)
                return
        from cflib.crtp.crtpstack import CRTPPacket

        pk = CRTPPacket()
        pk.set_header(PLATFORM_PORT, PLATFORM_COMMAND)
        pk.data = struct.pack("<BB", PLATFORM_REQUEST_ARMING, 1 if on else 0)
        self.cf.send_packet(pk)

    def hover(self, vx: float, vy: float, yawrate: float, z: float) -> None:
        self.cf.commander.send_hover_setpoint(vx, vy, yawrate, z)

    def stop_motors(self) -> None:
        self.cf.commander.send_stop_setpoint()
        notify = getattr(self.cf.commander, "send_notify_setpoint_stop", None)
        if notify is not None:
            notify()

    def emergency_stop(self) -> None:
        self.stop_motors()
        fn = getattr(getattr(self.cf, "supervisor", None), "send_emergency_stop", None) \
            or getattr(getattr(self.cf, "localization", None), "send_emergency_stop", None)
        if fn is not None:
            fn()
