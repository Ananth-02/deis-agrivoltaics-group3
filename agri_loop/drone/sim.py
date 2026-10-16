"""Simulated Crazyflie and AI deck, for testing C1 without hardware.

SimVehicle implements the same interface as vehicle.CrazyflieVehicle and logs
through the same FlightLogger. FakeAIDeck serves the real CPX wire format on a
local TCP port, with frames cut from a synthetic textured bed at the simulated
drone position, so the recorder, the mission and the analysis run unchanged.

Numbers here (time constants, noise, battery sag) are placeholders for the
software test only. They are not measurements of the Crazyflie.
"""
from __future__ import annotations

import math
import random
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from .cpx_stream import FMT_RAW, encode_packets
from .flightlog import FlightLogger, SUP_CAN_BE_ARMED, SUP_CAN_FLY, SUP_IS_ARMED, SUP_IS_FLYING


class SimVehicle:
    def __init__(self, logger: Optional[FlightLogger] = None, seed: int = 0, vbat0: float = 4.15,
                 vel_noise: float = 0.01, link_fail_at_s: Optional[float] = None) -> None:
        self.logger = logger
        self.rng = random.Random(seed)
        self.vbat0 = vbat0
        self.vel_noise = vel_noise
        self.link_fail_at_s = link_fail_at_s
        self.s = dict(x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        self.cmd = (0.0, 0.0, 0.0, 0.0)  # vx_body, vy_body, yawrate, z
        self.flying = False
        self.armed = False
        self.t_start = time.time()
        self.t_air = 0.0
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="sim-vehicle", daemon=True)

    def open(self) -> "SimVehicle":
        self._thread.start()
        time.sleep(0.15)
        return self

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def start_logging(self) -> None:
        pass

    def decks(self) -> Dict[str, bool]:
        return {"bcFlow2": True, "bcFlow": False, "bcAI": True, "bcLighthouse4": False, "bcLoco": False, "bcMultiranger": False}

    def param(self, name: str) -> Optional[str]:
        return None

    missing_vars: list = []

    def link_ok(self) -> bool:
        return self.link_fail_at_s is None or time.time() - self.t_start < self.link_fail_at_s

    def latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def arm(self, on: bool) -> None:
        self.armed = on

    def hover(self, vx: float, vy: float, yawrate: float, z: float) -> None:
        if self.armed:
            self.cmd = (vx, vy, yawrate, z)
            self.flying = True

    def stop_motors(self) -> None:
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.flying = False

    def emergency_stop(self) -> None:
        self.stop_motors()
        self.armed = False

    def _vbat(self) -> float:
        sag = 0.25 if self.flying else 0.0
        return self.vbat0 - 0.002 * self.t_air - sag + self.rng.gauss(0, 0.005)

    def _loop(self) -> None:
        dt = 0.01
        tau_v, tau_z = 0.25, 0.4
        n = 0
        t_next = time.time()
        while not self._stop.is_set():
            s = self.s
            vx_b, vy_b, yawrate, z_cmd = self.cmd
            yaw = math.radians(s["yaw"])
            vx_w = math.cos(yaw) * vx_b - math.sin(yaw) * vy_b
            vy_w = math.sin(yaw) * vx_b + math.cos(yaw) * vy_b
            if self.flying:
                ax = (vx_w - s["vx"]) / tau_v
                ay = (vy_w - s["vy"]) / tau_v
                s["vx"] += ax * dt + self.rng.gauss(0, self.vel_noise) * math.sqrt(dt)
                s["vy"] += ay * dt + self.rng.gauss(0, self.vel_noise) * math.sqrt(dt)
                s["vz"] = (z_cmd - s["z"]) / tau_z
                s["yaw"] += -yawrate * dt + self.rng.gauss(0, 0.2) * math.sqrt(dt)
                s["pitch"] = math.degrees(math.atan2(ax, 9.81))
                s["roll"] = -math.degrees(math.atan2(ay, 9.81))
                self.t_air += dt
            else:
                s["vx"] = s["vy"] = 0.0
                s["vz"] = -s["z"] / 0.1 if s["z"] > 0 else 0.0
                s["roll"] = s["pitch"] = 0.0
            s["x"] += s["vx"] * dt
            s["y"] += s["vy"] * dt
            s["z"] = max(0.0, s["z"] + s["vz"] * dt)
            sup = SUP_CAN_BE_ARMED | SUP_CAN_FLY
            if self.armed:
                sup |= SUP_IS_ARMED
            if self.flying:
                sup |= SUP_IS_FLYING
            slow = dict(roll=s["roll"], pitch=s["pitch"], yaw=s["yaw"], vbat=self._vbat(),
                        battery_level=max(0, min(100, int((self._vbat() - 3.0) / 1.2 * 100))),
                        pm_state=0, sup_info=sup, zrange_mm=int(s["z"] * 1000))
            fast = {k: s[k] for k in ("x", "y", "z", "vx", "vy", "vz")}
            now = time.time()
            with self._lock:
                self._latest.update(slow)
                self._latest.update(fast)
                self._latest["t"] = now
            if self.logger:
                if n % 10 == 0:
                    self.logger.update_slow(slow)
                if n % 5 == 0:
                    self.logger.add_fast(now, int((now - self.t_start) * 1000), fast)
            n += 1
            t_next += dt
            time.sleep(max(0.0, t_next - time.time()))

    def pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._latest.get("x", 0.0), self._latest.get("y", 0.0), self._latest.get("z", 0.0)


def synthetic_bed(w_px: int = 2400, h_px: int = 1400, seed: int = 1) -> np.ndarray:
    """Grey textured floor with a grid, so consecutive frames overlap with features."""
    rng = np.random.default_rng(seed)
    img = rng.integers(60, 120, size=(h_px // 8, w_px // 8), dtype=np.uint8)
    img = np.kron(img, np.ones((8, 8), dtype=np.uint8))
    img[::100, :] = 220
    img[:, ::100] = 220
    for _ in range(80):
        x, y = int(rng.integers(0, w_px - 40)), int(rng.integers(0, h_px - 40))
        img[y:y + 30, x:x + 30] = int(rng.integers(0, 255))
    return img


class FakeAIDeck:
    """TCP server speaking the wifi-img-streamer CPX format."""

    def __init__(self, pose: Callable[[], Tuple[float, float, float]], fps: float = 15.0,
                 width: int = 324, height: int = 244, port: int = 0, px_per_m: float = 600.0) -> None:
        self.pose, self.fps = pose, fps
        self.width, self.height = width, height
        self.px_per_m = px_per_m
        self.bed = synthetic_bed()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", port))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="fake-aideck", daemon=True)
        self.sent = 0

    def start(self) -> "FakeAIDeck":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        self._thread.join(timeout=2)

    def frame(self) -> np.ndarray:
        x, y, _z = self.pose()
        h, w = self.bed.shape
        cx = int(400 + x * self.px_per_m)
        cy = int(h / 2 - y * self.px_per_m)
        x0 = max(0, min(w - self.width, cx - self.width // 2))
        y0 = max(0, min(h - self.height, cy - self.height // 2))
        return self.bed[y0:y0 + self.height, x0:x0 + self.width].copy()

    def _serve(self) -> None:
        self._srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except (socket.timeout, OSError):
                continue
            t_next = time.time()
            try:
                while not self._stop.is_set():
                    conn.sendall(encode_packets(self.frame(), FMT_RAW))
                    self.sent += 1
                    t_next += 1.0 / self.fps
                    time.sleep(max(0.0, t_next - time.time()))
            except OSError:
                pass
            finally:
                conn.close()
