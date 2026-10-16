"""C1 mission: take off, fly N straight passes at constant speed, land.

Passes run along the x axis of the Flow deck frame (x forward at power-up,
origin where the drone took off). With `alternate` the passes go out and back
on the same line, so the room only needs to be pass_length_m long.

Each pass: ramp up at accel_m_s2, hold speed_m_s, brake so it stops on the
end of the line, hold position for pause_s. Lateral position and heading are
held with small proportional corrections, so the pass stays straight.

Safety (checked on every 20 Hz tick, any one lands or cuts the motors):
    link lost, battery below land_vbat for 1 s or pm.state LOW_POWER,
    tilt above max_tilt_deg or supervisor tumbled/crashed (motors off),
    position outside the geofence, Ctrl+C (lands).
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, Optional

from ..link.eventlog import EventLog
from .flightlog import PM_LOW_POWER, SUP_IS_CRASHED, SUP_IS_TUMBLED


@dataclass
class C1Config:
    n_passes: int = 10
    pass_length_m: float = 1.5
    speed_m_s: float = 0.20
    height_m: float = 0.60
    accel_m_s2: float = 0.40
    pause_s: float = 2.0
    takeoff_s: float = 2.5
    settle_s: float = 2.0
    land_s: float = 2.5
    rate_hz: float = 20.0
    alternate: bool = True
    lateral_gain: float = 1.0      # (m/s) per m of lateral error
    hold_gain: float = 1.0         # (m/s) per m of along-track error during pauses
    yaw_gain: float = 1.0          # (deg/s) per deg of heading error
    max_correction_m_s: float = 0.10
    geofence_margin_m: float = 0.5
    min_start_vbat: float = 3.8    # refuse take-off below this (resting voltage)
    land_vbat: float = 3.3         # land if below this for 1 s in flight
    max_tilt_deg: float = 45.0
    pass_timeout_factor: float = 2.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MissionAbort(Exception):
    def __init__(self, reason: str, cut_motors: bool = False) -> None:
        super().__init__(reason)
        self.reason = reason
        self.cut_motors = cut_motors


def _clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class Mission:
    def __init__(self, vehicle: Any, cfg: C1Config, events: EventLog,
                 clock: Callable[[], float] = time.time, sleep: Callable[[float], None] = time.sleep) -> None:
        self.v = vehicle
        self.cfg = cfg
        self.ev = events
        self.clock, self.sleep = clock, sleep
        self.home = (0.0, 0.0)
        self._low_since: Optional[float] = None
        self.passes_done = 0
        self.airborne = False
        self.in_pass = False   # read by the C4 beacon sender (mission type)

    # ---- helpers ----
    def _state(self) -> Dict[str, Any]:
        s = self.v.latest()
        if not s or "x" not in s:
            raise MissionAbort("no state estimate from the drone")
        return s

    def _check(self, s: Dict[str, Any], critical_only: bool = False) -> None:
        c = self.cfg
        sup = int(s.get("sup_info", 0) or 0)
        if sup & (SUP_IS_TUMBLED | SUP_IS_CRASHED):
            raise MissionAbort("supervisor reports tumbled or crashed", cut_motors=True)
        if abs(s.get("roll", 0.0)) > c.max_tilt_deg or abs(s.get("pitch", 0.0)) > c.max_tilt_deg:
            raise MissionAbort("tilt limit exceeded", cut_motors=True)
        if critical_only:
            return  # while landing only the motor-cut conditions apply
        if not self.v.link_ok():
            raise MissionAbort("radio link lost")
        if self.airborne:
            vb = s.get("vbat")
            now = self.clock()
            if int(s.get("pm_state", 0) or 0) == PM_LOW_POWER:
                raise MissionAbort("battery low (pm.state)")
            if vb is not None and vb < c.land_vbat:
                self._low_since = self._low_since or now
                if now - self._low_since > 1.0:
                    raise MissionAbort(f"battery below {c.land_vbat} V")
            else:
                self._low_since = None
        hx, hy = self.home
        m = c.geofence_margin_m
        x, y, z = s["x"] - hx, s["y"] - hy, s["z"]
        if x < -m or x > c.pass_length_m + m or abs(y) > m or z > c.height_m + m:
            raise MissionAbort(f"geofence: x={x:.2f} y={y:.2f} z={z:.2f}")

    def _send_world(self, s: Dict[str, Any], vx_w: float, vy_w: float, z: float) -> None:
        """World-frame velocity -> body-frame hover setpoint, with heading held at 0."""
        yaw_deg = s.get("yaw", 0.0) or 0.0
        yaw = math.radians(yaw_deg)
        vx_b = math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
        vy_b = -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w
        # positive yawrate in a hover setpoint turns clockwise (cflib MotionCommander convention)
        yawrate = _clip(self.cfg.yaw_gain * yaw_deg, 30.0)
        self.v.hover(vx_b, vy_b, yawrate, z)

    def _lateral(self, s: Dict[str, Any]) -> float:
        return _clip(-self.cfg.lateral_gain * (s["y"] - self.home[1]), self.cfg.max_correction_m_s)

    def _tick(self, t_next: float) -> float:
        t_next += 1.0 / self.cfg.rate_hz
        dt = t_next - self.clock()
        if dt > 0:
            self.sleep(dt)
        return t_next

    def _hold(self, x_target: float, z: float, duration: float, z_from: Optional[float] = None,
              critical_only: bool = False) -> None:
        t0 = self.clock()
        t_next = t0
        while self.clock() - t0 < duration:
            s = self._state()
            self._check(s, critical_only)
            zz = z if z_from is None else z_from + (z - z_from) * min(1.0, (self.clock() - t0) / duration)
            vx = _clip(self.cfg.hold_gain * (x_target - s["x"]), self.cfg.max_correction_m_s)
            self._send_world(s, vx, self._lateral(s), zz)
            t_next = self._tick(t_next)

    # ---- phases ----
    def takeoff(self) -> None:
        c = self.cfg
        s = self.v.latest()
        vb = s.get("vbat")
        if vb is not None and vb < c.min_start_vbat:
            raise MissionAbort(f"battery {vb:.2f} V below {c.min_start_vbat} V, not taking off")
        self.home = (s.get("x", 0.0), s.get("y", 0.0))
        self.v.arm(True)
        self.ev.add("armed", self.clock())
        self.sleep(1.0)
        self.ev.add("takeoff", self.clock(), height_m=c.height_m)
        self.airborne = True
        self._hold(self.home[0], c.height_m, c.takeoff_s, z_from=0.05)
        self._hold(self.home[0], c.height_m, c.settle_s)
        self.ev.add("hover_ok", self.clock(), **{k: self.v.latest().get(k) for k in ("x", "y", "z", "vbat")})

    def fly_pass(self, i: int) -> None:
        c = self.cfg
        direction = -1.0 if (c.alternate and i % 2 == 1) else 1.0
        x0 = self.home[0] if direction > 0 else self.home[0] + c.pass_length_m
        x1 = x0 + direction * c.pass_length_m
        if not c.alternate:
            x0, x1, direction = self.home[0], self.home[0] + c.pass_length_m, 1.0
            self._hold(x0, c.height_m, max(c.pause_s, c.pass_length_m / max(c.max_correction_m_s, 1e-3)))
        timeout = c.pass_timeout_factor * c.pass_length_m / c.speed_m_s + 3.0
        t0 = self.clock()
        self.ev.add("pass_start", t0, i=i, direction=int(direction), x0=x0, x1=x1, speed_cmd=c.speed_m_s)
        t_next = t0
        reason = "reached"
        self.in_pass = True
        while True:
            s = self._state()
            self._check(s)
            remaining = direction * (x1 - s["x"])
            if remaining <= 0.01:
                break
            if self.clock() - t0 > timeout:
                reason = "timeout"
                break
            v_up = c.accel_m_s2 * (self.clock() - t0) + 0.02
            v_brake = math.sqrt(2.0 * c.accel_m_s2 * max(remaining, 0.0))
            vx = direction * min(c.speed_m_s, v_up, v_brake)
            self._send_world(s, vx, self._lateral(s), c.height_m)
            t_next = self._tick(t_next)
        self.in_pass = False
        self.ev.add("pass_end", self.clock(), i=i, reason=reason)
        self.passes_done += 1
        self._hold(x1, c.height_m, c.pause_s)

    def land(self) -> None:
        c = self.cfg
        s = self.v.latest()
        z0 = s.get("z", c.height_m) or c.height_m
        self.ev.add("land", self.clock())
        self._hold(s.get("x", self.home[0]), 0.05, c.land_s, z_from=z0, critical_only=True)
        self.v.stop_motors()
        self.airborne = False
        self.v.arm(False)
        self.ev.add("landed", self.clock())

    def run(self) -> Dict[str, Any]:
        self.ev.add("mission_start", self.clock(), config=self.cfg.to_dict())
        result = {"passes_done": 0, "aborted": False, "reason": ""}
        try:
            self.takeoff()
            for i in range(self.cfg.n_passes):
                self.fly_pass(i)
            self.land()
        except MissionAbort as e:
            result.update(aborted=True, reason=e.reason)
            self.ev.add("abort", self.clock(), reason=e.reason, cut_motors=e.cut_motors)
            self._safe_stop(e.cut_motors)
        except KeyboardInterrupt:
            result.update(aborted=True, reason="operator Ctrl+C")
            self.ev.add("abort", self.clock(), reason="operator Ctrl+C", cut_motors=False)
            self._safe_stop(False)
        result["passes_done"] = self.passes_done
        self.ev.add("mission_end", self.clock(), **result)
        return result

    def _safe_stop(self, cut_motors: bool) -> None:
        if cut_motors or not self.airborne:
            self.v.emergency_stop() if cut_motors else self.v.stop_motors()
            self.airborne = False
            return
        try:
            self.land()
        except MissionAbort as e:
            self.ev.add("abort", self.clock(), reason=e.reason, cut_motors=True)
            self.v.emergency_stop()
            self.airborne = False
        except Exception:  # noqa: BLE001 - last resort
            self.v.emergency_stop()
            self.airborne = False
