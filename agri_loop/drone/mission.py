"""C1 mission: take off, fly N straight passes at constant speed, land.

Passes run along the x axis of the Flow deck frame (x forward at power-up,
origin where the drone took off), or along -x with direction = -1. With `alternate` the passes go out and back
on the same line, so the room only needs to be pass_length_m long.

Each pass follows a trapezoid speed profile: ramp up at accel_m_s2, hold
speed_m_s, brake so it stops on the end of the line, then hold for pause_s.
Two ways to fly it (pass_control):
    "position": a position target slides along the line on that profile and the
                drone's position controller tracks it. Needs absolute positioning
                (Lighthouse) and gives the steadiest speed.
    "velocity": velocity (hover) setpoints on the same profile, with small
                proportional corrections for sideways drift and heading. Works
                with the Flow deck, which has no absolute position.

Safety (checked on every 20 Hz tick, any one lands or cuts the motors):
    link lost, battery below land_vbat for 1 s or pm.state LOW_POWER,
    tilt above max_tilt_deg or supervisor tumbled/crashed (motors off),
    position outside the geofence, Ctrl+C (lands).
"""
from __future__ import annotations

import math
import signal
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
    accel_m_s2: float = 0.20
    pause_s: float = 2.0
    takeoff_s: float = 2.5
    settle_s: float = 2.0
    land_s: float = 2.5
    rate_hz: float = 20.0
    alternate: bool = True
    direction: int = 1             # 1 = passes go along +x from home, -1 = along -x
    lateral_gain: float = 1.0      # (m/s) per m of lateral error
    hold_gain: float = 1.0         # (m/s) per m of along-track error during pauses
    yaw_gain: float = 1.0          # (deg/s) per deg of heading error
    max_correction_m_s: float = 0.10
    geofence_margin_m: float = 0.5
    max_z_m: float = 1.3           # absolute z limit of the geofence, whatever the flight height
    min_start_vbat: float = 3.8    # refuse take-off below this (resting voltage)
    land_vbat: float = 3.3         # land if below this for 1 s in flight
    max_tilt_deg: float = 45.0
    pass_timeout_factor: float = 2.0
    pass_control: str = "velocity"   # "velocity" or "position" (see module docstring)

    def __post_init__(self) -> None:
        if self.direction not in (1, -1):
            raise ValueError(f"direction must be 1 or -1, not {self.direction!r}")
        if not self.geofence_margin_m > 0:
            raise ValueError("the geofence margin must be above 0")
        if not self.height_m < self.max_z_m:
            raise ValueError(f"height {self.height_m} m must be below the {self.max_z_m} m z limit")
        if not self.pass_length_m > 0:
            raise ValueError(f"pass length must be above 0 m, not {self.pass_length_m!r}")
        if not self.speed_m_s > 0 or not self.accel_m_s2 > 0:
            raise ValueError("speed and acceleration must be above 0")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MissionAbort(Exception):
    def __init__(self, reason: str, cut_motors: bool = False) -> None:
        super().__init__(reason)
        self.reason = reason
        self.cut_motors = cut_motors


def _clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def trapezoid(t: float, length: float, speed: float, accel: float):
    """Distance along the pass at time t for a trapezoid (or, if short, triangle) speed profile.

    Returns (distance, total_time)."""
    t_acc = speed / accel
    d_acc = 0.5 * accel * t_acc ** 2
    if 2 * d_acc > length:  # never reaches cruise speed
        t_acc = math.sqrt(length / accel)
        speed = accel * t_acc
        d_acc = 0.5 * length
    t_cruise = (length - 2 * d_acc) / speed
    total = 2 * t_acc + t_cruise
    if t <= 0:
        return 0.0, total
    if t < t_acc:
        return 0.5 * accel * t * t, total
    if t < t_acc + t_cruise:
        return d_acc + speed * (t - t_acc), total
    if t < total:
        left = total - t
        return length - 0.5 * accel * left * left, total
    return length, total


class Mission:
    def __init__(self, vehicle: Any, cfg: C1Config, events: EventLog,
                 clock: Callable[[], float] = time.time, sleep: Callable[[float], None] = time.sleep) -> None:
        self.v = vehicle
        self.cfg = cfg
        self.ev = events
        self.clock, self.sleep = clock, sleep
        self.home = (0.0, 0.0)
        self.ground_z = 0.0    # z the estimator reads on the ground at take-off (the floor need not read 0)
        self._low_since: Optional[float] = None
        self.passes_done = 0
        self.airborne = False
        self.in_pass = False   # read by the C4 beacon sender (mission type)
        self.pass_index: Optional[int] = None   # index of the running (or last) pass
        self.pass_direction = 0                 # +1 / -1 along x of the running (or last) pass
        self._stop_requested = False
        self._stopping = False   # the abort handler is running: further signals are ignored

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
        x, y, z = self._dir() * (s["x"] - hx), s["y"] - hy, s["z"]   # x: distance along the pass direction
        if x < -m or x > c.pass_length_m + m or abs(y) > m or z > min(c.height_m + m, c.max_z_m):
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

    def _dir(self) -> float:
        return -1.0 if self.cfg.direction < 0 else 1.0

    def _position_mode(self) -> bool:
        return self.cfg.pass_control == "position"

    def _send_pos(self, x: float, z: float) -> None:
        """Absolute position target on the pass line (y of the take-off point), heading 0."""
        self.v.position(x, self.home[1], z, 0.0)

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
            if self._position_mode():
                self._send_pos(x_target, zz)
            else:
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
        self.ground_z = float(s.get("z", 0.0) or 0.0)
        if abs(self.ground_z) > 0.10:
            raise MissionAbort(f"the drone reads z = {self.ground_z:.2f} m on the ground, not about 0: wait for "
                               "the Lighthouse estimate to settle (or restart the drone), then retry")
        self.v.arm(True)
        self.ev.add("armed", self.clock())
        self.sleep(1.0)
        self.ev.add("takeoff", self.clock(), height_m=c.height_m)
        self.airborne = True
        self._hold(self.home[0], c.height_m, c.takeoff_s, z_from=self.ground_z + 0.05)
        self._hold(self.home[0], c.height_m, c.settle_s)
        self.ev.add("hover_ok", self.clock(), **{k: self.v.latest().get(k) for k in ("x", "y", "z", "vbat")})

    def fly_pass(self, i: int) -> None:
        c = self.cfg
        d0 = self._dir()
        far = self.home[0] + d0 * c.pass_length_m
        direction = -d0 if (c.alternate and i % 2 == 1) else d0
        x0 = self.home[0] if direction == d0 else far
        x1 = x0 + direction * c.pass_length_m
        if not c.alternate:
            x0, x1, direction = self.home[0], far, d0
            self._hold(x0, c.height_m, max(c.pause_s, c.pass_length_m / max(c.max_correction_m_s, 1e-3)))
        timeout = c.pass_timeout_factor * c.pass_length_m / c.speed_m_s + 3.0
        t0 = self.clock()
        self.ev.add("pass_start", t0, i=i, direction=int(direction), x0=x0, x1=x1, speed_cmd=c.speed_m_s,
                    control=c.pass_control)
        t_next = t0
        reason = "reached"
        self.pass_index, self.pass_direction = i, int(direction)
        self.in_pass = True
        if self._position_mode():
            reason = self._pass_position(x0, x1, direction, t0, timeout)
        while not self._position_mode():
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

    def beacon_active(self, which: str = "all") -> bool:
        """Should the C4 beacon report the mission now? which = "all": during every pass; "outbound": only
        during the outward passes (even index in the alternating pattern: 0, 2, ...), so the return legs do not
        make the panel tilt."""
        if not self.in_pass:
            return False
        if which == "outbound":
            return self.pass_index is not None and self.pass_index % 2 == 0
        return True

    def _pass_position(self, x0: float, x1: float, direction: float, t0: float, timeout: float) -> str:
        """Slide the position target from x0 to x1 on the speed profile; done once the drone is at x1."""
        c = self.cfg
        t_next = t0
        while True:
            s = self._state()
            self._check(s)
            t = self.clock() - t0
            dist, total = trapezoid(t, c.pass_length_m, c.speed_m_s, c.accel_m_s2)
            if t >= total and abs(x1 - s["x"]) <= 0.03:
                return "reached"
            if t > timeout:
                return "timeout"
            self._send_pos(x0 + direction * dist, c.height_m)
            t_next = self._tick(t_next)

    def land(self) -> None:
        c = self.cfg
        s = self.v.latest()
        z0 = s.get("z", c.height_m) or c.height_m
        self.ev.add("land", self.clock())
        self._hold(s.get("x", self.home[0]), self.ground_z + 0.05, c.land_s, z_from=z0, critical_only=True)
        self.airborne = False     # first: a retry after this point must not take off again
        self.v.stop_motors()
        self.v.arm(False)
        self.ev.add("landed", self.clock())

    def run(self) -> Dict[str, Any]:
        self.ev.add("mission_start", self.clock(), config=self.cfg.to_dict())
        result = {"passes_done": 0, "aborted": False, "reason": ""}
        saved = self._install_stop_handler()
        try:
            try:
                self.takeoff()
                for i in range(self.cfg.n_passes):
                    self.fly_pass(i)
                self.land()
            except (MissionAbort, KeyboardInterrupt, Exception) as e:  # noqa: BLE001 - land first, whatever happened
                self._stopping = True     # from here on, further stop signals are ignored
                self.in_pass = False      # the beacon stops reporting the mission, so the panel can close
                cut = False
                if isinstance(e, MissionAbort):
                    reason, cut = e.reason, e.cut_motors
                elif isinstance(e, KeyboardInterrupt):
                    reason = "operator Ctrl+C"
                else:
                    reason = f"unexpected error: {type(e).__name__}: {e}"
                result.update(aborted=True, reason=reason)
                self.ev.add("abort", self.clock(), reason=reason, cut_motors=cut)
                self._safe_stop(cut)
        finally:
            _restore_signals(saved)
        result["passes_done"] = self.passes_done
        self.ev.add("mission_end", self.clock(), **result)
        return result

    def _safe_stop(self, cut_motors: bool) -> None:
        if cut_motors or not self.airborne:
            self.v.emergency_stop() if cut_motors else self.v.stop_motors()
            self.airborne = False
            try:
                self.v.arm(False)
            except Exception:  # noqa: BLE001 - not every vehicle can disarm; the motors are stopped anyway
                pass
            return
        try:
            while self.airborne:    # a landing that is interrupted starts again from where the drone is
                try:
                    self.land()
                except KeyboardInterrupt:
                    continue
        except MissionAbort as e:
            self.ev.add("abort", self.clock(), reason=e.reason, cut_motors=True)
            self.v.emergency_stop()
            self.airborne = False
        except Exception:  # noqa: BLE001 - last resort
            self.v.emergency_stop()
            self.airborne = False
        self.v.stop_motors()   # land() clears airborne before this; make sure the motors are off whatever happened

    def _install_stop_handler(self) -> Dict[int, Any]:
        """One-shot stop handler: the first SIGINT, SIGTERM or SIGHUP raises KeyboardInterrupt (the mission lands),
        every later one is ignored, so a second signal cannot interrupt the landing."""
        def handler(signum, frame):  # noqa: ARG001
            if self._stop_requested or self._stopping:
                self._stop_requested = True
                return
            self._stop_requested = True
            raise KeyboardInterrupt
        saved: Dict[int, Any] = {}
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            sig = getattr(signal, name, None)
            if sig is None:
                continue
            try:
                saved[sig] = signal.signal(sig, handler)
            except (ValueError, OSError):   # not the main thread
                pass
        return saved


def _restore_signals(saved: Dict[int, Any]) -> None:
    for sig, handler in saved.items():
        if handler is not None:
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass
