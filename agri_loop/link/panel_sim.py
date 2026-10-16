"""Panel simulator: reference implementation of the Arduino FSM (plan B5).

States: IDLE_TRACK -> ACTUATING -> HOLD -> RESTORE -> IDLE_TRACK, plus FAULT
when heartbeats stop. The servo is modelled as a slew-limited actuator and the
IMU as the true angle plus noise. The firmware written by the bachelor team
should behave identically on the same message sequence.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Optional

from .eventlog import EventLog
from .messages import (EVENT_ABORT, EVENT_APPROACH, EVENT_CLEARED, FSM_ACTUATING, FSM_FAULT, FSM_HOLD, FSM_IDLE_TRACK,
                       FSM_RESTORE, Heartbeat, PanelEvent, PanelState)


@dataclass
class PanelConfig:
    panel_id: int = 1
    omega_deg_s: float = 90.0     # servo + panel slew rate (measure on the real frame, C3)
    default_angle: float = 20.0   # fixed "energy" angle while idle (no real sun-tracking)
    safe_angle: float = 20.0
    imu_noise_deg: float = 0.5
    state_hz: float = 20.0
    max_event_age_s: float = 0.5
    hb_timeout_s: float = 1.0
    t_max_hold_s: float = 20.0
    tol_deg: float = 4.0
    settle_s: float = 0.2
    i0_a: float = 0.5             # panel current at the best angle (energy proxy)


class PanelSim:
    def __init__(self, send: Callable[[object], None], log: EventLog, cfg: Optional[PanelConfig] = None, seed: int = 1):
        self.cfg = cfg or PanelConfig()
        self._send = send
        self.log = log
        self.rng = random.Random(seed)
        self.angle = self.cfg.default_angle
        self.target = self.cfg.default_angle
        self.fsm = FSM_IDLE_TRACK
        self.ack_seq = 0
        self._seq = 0
        self._last_t: Optional[float] = None
        self._last_state_t = -1e9
        self._last_hb_t: Optional[float] = None
        self._hb_ok_since: Optional[float] = None
        self._in_tol_since: Optional[float] = None
        self._hold_start = 0.0

    # ---- inputs ----
    def on_heartbeat(self, h: Heartbeat, now: float) -> None:
        self._last_hb_t = now

    def on_event(self, e: PanelEvent, now: float) -> None:
        c = self.cfg
        if now - e.t_send > c.max_event_age_s:
            self.log.add("reject_stale", now, seq=e.seq, age=now - e.t_send)
            return
        if e.seq <= self.ack_seq:        # duplicate (retransmission): ack again, do nothing
            return
        if e.event_type == EVENT_APPROACH and self.fsm in (FSM_IDLE_TRACK, FSM_RESTORE) and 0.0 <= e.target_angle_deg <= 90.0:
            self.target = e.target_angle_deg
            self.fsm = FSM_ACTUATING
            self._in_tol_since = None
        elif e.event_type in (EVENT_CLEARED, EVENT_ABORT) and self.fsm in (FSM_ACTUATING, FSM_HOLD):
            self.target = c.default_angle
            self.fsm = FSM_RESTORE
        self.ack_seq = e.seq
        self.log.add("event_applied", now, seq=e.seq, etype=e.event_type)

    # ---- time ----
    def tick(self, now: float) -> None:
        c = self.cfg
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now
        # servo
        step = c.omega_deg_s * dt
        self.angle += max(-step, min(step, self.target - self.angle))
        measured = self.angle + self.rng.gauss(0.0, c.imu_noise_deg)

        # watchdog
        if self._last_hb_t is not None:
            hb_age = now - self._last_hb_t
            if hb_age > c.hb_timeout_s and self.fsm != FSM_FAULT:
                self.fsm = FSM_FAULT
                self.target = c.safe_angle
                self._hb_ok_since = None
                self.log.add("fault", now, reason="heartbeat_lost")
            elif self.fsm == FSM_FAULT and hb_age <= c.hb_timeout_s:
                self._hb_ok_since = self._hb_ok_since if self._hb_ok_since is not None else now
                if now - self._hb_ok_since >= 1.0 and abs(self.angle - c.default_angle) <= c.tol_deg:
                    self.fsm = FSM_IDLE_TRACK
                    self.log.add("fault_cleared", now)

        # FSM
        if self.fsm == FSM_ACTUATING:
            if abs(measured - self.target) <= c.tol_deg:
                self._in_tol_since = self._in_tol_since if self._in_tol_since is not None else now
                if now - self._in_tol_since >= c.settle_s:
                    self.fsm = FSM_HOLD
                    self._hold_start = now
            else:
                self._in_tol_since = None
        elif self.fsm == FSM_HOLD:
            if now - self._hold_start > c.t_max_hold_s:
                self.target = c.default_angle
                self.fsm = FSM_RESTORE
                self.log.add("hold_timeout", now)
        elif self.fsm == FSM_RESTORE:
            if abs(self.angle - c.default_angle) <= c.tol_deg:
                self.fsm = FSM_IDLE_TRACK

        # publish state
        if now - self._last_state_t >= 1.0 / c.state_hz:
            self._last_state_t = now
            self._seq += 1
            cur = c.i0_a * max(0.0, math.cos(math.radians(self.angle - c.default_angle)))
            st = PanelState(seq=self._seq, t_send=now, panel_id=c.panel_id, fsm_state=self.fsm,
                            angle_deg=measured, target_deg=self.target, current_a=cur, ack_seq=self.ack_seq)
            self.log.add("send", now, msg="PanelState", seq=st.seq)
            self._send(st)
