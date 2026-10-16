"""Gateway (roadside-unit role): turns drone beacons into panel events.

Sans-IO: the caller feeds messages and time, and gives a `send` callable.
Events are sent reliably (retransmitted until the panel acknowledges them),
beacons are best-effort, mirroring CAM and DENM.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple

from .eventlog import EventLog
from .messages import (EVENT_ABORT, EVENT_APPROACH, EVENT_CLEARED, MISSION_CROP_OCCLUSION, MISSION_CROP_SHADOW,
                       MISSION_NONE, MISSION_PANEL_VIEW, DroneBeacon, Heartbeat, PanelEvent, PanelState)


@dataclass
class GatewayConfig:
    panel_id: int = 1
    zone: Tuple[float, float, float, float] = (0.4, 0.8, -0.3, 0.3)  # panel occlusion zone in bed frame (m): xmin,xmax,ymin,ymax
    footprint_half: float = 0.25   # camera ground footprint half-size (m); about h*tan(FOV/2)
    lead_s: float = 1.4            # t_comm + t_actuate used for the prediction
    trigger_margin: float = 0.05   # m
    release_margin: float = 0.10   # m, larger than trigger margin (hysteresis)
    hold_s: float = 0.5            # footprint must be clear this long before CLEARED
    beacon_timeout_s: float = 2.0  # no beacon for this long while active -> CLEARED
    retransmit_s: float = 0.3
    max_retries: int = 10
    heartbeat_hz: float = 5.0
    angle_tol_deg: float = 4.0
    angle_for_mission: Dict[int, float] = field(default_factory=lambda: {
        MISSION_CROP_OCCLUSION: 90.0, MISSION_CROP_SHADOW: 60.0, MISSION_PANEL_VIEW: 0.0})


def _intersects(cx: float, cy: float, half: float, zone: Tuple[float, float, float, float]) -> bool:
    xmin, xmax, ymin, ymax = zone
    return not (cx + half < xmin or cx - half > xmax or cy + half < ymin or cy - half > ymax)


def _grow(zone: Tuple[float, float, float, float], m: float) -> Tuple[float, float, float, float]:
    return (zone[0] - m, zone[1] + m, zone[2] - m, zone[3] + m)


class Gateway:
    IDLE, PENDING, ACTIVE, CLEARING = "IDLE", "PENDING", "ACTIVE", "CLEARING"

    def __init__(self, send: Callable[[object], None], log: EventLog, cfg: Optional[GatewayConfig] = None):
        self.cfg = cfg or GatewayConfig()
        self._send = send
        self.log = log
        self.state = self.IDLE
        self._event_seq = 0
        self._hb_seq = 0
        self._pending: Optional[PanelEvent] = None
        self._pending_first_send = 0.0
        self._last_tx = 0.0
        self._retries = 0
        self._last_hb = -1e9
        self._last_beacon_t = -1e9
        self._clear_since: Optional[float] = None
        self._entered = False    # footprint has been inside the zone since the approach event
        self._acked = False      # pending event acknowledged by the panel (stop retransmitting)
        self._target = 0.0
        self.last_state: Optional[PanelState] = None

    # ---- inputs ----
    def on_beacon(self, b: DroneBeacon, now: float) -> None:
        self._last_beacon_t = now
        c = self.cfg
        if self.state == self.IDLE:
            if b.mission_type == MISSION_NONE or b.mission_type not in c.angle_for_mission:
                return
            px, py = b.x + b.vx * c.lead_s, b.y + b.vy * c.lead_s
            if _intersects(px, py, c.footprint_half + c.trigger_margin, c.zone):
                self._target = c.angle_for_mission[b.mission_type]
                self._send_event(EVENT_APPROACH, b.mission_type, self._target, now, first=True)
                self.state = self.PENDING
        elif self.state in (self.PENDING, self.ACTIVE):
            inside = _intersects(b.x, b.y, c.footprint_half, _grow(c.zone, c.release_margin))
            if inside:
                self._entered = True
                self._clear_since = None
            elif self.state == self.ACTIVE and (self._entered or b.mission_type == MISSION_NONE):
                # the drone has passed (or ended its mission): release after the hold time
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= c.hold_s:
                    self._send_event(EVENT_CLEARED, b.mission_type, 0.0, now, first=True)
                    self.state = self.CLEARING
            else:
                self._clear_since = None

    def on_state(self, s: PanelState, now: float) -> None:
        self.last_state = s
        p = self._pending
        if p is None or s.ack_seq < p.seq:
            return
        self._acked = True
        # the panel has processed the pending event
        if p.event_type == EVENT_APPROACH:
            if abs(s.angle_deg - p.target_angle_deg) <= self.cfg.angle_tol_deg:
                self.log.add("angle_reached", now, seq=p.seq, angle=s.angle_deg,
                             latency=now - self._pending_first_send)
                self._pending = None
                self.state = self.ACTIVE
            elif self.state == self.PENDING:
                self.log.add("event_acked", now, seq=p.seq, latency=now - self._pending_first_send)
                self.state = self.ACTIVE  # acked, still moving; keep pending for angle_reached
        else:
            self.log.add("restore_acked", now, seq=p.seq, latency=now - self._pending_first_send)
            self._pending = None
            self.state = self.IDLE
            self._clear_since = None

    # ---- time ----
    def tick(self, now: float) -> None:
        c = self.cfg
        if now - self._last_hb >= 1.0 / c.heartbeat_hz:
            self._last_hb = now
            self._hb_seq += 1
            self._tx(Heartbeat(seq=self._hb_seq, t_send=now), now)
        p = self._pending
        if p is not None and not self._acked and now - self._last_tx >= c.retransmit_s:
            if self._retries >= c.max_retries:
                self.log.add("event_failed", now, seq=p.seq)
                self._pending = None
                self.state = self.IDLE
                return
            self._retries += 1
            p.t_send = now
            self._tx(p, now, retransmit=True)
        if self.state == self.ACTIVE and now - self._last_beacon_t > c.beacon_timeout_s:
            self.log.add("beacon_timeout", now)
            self._send_event(EVENT_CLEARED, MISSION_NONE, 0.0, now, first=True)
            self.state = self.CLEARING

    # ---- helpers ----
    def _send_event(self, etype: int, mission: int, angle: float, now: float, first: bool) -> None:
        self._event_seq += 1
        ev = PanelEvent(seq=self._event_seq, t_send=now, panel_id=self.cfg.panel_id, event_type=etype,
                        mission_type=mission, target_angle_deg=angle)
        self._pending = ev
        self._pending_first_send = now
        self._retries = 0
        self._acked = False
        if etype == EVENT_APPROACH:
            self._entered = False
        self._tx(ev, now)

    def _tx(self, msg, now: float, retransmit: bool = False) -> None:
        self._last_tx = now if isinstance(msg, PanelEvent) else self._last_tx
        self.log.add("send", now, msg=type(msg).__name__, seq=msg.seq, retx=retransmit)
        self._send(msg)
