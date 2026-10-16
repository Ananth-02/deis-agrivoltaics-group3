"""C4 harness: drone -> gateway -> panel loop in virtual time.

One run reproduces the TG2 C4 test: beacons at 10 Hz for several minutes,
heartbeats at 5 Hz, panel state at 20 Hz, and N scripted passes of the drone
across the panel's occlusion zone. Link loss and delay are injected with SimNet
(a stand-in for `tc netem`; use netem for the real Wi-Fi measurements).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List

from .eventlog import EventLog
from .gateway import Gateway, GatewayConfig, _intersects
from .messages import MISSION_CROP_OCCLUSION, MISSION_NONE, DroneBeacon, Heartbeat, PanelEvent, PanelState, decode, encode
from .monitor import event_stats, stream_stats
from .netsim import SimNet
from .panel_sim import PanelConfig, PanelSim


@dataclass
class ScenarioConfig:
    duration_s: float = 600.0
    n_passes: int = 20
    pass_period_s: float = 28.0
    pass_start_s: float = 5.0
    speed_m_s: float = 0.15
    x0: float = -1.0
    x1: float = 2.0
    y: float = 0.0
    z: float = 0.6
    beacon_hz: float = 10.0
    dt: float = 0.01
    loss: float = 0.0
    delay_s: float = 0.010
    jitter_s: float = 0.005
    seed: int = 0


def run_scenario(sc: ScenarioConfig, gw_cfg: GatewayConfig = None, panel_cfg: PanelConfig = None) -> Dict[str, Any]:
    gw_cfg = gw_cfg or GatewayConfig()
    panel_cfg = panel_cfg or PanelConfig()
    log = EventLog()
    net_up = SimNet(sc.loss, sc.delay_s, sc.jitter_s, seed=sc.seed * 3 + 1)    # drone -> gateway
    net_dn = SimNet(sc.loss, sc.delay_s, sc.jitter_s, seed=sc.seed * 3 + 2)    # gateway -> panel
    net_st = SimNet(sc.loss, sc.delay_s, sc.jitter_s, seed=sc.seed * 3 + 3)    # panel -> gateway

    box: Dict[str, Any] = {"now": 0.0}

    def to_panel(msg) -> None:
        net_dn.send(encode(msg), box["now"], recv_panel)

    def to_gateway_state(msg) -> None:
        net_st.send(encode(msg), box["now"], recv_gateway)

    gw = Gateway(to_panel, log, gw_cfg)
    panel = PanelSim(to_gateway_state, log, panel_cfg, seed=sc.seed + 1)

    def recv_panel(data: bytes, t: float) -> None:
        m = decode(data)
        log.add("recv", t, msg=type(m).__name__, seq=m.seq)
        if isinstance(m, PanelEvent):
            panel.on_event(m, t)
        elif isinstance(m, Heartbeat):
            panel.on_heartbeat(m, t)

    def recv_gateway(data: bytes, t: float) -> None:
        m = decode(data)
        log.add("recv", t, msg=type(m).__name__, seq=m.seq)
        if isinstance(m, PanelState):
            gw.on_state(m, t)
        elif isinstance(m, DroneBeacon):
            gw.on_beacon(m, t)

    # scripted drone track
    pass_len = (sc.x1 - sc.x0) / sc.speed_m_s
    starts = [sc.pass_start_s + k * sc.pass_period_s for k in range(sc.n_passes)]

    def track(t: float):
        for t0 in starts:
            if t0 <= t <= t0 + pass_len:
                return sc.x0 + sc.speed_m_s * (t - t0), sc.speed_m_s, MISSION_CROP_OCCLUSION
        return sc.x0, 0.0, MISSION_NONE

    # ground-truth footprint entry times (for the timing-success criterion)
    enter_times: List[float] = []
    for t0 in starts:
        for i in range(int(pass_len / 0.005)):
            t = t0 + i * 0.005
            x, _, _ = track(t)
            if _intersects(x, sc.y, gw_cfg.footprint_half, gw_cfg.zone):
                enter_times.append(t)
                break

    angle_trace: List[tuple] = []
    next_beacon = 0.0
    bseq = 0
    n_steps = int(sc.duration_s / sc.dt)
    for i in range(n_steps + 1):
        now = i * sc.dt
        box["now"] = now
        if now >= next_beacon:
            next_beacon += 1.0 / sc.beacon_hz
            x, vx, mission = track(now)
            bseq += 1
            b = DroneBeacon(seq=bseq, t_send=now, station_id=1, x=x, y=sc.y, z=sc.z, vx=vx, vy=0.0, vz=0.0,
                            heading_deg=0.0, mission_type=mission)
            log.add("send", now, msg="DroneBeacon", seq=bseq)
            net_up.send(encode(b), now, recv_gateway)
        net_up.deliver_until(now)
        net_dn.deliver_until(now)
        net_st.deliver_until(now)
        gw.tick(now)
        panel.tick(now)
        if i % 10 == 0:
            angle_trace.append((now, panel.angle))

    # flush packets still in flight so the last messages are not counted as lost
    for net in (net_up, net_dn, net_st):
        net.deliver_until(sc.duration_s + 1.0)

    # timing success: panel true angle within tol of the target before the footprint enters the zone
    target = gw_cfg.angle_for_mission[MISSION_CROP_OCCLUSION]
    success = 0
    margins: List[float] = []
    reached_by_seq = [e for e in log.events if e["ev"] == "angle_reached"]
    for k, t_enter in enumerate(enter_times):
        t0 = starts[k]
        hits = [e for e in reached_by_seq if t0 <= e["t"] <= t0 + pass_len]
        if hits:
            m = t_enter - hits[0]["t"]
            margins.append(m)
            if m > 0:
                success += 1

    ev = log.events
    res = {
        "loss": sc.loss,
        "passes": len(starts),
        "timing_success": success / max(1, len(starts)),
        "timing_margin_s_min": min(margins) if margins else None,
        "streams": {m: stream_stats(ev, m) for m in ("DroneBeacon", "Heartbeat", "PanelEvent", "PanelState")},
        "events": event_stats(ev),
        "net": {"up_sent": net_up.sent, "up_dropped": net_up.dropped,
                "dn_sent": net_dn.sent, "dn_dropped": net_dn.dropped,
                "st_sent": net_st.sent, "st_dropped": net_st.dropped},
    }
    return {"summary": res, "log": log, "angle_trace": angle_trace}
