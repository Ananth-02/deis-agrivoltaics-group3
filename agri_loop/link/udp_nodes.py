"""Run the loop over real UDP sockets (separate processes, or --role demo in one process).

    python -m agri_loop.link.udp_nodes --role gateway --out data/c4_real
    python -m agri_loop.link.udp_nodes --role panel   --out data/c4_real
    python -m agri_loop.link.udp_nodes --role drone   --out data/c4_real --passes 20
    python -m agri_loop.link.udp_nodes --role demo --seconds 90 --passes 3 --loss 0.1   (all three, one process)

Ports: drone -> gateway 5001, gateway -> panel 5002, panel -> gateway 5003.
Use --host-gateway / --host-panel to put the roles on different machines (then
use chrony so timestamps agree) and `tc netem` on that link for real loss.
Each role writes <out>/<role>.jsonl; analyse with python -m agri_loop.link.c4_report <out>/*.jsonl
"""
from __future__ import annotations

import argparse
import os
import threading
import time

from .eventlog import EventLog
from .gateway import Gateway, GatewayConfig
from .messages import MISSION_CROP_OCCLUSION, MISSION_NONE, DroneBeacon, Heartbeat, PanelEvent, PanelState
from .panel_sim import PanelConfig, PanelSim
from .udp import UdpPort

P_GW, P_PANEL = 5001, 5002   # gateway listens on P_GW (beacons, panel state), panel on P_PANEL


def _loop(stop: threading.Event, step, period: float = 0.005):
    while not stop.is_set():
        step(time.time())
        time.sleep(period)


def run_drone(a, stop: threading.Event, log: EventLog) -> None:
    port = UdpPort(("0.0.0.0", 0), (a.host_gateway, P_GW))
    t0 = time.time()
    state = {"seq": 0, "next": t0}
    pass_len = (a.x1 - a.x0) / a.speed

    def track(t):
        rel = t - t0 - a.start
        k = int(rel // a.period) if rel >= 0 else -1
        if 0 <= k < a.passes and rel - k * a.period <= pass_len:
            return a.x0 + a.speed * (rel - k * a.period), a.speed, MISSION_CROP_OCCLUSION
        return a.x0, 0.0, MISSION_NONE

    def step(now):
        if now >= state["next"]:
            state["next"] += 0.1
            x, vx, mission = track(now)
            state["seq"] += 1
            b = DroneBeacon(state["seq"], now, 1, x, 0.0, 0.6, vx, 0.0, 0.0, 0.0, mission)
            log.add("send", now, msg="DroneBeacon", seq=b.seq)
            port.send(b)
    _loop(stop, step)
    port.close()


def run_gateway(a, stop: threading.Event, log: EventLog) -> None:
    rx = UdpPort(("0.0.0.0", P_GW), ("127.0.0.1", 9), rx_loss=a.loss, seed=11)
    tx = UdpPort(("0.0.0.0", 0), (a.host_panel, P_PANEL))
    gw = Gateway(tx.send, log, GatewayConfig())

    def step(now):
        for m, t in rx.poll():
            log.add("recv", t, msg=type(m).__name__, seq=m.seq)
            if isinstance(m, DroneBeacon):
                gw.on_beacon(m, t)
            elif isinstance(m, PanelState):
                gw.on_state(m, t)
        gw.tick(now)
    _loop(stop, step)
    rx.close()
    tx.close()


def run_panel(a, stop: threading.Event, log: EventLog) -> None:
    rx = UdpPort(("0.0.0.0", P_PANEL), ("127.0.0.1", 9), rx_loss=a.loss, seed=12)
    tx = UdpPort(("0.0.0.0", 0), (a.host_gateway, P_GW))
    panel = PanelSim(tx.send, log, PanelConfig())

    def step(now):
        for m, t in rx.poll():
            log.add("recv", t, msg=type(m).__name__, seq=m.seq)
            if isinstance(m, PanelEvent):
                panel.on_event(m, t)
            elif isinstance(m, Heartbeat):
                panel.on_heartbeat(m, t)
        panel.tick(now)
    _loop(stop, step)
    rx.close()
    tx.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["drone", "gateway", "panel", "demo"], required=True)
    ap.add_argument("--out", default="data/c4_real")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = run until Ctrl+C")
    ap.add_argument("--loss", type=float, default=0.0, help="software receive-side loss (use tc netem on Linux if you can)")
    ap.add_argument("--host-gateway", default="127.0.0.1")
    ap.add_argument("--host-panel", default="127.0.0.1")
    ap.add_argument("--passes", type=int, default=3)
    ap.add_argument("--period", type=float, default=28.0)
    ap.add_argument("--start", type=float, default=5.0)
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--x0", type=float, default=-1.0)
    ap.add_argument("--x1", type=float, default=2.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    roles = ["drone", "gateway", "panel"] if a.role == "demo" else [a.role]
    fns = {"drone": run_drone, "gateway": run_gateway, "panel": run_panel}
    stop = threading.Event()
    logs = {r: EventLog() for r in roles}
    threads = [threading.Thread(target=fns[r], args=(a, stop, logs[r]), daemon=True) for r in roles]
    for t in threads:
        t.start()
    try:
        t_end = time.time() + a.seconds if a.seconds else None
        while t_end is None or time.time() < t_end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(timeout=2)
    for r, lg in logs.items():
        lg.save(os.path.join(a.out, f"{r}.jsonl"))
    print("logs in", a.out)


if __name__ == "__main__":
    main()
