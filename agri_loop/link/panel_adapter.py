"""Panel adapter: takes the place of the panel simulator and drives Anton's real panel (Arduino UNO R4 WiFi).

    python -m agri_loop.link.panel_adapter --arduino 192.168.1.19 --out data/full_pass/run1
    python -m agri_loop.link.panel_adapter --fake-arduino --out data/dry          # dry run, no hardware

It listens on UDP 5002 for the gateway's binary PanelEvent and Heartbeat messages and answers on
gateway:5001 with PanelState, exactly like panel_sim. Towards the Arduino it speaks plain text on port 5005:
    Plant_inspection (servo 0, panel vertical, plant in view), Panel_inspection (90, flat),
    Done (back to 45, SUN_CATCH), Status (changes nothing).
The wanted command is repeated at 20 Hz (repeating is harmless). The Arduino replies "<STATE>,<angle>" to
every packet; the event is acknowledged to the gateway only once the reply names the wanted state, which
proves delivery. Heartbeats lost for 1 s send Done and log a fault. On exit Done is always sent a few times.
Do not run it next to Anton's arduino_bridge: that resends its own commands and fights this one.
Log: <out>/panel.jsonl (read it with agri_loop.link.c4_report together with gateway.jsonl).
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .eventlog import EventLog
from .messages import (EVENT_APPROACH, FSM_ACTUATING, FSM_FAULT, FSM_HOLD, FSM_IDLE_TRACK, FSM_RESTORE,
                       MISSION_CROP_OCCLUSION, MISSION_PANEL_VIEW, Heartbeat, PanelEvent, PanelState)
from .udp import UdpPort

P_GW, P_PANEL, P_ARDUINO = 5001, 5002, 5005

# command -> (state name the Arduino reports once it has taken the command, servo target angle)
COMMANDS = {"Plant_inspection": ("PLANT_INSPECTION", 0.0), "Panel_inspection": ("PANEL_INSPECTION", 90.0),
            "Done": ("SUN_CATCH", 45.0)}
STATE_ANGLE = {"PLANT_INSPECTION": 0.0, "PANEL_INSPECTION": 90.0, "SUN_CATCH": 45.0}
MISSION_COMMAND = {MISSION_CROP_OCCLUSION: "Plant_inspection", MISSION_PANEL_VIEW: "Panel_inspection"}


@dataclass
class AdapterConfig:
    panel_id: int = 1
    send_hz: float = 20.0
    max_event_age_s: float = 0.5
    hb_timeout_s: float = 1.0
    silent_s: float = 1.0        # no Arduino reply for this long -> warn
    tol_deg: float = 2.0         # reported angle is an integer; this is "at the target"
    done_repeats: int = 3
    shutdown_s: float = 2.0      # keep sending Done on exit until SUN_CATCH is reported, at most this long


def parse_reply(text: str) -> Optional[Tuple[str, float]]:
    name, _, ang = text.strip().partition(",")
    try:
        return name.strip(), float(ang)
    except ValueError:
        return None


class PanelAdapter:
    def __init__(self, send_state: Callable[[PanelState], None], send_arduino: Callable[[str], None], log: EventLog,
                 cfg: Optional[AdapterConfig] = None, warn: Callable[[str], None] = print,
                 sleep: Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.time) -> None:
        self.cfg = cfg or AdapterConfig()
        self._send_state, self._send_arduino, self.log = send_state, send_arduino, log
        self._warn, self._sleep, self._clock = warn, sleep, clock
        self._send_failing = False
        self.cmd = "Status"            # what is sent at 20 Hz; Status changes nothing
        self.target = 45.0
        self.seen_seq = 0              # newest event accepted
        self.ack_seq = 0               # newest event confirmed by an Arduino reply
        self._pending: Optional[int] = None
        self._seq = 0
        self._last_tx = -1e9
        self._t_start: Optional[float] = None
        self._last_reply_t: Optional[float] = None
        self._reply: Optional[Tuple[str, float]] = None
        self._silent = False
        self._last_hb: Optional[float] = None
        self.fault = False

    # ---- inputs ----
    def on_heartbeat(self, h: Heartbeat, now: float) -> None:
        self._last_hb = now
        if self.fault:
            self.fault = False
            self.log.add("fault_cleared", now)

    def on_event(self, e: PanelEvent, now: float) -> None:
        if now - e.t_send > self.cfg.max_event_age_s:
            self.log.add("reject_stale", now, seq=e.seq, age=now - e.t_send)
            return
        if e.seq <= self.seen_seq:     # duplicate or older: do not act again, ack again with the next state
            self._publish(now)
            return
        self.seen_seq = e.seq
        if e.event_type == EVENT_APPROACH:
            cmd = MISSION_COMMAND.get(e.mission_type)
            if cmd is None:
                self.log.add("reject_mission", now, seq=e.seq, mission=e.mission_type)
                self.ack_seq = e.seq
                return
        else:
            cmd = "Done"
        self._set_command(cmd)
        self._pending = e.seq
        self.log.add("event_applied", now, seq=e.seq, etype=e.event_type, cmd=cmd)

    def on_arduino(self, text: str, now: float) -> None:
        r = parse_reply(text)
        if r is None:
            self.log.add("arduino_bad_reply", now, text=text[:40])
            return
        if self._silent:
            self._silent = False
            self.log.add("arduino_back", now)
        self._last_reply_t, self._reply = now, r
        if self._pending is not None and r[0] == COMMANDS[self.cmd][0]:
            self.ack_seq, self._pending = self._pending, None   # the Arduino has taken the command
        self._publish(now)

    # ---- time ----
    def tick(self, now: float) -> None:
        c = self.cfg
        if self._t_start is None:
            self._t_start = now
        if self._last_hb is not None and now - self._last_hb > c.hb_timeout_s and not self.fault \
                and self.cmd != "Done":
            self.fault = True
            self._set_command("Done")
            self._pending = None
            self.log.add("fault", now, reason="heartbeat_lost")
        ref = self._last_reply_t if self._last_reply_t is not None else self._t_start
        if now - ref > c.silent_s and not self._silent:
            self._silent = True
            self.log.add("arduino_silent", now)
            self._warn(f"WARNING: the Arduino has not answered for {c.silent_s:.0f} s: check its Wi-Fi and IP address")
        if now - self._last_tx >= 1.0 / c.send_hz:
            self._last_tx = now
            self._tx_arduino(self.cmd, now)

    def shutdown(self, poll: Optional[Callable[[], list]] = None) -> bool:
        """Always leave the panel in its normal position: send Done every 0.1 s until the Arduino reports
        SUN_CATCH (poll returns the reply texts that arrived) or shutdown_s has passed. True if confirmed."""
        self._set_command("Done")
        waited, confirmed, n = 0.0, False, 0
        while True:
            self._tx_arduino("Done", self._clock())
            n += 1
            self._sleep(0.1)
            waited += 0.1
            for text in (poll() if poll else []):
                r = parse_reply(text)
                if r and r[0] == "SUN_CATCH":
                    confirmed = True
            if confirmed or (n >= self.cfg.done_repeats and (poll is None or waited >= self.cfg.shutdown_s)):
                break
        self.log.add("shutdown_done", self._clock(), confirmed=confirmed, sends=n)
        if not confirmed and poll is not None:
            self._warn("WARNING: the Arduino did not confirm SUN_CATCH on shutdown: check that the panel is "
                       "back in its normal position")
        return confirmed

    def _tx_arduino(self, cmd: str, now: float) -> None:
        try:
            self._send_arduino(cmd)
            self._send_failing = False
        except OSError as e:   # network down: no reply will come, so the silence warning follows
            if not self._send_failing:
                self._send_failing = True
                self.log.add("arduino_send_error", now, error=str(e))

    # ---- helpers ----
    def _set_command(self, cmd: str) -> None:
        self.cmd = cmd
        self.target = COMMANDS[cmd][1]

    def fsm_for(self, name: str, angle: float) -> int:
        if self.fault:
            return FSM_FAULT
        at = abs(angle - STATE_ANGLE.get(name, self.target)) <= self.cfg.tol_deg
        if name == "SUN_CATCH":
            return FSM_IDLE_TRACK if at else FSM_RESTORE
        return FSM_HOLD if at else FSM_ACTUATING

    def _publish(self, now: float) -> None:
        if self._reply is None:
            return
        name, angle = self._reply
        self._seq += 1
        st = PanelState(seq=self._seq, t_send=now, panel_id=self.cfg.panel_id, fsm_state=self.fsm_for(name, angle),
                        angle_deg=angle, target_deg=self.target, current_a=0.0, ack_seq=self.ack_seq)
        self.log.add("send", now, msg="PanelState", seq=st.seq)
        self._send_state(st)


class FakeArduino:
    """Text UDP server that behaves like panel.ino: same commands, replies and slew rate (1 degree per 15 ms)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0, deg_per_s: float = 1000.0 / 15.0) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.rate = deg_per_s
        self.state, self.angle, self.target = "SUN_CATCH", 45.0, 45.0
        self.commands: list = []
        self._t = time.time()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="fake_arduino", daemon=True)

    def start(self) -> "FakeArduino":
        self._t = time.time()
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.sock.close()

    def _advance(self, now: float) -> None:
        step = self.rate * (now - self._t)
        self._t = now
        self.angle += max(-step, min(step, self.target - self.angle))

    def handle(self, text: str, now: float) -> str:
        self._advance(now)
        if text in COMMANDS:
            self.state, self.target = COMMANDS[text]
            self.commands.append(text)
        return f"{self.state},{int(round(self.angle))}"

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            self.sock.sendto(self.handle(data.decode(errors="replace").strip(), time.time()).encode(), addr)


def _install_stop(stop: threading.Event) -> None:
    def handler(signum, frame):  # noqa: ARG001
        stop.set()
    for name in ("SIGINT", "SIGTERM", "SIGBREAK"):   # SIGBREAK: how full_pass stops us on Windows
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), handler)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arduino", default=os.environ.get("AGRI_ARDUINO_IP", ""), metavar="IP")
    ap.add_argument("--arduino-port", type=int, default=P_ARDUINO)
    ap.add_argument("--fake-arduino", action="store_true", help="run a fake Arduino on localhost (dry run)")
    ap.add_argument("--gateway", default="127.0.0.1")
    ap.add_argument("--out", default=os.path.join("data", "panel_adapter"))
    a = ap.parse_args(argv)

    fake = None
    if a.fake_arduino:
        fake = FakeArduino("127.0.0.1", a.arduino_port).start()
        host, port = "127.0.0.1", fake.port
        print(f"Fake Arduino on 127.0.0.1:{port}")
    else:
        if not a.arduino:
            print("Not starting: give the panel's address with --arduino IP (or --fake-arduino).")
            return 2
        from ..drone.measure_points import other_panel_senders

        busy = other_panel_senders()
        if busy:
            print(f"Not starting: {', '.join(busy)} is running. It resends its own panel commands and would "
                  "undo ours. Stop it first (Ctrl+C in its window).")
            return 2
        host, port = a.arduino, a.arduino_port

    os.makedirs(a.out, exist_ok=True)
    log = EventLog()
    stop = threading.Event()
    _install_stop(stop)
    rx = UdpPort(("0.0.0.0", P_PANEL), ("127.0.0.1", 9))
    tx = UdpPort(("0.0.0.0", 0), (a.gateway, P_GW))
    ard = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ard.bind(("0.0.0.0", 0))   # Windows needs a bound socket before recvfrom
    ard.setblocking(False)
    adapter = PanelAdapter(tx.send, lambda cmd: ard.sendto(cmd.encode(), (host, port)), log)
    print(f"Panel adapter: gateway {a.gateway}:{P_GW}, Arduino {host}:{port}, log {a.out}/panel.jsonl")
    try:
        while not stop.is_set():
            for m, t in rx.poll():
                log.add("recv", t, msg=type(m).__name__, seq=m.seq)
                if isinstance(m, PanelEvent):
                    adapter.on_event(m, t)
                elif isinstance(m, Heartbeat):
                    adapter.on_heartbeat(m, t)
            while True:
                try:
                    data, _ = ard.recvfrom(256)
                except BlockingIOError:
                    break
                except ConnectionResetError:   # Windows: nothing listening at the Arduino address yet
                    continue
                adapter.on_arduino(data.decode(errors="replace"), time.time())
            adapter.tick(time.time())
            time.sleep(0.005)
    finally:
        log.save(os.path.join(a.out, "panel.jsonl"))   # saved first: the log survives a failure below

        def poll() -> list:
            out = []
            while True:
                try:
                    out.append(ard.recvfrom(256)[0].decode(errors="replace"))
                except (BlockingIOError, OSError):
                    return out
        adapter.shutdown(poll)
        log.save(os.path.join(a.out, "panel.jsonl"))
        rx.close()
        tx.close()
        ard.close()
        if fake is not None:
            fake.stop()
        print("Panel adapter stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
