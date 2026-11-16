"""Owns the Crazyflie for the ROS2 mission: the drone-side twin of the Arduino panel.

    python -m agri_loop.drone.cf_server --sim        # simulated drone, no radio
    python -m agri_loop.drone.cf_server              # real drone over the Crazyradio ($AGRI_CF_URI)

Runs in ~/agri_venv (cflib), never inside ROS2. The ROS2 node `drone_bridge`
talks to it in plain text over UDP on localhost, the same way `arduino_bridge`
talks to the panel:

    drone_bridge -> cf_server (port 5010)   command, resent 5 times per second:
        Status                 change nothing, just report (the bridge's default)
        Takeoff,z | Goto,x,y,z | Land      flying commands (refused until step 3 of the plan)
    cf_server -> drone_bridge (port 5011)   20 times per second:
        STATE,x,y,z,yaw,vbat   e.g. LANDED,1.402,0.600,0.012,2.1,4.05
                               position in m (Lighthouse frame), yaw in deg, battery in V

STATE is only sent while the radio log is fresh (newest sample younger than
0.3 s). A drone that is off or out of range therefore goes silent instead of
repeating an old position, so the coordinator's 1 s staleness rule works end
to end.

Step 1 of docs/drone_integration_plan.md: pose only, the motors never start.
Ctrl+C stops the server and saves the run folder (flight_log.csv, events.jsonl).
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from typing import Any, Callable, Dict, Optional, Tuple

from ..link.eventlog import EventLog
from .flightlog import FlightLogger

CMD_PORT = 5010      # cf_server listens here for commands
STATE_PORT = 5011    # drone_bridge listens here for STATE
RATE_HZ = 20.0       # STATE rate; NFR-6 asks for 5 Hz or more
FRESH_S = 0.3        # a radio sample older than this is not reported
FLY_COMMANDS = {"Takeoff": 1, "Goto": 3, "Land": 0}   # name -> number of values after it


def parse_command(text: str) -> Optional[Tuple[str, Tuple[float, ...]]]:
    """'Goto,1.4,-0.1,0.8' -> ('Goto', (1.4, -0.1, 0.8)). None for anything malformed."""
    parts = text.strip().split(",")
    name = parts[0]
    n_values = 0 if name == "Status" else FLY_COMMANDS.get(name)
    if n_values is None or len(parts) != n_values + 1:
        return None
    try:
        return name, tuple(float(p) for p in parts[1:])
    except ValueError:
        return None


def format_state(name: str, s: Dict[str, Any]) -> str:
    g = lambda k: float(s.get(k) or 0.0)  # noqa: E731
    return f"{name},{g('x'):.3f},{g('y'):.3f},{g('z'):.3f},{g('yaw'):.1f},{g('vbat'):.2f}"


class CfServer:
    def __init__(self, vehicle: Any, sock: socket.socket, bridge: Tuple[str, int], events: EventLog,
                 clock: Callable[[], float] = time.time) -> None:
        self.v = vehicle
        self.sock = sock
        self.bridge = bridge
        self.ev = events
        self.clock = clock
        self.state = "LANDED"     # step 1: the motors never start
        self.command = "Status"
        self.command_time: Optional[float] = None   # used for the no-command hold in step 3
        self.sent = 0
        self.stale = 0            # ticks where the sample was too old to report
        self._was_fresh: Optional[bool] = None

    def step(self) -> None:
        """One 20 Hz tick: read commands, then report the pose if it is fresh."""
        self._read_commands()
        s = self.v.latest()
        now = self.clock()
        fresh = self.v.link_ok() and "x" in s and now - s.get("t", 0.0) < FRESH_S
        if fresh != self._was_fresh:
            self.ev.add("pose_fresh" if fresh else "pose_stale", now)
            self._was_fresh = fresh
        if not fresh:
            self.stale += 1
            return
        try:
            self.sock.sendto(format_state(self.state, s).encode(), self.bridge)
            self.sent += 1
        except OSError:   # bridge not running yet; nothing to do, the next tick tries again
            pass

    def _read_commands(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(128)
            except BlockingIOError:
                return
            except ConnectionResetError:   # Windows: ICMP "port unreachable" from an earlier send
                continue
            text = data.decode(errors="replace").strip()
            now = self.clock()
            self.command_time = now
            if text == self.command:
                continue   # the bridge resends the same command 5 times per second
            self.command = text
            cmd = parse_command(text)
            if cmd is None:
                self.ev.add("bad_command", now, text=text)
                print(f"Ignoring malformed command: {text!r}")
            elif cmd[0] == "Status":
                self.ev.add("command", now, text=text)
            else:
                self.ev.add("command_refused", now, text=text, reason="flying commands come in step 3")
                print(f"Refusing {text!r}: flying commands are not enabled yet (step 3 of the plan)")

    def status_line(self) -> str:
        s = self.v.latest()
        age = self.clock() - s.get("t", 0.0) if s else float("inf")
        return (f"{format_state(self.state, s) if s else self.state}  sent={self.sent} stale_ticks={self.stale} "
                f"sample_age={age * 1000:.0f} ms  cmd={self.command}")


def build_parser() -> argparse.ArgumentParser:
    from .c1_common import DEFAULT_URI

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated drone instead of the Crazyradio")
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--cmd-port", type=int, default=CMD_PORT, help="port this server listens on")
    ap.add_argument("--bridge", default=f"127.0.0.1:{STATE_PORT}", metavar="HOST:PORT",
                    help="where STATE goes (drone_bridge)")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default=os.path.join("data", "cf_server"))
    return ap


def main(argv=None) -> int:
    from .c1_common import new_run_dir, preflight, print_report, save_json
    from .mission import C1Config

    a = build_parser().parse_args(argv)
    host, port = a.bridge.rsplit(":", 1)
    run_dir = new_run_dir(a.out, a.label or ("sim" if a.sim else "real"))
    save_json(os.path.join(run_dir, "config.json"), dict(args=vars(a)))
    events = EventLog()
    logger = FlightLogger(os.path.join(run_dir, "flight_log.csv"))

    if a.sim:
        from .sim import SimVehicle

        vehicle = SimVehicle(logger).open()
    else:
        from .vehicle import CrazyflieVehicle

        vehicle = CrazyflieVehicle(a.uri, logger, console_path=os.path.join(run_dir, "console.log")).open()
        vehicle.start_logging()
        time.sleep(1.5)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", a.cmd_port))
    sock.setblocking(False)
    server = CfServer(vehicle, sock, (host, int(port)), events)
    try:
        _, report, _ = preflight(vehicle, C1Config(), need_camera=False)
        events.add("preflight", time.time(), **report)
        print_report("Drone (information only, motors stay off)", report)
        print(f"\nListening on 127.0.0.1:{a.cmd_port}, sending STATE to {host}:{port} at {RATE_HZ:g} Hz. "
              "Ctrl+C stops.")
        t_next = t_print = time.time()
        while True:
            server.step()
            if time.time() - t_print >= 2.0:
                print(server.status_line())
                t_print = time.time()
            t_next += 1.0 / RATE_HZ
            time.sleep(max(0.0, t_next - time.time()))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        events.add("server_end", time.time(), sent=server.sent, stale_ticks=server.stale)
        sock.close()
        vehicle.close()   # stop the radio log first, so no sample arrives after the log file is closed
        logger.close()
        events.save(os.path.join(run_dir, "events.jsonl"))
        print(f"Run saved to {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
