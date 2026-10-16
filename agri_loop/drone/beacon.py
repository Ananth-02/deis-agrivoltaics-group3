"""Join C1 to C4: send the drone's live pose to the gateway as DroneBeacon (CAM-like) messages.

Replaces the scripted track of `link.udp_nodes --role drone` with the real
Flow deck estimate. Positions are in the Flow deck frame (origin at take-off,
x along the passes) plus a fixed offset, so place the drone at a measured
point of the bed frame before take-off and pass that point as the offset.
Sends are logged as `<run_dir>/drone.jsonl` in the C4 format, so
`python -m agri_loop.link.c4_report` reads them with gateway.jsonl and panel.jsonl.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Tuple

from ..link.eventlog import EventLog
from ..link.messages import MISSION_CROP_OCCLUSION, MISSION_NONE, DroneBeacon
from ..link.udp import UdpPort

P_GW = 5001  # gateway port used by link.udp_nodes


class BeaconSender:
    def __init__(self, latest: Callable[[], dict], in_pass: Callable[[], bool], gateway: Tuple[str, int],
                 offset: Tuple[float, float] = (0.0, 0.0), rate_hz: float = 10.0, station_id: int = 1,
                 mission_type: int = MISSION_CROP_OCCLUSION) -> None:
        self.latest, self.in_pass = latest, in_pass
        self.offset, self.period = offset, 1.0 / rate_hz
        self.station_id, self.mission_type = station_id, mission_type
        self.port = UdpPort(("0.0.0.0", 0), gateway)
        self.log = EventLog()
        self.seq = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="beacon", daemon=True)

    def start(self) -> "BeaconSender":
        self._thread.start()
        return self

    def stop(self, path: str) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.port.close()
        self.log.save(path)

    def make(self, s: dict, now: float) -> DroneBeacon:
        self.seq += 1
        mission = self.mission_type if self.in_pass() else MISSION_NONE
        g = lambda k: float(s.get(k, 0.0) or 0.0)  # noqa: E731
        return DroneBeacon(self.seq, now, self.station_id, g("x") + self.offset[0], g("y") + self.offset[1], g("z"),
                           g("vx"), g("vy"), g("vz"), g("yaw"), mission)

    def _run(self) -> None:
        t_next = time.time()
        while not self._stop.is_set():
            s = self.latest()
            now = time.time()
            if s and "x" in s:
                b = self.make(s, now)
                self.log.add("send", now, msg="DroneBeacon", seq=b.seq)
                self.port.send(b)
            t_next += self.period
            time.sleep(max(0.0, t_next - time.time()))
