"""Measure the mission points in the Lighthouse frame, by one person, motors off.

    python -m agri_loop.drone.measure_points --arduino 192.168.1.19
    python -m agri_loop.drone.measure_points --sim            # dry run without hardware

The script walks through a list of points (section 6 of docs/drone_integration_plan.md).
For each one it prints what to do, then waits until the drone has been still for 2.5 s
at a new place (at least 8 cm from the last point) and records the average position.
So: place or hold the drone, keep still, wait for the drone's LEDs to flash, move on.
No need to come back to the keyboard.

Keys (optional, for example from a phone over SSH): Enter = record now, s = skip,
b = back one point, q = quit. The panel points send Plant_inspection / Panel_inspection
straight to the Arduino over UDP and wait for its confirmation; stop arduino_bridge and
mission_coordinator while measuring, or their resent commands fight the script.
At the end the panel gets Done, and the results go to data/measure/<time>/:
points.json, points.csv and summary.md (fence, heights, staging point, warnings).

The motors are never armed: the drone is only connected and logged.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
import socket
import sys
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from .flightlog import LH_TO_ESTIMATOR

Point = Tuple[float, float, float]

# (key, what to do, panel command to send first or None). Ordered to keep walking short.
POINTS: List[Tuple[str, str, Optional[str]]] = [
    ("origin", "Flat on the green X, facing the calibration direction.", None),
    ("towards_strip", "Flat on the floor, 0.5 m from the X towards the take-off strip.", None),
    ("big_panel_1", "Held level on the HIGHEST edge of the big panel above the X.", None),
    ("big_panel_2", "Held level on the HIGHEST edge of the other big panel.", None),
    ("takeoff", "Flat on the take-off spot in the middle of the strip (mark it with tape).", None),
    ("corner_strip", "Flat on the floor, pushed into the corner at the strip end, far from the rig.", None),
    ("staging", "Flat on the floor about 0.5 m from the pot towards the strip, clear of the panel swing, "
                "the big panels and the cables.", None),
    ("pot", "On the soil in the middle of the pot.", None),
    ("leaf", "Held level at the top of the highest leaf.", None),
    ("post_a", "On top of one wooden post.", None),
    ("post_b", "On top of the other wooden post.", None),
    ("panel_top_vertical", "Panel is now VERTICAL: hold the drone level on its top edge.", "Plant_inspection"),
    ("panel_flat_strip_side", "Panel is now FLAT: hold the drone on its edge nearest the take-off strip.",
     "Panel_inspection"),
    ("panel_flat_far_side", "Panel still FLAT: hold the drone on its edge furthest from the strip.", None),
    ("corner_rig", "Flat on the floor, pushed into the corner next to the rig.", None),
]
PANEL_REPLY = {"Plant_inspection": ("PLANT_INSPECTION", 0), "Panel_inspection": ("PANEL_INSPECTION", 90),
               "Done": ("SUN_CATCH", 45)}
FLOOR_POINTS = ("origin", "towards_strip", "takeoff", "corner_strip", "staging", "corner_rig")

WINDOW_S = 2.5      # how long the drone must be still
STILL_M = 0.015     # max distance of any sample from the window mean
MIN_MOVE_M = 0.08   # a new point must be this far from the last recorded one
FRESH_S = 0.3
CLEARANCE_M = 0.30  # flight height above the highest obstacle
KEEP_OUT_MARGIN_M = 0.15
FENCE_MARGIN_M = 0.20


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


class StillDetector:
    """Finds the moment the drone has been held still at a new place."""

    def __init__(self, window_s: float = WINDOW_S, still_m: float = STILL_M, min_move_m: float = MIN_MOVE_M) -> None:
        self.window_s, self.still_m, self.min_move_m = window_s, still_m, min_move_m
        self.buf: Deque[Tuple[float, Point]] = deque()
        self.last: Optional[Point] = None

    def add(self, t: float, p: Point) -> None:
        self.buf.append((t, p))
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.buf.popleft()

    def reset(self) -> None:
        self.buf.clear()

    def mean(self) -> Optional[Point]:
        if not self.buf:
            return None
        n = len(self.buf)
        return tuple(sum(p[i] for _, p in self.buf) / n for i in range(3))  # type: ignore[return-value]

    def spread(self) -> float:
        m = self.mean()
        return max(_dist(p, m) for _, p in self.buf) if m else float("inf")

    def full(self) -> bool:
        return len(self.buf) >= 10 and self.buf[-1][0] - self.buf[0][0] >= 0.95 * self.window_s

    def check(self) -> Optional[Point]:
        """The window mean if the drone is still at a new place, else None."""
        if not self.full() or self.spread() > self.still_m:
            return None
        m = self.mean()
        if self.last is not None and _dist(m, self.last) < self.min_move_m:
            return None
        return m

    def accept(self, p: Point) -> None:
        self.last = p
        self.reset()


def derive(points: Dict[str, Dict[str, Any]]) -> Tuple[Dict[str, Any], List[str]]:
    """Mission numbers from the measured points. Missing points just leave their numbers out."""
    p = {k: v["pos"] for k, v in points.items()}
    out: Dict[str, Any] = {}
    warn: List[str] = []

    floor = [p[k][2] for k in FLOOR_POINTS if k in p]
    z0 = sum(floor) / len(floor) if floor else 0.0
    out["floor_z"] = round(z0, 3)
    if abs(z0) > 0.05:
        warn.append(f"floor reads z = {z0:.2f} m, not about 0: heights below are measured from the floor reading")
    if "origin" in p and _dist(p["origin"][:2], (0.0, 0.0)) > 0.05:
        warn.append(f"the green X reads ({p['origin'][0]:.2f}, {p['origin'][1]:.2f}), not (0, 0): "
                    "check the Lighthouse geometry")
    if "origin" in p and "towards_strip" in p:
        dx, dy = (p["towards_strip"][i] - p["origin"][i] for i in range(2))
        axis = "x" if abs(dx) >= abs(dy) else "y"
        sign = "+" if (dx if axis == "x" else dy) > 0 else "-"
        out["towards_strip_axis"] = sign + axis

    corners = [p[k] for k in ("corner_strip", "corner_rig") if k in p]
    if len(corners) == 2:
        lo = [min(c[i] for c in corners) for i in range(2)]
        hi = [max(c[i] for c in corners) for i in range(2)]
        out["arena_size_m"] = [round(hi[i] - lo[i], 2) for i in range(2)]
        out["fence"] = {"x": [round(lo[0] + FENCE_MARGIN_M, 2), round(hi[0] - FENCE_MARGIN_M, 2)],
                        "y": [round(lo[1] + FENCE_MARGIN_M, 2), round(hi[1] - FENCE_MARGIN_M, 2)]}

    rig = [k for k in ("leaf", "post_a", "post_b", "panel_top_vertical", "pot") if k in p]
    if rig:
        top = max(p[k][2] for k in rig) - z0
        out["rig_top_m"] = round(top, 3)
        out["over_plant_z"] = round(top + CLEARANCE_M, 2)
    big = [p[k][2] - z0 for k in ("big_panel_1", "big_panel_2") if k in p]
    if big or "over_plant_z" in out:
        out["cruise_z"] = round(max([b + CLEARANCE_M for b in big] + [out.get("over_plant_z", 0.0)]), 2)
    if "pot" in p and "over_plant_z" in out:
        out["OVER_PLANT"] = [round(p["pot"][0], 2), round(p["pot"][1], 2), out["over_plant_z"]]
    if "takeoff" in p:
        out["HOME"] = [round(p["takeoff"][0], 2), round(p["takeoff"][1], 2), 0.0]
    if "staging" in p and "cruise_z" in out:
        out["STAGING"] = [round(p["staging"][0], 2), round(p["staging"][1], 2), out["cruise_z"]]

    swing = [p[k] for k in ("post_a", "post_b", "panel_top_vertical", "panel_flat_strip_side",
                            "panel_flat_far_side") if k in p]
    if swing:
        box = {"x": [round(min(s[0] for s in swing) - KEEP_OUT_MARGIN_M, 2),
                     round(max(s[0] for s in swing) + KEEP_OUT_MARGIN_M, 2)],
               "y": [round(min(s[1] for s in swing) - KEEP_OUT_MARGIN_M, 2),
                     round(max(s[1] for s in swing) + KEEP_OUT_MARGIN_M, 2)]}
        out["keep_out"] = box
        if "staging" in p and box["x"][0] <= p["staging"][0] <= box["x"][1] \
                and box["y"][0] <= p["staging"][1] <= box["y"][1]:
            warn.append("the staging point is inside the panel keep-out box: pick a spot further from the rig")
    if "fence" in out:
        f = out["fence"]
        for name in ("pot", "staging", "takeoff"):
            if name in p and not (f["x"][0] <= p[name][0] <= f["x"][1] and f["y"][0] <= p[name][1] <= f["y"][1]):
                warn.append(f"{name} is within {FENCE_MARGIN_M} m of a wall (outside the fence)")
    if out.get("cruise_z", 0.0) > 1.0:
        warn.append(f"cruise height {out['cruise_z']} m is above 1.0 m: check the net and the base stations' view")
    return out, warn


class PanelLink:
    """Sends one panel command to the Arduino (resent 5 times per second) until it confirms."""

    def __init__(self, ip: str, port: int = 5005) -> None:
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

    def set(self, cmd: str, timeout_s: float = 6.0) -> Tuple[bool, str]:
        want_state, want_angle = PANEL_REPLY[cmd]
        reply = ""
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            self.sock.sendto(cmd.encode(), self.addr)
            try:
                reply = self.sock.recvfrom(64)[0].decode(errors="replace").strip()
            except (socket.timeout, ConnectionResetError):
                continue
            name, _, angle = reply.partition(",")
            if name == want_state and angle.strip() == str(want_angle):
                return True, reply
            time.sleep(0.2)
        return False, reply

    def close(self) -> None:
        self.sock.close()


def _blink(vehicle: Any) -> None:
    """Flash the drone's LEDs three times as the 'recorded' signal (real drone only)."""
    cf = getattr(vehicle, "cf", None)
    if cf is None:
        return

    def run() -> None:
        try:
            for _ in range(3):
                cf.param.set_value("led.bitmask", 255)
                time.sleep(0.15)
                cf.param.set_value("led.bitmask", 0)
                time.sleep(0.15)
        except Exception:  # noqa: BLE001 - a missing LED parameter only loses the signal
            pass

    threading.Thread(target=run, daemon=True).start()


def _key() -> str:
    """One line from the keyboard if one is waiting (POSIX only), else ''."""
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):   # Windows consoles: keys not available, auto-capture still works
        return ""
    if not ready:
        return ""
    line = sys.stdin.readline()
    return line.strip().lower() or "enter"


def _fresh(s: Dict[str, Any], now: float) -> bool:
    if "x" not in s or now - s.get("t", 0.0) > FRESH_S:
        return False
    lh = s.get("lh_status")
    return lh is None or lh == LH_TO_ESTIMATOR


def write_results(run_dir: str, points: Dict[str, Dict[str, Any]]) -> str:
    derived, warnings = derive(points)
    with open(os.path.join(run_dir, "points.json"), "w", encoding="utf-8") as f:
        json.dump(dict(points=points, derived=derived, warnings=warnings), f, indent=2)
    with open(os.path.join(run_dir, "points.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["point", "x", "y", "z", "spread_m", "samples", "how"])
        for k, v in points.items():
            w.writerow([k] + [f"{c:.3f}" for c in v["pos"]] + [f"{v['spread_m']:.3f}", v["samples"], v["how"]])
    lines = ["# Measured points", "", "| point | x | y | z | spread (m) |", "|---|---|---|---|---|"]
    lines += [f"| {k} | {v['pos'][0]:.3f} | {v['pos'][1]:.3f} | {v['pos'][2]:.3f} | {v['spread_m']:.3f} |"
              for k, v in points.items()]
    lines += ["", "## Derived (to be checked)", "", "```", json.dumps(derived, indent=2), "```", ""]
    lines += ["## Warnings", ""] + ([f"- {w}" for w in warnings] or ["- none"])
    text = "\n".join(lines) + "\n"
    with open(os.path.join(run_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write(text)
    return text


def main(argv=None) -> int:
    from .c1_common import DEFAULT_URI, new_run_dir

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated drone (dry run; it never moves)")
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--arduino", default=os.environ.get("AGRI_ARDUINO_IP", ""), metavar="IP",
                    help="panel Arduino address; without it, set the panel by hand when asked")
    ap.add_argument("--start", default="", help="start at this point name (to redo a part)")
    ap.add_argument("--out", default=os.path.join("data", "measure"))
    a = ap.parse_args(argv)

    run_dir = new_run_dir(a.out, "sim" if a.sim else "")
    if a.sim:
        from .sim import SimVehicle

        vehicle = SimVehicle().open()
    else:
        from .vehicle import CrazyflieVehicle

        vehicle = CrazyflieVehicle(a.uri).open()
        vehicle.start_logging()
        time.sleep(1.5)
    panel = PanelLink(a.arduino) if a.arduino else None
    det = StillDetector()
    points: Dict[str, Dict[str, Any]] = {}
    idx = next((i for i, p in enumerate(POINTS) if p[0] == a.start), 0)
    entered = -1
    t_status = 0.0
    try:
        while idx < len(POINTS):
            key, what, cmd = POINTS[idx]
            if entered != idx:
                print(f"\n[{idx + 1}/{len(POINTS)}] {key}: {what}")
                if cmd and panel:
                    ok, reply = panel.set(cmd)
                    print(f"  panel {cmd}: {'confirmed' if ok else 'NOT confirmed'} ({reply or 'no reply'})")
                elif cmd:
                    print(f"  (no --arduino: put the panel in position yourself for this point)")
                print("  Keep the drone still for 2.5 s; the LEDs flash when it is recorded.")
                det.reset()
                entered = idx
            now = time.time()
            s = vehicle.latest()
            if _fresh(s, now):
                det.add(now, (s["x"], s["y"], s["z"]))
            else:
                det.reset()
                if now - t_status > 2.0:
                    print("  waiting for a fresh Lighthouse position (keep the deck uncovered, in view of the base stations)")
                    t_status = now
            k = _key()
            if k == "q":
                break
            if k == "s":
                print("  skipped")
                idx += 1
                continue
            if k == "b":
                idx = max(0, idx - 1)
                points.pop(POINTS[idx][0], None)
                det.last = None
                entered = -1
                continue
            pos = det.check()
            how = "auto"
            if pos is None and k == "enter" and det.mean() is not None:
                pos, how = det.mean(), "enter"
            if pos is not None:
                points[key] = dict(pos=[round(c, 4) for c in pos], spread_m=round(det.spread(), 4),
                                   samples=len(det.buf), how=how, t=round(now, 3))
                print(f"  recorded {key}: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} ({how})")
                _blink(vehicle)
                print("\a", end="", flush=True)
                det.accept(pos)
                idx += 1
                continue
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        if panel:
            ok, reply = panel.set("Done")
            print(f"Panel back to normal: {'confirmed' if ok else 'NOT confirmed'} ({reply or 'no reply'})")
            panel.close()
        vehicle.close()
        print("\n" + write_results(run_dir, points))
        print(f"Saved to {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
