"""One command for a whole autonomous test: fly passes over the plant while the panel tilts out of the way.

    python -m agri_loop.drone.full_pass --points data/measure/<run>/points.json --arduino 192.168.1.19
    python -m agri_loop.drone.full_pass --points PATH/points.json --sim --fake-arduino --yes   # dry run
    python -m agri_loop.drone.full_pass --arduino 192.168.1.19 --zone -1.15 -0.85 -0.8 -0.4 \\
        --height 0.84 --direction -1 --length 2.0                                              # no points file

The plan (zone, height, direction, length) comes from the measured points and can be overridden by hand.
Heights are above the floor reading; the drone flies at floor z + height. The plan is printed first; press
Enter to go. Then this script starts the gateway and the panel adapter, flies with c1_run (take-off at the
current position, which must be the planned take-off spot, passes along the x axis towards the plant, the
camera records, the beacon feeds the gateway), lands, stops the adapter (which sends Done) and the gateway,
and runs the C4 report. Everything goes to data/full_pass/<time>/. Ctrl+C (or SIGTERM) lands the drone
(c1_run handles it). The exit code is non-zero if the flight did not complete.
On the Pi, start it through scripts/pi/full_pass.sh so a dropped SSH session does not stop the flight.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Sequence

from ..link.gateway import GatewayConfig

_GW = GatewayConfig()
FOOTPRINT_M = _GW.footprint_half            # camera footprint half-size used by the gateway (m)
TRIGGER_MARGIN_M = _GW.trigger_margin
# The pass ends this far past the zone, beyond the footprint: the drone stops a few cm short of the end,
# and the gateway only releases the panel once the footprint is release_margin clear of the zone.
END_MARGIN_M = _GW.release_margin + 0.10
ZONE_FALLBACK_M = 0.15  # zone half-size around the pot when the panel points are missing
MAX_TAKEOFF_Y_OFFSET_M = 0.10
CLEARANCE_M = 0.30      # flight height above the highest part of the rig
MAX_FLY_Z_M = 1.2       # never fly higher than this (net, base station view)
MAX_FLOOR_Z_M = 0.10     # the floor must read about 0 in the Lighthouse frame
GEOFENCE_MARGIN_M = 0.10  # tight mission geofence around the pass line
SETTLE_S = 3.0          # after landing: let the gateway release the panel


class PlanError(Exception):
    pass


def _pos(points: Dict[str, Any], name: str) -> Optional[List[float]]:
    p = (points.get("points") or {}).get(name)
    return list(p["pos"]) if p else None


def _arg(args: Any, name: str, default: Any = None) -> Any:
    v = getattr(args, name, None)
    return default if v is None else v


def _rig_top(points: Dict[str, Any], derived: Dict[str, Any], floor: float) -> Optional[float]:
    """Highest part of the rig (leaf, posts, vertical panel top) above the floor reading."""
    if derived.get("rig_top_m") is not None:
        return float(derived["rig_top_m"])
    zs = [q[2] for q in (_pos(points, k) for k in ("leaf", "post_a", "post_b", "panel_top_vertical")) if q]
    return max(zs) - floor if zs else None


def plan(points: Optional[Dict[str, Any]], args: Any) -> Dict[str, Any]:
    """Turn the measured points (and manual overrides in args) into the numbers for one test. Pure function."""
    points = points or {}
    derived = points.get("derived") or {}
    warnings: List[str] = []
    speed, lead_s = float(_arg(args, "speed", 0.2)), float(_arg(args, "lead_s", 2.2))
    fp = float(_arg(args, "footprint", FOOTPRINT_M))
    fov = _arg(args, "fov_deg")
    if fov is not None and not 0 < float(fov) < 180:
        raise PlanError(f"--fov-deg {fov} must be between 0 and 180")
    fp_note = f"assumed {fp:.2f} m (give --fov-deg to size it from the geometry)"
    passes = int(_arg(args, "passes", 2))
    home_tol = float(_arg(args, "home_tol", 0.10))
    if not speed > 0:
        raise PlanError("the speed must be above 0")
    if passes < 2 or passes % 2:
        raise PlanError(f"--passes {passes}: use an even number (2 or more) so the drone lands back at the "
                        "take-off spot")
    takeoff, pot = _pos(points, "takeoff"), _pos(points, "pot")

    # zone
    if _arg(args, "zone") is not None:
        zone = [float(v) for v in args.zone]
    else:
        ext = [q for q in (_pos(points, k) for k in ("panel_flat_strip_side", "panel_flat_far_side", "post_a", "post_b"))
               if q]
        if ext:
            zone = [min(q[0] for q in ext), max(q[0] for q in ext), min(q[1] for q in ext), max(q[1] for q in ext)]
        elif pot:
            zone = [pot[0] - ZONE_FALLBACK_M, pot[0] + ZONE_FALLBACK_M, pot[1] - ZONE_FALLBACK_M,
                    pot[1] + ZONE_FALLBACK_M]
            warnings.append(f"no panel or post points: the zone is the pot +- {ZONE_FALLBACK_M} m")
        else:
            raise PlanError("no zone: measure the panel points or the pot, or give --zone XMIN XMAX YMIN YMAX")
    zone = [round(v, 3) for v in zone]

    # height: --height and over_plant_z are above the floor reading; the drone flies at floor_z + height
    floor = float(derived.get("floor_z") or 0.0)
    if abs(floor) > MAX_FLOOR_Z_M:
        raise PlanError(f"the floor reads z = {floor:.2f} m: check the Lighthouse set-up before flying")
    height = _arg(args, "height")
    if height is None:
        height = derived.get("over_plant_z")
        if height is None:
            raise PlanError("no flight height: the points file has no over_plant_z, give --height")
    height = float(height)
    rig = _rig_top(points, derived, floor)
    if rig is not None:
        if height < rig + CLEARANCE_M - 1e-6:
            raise PlanError(f"height {height:.2f} m is below the rig top ({rig:.2f} m above the floor) + "
                            f"{CLEARANCE_M} m: the drone would hit the plant or the panel")
    else:
        warnings.append("no rig height known (no points file): the height is NOT checked against the plant")
    fly_z = round(floor + height, 3)
    if fly_z > MAX_FLY_Z_M:
        raise PlanError(f"flight height z = {fly_z:.2f} m is above the {MAX_FLY_Z_M} m limit")

    # camera patch under the drone (the camera looks straight down): half-size = height above the plant top * tan(fov/2)
    if fov is not None:
        top_pt = _pos(points, "leaf") or pot
        if top_pt is None:
            raise PlanError("--fov-deg needs the leaf or pot point to know the height above the plant")
        above = fly_z - top_pt[2]
        if above <= 0:
            raise PlanError(f"the flight height z = {fly_z:.2f} m is not above the plant top ({top_pt[2]:.2f} m)")
        fp = round(above * math.tan(math.radians(float(fov)) / 2), 2)
        fp_note = f"{fp:.2f} m = {above:.2f} m above the plant top x tan({float(fov):g} deg / 2)"
    # direction and length
    zc = (zone[0] + zone[1]) / 2
    direction = _arg(args, "direction")
    if direction is None:
        if not takeoff:
            raise PlanError("no take-off point: give --direction and --length")
        direction = 1 if (pot[0] if pot else zc) >= takeoff[0] else -1
    if direction not in (1, -1):
        raise PlanError(f"direction must be 1 or -1, not {direction}")
    direction = int(direction)
    near, far = (zone[0], zone[1]) if direction > 0 else (zone[1], zone[0])
    approach = speed * lead_s + fp + TRIGGER_MARGIN_M     # the tilt starts this far before the zone
    length = _arg(args, "length")
    end_margin = fp + END_MARGIN_M
    if takeoff:
        dist_near, dist_far = direction * (near - takeoff[0]), direction * (far - takeoff[0])
        if dist_far <= 0:
            raise PlanError("the zone is not ahead of the take-off spot in this direction: "
                            "check --direction or the take-off point")
        zx0, zx1, zy0, zy1 = zone[0] - fp, zone[1] + fp, zone[2] - fp, zone[3] + fp
        if zx0 <= takeoff[0] <= zx1 and zy0 <= takeoff[1] <= zy1:
            raise PlanError("the take-off spot is inside the zone grown by the camera patch: the climb would go "
                            "through the rig")
        need = dist_far + end_margin
        fence = (derived.get("fence") or {}).get("x")
        room = None
        if fence:
            room = (fence[1] - takeoff[0]) if direction > 0 else (takeoff[0] - fence[0])
            room -= home_tol    # the drone may start up to home_tol from the planned spot
        if length is None:
            length = need if room is None else min(need, room)
        length = float(length)
        if length <= 0:
            raise PlanError(f"the pass length {length:.2f} m must be above 0")
        if room is not None and length > room + 1e-9:
            raise PlanError(f"the pass of {length:.2f} m ends beyond the fence (room {room:.2f} m)")
        if length < need - 1e-9:
            raise PlanError(f"the pass cannot get past the zone: it needs {need:.2f} m (zone far edge + footprint + "
                            f"{END_MARGIN_M:.2f} m) but is {length:.2f} m"
                            + (f", the fence allows {room:.2f} m" if room is not None else ""))
        if dist_near < approach:
            warnings.append(f"the approach distance {approach:.2f} m (speed x lead + footprint) does not fit before "
                            f"the zone ({dist_near:.2f} m from the take-off spot): the tilt starts as the pass starts")
        runway = length - dist_far - fp     # return passes: start-to-footprint-contact distance
        if passes > 1 and runway < speed * lead_s:
            warnings.append(f"return passes start only {runway:.2f} m before the camera footprint reaches the zone, "
                            f"the panel needs about {speed * lead_s:.2f} m of travel (speed x lead): it may still "
                            "be moving when the camera reaches the zone")
        if pot and abs(takeoff[1] - pot[1]) > MAX_TAKEOFF_Y_OFFSET_M:
            warnings.append(f"the take-off spot is {abs(takeoff[1] - pot[1]):.2f} m off the pot in y: the passes "
                            "keep the take-off y and will not cross the plant")
    else:
        if length is None:
            raise PlanError("no take-off point: give --length")
        length = float(length)
        if length <= 0:
            raise PlanError(f"the pass length {length:.2f} m must be above 0")
        warnings.append("no take-off point known: the pass length is NOT checked against the zone or the fence")
    return dict(zone=zone, height=round(height, 3), floor_z=round(floor, 3), fly_z=fly_z, direction=direction,
                length=round(length, 3), speed=speed, lead_s=lead_s, footprint=fp, passes=passes,
                approach_m=round(approach, 3), footprint_note=fp_note, takeoff=takeoff, pot=pot, warnings=warnings)


def print_plan(p: Dict[str, Any]) -> None:
    z = p["zone"]
    print("\n== Plan ==")
    print(f"  zone x {z[0]:.2f} to {z[1]:.2f}, y {z[2]:.2f} to {z[3]:.2f} m")
    print(f"  height {p['height']:.2f} m above the floor reading ({p['floor_z']:.2f} m): flying at absolute "
          f"z = {p['fly_z']:.2f} m")
    print(f"  direction {'+x' if p['direction'] > 0 else '-x'}, pass length {p['length']:.2f} m, "
          f"{p['passes']} passes at {p['speed']} m/s")
    print(f"  camera patch under the drone: half-size {p['footprint']:.2f} m, {p['footprint_note']}")
    print(f"  the tilt starts {p['approach_m']:.2f} m before the zone (speed x {p['lead_s']} s + footprint)")
    for w in p["warnings"]:
        print(f"  WARNING: {w}")


def to_sim_frame(p: Dict[str, Any]) -> Dict[str, Any]:
    """The simulator takes off at (0, 0): move the zone so the geometry relative to the take-off spot is kept."""
    if not p.get("takeoff"):
        return p
    tx, ty = p["takeoff"][0], p["takeoff"][1]
    z = p["zone"]
    return dict(p, zone=[round(z[0] - tx, 3), round(z[1] - tx, 3), round(z[2] - ty, 3), round(z[3] - ty, 3)])


# ---- running ----
def _spawn(cmd: Sequence[str], **kw) -> subprocess.Popen:
    """Own process group: Ctrl+C in this console goes to c1_run only, we stop these ourselves."""
    if os.name == "nt":
        kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kw["start_new_session"] = True
    return subprocess.Popen(list(cmd), **kw)


def _stop(proc: subprocess.Popen, name: str, wait_s: float = 8.0) -> None:
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
        proc.wait(timeout=wait_s)
    except (subprocess.TimeoutExpired, OSError, KeyboardInterrupt):
        print(f"{name} did not stop, terminating it")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def check_arduino(ip: str) -> str:
    """Why the real panel cannot be used ('' when fine)."""
    from .measure_points import PanelLink, other_panel_senders

    busy = other_panel_senders()
    if busy:
        return f"{', '.join(busy)} is running and would undo the panel commands: stop it first"
    link = PanelLink(ip)
    try:
        reply = link.ping()
    finally:
        link.close()
    return "" if reply else f"the Arduino at {ip} does not answer (check its Wi-Fi, IP address and ping)"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--points", default="", help="points.json from measure_points")
    ap.add_argument("--arduino", default=os.environ.get("AGRI_ARDUINO_IP", ""), metavar="IP")
    ap.add_argument("--sim", action="store_true", help="simulated drone (implies the fake Arduino)")
    ap.add_argument("--fake-arduino", action="store_true", help="fake Arduino on localhost, real drone")
    ap.add_argument("--passes", type=int, default=2, help="even number, so the drone lands at the take-off spot")
    ap.add_argument("--speed", type=float, default=0.2, help="pass speed, m/s")
    ap.add_argument("--lead-s", type=float, default=2.2, help="how far ahead the patch under the drone is predicted (comms + panel actuation), s")
    ap.add_argument("--footprint", type=float, default=FOOTPRINT_M,
                    help="half-size of the camera patch under the drone, m (assumed; --fov-deg overrides it)")
    ap.add_argument("--fov-deg", type=float, default=None,
                    help="camera field of view across the narrower image side; sizes the patch from the geometry")
    ap.add_argument("--zone", type=float, nargs=4, default=None, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    ap.add_argument("--height", type=float, default=None, help="flight height above the floor, m")
    ap.add_argument("--direction", type=int, choices=[1, -1], default=None)
    ap.add_argument("--length", type=float, default=None, help="pass length, m")
    ap.add_argument("--home-tol", type=float, default=0.10, help="allowed distance from the planned take-off spot, m")
    ap.add_argument("--yes", action="store_true", help="do not wait for Enter")
    ap.add_argument("--out", default=os.path.join("data", "full_pass"))
    return ap


def build_commands(a: Any, p: Dict[str, Any], run: str, fake: bool) -> Dict[str, List[str]]:
    py = sys.executable
    gw = [py, "-m", "agri_loop.link.udp_nodes", "--role", "gateway", "--out", run,
          "--zone", *[str(v) for v in p["zone"]], "--lead-s", str(p["lead_s"]),
          "--footprint", str(p["footprint"]), "--angle-plant", "0"]
    pa = [py, "-m", "agri_loop.link.panel_adapter", "--out", run] + (
        ["--fake-arduino"] if fake else ["--arduino", a.arduino])
    c1 = [py, "-m", "agri_loop.drone.c1_run", "--sim" if a.sim else "--fly", "--beacon", "127.0.0.1",
          "--height", str(p["fly_z"]), "--length", str(p["length"]), "--speed", str(p["speed"]),
          "--passes", str(p["passes"]), "--geofence-margin", str(GEOFENCE_MARGIN_M), "--direction", str(p["direction"]), "--fps-min", "5",
          "--out", os.path.join(run, "c1")]
    if not a.sim:   # the real flight needs Lighthouse positions, and must start on the planned spot
        c1 += ["--control", "position"]
        if p.get("takeoff"):
            c1 += ["--expect-home", str(p["takeoff"][0]), str(p["takeoff"][1]), "--home-tol", str(a.home_tol)]
    return dict(gateway=gw, adapter=pa, c1=c1)


def main(argv=None) -> int:
    for name in ("SIGTERM", "SIGHUP"):   # a stop request behaves like Ctrl+C, so the helpers are always stopped
        if hasattr(signal, name):
            try:
                signal.signal(getattr(signal, name), signal.default_int_handler)
            except (ValueError, OSError):
                pass
    try:
        return _main(argv)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1


def _main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    points = None
    if a.points:
        with open(a.points, encoding="utf-8") as f:
            points = json.load(f)
        for w in points.get("warnings", []):
            print(f"points file warning: {w}")
    try:
        p = plan(points, a)
    except PlanError as e:
        print(f"Not flying: {e}")
        return 2
    fake = a.sim or a.fake_arduino
    if not fake and not a.arduino:
        print("Not flying: give the panel's address with --arduino IP (or use --sim / --fake-arduino).")
        return 2
    print_plan(p)
    if a.sim:
        p = to_sim_frame(p)
        print(f"  (simulator: zone moved to the take-off origin: x {p['zone'][0]:.2f} to {p['zone'][1]:.2f}, "
              f"y {p['zone'][2]:.2f} to {p['zone'][3]:.2f})")
    if not fake:
        why = check_arduino(a.arduino)
        if why:
            print(f"Not flying: {why}")
            return 2
    if not a.yes:
        try:
            input("\nPress Enter to start (Ctrl+C cancels): ")
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            return 1

    run = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(os.path.join(run, "c1"), exist_ok=True)
    with open(os.path.join(run, "plan.json"), "w", encoding="utf-8") as f:
        json.dump(dict(plan=p, args=vars(a)), f, indent=2)
    cmds = build_commands(a, p, run, fake)
    gateway = adapter = None
    rc = 1
    try:
        gateway = _spawn(cmds["gateway"])
        adapter = _spawn(cmds["adapter"])
        time.sleep(1.5)
        for proc, name in ((gateway, "gateway"), (adapter, "panel adapter")):
            if proc.poll() is not None:
                print(f"Not flying: the {name} stopped at start (exit {proc.returncode}).")
                return 2
        print(f"\nGateway and panel adapter running, flying now. Output: {run}")
        rc = run_foreground(cmds["c1"])
        for proc, name in ((gateway, "gateway"), (adapter, "panel adapter")):
            if proc.poll() is not None:
                print(f"WARNING: the {name} stopped during the flight (exit {proc.returncode})")
                rc = rc or 5
        try:
            time.sleep(SETTLE_S)
        except KeyboardInterrupt:
            print("\nSkipping the wait for the panel to release.")
    except KeyboardInterrupt:
        print("\nInterrupted before or after the flight.")
        rc = rc or 1
    finally:
        for proc, name in ((adapter, "panel adapter"), (gateway, "gateway")):   # adapter first: it sends Done
            if proc is not None:
                _stop(proc, name)
    report(run)
    if rc:
        print(f"\nThe flight did not complete cleanly (exit code {rc}).")
    return rc


def run_foreground(cmd: Sequence[str]) -> int:
    """Run c1_run on this console. Ctrl+C reaches it too (it lands the drone); we just keep waiting.
    SIGTERM or SIGHUP sent to this script is forwarded to c1_run as SIGINT, so it lands."""
    proc = None
    early = []     # a signal that arrives before c1_run exists is passed on right after the start
    old = {}

    def forward(signum, frame):  # noqa: ARG001
        if proc is None:
            early.append(signum)
            return
        try:
            proc.send_signal(signal.SIGINT)    # first: nothing before this can fail or block
            print("\nStop requested: telling c1_run to land the drone ...")
        except (OSError, ValueError):
            pass
    if os.name != "nt":
        for name in ("SIGTERM", "SIGHUP"):
            if hasattr(signal, name):
                try:
                    old[name] = signal.signal(getattr(signal, name), forward)
                except (ValueError, OSError):
                    pass
    try:
        proc = subprocess.Popen(list(cmd))
        if early:
            forward(early[0], None)
        while True:
            try:
                return proc.wait()
            except KeyboardInterrupt:
                print("\nCtrl+C: waiting for c1_run to land the drone ...")
    finally:
        for name, h in old.items():
            signal.signal(getattr(signal, name), h)


def report(run: str) -> None:
    drone = sorted(glob.glob(os.path.join(run, "c1", "*", "drone.jsonl")))
    logs = [os.path.join(run, n) for n in ("gateway.jsonl", "panel.jsonl")] + drone[-1:]
    logs = [f for f in logs if os.path.exists(f)]
    print("\n== C4 report ==")
    if logs:
        subprocess.call([sys.executable, "-m", "agri_loop.link.c4_report", *logs])
    c1_dirs = sorted(d for d in glob.glob(os.path.join(run, "c1", "*")) if os.path.isdir(d))
    print(f"\nRun folder: {run}")
    print("  plan.json, gateway.jsonl, panel.jsonl")
    if c1_dirs:
        print(f"  C1 run (frames, flight log, summary): {c1_dirs[-1]}")


if __name__ == "__main__":
    sys.exit(main())
