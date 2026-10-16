"""C1 step 2: fly the scripted passes while recording frames, pose and battery, then score the run.

    python -m agri_loop.drone.c1_run --sim                      # whole chain on the simulator
    python -m agri_loop.drone.c1_run --fly                      # real drone, 10 passes
    python -m agri_loop.drone.c1_run --fly --passes 5 --label bat1   # split over two batteries

The real drone only flies with --fly, after the pre-flight checks pass and the
camera stream has delivered frames. Ctrl+C at any time lands the drone.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from ..link.eventlog import EventLog
from .c1_analysis import C1Criteria, analyse_run, combine, contact_sheet, plot_dashboard, write_summary
from .c1_common import DEFAULT_URI, new_run_dir, preflight, print_report, save_json, stream_check
from .cpx_stream import DEFAULT_IP, DEFAULT_PORT, FrameRecorder
from .flightlog import FlightLogger
from .mission import C1Config, Mission


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="simulated drone and AI deck")
    mode.add_argument("--fly", action="store_true", help="fly the real Crazyflie")
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--aideck-ip", default=DEFAULT_IP)
    ap.add_argument("--aideck-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-camera", action="store_true", help="fly without the AI deck stream")
    ap.add_argument("--debayer", action="store_true", help="only for a colour (Bayer) camera module")
    ap.add_argument("--flip", default="none", choices=["none", "h", "v", "hv"], help="flip frames (mirror mount)")
    ap.add_argument("--passes", type=int, default=10)
    ap.add_argument("--length", type=float, default=1.5, help="pass length, m")
    ap.add_argument("--speed", type=float, default=0.20, help="pass speed, m/s")
    ap.add_argument("--height", type=float, default=0.60, help="flight height, m")
    ap.add_argument("--pause", type=float, default=2.0, help="hover between passes, s")
    ap.add_argument("--min-vbat", type=float, default=3.8, help="refuse take-off below this voltage")
    ap.add_argument("--fps-min", type=float, default=20.0, help="C1 frame-rate threshold")
    ap.add_argument("--sim-fps", type=float, default=15.0, help="frame rate of the simulated AI deck")
    ap.add_argument("--beacon", default="", metavar="HOST",
                    help="also send DroneBeacon messages to the C4 gateway on HOST (UDP 5001)")
    ap.add_argument("--beacon-offset", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"),
                    help="take-off point in the bed frame, m")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default=os.path.join("data", "c1"))
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    cfg = C1Config(n_passes=a.passes, pass_length_m=a.length, speed_m_s=a.speed, height_m=a.height,
                   pause_s=a.pause, min_start_vbat=a.min_vbat)
    run_dir = new_run_dir(a.out, a.label or ("sim" if a.sim else "fly"))
    save_json(os.path.join(run_dir, "config.json"), dict(args=vars(a), mission=cfg.to_dict()))
    events = EventLog()
    logger = FlightLogger(os.path.join(run_dir, "flight_log.csv"))
    deck = None
    ip, port = a.aideck_ip, a.aideck_port

    if a.sim:
        from .sim import FakeAIDeck, SimVehicle

        vehicle = SimVehicle(logger).open()
        if not a.no_camera:
            deck = FakeAIDeck(vehicle.pose, fps=a.sim_fps).start()
            ip, port = "127.0.0.1", deck.port
    else:
        from .vehicle import CrazyflieVehicle

        vehicle = CrazyflieVehicle(a.uri, logger).open()
        vehicle.start_logging()
        time.sleep(1.5)

    recorder = None
    beacon = None
    try:
        ok, report, problems = preflight(vehicle, cfg, need_camera=not a.no_camera)
        events.add("preflight", time.time(), **report)
        print_report("Pre-flight", report)
        if not ok:
            print("Not flying: " + "; ".join(problems))
            return 2
        if not a.no_camera:
            recorder = FrameRecorder(os.path.join(run_dir, "frames"), ip, port, debayer=a.debayer, flip=a.flip).start()
            st = stream_check(recorder, 2.0)
            events.add("stream_check", time.time(), **st)
            print_report("Camera stream", st)
            if not st["ok"]:
                print("Not flying: no frames from the AI deck (use --no-camera to fly without it)")
                return 3
        if a.fly:
            print("\nTaking off in 3 s. Ctrl+C lands the drone.")
            time.sleep(3.0)
        mission = Mission(vehicle, cfg, events)
        if a.beacon:
            from .beacon import P_GW, BeaconSender

            beacon = BeaconSender(vehicle.latest, lambda: mission.in_pass, (a.beacon, P_GW),
                                  offset=tuple(a.beacon_offset)).start()
        result = mission.run()
        print_report("Mission", result)
    finally:
        if recorder is not None:
            stats = recorder.stop()
            events.add("stream_stats", time.time(), frames=stats.frames, fps=stats.fps, width=stats.width,
                       height=stats.height, format=stats.fmt, dropped=stats.dropped,
                       decode_errors=stats.decode_errors, error=stats.error)
        if deck is not None:
            deck.stop()
        if beacon is not None:
            beacon.stop(os.path.join(run_dir, "drone.jsonl"))
        logger.close()
        events.save(os.path.join(run_dir, "events.jsonl"))
        vehicle.close()

    crit = C1Criteria(n_passes=a.passes, fps_min=a.fps_min)
    run = analyse_run(run_dir, crit)
    summary = combine([run], crit)
    write_summary(summary, run_dir)
    plot_dashboard([run], summary, run_dir)
    contact_sheet([run], run_dir)
    print("\n" + open(os.path.join(run_dir, "c1_summary.md"), encoding="utf-8").read())
    print(f"Run saved to {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
