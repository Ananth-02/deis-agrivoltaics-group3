"""C1 step 1: connect, check decks, battery and logging, and measure the camera stream. Motors stay off.

    python -m agri_loop.drone.c1_preflight                      # radio + decks + battery
    python -m agri_loop.drone.c1_preflight --stream 10          # also record 10 s from the AI deck
    python -m agri_loop.drone.c1_preflight --stream 10 --no-radio   # camera only (laptop on the AI deck Wi-Fi)
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from .c1_common import DEFAULT_URI, new_run_dir, preflight, print_report, save_json, stream_check
from .cpx_stream import DEFAULT_IP, DEFAULT_PORT, FrameRecorder
from .mission import C1Config


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--no-radio", action="store_true", help="skip the Crazyradio checks")
    ap.add_argument("--stream", type=float, default=0.0, metavar="S", help="record S seconds from the AI deck")
    ap.add_argument("--aideck-ip", default=DEFAULT_IP)
    ap.add_argument("--aideck-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--flip", default="none", choices=["none", "h", "v", "hv"])
    ap.add_argument("--out", default=os.path.join("data", "c1"))
    a = ap.parse_args(argv)

    run_dir = new_run_dir(a.out, "preflight")
    result = {}
    ok = True
    if not a.no_radio:
        from .vehicle import CrazyflieVehicle

        v = CrazyflieVehicle(a.uri)
        try:
            v.open()
            v.start_logging()
            time.sleep(1.5)
            ok, report, problems = preflight(v, C1Config(), need_camera=a.stream > 0)
            result["drone"] = report
            print_report("Drone", report)
        finally:
            v.close()
    if a.stream > 0:
        rec = FrameRecorder(os.path.join(run_dir, "frames"), a.aideck_ip, a.aideck_port, flip=a.flip).start()
        st = stream_check(rec, a.stream)
        stats = rec.stop()
        st.update(dropped=stats.dropped, decode_errors=stats.decode_errors)
        result["stream"] = st
        print_report("AI deck stream", st)
        ok = ok and st["ok"]
    save_json(os.path.join(run_dir, "preflight.json"), result)
    print(f"\nPre-flight {'OK' if ok else 'FAILED'}. Saved to {run_dir}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
