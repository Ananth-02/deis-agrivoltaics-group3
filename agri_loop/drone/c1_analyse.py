"""C1 step 3: score one or several runs together (for example two batteries of 5 passes).

    python -m agri_loop.drone.c1_analyse data/c1/20261019_101500_bat1 data/c1/20261019_103000_bat2
    python -m agri_loop.drone.c1_analyse data/c1/<run> --fps-min 5
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from .c1_analysis import C1Criteria, analyse_run, combine, contact_sheet, plot_dashboard, write_summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run folders from c1_run")
    ap.add_argument("--passes", type=int, default=10, help="passes needed in total")
    ap.add_argument("--fps-min", type=float, default=20.0)
    ap.add_argument("--speed-tol", type=float, default=0.15, help="allowed mean speed error, fraction of command")
    ap.add_argument("--cv-max", type=float, default=0.15, help="allowed speed coefficient of variation")
    ap.add_argument("--lateral-max", type=float, default=0.10, help="allowed lateral RMS, m")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)

    crit = C1Criteria(n_passes=a.passes, fps_min=a.fps_min, speed_tol=a.speed_tol, cv_max=a.cv_max,
                      lateral_rms_max_m=a.lateral_max)
    runs = [analyse_run(r, crit) for r in a.runs]
    summary = combine(runs, crit)
    out = a.out or (a.runs[0] if len(a.runs) == 1 else os.path.join("data", "c1", "summary_" + time.strftime("%Y%m%d_%H%M%S")))
    write_summary(summary, out)
    plot_dashboard(runs, summary, out)
    contact_sheet(runs, out)
    print(open(os.path.join(out, "c1_summary.md"), encoding="utf-8").read())
    print(f"Written to {out}")
    return 0 if summary["c1_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
