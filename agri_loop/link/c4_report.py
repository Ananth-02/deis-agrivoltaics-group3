"""Analyse one or more JSON-lines logs from a real run (merged by timestamp).

    python -m agri_loop.link.c4_report data/c4_real/*.jsonl
"""
from __future__ import annotations

import argparse
import glob
import json

from .eventlog import EventLog
from .monitor import event_stats, stream_stats


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json", action="store_true", help="print machine-readable output")
    a = ap.parse_args()
    events = []
    for pat in a.logs:
        for f in glob.glob(pat):
            events += EventLog.load(f)
    events.sort(key=lambda e: e["t"])
    out = {"streams": {m: stream_stats(events, m) for m in ("DroneBeacon", "Heartbeat", "PanelEvent", "PanelState")},
           "events": event_stats(events)}
    for m in out["streams"].values():
        m.pop("lat_ms", None)
    out["events"].pop("latency_ms", None)
    if a.json:
        print(json.dumps(out, indent=1))
        return
    print(f"{'stream':<12} {'sent':>6} {'recv':>6} {'PDR':>7} {'lat med ms':>11} {'p95 ms':>8}")
    for m, s in out["streams"].items():
        if s["sent"]:
            print(f"{m:<12} {s['sent']:>6} {s['received']:>6} {s['pdr']:>7.3f} {s['lat_ms_median']:>11.1f} {s['lat_ms_p95']:>8.1f}")
    e = out["events"]
    print(f"\nevents reached {e['events_reached']}, failed {e['events_failed']}, retransmissions {e['retransmissions']}, "
          f"faults {e['faults']}, stale rejected {e['stale_rejected']}")
    if e["latency_ms_median"] is not None:
        print(f"event to angle reached: median {e['latency_ms_median']:.0f} ms, p95 {e['latency_ms_p95']:.0f} ms, "
              f"max {e['latency_ms_max']:.0f} ms (requirement: 2000 ms)")


if __name__ == "__main__":
    main()
