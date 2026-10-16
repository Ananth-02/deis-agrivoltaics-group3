"""C4 runner: link reliability and event latency over a loss sweep.

    python -m agri_loop.link.c4_run --loss 0 0.05 0.1 0.2 --minutes 10 --passes 20

Writes data/c4/<timestamp>/{summary.json, summary.csv, latency.png, log_loss<x>.jsonl}.
This is the simulated harness. For the real measurement run the same
components over UdpPort (udp_nodes.py) or the ROS2 nodes, with tc netem on the
link, and analyse the logs with `python -m agri_loop.link.c4_report <log.jsonl>`.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import time

from .scenario import ScenarioConfig, run_scenario


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--loss", type=float, nargs="+", default=[0.0, 0.05, 0.10, 0.20])
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--passes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="data/c4")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    out = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    rows, results = [], []
    for loss in a.loss:
        sc = ScenarioConfig(duration_s=a.minutes * 60, n_passes=a.passes, loss=loss, seed=a.seed)
        r = run_scenario(sc)
        s = r["summary"]
        r["log"].save(os.path.join(out, f"log_loss{int(loss * 100):02d}.jsonl"))
        results.append(s)
        b, h, st, ev = s["streams"]["DroneBeacon"], s["streams"]["Heartbeat"], s["streams"]["PanelState"], s["events"]
        rows.append({
            "loss_pct": loss * 100,
            "beacon_pdr": round(b["pdr"], 4), "beacon_lat_ms_p95": round(b["lat_ms_p95"], 1),
            "heartbeat_pdr": round(h["pdr"], 4), "state_pdr": round(st["pdr"], 4),
            "events_reached": ev["events_reached"], "events_failed": ev["events_failed"],
            "retransmissions": ev["retransmissions"],
            "event_lat_ms_median": round(ev["latency_ms_median"], 0) if ev["latency_ms_median"] else None,
            "event_lat_ms_p95": round(ev["latency_ms_p95"], 0) if ev["latency_ms_p95"] else None,
            "timing_success": s["timing_success"], "faults": ev["faults"],
        })
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump([{k: v for k, v in s.items()} for s in results], f, indent=1, default=lambda o: None)
    with open(os.path.join(out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"{'loss%':>6} {'beaconPDR':>10} {'hbPDR':>7} {'statePDR':>9} {'ev ok/fail':>11} {'retx':>5} "
          f"{'lat med/p95 ms':>15} {'timing ok':>10}")
    for r in rows:
        print(f"{r['loss_pct']:>6.0f} {r['beacon_pdr']:>10.3f} {r['heartbeat_pdr']:>7.3f} {r['state_pdr']:>9.3f} "
              f"{str(r['events_reached']) + '/' + str(r['events_failed']):>11} {r['retransmissions']:>5} "
              f"{str(r['event_lat_ms_median']) + '/' + str(r['event_lat_ms_p95']):>15} {r['timing_success']:>10.2f}")

    if not a.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
        for s in results:
            lat = s["streams"]["DroneBeacon"]["lat_ms"]
            ax[0].hist(lat, bins=40, histtype="step", label=f"loss {s['loss'] * 100:.0f}%")
            ev = s["events"]["latency_ms"]
            ax[1].hist(ev, bins=20, histtype="step", label=f"loss {s['loss'] * 100:.0f}%")
        ax[0].set_xlabel("beacon one-way latency (ms)"); ax[0].set_ylabel("count"); ax[0].legend()
        ax[1].set_xlabel("event to angle reached (ms)"); ax[1].axvline(2000, ls="--", c="k", lw=0.8)
        ax[1].set_title("dashed: 2 s requirement", fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "latency.png"), dpi=130)
    print("written to", out)


if __name__ == "__main__":
    main()
