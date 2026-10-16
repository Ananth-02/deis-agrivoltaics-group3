"""C1 analysis: score one or more recorded runs against the TG2 drone check.

C1 pass criterion (TG1 plan): 10 scripted straight passes at constant speed,
frames saved at >= 20 fps, no crashes, battery log.

"Constant speed" and "straight" are made measurable here (our own thresholds,
printed in the summary so they can be changed and defended):
    steady part of each pass = pass minus the ramp-up and braking time,
    mean along-track speed within +-speed_tol of the command,
    coefficient of variation of speed <= cv_max,
    lateral deviation (RMS) <= lateral_rms_max_m.
Pose comes from the Flow deck state estimate, which drifts; C2 (overhead
camera) is the independent check of the same passes.
"""
from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional

import numpy as np

from ..link.eventlog import EventLog
from .flightlog import SUP_IS_CRASHED, SUP_IS_TUMBLED, load_flight_log


@dataclass
class C1Criteria:
    n_passes: int = 10
    fps_min: float = 20.0
    speed_tol: float = 0.15
    cv_max: float = 0.15
    lateral_rms_max_m: float = 0.10
    max_tilt_deg: float = 45.0
    battery_gap_max_s: float = 1.0


def load_frames_index(frames_dir: str) -> List[float]:
    p = os.path.join(frames_dir, "index.csv")
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return [float(r["t"]) for r in csv.DictReader(f)]


def _pairs(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    starts = {e["i"]: e for e in events if e["ev"] == "pass_start"}
    ends = {e["i"]: e for e in events if e["ev"] == "pass_end"}
    return [dict(start=starts[i], end=ends[i]) for i in sorted(starts) if i in ends]


def analyse_run(run_dir: str, crit: Optional[C1Criteria] = None) -> Dict[str, Any]:
    crit = crit or C1Criteria()
    events = EventLog.load(os.path.join(run_dir, "events.jsonl"))
    log = {k: np.asarray(v) for k, v in load_flight_log(os.path.join(run_dir, "flight_log.csv")).items()}
    frames_t = np.asarray(load_frames_index(os.path.join(run_dir, "frames")))
    cfg = next((e.get("config", {}) for e in events if e["ev"] == "mission_start"), {})
    simulated = False
    cfg_path = os.path.join(run_dir, "config.json")
    if os.path.exists(cfg_path):
        with open(cfg_path, encoding="utf-8") as f:
            simulated = bool(json.load(f).get("args", {}).get("sim", False))
    hover = next((e for e in events if e["ev"] == "hover_ok"), None)
    y_line = float(hover["y"]) if hover and hover.get("y") is not None else float(np.nanmean(log["y"][:20]))
    speed_cmd = float(cfg.get("speed_m_s", 0.2))
    accel = float(cfg.get("accel_m_s2", 0.4))
    t_ramp = speed_cmd / accel + 0.3
    t = log["t"]

    passes = []
    for p in _pairs(events):
        t0, t1 = p["start"]["t"], p["end"]["t"]
        d = float(p["start"].get("direction", 1))
        m = (t >= t0) & (t <= t1)
        ms = (t >= t0 + t_ramp) & (t <= t1 - t_ramp)
        r: Dict[str, Any] = dict(i=p["start"]["i"], direction=int(d), t0=t0, t1=t1, duration_s=t1 - t0,
                                 end_reason=p["end"].get("reason", ""))
        if ms.sum() >= 3:
            v = d * log["vx"][ms]
            r["speed_mean"] = float(np.nanmean(v))
            r["speed_std"] = float(np.nanstd(v))
            r["speed_cv"] = r["speed_std"] / r["speed_mean"] if r["speed_mean"] > 1e-6 else float("inf")
            tt, xx = t[ms], log["x"][ms]
            r["speed_from_x"] = float(d * (xx[-1] - xx[0]) / (tt[-1] - tt[0])) if tt[-1] > tt[0] else float("nan")
            r["steady_s"] = float(tt[-1] - tt[0])
        else:
            r.update(speed_mean=float("nan"), speed_std=float("nan"), speed_cv=float("inf"),
                     speed_from_x=float("nan"), steady_s=0.0)
        lat = log["y"][m] - y_line
        r["lateral_rms_m"] = float(np.sqrt(np.nanmean(lat ** 2))) if m.any() else float("nan")
        r["lateral_max_m"] = float(np.nanmax(np.abs(lat))) if m.any() else float("nan")
        r["height_mean_m"] = float(np.nanmean(log["z"][m])) if m.any() else float("nan")
        r["height_std_m"] = float(np.nanstd(log["z"][m])) if m.any() else float("nan")
        r["distance_m"] = float(d * (log["x"][m][-1] - log["x"][m][0])) if m.sum() >= 2 else 0.0
        ft = frames_t[(frames_t >= t0) & (frames_t <= t1)] if frames_t.size else frames_t
        r["frames"] = int(ft.size)
        r["fps"] = float((ft.size - 1) / (ft[-1] - ft[0])) if ft.size >= 2 and ft[-1] > ft[0] else 0.0
        r["max_frame_gap_s"] = float(np.max(np.diff(ft))) if ft.size >= 2 else float("nan")
        vb = log["vbat"][m]
        r["vbat_start"] = float(vb[0]) if vb.size else float("nan")
        r["vbat_end"] = float(vb[-1]) if vb.size else float("nan")
        tilt = np.nanmax(np.maximum(np.abs(log["roll"][m]), np.abs(log["pitch"][m]))) if m.any() else 0.0
        sup = log["sup_info"][m]
        sup = sup[~np.isnan(sup)].astype(int) if sup.size else sup
        r["max_tilt_deg"] = float(tilt)
        r["crash_flag"] = bool(tilt > crit.max_tilt_deg or (sup.size and np.any(sup & (SUP_IS_TUMBLED | SUP_IS_CRASHED))))
        r["speed_ok"] = bool(abs(r["speed_mean"] - speed_cmd) <= crit.speed_tol * speed_cmd and r["speed_cv"] <= crit.cv_max)
        r["straight_ok"] = bool(r["lateral_rms_m"] <= crit.lateral_rms_max_m)
        r["fps_ok"] = bool(r["fps"] >= crit.fps_min)
        r["pass_ok"] = bool(r["end_reason"] == "reached" and r["speed_ok"] and r["straight_ok"] and not r["crash_flag"])
        passes.append(r)

    # battery log quality over the whole flight
    t_air0 = next((e["t"] for e in events if e["ev"] == "takeoff"), t[0] if t.size else 0.0)
    t_air1 = next((e["t"] for e in events if e["ev"] in ("landed", "mission_end")), t[-1] if t.size else 0.0)
    ma = (t >= t_air0) & (t <= t_air1)
    vb_t = t[ma][~np.isnan(log["vbat"][ma])]
    batt = dict(samples=int(vb_t.size),
                max_gap_s=float(np.max(np.diff(vb_t))) if vb_t.size >= 2 else float("inf"),
                vbat_takeoff=float(log["vbat"][ma][0]) if ma.any() else float("nan"),
                vbat_landing=float(log["vbat"][ma][-1]) if ma.any() else float("nan"),
                flight_s=float(t_air1 - t_air0))
    batt["ok"] = bool(batt["samples"] > 0 and batt["max_gap_s"] <= crit.battery_gap_max_s)

    aborts = [e for e in events if e["ev"] == "abort"]
    end = next((e for e in events if e["ev"] == "mission_end"), {})
    fps_all = float((frames_t.size - 1) / (frames_t[-1] - frames_t[0])) if frames_t.size >= 2 else 0.0
    return dict(run_dir=run_dir, simulated=simulated, config=cfg, passes=passes, battery=batt, aborts=aborts,
                landed=any(e["ev"] == "landed" for e in events), fps_overall=fps_all,
                frames_total=int(frames_t.size), mission_end=end)


def combine(runs: List[Dict[str, Any]], crit: Optional[C1Criteria] = None) -> Dict[str, Any]:
    crit = crit or C1Criteria()
    passes = [dict(p, run=os.path.basename(r["run_dir"])) for r in runs for p in r["passes"]]
    good = [p for p in passes if p["pass_ok"]]
    fps = [p["fps"] for p in passes]
    crashed = any(p["crash_flag"] for p in passes) or any(a.get("cut_motors") for r in runs for a in r["aborts"])
    checks = {
        "passes": dict(ok=len(good) >= crit.n_passes, value=len(good), target=crit.n_passes,
                       text=f"{len(good)} of {len(passes)} passes straight at constant speed (need {crit.n_passes})"),
        "frame_rate": dict(ok=bool(fps) and min(fps) >= crit.fps_min, value=min(fps) if fps else 0.0,
                           target=crit.fps_min,
                           text=f"lowest pass frame rate {min(fps) if fps else 0:.1f} fps (target {crit.fps_min:g})"),
        "no_crash": dict(ok=not crashed and all(r["landed"] for r in runs), value=int(crashed), target=0,
                         text="no crash, every run landed" if not crashed and all(r["landed"] for r in runs)
                         else "crash or unlanded run"),
        "battery_log": dict(ok=all(r["battery"]["ok"] for r in runs),
                            value=max(r["battery"]["max_gap_s"] for r in runs) if runs else float("inf"),
                            target=crit.battery_gap_max_s,
                            text="battery logged through every flight" if all(r["battery"]["ok"] for r in runs)
                            else "battery log has gaps"),
    }
    sp = np.array([p["speed_mean"] for p in passes], dtype=float)
    stats = dict(speed_mean=float(np.nanmean(sp)) if sp.size else float("nan"),
                 speed_cv_median=float(np.nanmedian([p["speed_cv"] for p in passes])) if passes else float("nan"),
                 lateral_rms_median_m=float(np.nanmedian([p["lateral_rms_m"] for p in passes])) if passes else float("nan"),
                 fps_median=float(np.median(fps)) if fps else 0.0)
    return dict(criteria=asdict(crit), checks=checks, c1_pass=all(c["ok"] for c in checks.values()),
                simulated=any(r.get("simulated") for r in runs),
                stats=stats, passes=passes, runs=[{k: v for k, v in r.items() if k != "passes"} for r in runs])


def _clean(o: Any) -> Any:
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_clean(v) for v in o]
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    return o


def write_summary(summary: Dict[str, Any], out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "c1_summary.json"), "w", encoding="utf-8") as f:
        json.dump(_clean(summary), f, indent=2)
    cols = ["run", "i", "direction", "end_reason", "duration_s", "distance_m", "speed_mean", "speed_cv",
            "speed_from_x", "lateral_rms_m", "height_mean_m", "frames", "fps", "max_frame_gap_s",
            "vbat_start", "vbat_end", "max_tilt_deg", "pass_ok"]
    with open(os.path.join(out_dir, "c1_passes.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for p in summary["passes"]:
            w.writerow([f"{p[c]:.3f}" if isinstance(p[c], float) else p[c] for c in cols])
    sim = "SIMULATED DATA, not a hardware result. " if summary.get("simulated") else ""
    lines = ["# C1 drone check", "", f"{sim}Result: {'PASS' if summary['c1_pass'] else 'NOT PASSED'}", ""]
    for k, c in summary["checks"].items():
        lines.append(f"- {k}: {'ok' if c['ok'] else 'FAIL'}, {c['text']}")
    lines += ["", "| Pass | Dir | Speed (m/s) | CV | Lateral RMS (m) | Height (m) | Frames | fps | Vbat start-end (V) | OK |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for p in summary["passes"]:
        lines.append(f"| {p['i'] + 1} | {'+x' if p['direction'] > 0 else '-x'} | {p['speed_mean']:.3f} | "
                     f"{p['speed_cv']:.2f} | {p['lateral_rms_m']:.3f} | {p['height_mean_m']:.2f} | {p['frames']} | "
                     f"{p['fps']:.1f} | {p['vbat_start']:.2f}-{p['vbat_end']:.2f} | {'yes' if p['pass_ok'] else 'no'} |")
    with open(os.path.join(out_dir, "c1_summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def plot_dashboard(runs: List[Dict[str, Any]], summary: Dict[str, Any], out_dir: str) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    crit = summary["criteria"]
    t_base = None
    for r in runs:
        log = {k: np.asarray(v) for k, v in load_flight_log(os.path.join(r["run_dir"], "flight_log.csv")).items()}
        t_base = log["t"][0] if t_base is None else t_base
        tt = log["t"] - t_base
        for p in r["passes"]:
            m = (log["t"] >= p["t0"]) & (log["t"] <= p["t1"])
            ax[0, 0].plot(log["x"][m], log["y"][m], lw=1)
            ax[0, 1].axvspan(p["t0"] - t_base, p["t1"] - t_base, color="0.92")
            ax[1, 1].axvspan(p["t0"] - t_base, p["t1"] - t_base, color="0.92")
        speed = np.hypot(log["vx"], log["vy"])
        ax[0, 1].plot(tt, speed, lw=0.8, color="tab:blue")
        ax[1, 1].plot(tt, log["vbat"], lw=0.8, color="tab:green")
    v_cmd = runs[0]["config"].get("speed_m_s", 0.2) if runs else 0.2
    ax[0, 0].set(title="Pass tracks (Flow deck estimate)", xlabel="x (m)", ylabel="y (m)")
    ax[0, 0].set_aspect("equal", adjustable="datalim")
    ax[0, 1].axhline(v_cmd, ls="--", color="k", lw=0.8)
    ax[0, 1].set(title="Ground speed, passes shaded", xlabel="time (s)", ylabel="speed (m/s)")
    ps = summary["passes"]
    ax[1, 0].bar([p["i"] + 1 for p in ps], [p["fps"] for p in ps], color=["tab:green" if p["fps_ok"] else "tab:red" for p in ps])
    ax[1, 0].axhline(crit["fps_min"], ls="--", color="k", lw=0.8)
    ax[1, 0].set(title="Saved frame rate per pass", xlabel="pass", ylabel="frames per second")
    ax[1, 1].set(title="Battery voltage (pm.vbat)", xlabel="time (s)", ylabel="V")
    res = "PASS" if summary["c1_pass"] else "NOT PASSED"
    sim = "SIMULATED, " if summary.get("simulated") else ""
    fig.suptitle(f"C1 drone check ({sim}not for results): {res}," if sim else f"C1 drone check: {res}, {summary['checks']['passes']['value']}/{crit['n_passes']} passes ok, "
                 f"median {summary['stats']['fps_median']:.1f} fps")
    fig.tight_layout()
    path = os.path.join(out_dir, "c1_dashboard.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def contact_sheet(runs: List[Dict[str, Any]], out_dir: str, cols: int = 5) -> Optional[str]:
    """One frame from the middle of each pass, for the slides."""
    import cv2

    tiles = []
    for r in runs:
        idx_path = os.path.join(r["run_dir"], "frames", "index.csv")
        if not os.path.exists(idx_path):
            continue
        with open(idx_path, newline="") as f:
            rows = list(csv.DictReader(f))
        for p in r["passes"]:
            tm = 0.5 * (p["t0"] + p["t1"])
            if not rows:
                continue
            best = min(rows, key=lambda q: abs(float(q["t"]) - tm))
            img = cv2.imread(os.path.join(r["run_dir"], "frames", best["file"]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            img = cv2.resize(img, (324, int(324 * img.shape[0] / img.shape[1])))
            cv2.putText(img, f"pass {p['i'] + 1}", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
            tiles.append(img)
    if not tiles:
        return None
    h = max(t.shape[0] for t in tiles)
    tiles = [np.pad(t, ((0, h - t.shape[0]), (0, 0))) for t in tiles]
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows_img = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    path = os.path.join(out_dir, "c1_frames.png")
    cv2.imwrite(path, np.vstack(rows_img))
    return path
