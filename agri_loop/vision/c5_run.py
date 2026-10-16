"""C5 runner: panel mask, mosaic, visible crop ratio, anomaly (dead patch) detection.

Synthetic demo (no hardware needed):
    python -m agri_loop.vision.c5_run --synthetic

Real data, three passes over the same bed (folders of frames or video files):
    python -m agri_loop.vision.c5_run --gt data/c1/gt --static data/c1/static --actuated data/c1/act

gt = no panel, static = panel always horizontal, actuated = panel moved away
while the camera passes. Use --poses to add a poses CSV later; the default
registers frames from image features only.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from typing import Dict

import cv2
import numpy as np

from .loader import load
from .mask import panel_mask
from .metrics import anomaly_mask, evaluate
from .mosaic import Mosaic, align_to, build_mosaic
from .synth import make_scene


def process(frames, label: str, **mask_kw):
    masks = [panel_mask(f, **mask_kw) for f in frames]
    mos = build_mosaic(frames, masks)
    frac = float(np.mean([(m > 0).mean() for m in masks]))
    print(f"  {label}: {len(frames)} frames, mean masked fraction {frac:.3f}, "
          f"registration failures {mos.failures}, mosaic {mos.image.shape[1]}x{mos.image.shape[0]}")
    return masks, mos


def run(runs: Dict[str, list], out_dir: str) -> Dict[str, dict]:
    os.makedirs(out_dir, exist_ok=True)
    mosaics: Dict[str, Mosaic] = {}
    masks: Dict[str, list] = {}
    for name in ("gt", "static", "actuated"):
        masks[name], mosaics[name] = process(runs[name], name)
    gt = mosaics["gt"]
    results: Dict[str, dict] = {}
    aligned = {"gt": (gt.image, gt.observed)}
    for name in ("static", "actuated"):
        al = align_to(gt, mosaics[name])
        if al is None:
            print(f"  {name}: could not align to ground truth mosaic")
            results[name] = {"error": "alignment failed"}
            continue
        aligned[name] = al
        results[name] = evaluate(al[0], al[1], gt.image, gt.observed)
    with open(os.path.join(out_dir, "c5_metrics.json"), "w") as f:
        json.dump(results, f, indent=1)

    print("\n  condition   visible crop ratio   anomaly recall   detected   patch area / GT   grey error")
    for name, r in results.items():
        if "error" in r:
            continue
        print(f"  {name:<10}  {r['visible_crop_ratio']:>17.3f}   {r['anomaly_recall']:>13.3f}   "
              f"{str(r['anomaly_detected']):>8}   {r['anomaly_area_ratio']:>14.2f}   {r['mean_gray_error']:>9.2f}")

    # figure: example frame + mask, mosaic, anomaly overlay per run
    rows = []
    for name in ("gt", "static", "actuated"):
        frames, ms = runs[name], masks[name]
        k = int(np.argmax([(m > 0).sum() for m in ms])) if any((m > 0).any() for m in ms) else len(frames) // 2
        ex = cv2.cvtColor(frames[k], cv2.COLOR_GRAY2BGR)
        ex[ms[k] > 0] = (0.5 * ex[ms[k] > 0] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
        img, obs = aligned.get(name, (mosaics[name].image, mosaics[name].observed))
        mos = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        mos[~obs] = (60, 0, 60)                       # unobserved in purple
        an = anomaly_mask(img, obs)
        over = mos.copy()
        over[an] = (0.4 * over[an] + 0.6 * np.array([0, 255, 255])).astype(np.uint8)
        H = 260
        def fit(a):
            return cv2.resize(a, (int(a.shape[1] * H / a.shape[0]), H))
        row = np.hstack([fit(ex), fit(over)])
        cv2.putText(row, name, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        rows.append(row)
    W = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, W - r.shape[1], cv2.BORDER_CONSTANT) for r in rows]
    cv2.imwrite(os.path.join(out_dir, "c5_overview.png"), np.vstack(rows))
    print(f"\n  written to {out_dir} (c5_metrics.json, c5_overview.png)")
    return results


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gt"); ap.add_argument("--static"); ap.add_argument("--actuated")
    ap.add_argument("--step", type=int, default=1, help="use every n-th frame")
    ap.add_argument("--out", default="data/c5")
    a = ap.parse_args()
    out = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S"))
    if a.synthetic:
        sc = make_scene(a.seed)
        runs = {k: v.frames for k, v in sc.runs.items()}
    else:
        if not (a.gt and a.static and a.actuated):
            ap.error("give --synthetic or all of --gt --static --actuated")
        runs = {k: load(getattr(a, k), a.step)[0] for k in ("gt", "static", "actuated")}
    run(runs, out)


if __name__ == "__main__":
    main()
