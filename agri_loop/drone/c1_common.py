"""Shared pieces of the C1 command-line tools: run folders, pre-flight checks, stream check."""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, List, Tuple

from .cpx_stream import FrameRecorder
from .flightlog import SUP_CAN_BE_ARMED
from .mission import C1Config

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"
REQUIRED_VARS = ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                 "stateEstimate.vx", "stateEstimate.vy", "pm.vbat"]


def new_run_dir(root: str, label: str = "") -> str:
    name = time.strftime("%Y%m%d_%H%M%S") + (f"_{label}" if label else "")
    path = os.path.join(root, name)
    os.makedirs(os.path.join(path, "frames"), exist_ok=True)
    return path


def save_json(path: str, obj: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, default=str)


def preflight(vehicle: Any, cfg: C1Config, need_camera: bool) -> Tuple[bool, Dict[str, Any], List[str]]:
    """Checks that must pass before the motors start. Returns (ok, report, problems)."""
    problems: List[str] = []
    warnings: List[str] = []
    decks = vehicle.decks()
    if not decks.get("bcFlow2"):
        problems.append("Flow deck v2 not detected (deck.bcFlow2): the drone cannot hold position or height")
    if not decks.get("bcAI"):
        (problems if need_camera else warnings).append("AI deck not detected (deck.bcAI)")
    missing = [v for v in getattr(vehicle, "missing_vars", []) if v in REQUIRED_VARS]
    if missing:
        problems.append("log variables missing from the firmware: " + ", ".join(missing))
    s = vehicle.latest()
    if not s:
        problems.append("no log data received from the drone")
    vb = s.get("vbat")
    if vb is None:
        problems.append("battery voltage not available")
    elif vb < cfg.min_start_vbat:
        problems.append(f"battery {vb:.2f} V below {cfg.min_start_vbat} V: charge or swap it")
    if s.get("z") is not None and s["z"] > 0.10:
        warnings.append(f"height estimate {s['z']:.2f} m while on the ground: restart the drone on the floor")
    if abs(s.get("roll", 0.0) or 0.0) > 10 or abs(s.get("pitch", 0.0) or 0.0) > 10:
        problems.append("drone not level: place it flat before take-off")
    sup = s.get("sup_info")
    if sup is not None and not (int(sup) & SUP_CAN_BE_ARMED):
        problems.append("supervisor says the system cannot be armed (check for a tumble or a locked state; restart)")
    report = dict(decks=decks, state={k: s.get(k) for k in ("x", "y", "z", "roll", "pitch", "yaw", "vbat",
                                                            "battery_level", "pm_state", "sup_info")},
                  missing_vars=getattr(vehicle, "missing_vars", []), warnings=warnings, problems=problems)
    return not problems, report, problems


def stream_check(recorder: FrameRecorder, seconds: float) -> Dict[str, Any]:
    """Wait for the stream and measure it for `seconds` before anything flies."""
    if not recorder.connected.wait(timeout=recorder.timeout_s + 1):
        return dict(ok=False, error=recorder.stats.error or "could not connect to the AI deck")
    n0 = recorder.stats.frames
    t0 = time.time()
    time.sleep(seconds)
    n = recorder.stats.frames - n0
    st = recorder.stats
    return dict(ok=n > 0 and not st.error, frames=n, fps=n / max(time.time() - t0, 1e-6),
                width=st.width, height=st.height, format="jpeg" if st.fmt == 1 else "raw", error=st.error)


def print_report(title: str, d: Dict[str, Any]) -> None:
    print(f"\n== {title} ==")
    print(json.dumps(d, indent=2, default=str))
