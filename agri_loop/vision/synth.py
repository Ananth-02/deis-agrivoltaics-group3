"""Synthetic grayscale data for testing the C5 pipeline without hardware.

World: a textured crop bed seen from above (nadir). A dark panel lies over part
of the bed (horizontal) and hides a bright "dead patch" (paper). The camera
mimics the AI deck: small mono frames, noise, vignette, exposure flicker, a
little yaw and scale jitter. Parallax from the panel height is ignored.

Three runs over the same bed:
  gt        no panel at all (ground truth)
  static    panel always horizontal (control)
  actuated  panel vertical while the camera footprint is near the panel
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import cv2
import numpy as np


@dataclass
class SynthRun:
    frames: List[np.ndarray]
    centers: np.ndarray            # true camera centre in world px, Nx2
    panel_state: List[str]         # "horizontal" or "vertical" (or "none")
    panel_truth: List[np.ndarray]  # true panel mask per frame (uint8 0/255)
    t: np.ndarray


@dataclass
class SynthScene:
    world_clean: np.ndarray
    panel_box: Tuple[int, int, int, int]     # x0, y0, x1, y1 in world px
    patch_box: Tuple[int, int, int, int]
    runs: Dict[str, SynthRun] = field(default_factory=dict)


def _norm(a: np.ndarray) -> np.ndarray:
    return a / (a.std() + 1e-9)


def make_bed(w: int, h: int, rng: np.random.Generator) -> np.ndarray:
    def noise(s):
        return _norm(cv2.GaussianBlur(rng.standard_normal((h, w)).astype(np.float32), (0, 0), s))
    yy = np.arange(h, dtype=np.float32)[:, None]
    bed = 112 + 24 * noise(2.0) + 12 * noise(6.0) + 6 * noise(20.0) + 8 * np.sin(2 * np.pi * yy / 34.0)
    return np.clip(bed, 25, 235).astype(np.float32)


def add_patch(bed: np.ndarray, box: Tuple[int, int, int, int], rng: np.random.Generator, gray: float = 215.0) -> None:
    x0, y0, x1, y1 = box
    bed[y0:y1, x0:x1] = gray + rng.normal(0, 4, (y1 - y0, x1 - x0))


def draw_panel(img: np.ndarray, box: Tuple[int, int, int, int], rng: np.random.Generator) -> np.ndarray:
    x0, y0, x1, y1 = box
    out = img.copy()
    out[y0:y1, x0:x1] = 34
    for gx in range(x0 + 24, x1, 24):          # cell grid lines
        out[y0:y1, gx:gx + 1] = 62
    for gy in range(y0 + 24, y1, 24):
        out[gy:gy + 1, x0:x1] = 62
    out[y0:y1, x0:x0 + 3] = 85                 # frame edge
    out[y0:y1, x1 - 3:x1] = 85
    out[y0:y0 + 3, x0:x1] = 85
    out[y1 - 3:y1, x0:x1] = 85
    # glossy glare streak across part of the panel
    gx0 = int(rng.integers(x0 + 20, x1 - 60))
    out[y0 + 30:y0 + 70, gx0:gx0 + 45] = cv2.GaussianBlur(out[y0 + 30:y0 + 70, gx0:gx0 + 45], (0, 0), 6) + 70
    return out


def _render(world: np.ndarray, center: Tuple[float, float], fw: int, fh: int, yaw: float, scale: float):
    M = cv2.getRotationMatrix2D(center, yaw, scale)
    M[0, 2] += fw / 2 - center[0]
    M[1, 2] += fh / 2 - center[1]
    return cv2.warpAffine(world, M, (fw, fh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE), M


def _degrade(img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    h, w = img.shape
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    r2 = ((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2
    out = cv2.GaussianBlur(img, (0, 0), 0.7) * (1 - 0.10 * r2) * rng.uniform(0.95, 1.05)
    out = out + rng.normal(0, 3.0, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def make_scene(seed: int = 0, world_w: int = 1000, world_h: int = 300, fw: int = 324, fh: int = 244,
               speed_px: float = 6.0, early_px: float = 40.0) -> SynthScene:
    rng = np.random.default_rng(seed)
    clean = make_bed(world_w, world_h, rng)
    panel_box = (430, 30, 580, 270)
    patch_box = (470, 120, 540, 170)
    add_patch(clean, patch_box, rng)
    world_panel = draw_panel(clean, panel_box, rng)
    world_vertical = clean.copy()                       # vertical panel seen end-on: a thin line
    x0, y0, x1, y1 = panel_box
    world_vertical[y0:y1, x0:x0 + 4] = 70
    pmask_world = np.zeros((world_h, world_w), np.float32)
    pmask_world[y0:y1, x0:x1] = 255

    xs = np.arange(fw / 2, world_w - fw / 2, speed_px)
    scene = SynthScene(world_clean=clean, panel_box=panel_box, patch_box=patch_box)
    for name in ("gt", "static", "actuated"):
        frames, centers, states, truths = [], [], [], []
        r = np.random.default_rng(seed * 10 + {"gt": 1, "static": 2, "actuated": 3}[name])
        for x in xs:
            cy = world_h / 2 + r.normal(0, 1.5)
            cx = x + r.normal(0, 1.0)
            yaw, sc = r.normal(0, 0.6), 1 + r.normal(0, 0.004)
            near = (x + fw / 2 + early_px >= x0) and (x - fw / 2 - early_px <= x1)
            if name == "gt":
                state, world = "none", clean
            elif name == "static":
                state, world = "horizontal", world_panel
            else:
                state, world = ("vertical", world_vertical) if near else ("horizontal", world_panel)
            img, M = _render(world, (cx, cy), fw, fh, yaw, sc)
            truth = np.zeros((fh, fw), np.uint8)
            if state == "horizontal":
                truth = (cv2.warpAffine(pmask_world, M, (fw, fh), flags=cv2.INTER_NEAREST) > 127).astype(np.uint8) * 255
            frames.append(_degrade(img, r))
            centers.append((cx, cy))
            states.append(state)
            truths.append(truth)
        scene.runs[name] = SynthRun(frames, np.array(centers), states, truths, np.arange(len(frames)) * 0.1)
    return scene
