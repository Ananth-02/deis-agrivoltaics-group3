"""Panel segmentation for a monochrome camera (no colour available).

The panel is a large, dark, compact region. Steps:
 1. blur, estimate the bed level as a high percentile of the frame (robust when
    the panel fills up to about 45 % of the frame),
 2. threshold at a fraction of that level,
 3. close small holes (cell grid lines, glare streaks), open away speckle,
 4. keep only large, compact components (area and bounding-box fill),
 5. dilate a few pixels so the panel edge does not leak into the mosaic.
Thin dark structures (a vertical panel seen end-on, shadows of leaves) fail the
area test and stay unmasked.
"""
from __future__ import annotations

import cv2
import numpy as np


def panel_mask(img: np.ndarray, dark_frac: float = 0.45, level_pct: float = 75.0, min_area: int = 1500,
               min_area_border: int = 600, min_fill: float = 0.55, close_px: int = 15,
               dilate_px: int = 5) -> np.ndarray:
    g = cv2.GaussianBlur(img, (0, 0), 1.5)
    level = float(np.percentile(g, level_pct))
    dark = (g < dark_frac * level).astype(np.uint8) * 255
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k_close)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k_open)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = np.zeros_like(m)
    H, W = m.shape
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        bx, by = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        bw, bh = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        touches = bx == 0 or by == 0 or bx + bw >= W or by + bh >= H
        # a panel entering the frame shows only a sliver at the border: accept a smaller area there
        if area >= (min_area_border if touches else min_area) and area / float(max(1, bw * bh)) >= min_fill:
            out[lab == i] = 255
    if out.any():
        # glare on a glossy panel is bright, not dark, and may touch the frame edge, so filling holes
        # is not enough. The panel is a convex shape seen from above: fill each region by its hull.
        cs, _ = cv2.findContours(out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = np.zeros_like(out)
        cv2.fillPoly(out, [cv2.convexHull(c) for c in cs], 255)
    if dilate_px > 0 and out.any():
        out = cv2.dilate(out, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilate_px + 1, 2 * dilate_px + 1)))
    return out


def iou(a: np.ndarray, b: np.ndarray) -> float:
    a, b = a > 0, b > 0
    u = np.logical_or(a, b).sum()
    return 1.0 if u == 0 else float(np.logical_and(a, b).sum() / u)
