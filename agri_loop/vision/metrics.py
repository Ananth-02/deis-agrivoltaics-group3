"""C5 metrics, all relative to the ground-truth mosaic (panel-free pass).

With a mono camera there is no colour index, so:
  visible_crop_ratio   observed area / ground-truth observed area
  anomaly detection    a "dead patch" (bright or dark paper) appears as a
                       grey-level outlier on a smoothed mosaic; recall is
                       measured against the anomaly found in the ground truth
  mean_gray_error      |mean grey (test) - mean grey (ground truth)| on the
                       common observed region, the stand-in for index error
"""
from __future__ import annotations

from typing import Dict, Tuple

import cv2
import numpy as np


def visible_crop_ratio(obs: np.ndarray, gt_obs: np.ndarray) -> float:
    gt = gt_obs.sum()
    return float(np.logical_and(obs, gt_obs).sum() / gt) if gt else 0.0


def anomaly_mask(img: np.ndarray, obs: np.ndarray, sigma: float = 5.0, z: float = 4.0, min_area: int = 400,
                 min_support: float = 0.8) -> np.ndarray:
    """Grey-level outliers on a smoothed image, using normalised convolution so
    unobserved pixels do not bleed into the statistics."""
    o = obs.astype(np.float32)
    num = cv2.GaussianBlur(img * o, (0, 0), sigma)
    den = cv2.GaussianBlur(o, (0, 0), sigma)
    g = num / np.maximum(den, 1e-6)
    support = den >= min_support
    vals = g[support]
    if vals.size < 100:
        return np.zeros(img.shape, bool)
    med = np.median(vals)
    mad = 1.4826 * np.median(np.abs(vals - med)) + 1e-6
    zmap = np.abs(g - med) / mad
    m = ((zmap > z) & support).astype(np.uint8) * 255
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = np.zeros(img.shape, bool)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out |= lab == i
    return out


def anomaly_recall(test: np.ndarray, gt: np.ndarray) -> float:
    g = gt.sum()
    return float(np.logical_and(test, gt).sum() / g) if g else float("nan")


def mean_gray_error(img: np.ndarray, obs: np.ndarray, gt_img: np.ndarray, gt_obs: np.ndarray) -> float:
    common = np.logical_and(obs, gt_obs)
    if not common.any():
        return float("nan")
    return float(abs(img[common].mean() - gt_img[common].mean()))


def evaluate(test_img, test_obs, gt_img, gt_obs, patch_detect_recall: float = 0.5) -> Dict[str, float]:
    gt_anom = anomaly_mask(gt_img, gt_obs)
    t_anom = anomaly_mask(test_img, test_obs)
    rec = anomaly_recall(t_anom, gt_anom)
    return {
        "visible_crop_ratio": visible_crop_ratio(test_obs, gt_obs),
        "anomaly_recall": rec,
        "anomaly_detected": bool(rec >= patch_detect_recall) if rec == rec else False,
        "anomaly_area_ratio": float(t_anom.sum() / gt_anom.sum()) if gt_anom.sum() else float("nan"),
        "mean_gray_error": mean_gray_error(test_img, test_obs, gt_img, gt_obs),
        "gt_anomaly_px": int(gt_anom.sum()),
    }
