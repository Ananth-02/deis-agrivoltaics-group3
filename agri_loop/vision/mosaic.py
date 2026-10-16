"""Mosaic from nadir mono frames with the panel masked out.

Registration: SIFT features (panel region excluded), similarity transform
(translation, rotation, scale) between consecutive frames with RANSAC, chained
to the first frame. A similarity model is used because the scene is a plane
seen from above; a full homography drifts more on weak texture. If matching
fails for a frame, the last good motion is repeated and the failure is counted.

Compositing: every mosaic pixel is the mean of the unmasked observations.
Pixels never seen unmasked are "unobserved" and are reported as such.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class Mosaic:
    image: np.ndarray            # float32, valid where observed
    observed: np.ndarray         # bool
    count: np.ndarray            # float32 effective number of observations
    transforms: List[np.ndarray]  # frame -> mosaic, 3x3, in the first-frame system (before canvas shift)
    offset: Tuple[float, float]  # canvas shift applied to the transforms
    failures: int = 0
    inliers: List[int] = field(default_factory=list)


def _to3(M: np.ndarray) -> np.ndarray:
    T = np.eye(3)
    T[:2, :] = M
    return T


def _features(img: np.ndarray, mask: Optional[np.ndarray], sift, clahe):
    det_mask = None
    if mask is not None:
        det_mask = cv2.erode((mask == 0).astype(np.uint8) * 255, np.ones((7, 7), np.uint8))
    return sift.detectAndCompute(clahe.apply(img), det_mask)


def _pair_transform(kp_a, des_a, kp_b, des_b, matcher, ratio: float = 0.75, min_matches: int = 15):
    """Similarity mapping coordinates of frame a onto frame b."""
    if des_a is None or des_b is None or len(kp_a) < 8 or len(kp_b) < 8:
        return None, 0
    good = []
    for pair in matcher.knnMatch(des_a, des_b, k=2):
        if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance:
            good.append(pair[0])
    if len(good) < min_matches:
        return None, len(good)
    src = np.float32([kp_a[m.queryIdx].pt for m in good])
    dst = np.float32([kp_b[m.trainIdx].pt for m in good])
    M, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=2.5, maxIters=3000,
                                         confidence=0.995)
    if M is None:
        return None, 0
    n = int(inl.sum())
    return (M, n) if n >= min_matches else (None, n)


def register(frames: Sequence[np.ndarray], masks: Sequence[Optional[np.ndarray]], mode: str = "feature",
             poses_px: Optional[np.ndarray] = None):
    """Return (list of 3x3 frame->mosaic transforms, failure count, inlier counts).

    mode "pose": translation only from known camera positions in pixels
    (nadir, constant height, no yaw), e.g. from the overhead tracker.
    """
    n = len(frames)
    T = [np.eye(3) for _ in range(n)]
    if mode == "pose":
        assert poses_px is not None and len(poses_px) == n
        for i in range(n):
            T[i][0, 2] = poses_px[i][0] - poses_px[0][0]
            T[i][1, 2] = poses_px[i][1] - poses_px[0][1]
        return T, 0, []
    sift = cv2.SIFT_create(nfeatures=1500)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    feats = [_features(frames[i], masks[i], sift, clahe) for i in range(n)]
    failures, inliers = 0, []
    last_delta = np.eye(3)
    for i in range(1, n):
        M, ninl = None, 0
        for back in (1, 2, 3):
            j = i - back
            if j < 0:
                break
            M, ninl = _pair_transform(feats[i][0], feats[i][1], feats[j][0], feats[j][1], matcher)
            if M is not None:
                # chain through frame j; also remember the motion relative to the previous frame
                T[i] = T[j] @ _to3(M)
                last_delta = np.linalg.inv(T[i - 1]) @ T[i]
                break
        if M is None:
            failures += 1
            T[i] = T[i - 1] @ last_delta
        inliers.append(ninl)
    return T, failures, inliers


def composite(frames: Sequence[np.ndarray], masks: Sequence[Optional[np.ndarray]], T: Sequence[np.ndarray],
              border: int = 4) -> Mosaic:
    h, w = frames[0].shape
    corners = np.array([[0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1]], dtype=np.float64).T
    pts = np.concatenate([(t @ corners)[:2].T for t in T])
    x0, y0 = np.floor(pts.min(axis=0)).astype(int)
    x1, y1 = np.ceil(pts.max(axis=0)).astype(int)
    W, H = int(x1 - x0) + 1, int(y1 - y0) + 1
    shift = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1.0]])
    acc = np.zeros((H, W), np.float32)
    cnt = np.zeros((H, W), np.float32)
    inner = np.zeros((h, w), np.uint8)
    inner[border:h - border, border:w - border] = 255
    for img, m, t in zip(frames, masks, T):
        valid = inner.copy()
        if m is not None:
            valid[m > 0] = 0
        Mw = (shift @ t)[:2]
        v = valid.astype(np.float32) / 255.0
        acc += cv2.warpAffine(img.astype(np.float32) * v, Mw, (W, H), flags=cv2.INTER_LINEAR)
        cnt += cv2.warpAffine(v, Mw, (W, H), flags=cv2.INTER_LINEAR)
    observed = cnt > 0.5
    image = np.where(observed, acc / np.maximum(cnt, 1e-6), 0).astype(np.float32)
    return Mosaic(image=image, observed=observed, count=cnt, transforms=[shift @ t for t in T], offset=(-x0, -y0))


def build_mosaic(frames, masks, mode: str = "feature", poses_px=None) -> Mosaic:
    T, fails, inl = register(frames, masks, mode, poses_px)
    mos = composite(frames, masks, T)
    mos.failures, mos.inliers = fails, inl
    return mos


def align_to(ref: Mosaic, other: Mosaic, min_matches: int = 20):
    """Similarity-align `other` onto the canvas of `ref` using mosaic features.

    Returns (image, observed) in ref's canvas, or None if alignment fails.
    """
    sift = cv2.SIFT_create(nfeatures=4000)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def prep(m: Mosaic):
        u8 = np.clip(m.image, 0, 255).astype(np.uint8)
        mask = cv2.erode(m.observed.astype(np.uint8) * 255, np.ones((9, 9), np.uint8))
        return sift.detectAndCompute(clahe.apply(u8), mask)

    kr, dr = prep(ref)
    ko, do = prep(other)
    M, n = _pair_transform(ko, do, kr, dr, cv2.BFMatcher(cv2.NORM_L2), min_matches=min_matches)
    if M is None:
        return None
    H, W = ref.image.shape
    img = cv2.warpAffine(other.image, M, (W, H), flags=cv2.INTER_LINEAR)
    obs = cv2.warpAffine(other.observed.astype(np.uint8) * 255, M, (W, H), flags=cv2.INTER_NEAREST) > 127
    return img, obs
