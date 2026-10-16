"""Load frames for C5 from a folder, an index.csv, or a video file.

Folder layout written by the C1 capture: PNG/JPG files plus optional
index.csv with columns idx,t,file (extra columns are ignored). Without an
index, files are used in name order. Colour images are converted to grey.
"""
from __future__ import annotations

import csv
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")
VID_EXT = (".mp4", ".avi", ".mov", ".mkv")


def _gray(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img


def load_folder(path: str, step: int = 1, limit: Optional[int] = None) -> Tuple[List[np.ndarray], List[float]]:
    idx = os.path.join(path, "index.csv")
    files: List[str] = []
    times: List[float] = []
    if os.path.isfile(idx):
        with open(idx, newline="") as f:
            for row in csv.DictReader(f):
                files.append(os.path.join(path, row["file"]))
                times.append(float(row.get("t", len(files))))
    else:
        files = sorted(os.path.join(path, n) for n in os.listdir(path) if n.lower().endswith(IMG_EXT))
        times = [float(i) for i in range(len(files))]
    frames = []
    keep_t = []
    for i in range(0, len(files), step):
        im = cv2.imread(files[i], cv2.IMREAD_UNCHANGED)
        if im is None:
            continue
        frames.append(_gray(im))
        keep_t.append(times[i])
        if limit and len(frames) >= limit:
            break
    return frames, keep_t


def load_video(path: str, step: int = 3, limit: Optional[int] = None) -> Tuple[List[np.ndarray], List[float]]:
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames, times, i = [], [], 0
    while True:
        ok, im = cap.read()
        if not ok:
            break
        if i % step == 0:
            frames.append(_gray(im))
            times.append(i / fps)
            if limit and len(frames) >= limit:
                break
        i += 1
    cap.release()
    return frames, times


def load(path: str, step: int = 1, limit: Optional[int] = None):
    if os.path.isdir(path):
        return load_folder(path, step, limit)
    if path.lower().endswith(VID_EXT):
        return load_video(path, step, limit)
    raise ValueError(f"not a folder or video: {path}")
