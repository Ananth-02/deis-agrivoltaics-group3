import numpy as np
import pytest

from agri_loop.vision.mask import panel_mask
from agri_loop.vision.metrics import evaluate
from agri_loop.vision.mosaic import align_to, build_mosaic, register
from agri_loop.vision.synth import make_scene


@pytest.fixture(scope="module")
def scene():
    return make_scene(seed=3)


def test_mask_finds_panel_and_ignores_clean_bed(scene):
    cover, fp = [], 0
    for name, run in scene.runs.items():
        for img, truth in zip(run.frames, run.panel_truth):
            m = panel_mask(img) > 0
            t = truth > 0
            if t.sum() > 2000:                        # panel clearly in view
                cover.append((m & t).sum() / t.sum())
            elif not t.any() and m.any():
                fp += 1
    assert np.mean(cover) > 0.95
    assert fp == 0                                    # no panel in view, nothing masked (also the end-on vertical panel)


def test_mask_has_no_hole_from_glare(scene):
    run = scene.runs["static"]
    img, truth = max(zip(run.frames, run.panel_truth), key=lambda p: (p[1] > 0).sum())
    m = panel_mask(img) > 0
    assert (truth > 0).sum() > 20000
    assert ((truth > 0) & ~m).sum() / (truth > 0).sum() < 0.03


def test_registration_error_and_no_failures(scene):
    run = scene.runs["actuated"]
    masks = [panel_mask(f) for f in run.frames]
    T, fails, inl = register(run.frames, masks)
    err = [np.linalg.norm(np.array([t[0, 2], t[1, 2]]) - (c - run.centers[0])) for t, c in zip(T, run.centers)]
    assert fails == 0
    assert np.median(err) < 4.0 and max(err) < 12.0    # px over a 700 px pass


def test_actuated_recovers_crop_and_hidden_patch_static_does_not(scene):
    mos, masks = {}, {}
    for name in ("gt", "static", "actuated"):
        masks[name] = [panel_mask(f) for f in scene.runs[name].frames]
        mos[name] = build_mosaic(scene.runs[name].frames, masks[name])
    res = {}
    for name in ("static", "actuated"):
        al = align_to(mos["gt"], mos[name])
        assert al is not None
        res[name] = evaluate(al[0], al[1], mos["gt"].image, mos["gt"].observed)
    assert res["static"]["visible_crop_ratio"] < 0.90          # panel hides part of the bed
    assert res["actuated"]["visible_crop_ratio"] > 0.97
    assert not res["static"]["anomaly_detected"]               # hidden patch stays hidden
    assert res["actuated"]["anomaly_detected"]
    assert 0.8 < res["actuated"]["anomaly_area_ratio"] < 1.2
