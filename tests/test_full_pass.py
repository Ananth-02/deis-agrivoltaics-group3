import math
from types import SimpleNamespace

import pytest

from agri_loop.drone.full_pass import END_MARGIN_M, FOOTPRINT_M, PlanError, plan, to_sim_frame
from agri_loop.link.eventlog import EventLog
from agri_loop.link.gateway import Gateway, GatewayConfig
from agri_loop.link.messages import (EVENT_APPROACH, EVENT_CLEARED, FSM_HOLD, MISSION_CROP_OCCLUSION, MISSION_NONE,
                                     DroneBeacon, PanelEvent, PanelState)
from agri_loop.drone.measure_points import derive


def _pt(x, y, z):
    return dict(pos=[x, y, z], spread_m=0.005, samples=50, how="auto")


def _points(takeoff=(1.4, -0.6, 0.01)):
    pts = {
        "origin": _pt(0.0, 0.0, 0.01), "towards_strip": _pt(0.5, 0.02, 0.01),
        "takeoff": _pt(*takeoff), "corner_strip": _pt(1.6, 0.9, 0.01),
        "pot": _pt(-1.0, -0.6, 0.15), "leaf": _pt(-1.0, -0.6, 0.30),
        "post_a": _pt(-1.0, -0.4, 0.45), "post_b": _pt(-1.0, -0.8, 0.45),
        "panel_top_vertical": _pt(-1.0, -0.6, 0.55), "panel_flat_strip_side": _pt(-0.85, -0.6, 0.45),
        "panel_flat_far_side": _pt(-1.15, -0.6, 0.45), "corner_rig": _pt(-2.0, -1.1, 0.01),
    }
    d, w = derive(pts)
    return dict(points=pts, derived=d, warnings=w)


def _args(**kw):
    base = dict(speed=0.2, lead_s=2.2, passes=2, zone=None, height=None, direction=None, length=None)
    base.update(kw)
    return SimpleNamespace(**base)


END = FOOTPRINT_M + END_MARGIN_M     # past the zone far edge


def test_plan_minus_x_from_points():
    p = plan(_points(), _args())
    assert p["direction"] == -1
    assert p["zone"] == [-1.15, -0.85, -0.8, -0.4]
    assert p["height"] == 0.84 and p["floor_z"] == 0.01 and p["fly_z"] == 0.85
    assert p["length"] == pytest.approx(1.4 + 1.15 + END)
    assert p["approach_m"] == pytest.approx(0.2 * 2.2 + FOOTPRINT_M + 0.05)
    assert not any("off the pot" in w for w in p["warnings"])
    assert "assumed" in p["footprint_note"]


def test_plan_plus_x_when_plant_is_on_the_positive_side():
    p = plan(_points(takeoff=(-3.0, -0.6, 0.01)), _args())
    assert p["direction"] == 1 and p["length"] == pytest.approx(3.0 - 0.85 + END)


def test_plan_warns_for_sideways_takeoff():
    p = plan(_points(takeoff=(1.4, -0.4, 0.01)), _args())
    assert any("off the pot" in w for w in p["warnings"])


def test_plan_refuses_a_pass_that_cannot_get_past_the_zone_or_leaves_the_fence():
    pts = _points()
    pts["derived"]["fence"] = {"x": [-1.45, 1.4], "y": [-1.0, 1.0]}     # room 2.85 m, needs 3.0 m
    with pytest.raises(PlanError, match="cannot get past"):
        plan(pts, _args())
    with pytest.raises(PlanError, match="beyond the fence"):
        plan(pts, _args(length=3.5))
    with pytest.raises(PlanError):
        plan(_points(), _args(length=0.0))
    with pytest.raises(PlanError, match="cannot get past"):
        plan(_points(), _args(length=1.0))


def test_plan_warns_when_approach_does_not_fit():
    p = plan(_points(takeoff=(-0.5, -0.6, 0.01)), _args(speed=0.3))
    assert p["direction"] == -1
    assert any("approach distance" in w for w in p["warnings"])


def test_plan_refuses_low_high_or_missing_height():
    pts = _points()
    with pytest.raises(PlanError):
        plan(pts, _args(height=0.7))                                # rig top 0.54 + 0.3 = 0.84
    with pytest.raises(PlanError, match="limit"):
        plan(pts, _args(height=1.3))
    del pts["derived"]["over_plant_z"]
    with pytest.raises(PlanError):
        plan(pts, _args())
    assert plan(pts, _args(height=0.9))["fly_z"] == 0.91


def test_plan_refuses_odd_or_bad_passes_and_direction():
    for n in (1, 3, 0):
        with pytest.raises(PlanError, match="even"):
            plan(_points(), _args(passes=n))
    with pytest.raises(PlanError):
        plan(_points(), _args(direction=2))


def test_plan_zone_falls_back_to_pot():
    pts = _points()
    for k in ("panel_flat_strip_side", "panel_flat_far_side", "post_a", "post_b"):
        del pts["points"][k]
    p = plan(pts, _args())
    assert p["zone"] == [-1.15, -0.85, -0.75, -0.45]
    assert any("pot +- 0.15" in w for w in p["warnings"])


def test_plan_manual_overrides_without_points_file():
    p = plan(None, _args(zone=[0.4, 0.8, -0.3, 0.3], height=0.7, direction=-1, length=1.5))
    assert p["zone"] == [0.4, 0.8, -0.3, 0.3] and p["direction"] == -1 and p["length"] == 1.5
    assert any("NOT checked against the plant" in w for w in p["warnings"])
    with pytest.raises(PlanError):
        plan(None, _args(height=0.7))                               # no zone
    with pytest.raises(PlanError):
        plan(None, _args(zone=[0, 1, 0, 1]))                        # no height
    with pytest.raises(PlanError):
        plan(None, _args(zone=[0, 1, 0, 1], height=0.7))            # no direction and length


def test_plan_rejects_zone_behind_takeoff_and_sim_frame_shift():
    with pytest.raises(PlanError):
        plan(_points(), _args(direction=1))
    p = to_sim_frame(plan(_points(), _args()))
    assert p["zone"] == [-2.55, -2.25, -0.2, 0.2]


def test_plan_sizes_the_footprint_from_the_field_of_view():
    p = plan(_points(), _args(fov_deg=60.0))
    assert p["footprint"] == round((0.85 - 0.30) * math.tan(math.radians(30)), 2) == 0.32   # leaf top 0.30
    assert "tan(60 deg / 2)" in p["footprint_note"]
    assert p["length"] == pytest.approx(1.4 + 1.15 + 0.32 + END_MARGIN_M)
    assert p["approach_m"] == pytest.approx(0.2 * 2.2 + 0.32 + 0.05)
    pts = _points()
    del pts["points"]["leaf"]                                       # falls back to the pot (0.15)
    assert plan(pts, _args(fov_deg=60.0))["footprint"] == round(0.70 * math.tan(math.radians(30)), 2)


def test_gateway_releases_the_panel_during_the_pause_at_the_end_of_the_pass():
    p = plan(_points(), _args())
    cfg = GatewayConfig(zone=tuple(p["zone"]), footprint_half=p["footprint"], lead_s=p["lead_s"])
    sent = []
    t_now = [0.0]
    gw = Gateway(lambda m: sent.append((t_now[0], m)), EventLog(), cfg)
    x0, d, speed = 1.4, p["direction"], p["speed"]
    t_stop = (p["length"] - 0.03) / speed          # the drone stops 3 cm short of the end of the pass
    pause = 2.0
    t, last_ack = 0.0, 0
    while t < t_stop + pause:
        t_now[0] = t
        moving = t < t_stop
        x = x0 + d * speed * min(t, t_stop)
        gw.on_beacon(DroneBeacon(int(t * 10) + 1, t, 1, x, -0.6, 0.85, d * speed if moving else 0.0, 0.0, 0.0, 0.0,
                                 MISSION_CROP_OCCLUSION if moving else MISSION_NONE), t)
        ev = [m for _, m in sent if isinstance(m, PanelEvent)]
        if ev and ev[-1].seq > last_ack:      # the panel confirms at once
            last_ack = ev[-1].seq
            gw.on_state(PanelState(1, t, 1, FSM_HOLD, ev[-1].target_angle_deg, ev[-1].target_angle_deg, 0.0,
                                   last_ack), t)
        gw.tick(t)
        t += 0.1
    events = [(ts, m) for ts, m in sent if isinstance(m, PanelEvent)]
    assert [m.event_type for _, m in events] == [EVENT_APPROACH, EVENT_CLEARED]
    assert events[0][0] < t_stop and t_stop < events[1][0] < t_stop + pause
