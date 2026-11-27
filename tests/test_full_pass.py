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
    p = plan(_points(), _args(fov_deg=60.0, home_tol=0.0))
    assert p["footprint"] == round((0.85 - 0.30) * math.tan(math.radians(30)), 2) == 0.32   # leaf top 0.30
    assert "tan(60 deg / 2)" in p["footprint_note"]
    assert p["length"] == pytest.approx(1.4 + 1.15 + 0.32 + END_MARGIN_M)
    assert p["approach_m"] == pytest.approx(0.2 * 2.2 + 0.32 + 0.05)
    pts = _points()
    del pts["points"]["leaf"]                                       # falls back to the pot (0.15)
    assert plan(pts, _args(fov_deg=60.0, home_tol=0.0))["footprint"] == round(0.70 * math.tan(math.radians(30)), 2)


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


def test_plan_fence_room_is_reduced_by_the_home_tolerance():
    pts = _points()
    pts["derived"]["fence"] = {"x": [-1.60, 1.4], "y": [-1.0, 1.0]}      # room 3.0 m, need 3.0 m
    assert plan(pts, _args(home_tol=0.0))["length"] == pytest.approx(3.0)
    with pytest.raises(PlanError, match="cannot get past"):
        plan(pts, _args(home_tol=0.10))                                   # room shrinks to 2.9 m


def test_plan_refuses_a_takeoff_spot_inside_the_grown_zone():
    with pytest.raises(PlanError, match="inside the zone"):
        plan(_points(takeoff=(-1.0, -0.6, 0.01)), _args(direction=1))
    with pytest.raises(PlanError, match="inside the zone"):
        plan(_points(takeoff=(-0.7, -0.6, 0.01)), _args(direction=-1))     # 0.15 m from the zone, patch 0.25 m
    plan(_points(takeoff=(-0.5, -0.6, 0.01)), _args(direction=-1))        # 0.35 m away: fine


def test_plan_validates_the_field_of_view_and_the_floor():
    for fov in (0.0, -5.0, 180.0, 200.0):
        with pytest.raises(PlanError, match="fov"):
            plan(_points(), _args(fov_deg=fov))
    pts = _points()
    pts["derived"]["floor_z"] = 0.15
    with pytest.raises(PlanError, match="floor"):
        plan(pts, _args())


def test_beacon_reports_the_mission_only_on_outbound_passes(tmp_path):
    import threading
    import time

    from agri_loop.drone.mission import C1Config, Mission
    from agri_loop.drone.flightlog import FlightLogger
    from agri_loop.drone.sim import SimVehicle

    seen = {"all": [], "outbound": []}
    logger = FlightLogger(str(tmp_path / "flight_log.csv"))
    v = SimVehicle(logger).open()
    cfg = C1Config(n_passes=2, pass_length_m=0.4, speed_m_s=0.3, accel_m_s2=1.0, pause_s=0.3, takeoff_s=1.0,
                   settle_s=0.5, land_s=0.8)
    m = Mission(v, cfg, EventLog())
    stop = threading.Event()

    def sample():
        while not stop.is_set():
            if m.in_pass:
                for k in seen:
                    seen[k].append((m.pass_index, m.pass_direction, m.beacon_active(k)))
            time.sleep(0.01)
    th = threading.Thread(target=sample, daemon=True)
    th.start()
    try:
        m.run()
    finally:
        stop.set()
        th.join()
        v.close()
        logger.close()
    assert not m.beacon_active("outbound") and not m.beacon_active("all")    # not in a pass any more
    assert {(i, d) for i, d, _ in seen["all"]} == {(0, 1), (1, -1)}
    assert all(a for _, _, a in seen["all"])
    assert all(a == (i == 0) for i, _, a in seen["outbound"]) and any(a for _, _, a in seen["outbound"])


def _demo_points():
    pts = _points()
    pts["derived"]["fence"] = {"x": [-1.45, 1.4], "y": [-1.0, 1.0]}     # room 2.85 m with the home tolerance
    return pts


def test_plan_demo_fits_where_the_normal_mode_refuses():
    pts = _demo_points()
    with pytest.raises(PlanError, match="cannot get past"):
        plan(pts, _args())
    p = plan(pts, _args(demo=True, passes=10))
    assert p["demo"] and p["passes"] == 2
    # plant far edge is the pot/leaf at x = -1.0, 2.4 m from the take-off spot, + 0.10 m
    assert p["length"] == pytest.approx(2.5)
    assert not any("return passes" in w for w in p["warnings"])


def test_plan_demo_refuses_when_even_the_plant_cannot_be_passed():
    pts = _demo_points()
    pts["derived"]["fence"] = {"x": [-1.0, 1.4], "y": [-1.0, 1.0]}      # fence ends at the plant
    with pytest.raises(PlanError, match="cannot get past the plant"):
        plan(pts, _args(demo=True))
    with pytest.raises(PlanError, match="cannot get past the plant"):
        plan(_points(), _args(demo=True, length=2.0))


def test_gateway_one_approach_and_one_cleared_when_the_drone_turns_back_without_the_mission():
    p = plan(_demo_points(), _args(demo=True))
    cfg = GatewayConfig(zone=tuple(p["zone"]), footprint_half=p["footprint"], lead_s=p["lead_s"])
    sent = []
    gw = Gateway(lambda m: sent.append(m), EventLog(), cfg)
    x0, d, speed = 1.4, p["direction"], p["speed"]
    x_stop = -1.0 + d * 0.05      # 0.05 m past the plant (pot/leaf at x = -1.0), in the pass direction
    t_out = abs(x_stop - x0) / speed
    t_back = t_out + 2.0 + abs(x_stop - x0) / speed
    t, last_ack = 0.0, 0
    while t < t_back + 3.0:
        if t < t_out:
            x, mission, v = x0 + d * speed * t, MISSION_CROP_OCCLUSION, d * speed
        elif t < t_out + 2.0:
            x, mission, v = x_stop, MISSION_NONE, 0.0
        elif t < t_back:
            x, mission, v = x_stop - d * speed * (t - t_out - 2.0), MISSION_NONE, -d * speed
        else:
            x, mission, v = x0, MISSION_NONE, 0.0
        gw.on_beacon(DroneBeacon(int(t * 10) + 1, t, 1, x, -0.6, 0.85, v, 0.0, 0.0, 0.0, mission), t)
        ev = [m for m in sent if isinstance(m, PanelEvent)]
        if ev and ev[-1].seq > last_ack:
            last_ack = ev[-1].seq
            gw.on_state(PanelState(1, t, 1, FSM_HOLD, ev[-1].target_angle_deg, ev[-1].target_angle_deg, 0.0,
                                   last_ack), t)
        gw.tick(t)
        t += 0.1
    assert [m.event_type for m in sent if isinstance(m, PanelEvent)] == [EVENT_APPROACH, EVENT_CLEARED]


def test_plant_frames_lists_the_outbound_frames_within_the_grown_zone(tmp_path):
    import os

    from agri_loop.drone.full_pass import plant_frames

    c1 = tmp_path / "c1"
    (c1 / "frames").mkdir(parents=True)
    ev = EventLog()
    ev.add("pass_start", 100.0, i=0, direction=1)
    ev.add("pass_end", 110.0, i=0, reason="reached")
    ev.add("pass_start", 112.0, i=1, direction=-1)
    ev.add("pass_end", 122.0, i=1, reason="reached")
    ev.save(str(c1 / "events.jsonl"))
    with open(c1 / "flight_log.csv", "w", newline="") as f:     # x = 0.2 * (t - 100) on the way out, then back
        from agri_loop.drone.flightlog import COLS
        f.write(",".join(COLS) + "\n")
        for k in range(0, 241):
            t = 100.0 + k * 0.1
            x = 0.2 * (t - 100) if t <= 110 else (2.0 if t <= 112 else 2.0 - 0.2 * (t - 112))
            f.write(",".join(repr(t) if c == "t" else repr(x) if c == "x" else "0" for c in COLS) + "\n")
    with open(c1 / "frames" / "index.csv", "w", newline="") as f:
        f.write("idx,t,file\n")
        for k in range(0, 24):
            f.write(f"{k},{100.0 + k:.1f},f{k:03d}.png\n")
    got = plant_frames(str(c1), {"zone": [1.0, 1.2, -0.1, 0.1], "footprint": 0.25})
    # outbound x in [0.75, 1.45] -> t in [103.75, 107.25]: frames at 104..107 only (none from the return leg)
    assert got == ["f004.png", "f005.png", "f006.png", "f007.png"]
