import socket
import threading

from agri_loop.drone.measure_points import PanelLink, StillDetector, derive


def _feed(det, start_t, pos, seconds, jitter=0.0, rate=20):
    n = int(seconds * rate)
    for i in range(n):
        j = jitter if i % 2 else -jitter
        det.add(start_t + i / rate, (pos[0] + j, pos[1], pos[2]))
    return start_t + n / rate


def test_still_detector_waits_for_a_full_still_window():
    det = StillDetector()
    _feed(det, 0.0, (1.0, 0.0, 0.0), 1.0)
    assert det.check() is None                      # not still long enough yet
    _feed(det, 1.0, (1.0, 0.0, 0.0), 2.0, jitter=0.003)
    m = det.check()
    assert m is not None and abs(m[0] - 1.0) < 0.005


def test_still_detector_ignores_shaking_and_the_last_spot():
    det = StillDetector()
    _feed(det, 0.0, (0.0, 0.0, 0.0), 3.0, jitter=0.05)
    assert det.check() is None                      # moving too much
    det.reset()
    t = _feed(det, 10.0, (0.0, 0.0, 0.0), 3.0)
    det.accept(det.check())
    _feed(det, t, (0.03, 0.0, 0.0), 3.0)            # still, but only 3 cm from the last point
    assert det.check() is None
    det.reset()
    _feed(det, 20.0, (0.5, 0.0, 0.0), 3.0)
    assert det.check() is not None


def _pt(x, y, z):
    return dict(pos=[x, y, z], spread_m=0.005, samples=50, how="auto")


def test_derive_mission_numbers():
    pts = {
        "origin": _pt(0.0, 0.0, 0.01), "towards_strip": _pt(0.5, 0.02, 0.01),
        "big_panel_1": _pt(0.1, 0.2, 0.35), "big_panel_2": _pt(0.7, 0.2, 0.40),
        "takeoff": _pt(1.4, -0.1, 0.01), "corner_strip": _pt(1.6, 0.9, 0.01),
        "staging": _pt(-0.6, -0.5, 0.01), "pot": _pt(-1.0, -0.6, 0.15), "leaf": _pt(-1.0, -0.6, 0.30),
        "post_a": _pt(-1.0, -0.4, 0.45), "post_b": _pt(-1.0, -0.8, 0.45),
        "panel_top_vertical": _pt(-1.0, -0.6, 0.55), "panel_flat_strip_side": _pt(-0.85, -0.6, 0.45),
        "panel_flat_far_side": _pt(-1.15, -0.6, 0.45), "corner_rig": _pt(-1.1, -1.1, 0.01),
    }
    d, warn = derive(pts)
    assert d["towards_strip_axis"] == "+x"
    assert d["arena_size_m"] == [2.7, 2.0]
    assert d["fence"] == {"x": [-0.9, 1.4], "y": [-0.9, 0.7]}
    assert d["over_plant_z"] == 0.84                 # panel top 0.54 above the floor + 0.3
    assert d["cruise_z"] == 0.84
    assert d["OVER_PLANT"] == [-1.0, -0.6, 0.84]
    assert d["STAGING"] == [-0.6, -0.5, 0.84]
    assert d["keep_out"]["x"] == [-1.3, -0.7]
    assert any("pot is within" in w for w in warn)   # pot only 0.1 m from the rig-side wall here
    assert not any("staging point is inside" in w for w in warn)


def test_derive_warns_for_staging_inside_keep_out():
    pts = {"post_a": _pt(0.0, 0.0, 0.4), "panel_flat_strip_side": _pt(0.3, 0.0, 0.4),
           "staging": _pt(0.2, 0.0, 0.0)}
    _, warn = derive(pts)
    assert any("staging point is inside" in w for w in warn)


def test_panel_link_waits_for_confirmation():
    ard = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ard.bind(("127.0.0.1", 0))
    ard.settimeout(2.0)
    replies = iter(["SUN_CATCH,30", "PLANT_INSPECTION,10", "PLANT_INSPECTION,0"])

    def arduino():
        for r in replies:
            _, addr = ard.recvfrom(64)
            ard.sendto(r.encode(), addr)

    th = threading.Thread(target=arduino)
    th.start()
    link = PanelLink("127.0.0.1", ard.getsockname()[1])
    ok, reply = link.set("Plant_inspection", timeout_s=3.0)
    th.join()
    link.close()
    ard.close()
    assert ok and reply == "PLANT_INSPECTION,0"


def test_not_ready_names_the_reason():
    from agri_loop.drone.measure_points import not_ready

    now = 100.0
    ok = dict(x=0.0, y=0.0, z=0.0, t=now - 0.05, lh_status=2, lh_active=13)
    assert not_ready(ok, now) == ""
    assert not_ready({}, now).startswith("no position")
    assert "ms old" in not_ready(dict(ok, t=now - 0.5), now)
    assert "base stations=1" in not_ready(dict(ok, lh_status=1, lh_active=1), now)
