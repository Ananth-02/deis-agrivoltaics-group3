import pytest

from agri_loop.link.eventlog import EventLog
from agri_loop.link.messages import (EVENT_APPROACH, FSM_FAULT, FSM_HOLD, FSM_IDLE_TRACK, DroneBeacon, Heartbeat,
                                     PanelEvent, PanelState, decode, encode)
from agri_loop.link.panel_sim import PanelConfig, PanelSim
from agri_loop.link.scenario import ScenarioConfig, run_scenario


def test_codec_roundtrip():
    msgs = [
        DroneBeacon(seq=7, t_send=12.5, station_id=1, x=0.1, y=-0.2, z=0.6, vx=0.15, vy=0.0, vz=0.0, heading_deg=90.0, mission_type=1),
        PanelEvent(seq=3, t_send=1.25, panel_id=1, event_type=EVENT_APPROACH, mission_type=1, target_angle_deg=90.0),
        PanelState(seq=9, t_send=2.0, panel_id=1, fsm_state=3, angle_deg=89.5, target_deg=90.0, current_a=0.2, ack_seq=3),
        Heartbeat(seq=4, t_send=3.0),
    ]
    for m in msgs:
        d = decode(encode(m))
        assert type(d) is type(m)
        for k, v in m.__dict__.items():
            assert getattr(d, k) == pytest.approx(v, abs=1e-5)


def test_codec_rejects_bad_packets():
    with pytest.raises(ValueError):
        decode(b"")
    with pytest.raises(ValueError):
        decode(b"\x09abc")
    with pytest.raises(ValueError):
        decode(encode(Heartbeat(1, 1.0))[:-1])


def test_no_loss_meets_requirements():
    r = run_scenario(ScenarioConfig(duration_s=200, n_passes=6, loss=0.0))["summary"]
    assert r["streams"]["DroneBeacon"]["pdr"] == 1.0
    assert r["events"]["events_reached"] == 6
    assert r["events"]["events_failed"] == 0
    assert r["timing_success"] == 1.0
    # 70 degrees at 90 deg/s is about 0.78 s, so well under the 2 s requirement
    assert 0.7 < r["events"]["latency_ms_median"] / 1000 < 1.2


def test_loss_is_visible_in_pdr_but_events_still_arrive():
    r = run_scenario(ScenarioConfig(duration_s=300, n_passes=10, loss=0.10, seed=2))["summary"]
    assert 0.85 < r["streams"]["DroneBeacon"]["pdr"] < 0.95
    assert r["events"]["events_failed"] == 0
    assert r["events"]["events_reached"] >= 9


def test_panel_released_only_after_drone_has_passed():
    """Regression: the gateway once sent CLEARED before the drone reached the zone."""
    sc = ScenarioConfig(duration_s=60, n_passes=1, loss=0.0)
    log = run_scenario(sc)["log"].events
    approach = next(e for e in log if e["ev"] == "send" and e["msg"] == "PanelEvent" and e["seq"] == 1)
    clear = next(e for e in log if e["ev"] == "send" and e["msg"] == "PanelEvent" and e["seq"] == 2)
    # drone x(t) = -1 + 0.15 (t - 5); the zone ends at x = 0.8 (+ footprint 0.25 + release margin 0.1)
    t_leaves = 5 + (0.8 + 0.25 + 0.10 + 1.0) / 0.15
    assert approach["t"] < clear["t"]
    assert clear["t"] >= t_leaves


def _panel():
    sent, log = [], EventLog()
    p = PanelSim(sent.append, log, PanelConfig(imu_noise_deg=0.0))
    return p, sent, log


def test_panel_rejects_stale_event_and_ignores_duplicates():
    p, sent, log = _panel()
    p.tick(0.0)
    p.on_event(PanelEvent(1, t_send=0.0, panel_id=1, event_type=EVENT_APPROACH, mission_type=1, target_angle_deg=90.0), now=1.0)
    assert p.fsm == FSM_IDLE_TRACK and p.ack_seq == 0          # too old (> 0.5 s)
    p.on_event(PanelEvent(1, t_send=1.0, panel_id=1, event_type=EVENT_APPROACH, mission_type=1, target_angle_deg=90.0), now=1.1)
    assert p.ack_seq == 1
    p.on_event(PanelEvent(1, t_send=1.2, panel_id=1, event_type=EVENT_APPROACH, mission_type=1, target_angle_deg=45.0), now=1.3)
    assert p.target == 90.0                                    # duplicate does not change the target


def test_panel_rejects_out_of_range_angle():
    p, sent, log = _panel()
    p.tick(0.0)
    p.on_event(PanelEvent(1, t_send=0.0, panel_id=1, event_type=EVENT_APPROACH, mission_type=1, target_angle_deg=140.0), now=0.1)
    assert p.target == p.cfg.default_angle


def test_watchdog_goes_to_fault_and_recovers():
    p, sent, log = _panel()
    p.on_heartbeat(Heartbeat(1, 0.0), now=0.0)
    t = 0.0
    while t < 2.0:                       # no further heartbeats
        p.tick(t)
        t += 0.01
    assert p.fsm == FSM_FAULT
    while t < 5.0:                       # heartbeats back
        p.on_heartbeat(Heartbeat(2, t), now=t)
        p.tick(t)
        t += 0.01
    assert p.fsm == FSM_IDLE_TRACK
