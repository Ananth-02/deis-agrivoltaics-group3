import socket
import time

from agri_loop.link.eventlog import EventLog
from agri_loop.link.messages import (EVENT_ABORT, EVENT_APPROACH, EVENT_CLEARED, FSM_ACTUATING, FSM_FAULT, FSM_HOLD,
                                     FSM_IDLE_TRACK, FSM_RESTORE, MISSION_CROP_OCCLUSION, MISSION_PANEL_VIEW,
                                     Heartbeat, PanelEvent)
from agri_loop.link.panel_adapter import FakeArduino, PanelAdapter


def _make():
    states, sent, warns = [], [], []
    ad = PanelAdapter(states.append, sent.append, EventLog(), warn=warns.append, sleep=lambda s: None)
    return ad, states, sent, warns


def _ev(seq, t, etype=EVENT_APPROACH, mission=MISSION_CROP_OCCLUSION, angle=0.0):
    return PanelEvent(seq=seq, t_send=t, panel_id=1, event_type=etype, mission_type=mission, target_angle_deg=angle)


def test_sends_status_before_any_event_and_maps_missions():
    ad, states, sent, _ = _make()
    ad.tick(0.0)
    assert sent[-1] == "Status"
    ad.on_event(_ev(1, 0.1), 0.1)
    ad.tick(0.2)
    assert sent[-1] == "Plant_inspection"
    ad.on_event(_ev(2, 0.3, mission=MISSION_PANEL_VIEW), 0.3)
    ad.tick(0.4)
    assert sent[-1] == "Panel_inspection"
    for i, et in enumerate((EVENT_CLEARED, EVENT_ABORT)):
        ad.on_event(_ev(3 + i, 0.5 + i, etype=et), 0.5 + i)
        ad.tick(0.6 + i)
        assert sent[-1] == "Done"


def test_commands_repeat_at_20_hz():
    ad, _, sent, _ = _make()
    for k in range(21):
        ad.tick(k * 0.01)       # 0.2 s at 100 Hz
    assert 4 <= len(sent) <= 5


def test_ack_only_after_matching_reply_and_fsm_mapping():
    ad, states, _, _ = _make()
    ad.tick(0.0)
    ad.on_arduino("SUN_CATCH,45", 0.05)
    assert states[-1].fsm_state == FSM_IDLE_TRACK and states[-1].ack_seq == 0
    ad.on_event(_ev(1, 0.1), 0.1)
    ad.on_arduino("SUN_CATCH,30", 0.15)                 # old state still: no ack
    assert states[-1].ack_seq == 0 and states[-1].fsm_state == FSM_RESTORE
    ad.on_arduino("PLANT_INSPECTION,40", 0.2)           # command taken, still moving
    assert states[-1].ack_seq == 1 and states[-1].fsm_state == FSM_ACTUATING
    assert states[-1].target_deg == 0.0 and states[-1].angle_deg == 40.0
    ad.on_arduino("PLANT_INSPECTION,0", 0.3)
    assert states[-1].fsm_state == FSM_HOLD
    ad.on_event(_ev(2, 0.4, etype=EVENT_CLEARED), 0.4)
    ad.on_arduino("PLANT_INSPECTION,0", 0.45)
    assert states[-1].ack_seq == 1
    ad.on_arduino("SUN_CATCH,10", 0.5)
    assert states[-1].ack_seq == 2 and states[-1].fsm_state == FSM_RESTORE


def test_duplicate_old_and_stale_events():
    ad, states, sent, _ = _make()
    ad.on_event(_ev(5, 0.0), 0.0)
    ad.on_arduino("PLANT_INSPECTION,10", 0.1)
    ad.on_event(_ev(5, 0.2), 0.2)                       # duplicate: acked again, no second action
    ad.on_event(_ev(4, 0.2, etype=EVENT_CLEARED), 0.2)  # older: ignored
    ad.tick(0.3)
    assert sent[-1] == "Plant_inspection" and states[-1].ack_seq == 5
    ad.on_event(_ev(6, 0.0, etype=EVENT_CLEARED), 0.9)  # 0.9 s old
    ad.tick(1.0)
    assert sent[-1] == "Plant_inspection"
    assert any(e["ev"] == "reject_stale" for e in ad.log.events)


def test_heartbeat_timeout_sends_done_and_logs_fault():
    ad, states, sent, _ = _make()
    ad.on_heartbeat(Heartbeat(1, 0.0), 0.0)
    ad.on_event(_ev(1, 0.1), 0.1)
    ad.tick(0.5)
    assert sent[-1] == "Plant_inspection"
    ad.tick(1.2)                                        # 1.2 s without heartbeat
    assert sent[-1] == "Done"
    assert sum(e["ev"] == "fault" for e in ad.log.events) == 1
    ad.on_arduino("SUN_CATCH,45", 1.3)
    assert states[-1].fsm_state == FSM_FAULT
    ad.on_heartbeat(Heartbeat(2, 1.4), 1.4)
    assert any(e["ev"] == "fault_cleared" for e in ad.log.events)


def test_warns_once_when_arduino_is_silent_and_shutdown_sends_done():
    ad, _, sent, warns = _make()
    ad.tick(0.0)
    ad.tick(1.5)
    ad.tick(2.0)
    assert len(warns) == 1
    ad.shutdown()
    assert sent[-3:] == ["Done"] * 3


def test_fake_arduino_round_trip_over_udp():
    fake = FakeArduino("127.0.0.1", 0, deg_per_s=2000.0).start()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)

    def ask(text):
        s.sendto(text.encode(), ("127.0.0.1", fake.port))
        return s.recvfrom(64)[0].decode()

    try:
        assert ask("Status") == "SUN_CATCH,45"
        assert ask("Plant_inspection").startswith("PLANT_INSPECTION,")
        time.sleep(0.1)
        assert ask("Status") == "PLANT_INSPECTION,0"
        assert ask("Panel_inspection").startswith("PANEL_INSPECTION,")
        time.sleep(0.1)
        assert ask("Done").startswith("SUN_CATCH,")
        time.sleep(0.1)
        assert ask("Status") == "SUN_CATCH,45"
        assert fake.commands == ["Plant_inspection", "Panel_inspection", "Done"]
    finally:
        s.close()
        fake.stop()


def test_fake_arduino_slews_one_degree_per_15_ms():
    fake = FakeArduino("127.0.0.1", 0)
    fake.handle("Plant_inspection", fake._t)
    assert fake.handle("Status", fake._t + 0.15) == "PLANT_INSPECTION,35"
    fake.sock.close()


def test_shutdown_repeats_done_until_sun_catch_is_confirmed():
    ad, _, sent, warns = _make()
    replies = [[], [], ["SUN_CATCH,45"]]
    assert ad.shutdown(lambda: replies.pop(0) if replies else []) is True
    assert sent == ["Done"] * 3 and not warns
    assert ad.log.events[-1]["ev"] == "shutdown_done" and ad.log.events[-1]["confirmed"] is True
    ad2, _, sent2, warns2 = _make()
    assert ad2.shutdown(lambda: ["PLANT_INSPECTION,0"]) is False       # never confirmed: gives up after 2 s
    assert len(sent2) >= 19 and len(warns2) == 1


def test_a_failing_send_does_not_stop_the_adapter():
    states, warns = [], []

    def boom(cmd):
        raise OSError("network is unreachable")
    ad = PanelAdapter(states.append, boom, EventLog(), warn=warns.append, sleep=lambda s: None)
    ad.tick(0.0)
    ad.tick(0.1)
    ad.tick(1.5)                                          # counts as silence
    assert len(warns) == 1
    assert sum(e["ev"] == "arduino_send_error" for e in ad.log.events) == 1
