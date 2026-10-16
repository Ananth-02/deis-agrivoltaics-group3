import csv
import os
import threading
import time

import numpy as np
import pytest

from agri_loop.drone.c1_analysis import C1Criteria, analyse_run, combine, write_summary
from agri_loop.drone.cpx_stream import FMT_JPEG, FMT_RAW, FrameRecorder, decode_frame, encode_packets, read_frame
from agri_loop.drone.flightlog import FlightLogger
from agri_loop.drone.mission import C1Config, Mission
from agri_loop.drone.sim import FakeAIDeck, SimVehicle
from agri_loop.link.eventlog import EventLog


def _reader(buf: bytes):
    pos = [0]

    def rx(n):
        b = buf[pos[0]:pos[0] + n]
        if len(b) < n:
            raise EOFError
        pos[0] += n
        return b
    return rx


def test_cpx_roundtrip_raw_reads_size_from_header():
    img = (np.arange(324 * 244) % 251).astype(np.uint8).reshape(244, 324)
    fr = read_frame(_reader(encode_packets(img, FMT_RAW)))
    assert (fr.width, fr.height, fr.fmt) == (324, 244, FMT_RAW)
    assert np.array_equal(decode_frame(fr), img)


def test_cpx_native_size_and_jpeg():
    img = np.full((324, 324), 128, np.uint8)
    img[100:200, 50:150] = 30
    fr = read_frame(_reader(encode_packets(img, FMT_JPEG)))
    assert (fr.width, fr.height, fr.fmt) == (324, 324, FMT_JPEG)
    out = decode_frame(fr)
    assert out.shape == (324, 324) and abs(int(out[150, 100]) - 30) < 10


def test_cpx_skips_garbage_packet_and_flips():
    import struct
    img = np.zeros((4, 6), np.uint8)
    img[:, 0] = 255
    junk = struct.pack("<HBB", 5, 9, 0) + b"abc"
    fr = read_frame(_reader(junk + encode_packets(img)))
    assert np.array_equal(decode_frame(fr, flip="h")[:, -1], np.full(4, 255, np.uint8))


def test_recorder_writes_c5_index(tmp_path):
    deck = FakeAIDeck(lambda: (0.0, 0.0, 0.5), fps=30).start()
    rec = FrameRecorder(str(tmp_path / "frames"), "127.0.0.1", deck.port).start()
    time.sleep(1.0)
    st = rec.stop()
    deck.stop()
    assert st.frames >= 10 and st.width == 324 and st.height == 244
    with open(tmp_path / "frames" / "index.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0].keys()) == ["idx", "t", "file"]
    assert os.path.exists(tmp_path / "frames" / rows[0]["file"])


def _sim_run(tmp_path, n_passes=2, link_fail_at_s=None, vbat0=4.15):
    run = tmp_path / "run"
    (run / "frames").mkdir(parents=True)
    logger = FlightLogger(str(run / "flight_log.csv"))
    v = SimVehicle(logger, vbat0=vbat0, link_fail_at_s=link_fail_at_s).open()
    deck = FakeAIDeck(v.pose, fps=25).start()
    rec = FrameRecorder(str(run / "frames"), "127.0.0.1", deck.port).start()
    ev = EventLog()
    cfg = C1Config(n_passes=n_passes, pass_length_m=0.4, speed_m_s=0.3, accel_m_s2=1.0, pause_s=0.3,
                   takeoff_s=1.0, settle_s=0.5, land_s=0.8)
    res = Mission(v, cfg, ev).run()
    rec.stop()
    deck.stop()
    logger.close()
    v.close()
    ev.save(str(run / "events.jsonl"))
    return str(run), res


def test_sim_mission_scores_as_c1(tmp_path):
    run, res = _sim_run(tmp_path, n_passes=2)
    assert res["passes_done"] == 2 and not res["aborted"]
    crit = C1Criteria(n_passes=2, fps_min=15)
    s = combine([analyse_run(run, crit)], crit)
    write_summary(s, run)
    assert s["checks"]["passes"]["ok"], s["passes"]
    assert s["checks"]["battery_log"]["ok"] and s["checks"]["no_crash"]["ok"]
    p = s["passes"][0]
    assert p["direction"] == 1 and s["passes"][1]["direction"] == -1
    assert abs(p["speed_mean"] - 0.3) < 0.05 and p["frames"] > 5
    assert os.path.exists(os.path.join(run, "c1_summary.md"))


def test_mission_refuses_takeoff_on_low_battery(tmp_path):
    _run, res = _sim_run(tmp_path, n_passes=1, vbat0=3.6)
    assert res["aborted"] and "battery" in res["reason"] and res["passes_done"] == 0


def test_link_loss_lands(tmp_path):
    run, res = _sim_run(tmp_path, n_passes=3, link_fail_at_s=2.0)
    assert res["aborted"] and res["reason"] == "radio link lost"
    ev = EventLog.load(os.path.join(run, "events.jsonl"))
    assert any(e["ev"] == "landed" for e in ev)


def test_beacon_reaches_udp_gateway(tmp_path):
    from agri_loop.drone.beacon import BeaconSender
    from agri_loop.link.messages import MISSION_CROP_OCCLUSION, DroneBeacon
    from agri_loop.link.udp import UdpPort

    gw = UdpPort(("127.0.0.1", 0), ("127.0.0.1", 9))
    port = gw.sock.getsockname()[1]
    flag = {"p": True}
    b = BeaconSender(lambda: {"x": 0.5, "y": 0.1, "z": 0.6, "vx": 0.2, "yaw": 1.0}, lambda: flag["p"],
                     ("127.0.0.1", port), offset=(1.0, 2.0)).start()
    time.sleep(0.55)
    b.stop(str(tmp_path / "drone.jsonl"))
    got = [m for m, _t in gw.poll()]
    gw.close()
    assert len(got) >= 4 and all(isinstance(m, DroneBeacon) for m in got)
    assert got[0].x == pytest.approx(1.5) and got[0].y == pytest.approx(2.1)
    assert got[0].mission_type == MISSION_CROP_OCCLUSION
    assert EventLog.load(str(tmp_path / "drone.jsonl"))[0]["msg"] == "DroneBeacon"


class _FakeCF:
    def __init__(self, decks, **state):
        self._decks, self._s, self.missing_vars = decks, state, []

    def decks(self):
        return self._decks

    def latest(self):
        return self._s


_GROUND = dict(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, vbat=4.1, sup_info=1)


@pytest.mark.parametrize("decks,extra,ok,source,text", [
    ({"bcFlow2": True, "bcAI": True}, {}, True, "flow", ""),
    ({"bcLighthouse4": True, "bcAI": True}, {"lh_status": 2, "lh_active": 0b11}, True, "lighthouse", ""),
    ({"bcLighthouse4": True, "bcAI": True}, {"lh_status": 1, "lh_active": 0}, False, "none", "geometry"),
    ({"bcLighthouse4": True, "bcAI": True}, {"lh_status": 0, "lh_active": 0}, False, "none", "no base stations"),
    ({"bcLighthouse4": True, "bcAI": True}, {"lh_status": 2, "lh_active": 0b1}, False, "none", "at least 2"),
    ({"bcLighthouse4": True, "bcFlow2": True, "bcAI": True}, {"lh_status": 0}, True, "flow", ""),
    ({"bcAI": True}, {}, False, "none", "no positioning"),
])
def test_preflight_positioning(decks, extra, ok, source, text):
    from agri_loop.drone.c1_common import preflight
    passed, report, problems = preflight(_FakeCF(decks, **dict(_GROUND, **extra)), C1Config(), need_camera=True)
    assert passed is ok and report["positioning"] == source
    assert text in " ".join(problems)


def test_preflight_warns_when_not_facing_lighthouse_x():
    from agri_loop.drone.c1_common import preflight
    s = dict(_GROUND, yaw=40.0, lh_status=2, lh_active=0b11)
    passed, report, _ = preflight(_FakeCF({"bcLighthouse4": True, "bcAI": True}, **s), C1Config(), need_camera=True)
    assert passed and any("Lighthouse x axis" in w for w in report["warnings"])
