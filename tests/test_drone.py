import csv
import os
import threading
import time

import numpy as np
import pytest

from agri_loop.drone.c1_analysis import C1Criteria, analyse_run, combine, write_summary
from agri_loop.drone.cpx_stream import (FMT_JPEG, FMT_RAW, PROTO_NANOCOCKPIT, FrameRecorder, decode_frame,
                                        default_host, default_protocol, encode_nanocockpit_packets, encode_packets,
                                        read_frame, read_frame_nanocockpit)
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


def test_nanocockpit_hand_built_packets():
    """Bytes written out from the NanoCockpit source (tcp.py, streamer.c, streamer.h), not from our encoder."""
    import struct
    w, h = 4, 3
    pixels = bytes(range(12))
    meta = bytearray(148)
    meta[0] = 10                                   # metadata_version
    meta[1:5] = struct.pack("<HH", w, h)           # frame_width, frame_height
    meta[5], meta[6], meta[7] = 1, 0, 42           # frame_bpp, frame_format GRAY_8, frame_id
    meta[8:12] = struct.pack("<I", 123456)         # frame_timestamp [usec]
    buf = bytes(meta) + pixels                     # 160 bytes
    first, rest = buf[:100], buf[100:]             # 60 bytes left, already a multiple of 4 (no padding)
    begin = b"\x10\x01" + struct.pack("<II", len(buf), 0) + b"\x00\x00" + first
    data = b"\x11\x00\x00\x00" + rest
    console = b"hello"                             # a CPX console packet (function 0x02) is skipped
    stream = (struct.pack("<H", len(console)) + b"\x61\x02" + console
              + struct.pack("<H", len(begin)) + b"\x63\x06" + begin
              + struct.pack("<H", len(data)) + b"\x63\x06" + data)
    fr = read_frame_nanocockpit(_reader(stream))
    assert (fr.width, fr.height, fr.deck_id, fr.deck_t_us) == (4, 3, 42, 123456)
    assert np.array_equal(decode_frame(fr), np.frombuffer(pixels, np.uint8).reshape(3, 4))


def test_nanocockpit_roundtrip_padding_checksum_and_skips():
    rng = np.random.default_rng(1)
    img = rng.integers(0, 256, (160, 160), dtype=np.uint8)
    other = encode_nanocockpit_packets(np.zeros((2, 2), np.uint8))
    other = other[:5] + b"\xf0" + other[6:]          # same buffer marked INFERENCE (0xF0): not an image
    bad = bytearray(encode_nanocockpit_packets(img, checksum=True, mtu=1000))
    bad[-10] ^= 0xFF                                  # corrupt a pixel: crc32 fails, frame dropped
    good = encode_nanocockpit_packets(img, frame_id=300, t_us=7, checksum=True, mtu=1000)
    fr = read_frame_nanocockpit(_reader(other + bytes(bad) + good))
    assert (fr.width, fr.height, fr.deck_id, fr.deck_t_us) == (160, 160, 300 & 0xFF, 7)
    assert np.array_equal(decode_frame(fr), img)


def test_nanocockpit_unknown_metadata_version_is_reported():
    pk = bytearray(encode_nanocockpit_packets(np.zeros((2, 2), np.uint8)))
    pk[4 + 12] = 11                                   # metadata_version right after the BEGIN header
    with pytest.raises(ValueError, match="v11"):
        read_frame_nanocockpit(_reader(bytes(pk)))


def test_recorder_nanocockpit(tmp_path):
    deck = FakeAIDeck(lambda: (0.0, 0.0, 0.5), fps=30, protocol=PROTO_NANOCOCKPIT).start()
    rec = FrameRecorder(str(tmp_path / "frames"), "127.0.0.1", deck.port, protocol=PROTO_NANOCOCKPIT).start()
    time.sleep(1.0)
    st = rec.stop()
    deck.stop()
    assert st.frames >= 10 and (st.width, st.height) == (160, 160) and not st.error
    with open(tmp_path / "frames" / "meta.csv", newline="") as f:
        ids = [int(r["deck_id"]) for r in csv.DictReader(f)]
    assert ids[:3] == [0, 1, 2]


def test_aideck_defaults_from_environment(monkeypatch):
    monkeypatch.delenv("AGRI_AIDECK_PROTO", raising=False)
    monkeypatch.delenv("AGRI_AIDECK_IP", raising=False)
    assert default_protocol() == "bitcraze" and default_host("bitcraze") == "192.168.4.1"
    assert default_host("nanocockpit") == "aideck.local"
    monkeypatch.setenv("AGRI_AIDECK_PROTO", "NanoCockpit")
    monkeypatch.setenv("AGRI_AIDECK_IP", "192.168.1.37")
    assert default_protocol() == "nanocockpit" and default_host("nanocockpit") == "192.168.1.37"
    monkeypatch.setenv("AGRI_AIDECK_PROTO", "tello")
    with pytest.raises(ValueError):
        default_protocol()


def test_flightlog_ignores_samples_after_close(tmp_path):
    """A radio log packet can still arrive while the link shuts down (seen on the Pi, 9 Nov)."""
    log = FlightLogger(str(tmp_path / "flight_log.csv"))
    log.add_fast(1.0, 10, {"x": 0.1})
    log.close()
    log.add_fast(2.0, 20, {"x": 0.2})   # must not raise
    log.close()                          # closing twice is harmless
    assert log.rows == 1


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


def test_trapezoid_profile_ends_and_cruises():
    from agri_loop.drone.mission import trapezoid
    d, total = trapezoid(0.0, 1.5, 0.2, 0.2)
    assert d == 0.0 and total == pytest.approx(1.5 / 0.2 + 0.2 / 0.2)
    assert trapezoid(total, 1.5, 0.2, 0.2)[0] == pytest.approx(1.5)
    mid1, _ = trapezoid(3.0, 1.5, 0.2, 0.2)
    mid2, _ = trapezoid(4.0, 1.5, 0.2, 0.2)
    assert mid2 - mid1 == pytest.approx(0.2)            # cruising at 0.2 m/s
    short, t_short = trapezoid(99, 0.05, 0.2, 0.2)       # too short to reach cruise speed
    assert short == pytest.approx(0.05) and t_short == pytest.approx(2 * (0.05 / 0.2) ** 0.5)


def test_sim_mission_position_control(tmp_path):
    run = tmp_path / "run"
    (run / "frames").mkdir(parents=True)
    logger = FlightLogger(str(run / "flight_log.csv"))
    v = SimVehicle(logger).open()
    ev = EventLog()
    cfg = C1Config(n_passes=2, pass_length_m=0.6, speed_m_s=0.2, accel_m_s2=0.4, pause_s=0.3,
                   takeoff_s=1.0, settle_s=0.5, land_s=0.8, pass_control="position")
    res = Mission(v, cfg, ev).run()
    logger.close()
    v.close()
    ev.save(str(run / "events.jsonl"))
    assert res["passes_done"] == 2 and not res["aborted"]
    ends = [e for e in EventLog.load(str(run / "events.jsonl")) if e["ev"] == "pass_end"]
    assert [e["reason"] for e in ends] == ["reached", "reached"]
    s = analyse_run(str(run), C1Criteria(n_passes=2, fps_min=0))
    assert all(abs(p["speed_mean"] - 0.2) < 0.03 for p in s["passes"]), s["passes"]
    assert all(p["lateral_rms_m"] < 0.02 for p in s["passes"])


def test_recorder_reconnects_after_drop(tmp_path):
    deck = FakeAIDeck(lambda: (0.0, 0.0, 0.5), fps=30, drop_after=10).start()
    rec = FrameRecorder(str(tmp_path / "frames"), "127.0.0.1", deck.port, retry_s=0.2).start()
    time.sleep(2.0)
    st = rec.stop()
    deck.stop()
    assert st.reconnects >= 1 and len(st.gaps) == st.reconnects
    assert st.frames > 20                      # frames kept coming after the drop
    with open(tmp_path / "frames" / "index.csv", newline="") as f:
        idx = [int(r["idx"]) for r in csv.DictReader(f)]
    assert idx == list(range(len(idx)))       # numbering continues across the reconnect
