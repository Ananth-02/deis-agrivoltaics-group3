"""cf_server and drone_bridge (step 1 of docs/drone_integration_plan.md): pose only, no flying.

drone_bridge needs ROS2, which is not installed on the laptops, so its test swaps in
small stand-ins for rclpy and the message types and checks only the translation.
"""
import importlib.util
import math
import os
import socket
import sys
import time
import types
from types import SimpleNamespace

from agri_loop.drone.cf_server import CfServer, format_state, parse_command
from agri_loop.drone.sim import SimVehicle
from agri_loop.link.eventlog import EventLog

BRIDGE_FILE = os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "group3_nodes", "group3_nodes",
                           "drone_bridge.py")


def _udp():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.setblocking(False)
    return s


def _drain(sock, wait=0.2):
    got, t_end = [], time.time() + wait
    while time.time() < t_end:
        try:
            got.append(sock.recvfrom(256)[0].decode())
        except BlockingIOError:
            time.sleep(0.005)
    return got


class FakeVehicle:
    def __init__(self, sample, link=True):
        self.sample, self.link = sample, link

    def latest(self):
        return dict(self.sample)

    def link_ok(self):
        return self.link


def _server(vehicle):
    bridge = _udp()
    srv_sock = _udp()
    server = CfServer(vehicle, srv_sock, bridge.getsockname(), EventLog())
    return server, srv_sock, bridge


def test_parse_command():
    assert parse_command("Status") == ("Status", ())
    assert parse_command("Goto,1.4,-0.1,0.8") == ("Goto", (1.4, -0.1, 0.8))
    assert parse_command("Takeoff,0.8") == ("Takeoff", (0.8,))
    assert parse_command("Land") == ("Land", ())
    for bad in ("", "Goto,1,2", "Goto,a,b,c", "Fly", "Land,1", "status"):
        assert parse_command(bad) is None


def test_format_state():
    s = dict(x=1.4021, y=0.6, z=0.0123, yaw=2.14, vbat=4.051)
    assert format_state("LANDED", s) == "LANDED,1.402,0.600,0.012,2.1,4.05"


def test_fresh_sample_is_reported():
    server, srv, bridge = _server(FakeVehicle(dict(x=1.0, y=2.0, z=0.0, yaw=0.0, vbat=4.0, t=time.time())))
    server.step()
    assert _drain(bridge) == ["LANDED,1.000,2.000,0.000,0.0,4.00"]


def test_stale_sample_or_lost_link_goes_silent():
    old = dict(x=1.0, y=2.0, z=0.0, vbat=4.0, t=time.time() - 0.5)
    server, srv, bridge = _server(FakeVehicle(old))
    server.step()
    assert _drain(bridge) == [] and server.stale == 1
    server.v = FakeVehicle(dict(old, t=time.time()), link=False)
    server.step()
    assert _drain(bridge) == []
    server.v = FakeVehicle({})   # nothing received from the drone yet
    server.step()
    assert _drain(bridge) == []
    assert [e["ev"] for e in server.ev.events] == ["pose_stale"]


def test_flying_commands_are_refused_once_and_never_change_state():
    server, srv, bridge = _server(FakeVehicle(dict(x=0.0, y=0.0, z=0.0, vbat=4.0, t=time.time())))
    sender = _udp()
    for _ in range(3):   # the bridge resends the same command
        sender.sendto(b"Goto,1.0,0.0,0.8", srv.getsockname())
    sender.sendto(b"Goto,nonsense", srv.getsockname())
    time.sleep(0.05)
    server.step()
    evs = [e["ev"] for e in server.ev.events]
    assert evs.count("command_refused") == 1 and evs.count("bad_command") == 1
    assert server.state == "LANDED"
    assert server.command_time is not None


def _load_bridge(monkeypatch):
    """Import drone_bridge with stand-ins for the ROS2 modules it uses."""
    class Node:
        def __init__(self, name):
            self.pubs, self.subs, self.timers, self.logged = {}, {}, [], []

        def create_publisher(self, _type, topic, _depth):
            self.pubs[topic] = []
            return SimpleNamespace(publish=self.pubs[topic].append)

        def create_subscription(self, _type, topic, cb, _depth):
            self.subs[topic] = cb

        def create_timer(self, period, cb):
            self.timers.append((period, cb))

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))

        def get_logger(self):
            return SimpleNamespace(info=self.logged.append)

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                                        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    class String:
        def __init__(self, data=""):
            self.data = data

    mods = {"rclpy": types.ModuleType("rclpy"), "rclpy.node": types.ModuleType("rclpy.node"),
            "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
            "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": types.ModuleType("std_msgs.msg")}
    mods["rclpy.node"].Node = Node
    mods["geometry_msgs.msg"].PoseStamped = PoseStamped
    mods["std_msgs.msg"].String = String
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)
    spec = importlib.util.spec_from_file_location("drone_bridge", BRIDGE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, String


def test_bridge_parse_state(monkeypatch):
    db, _ = _load_bridge(monkeypatch)
    assert db.parse_state("FLYING,1.4,0.6,0.8,2.1,3.95") == ("FLYING", 1.4, 0.6, 0.8, 2.1, 3.95)
    for bad in ("", "FLYING,1,2,3", "FLYING,a,0,0,0,4"):
        assert db.parse_state(bad) is None


def test_sim_drone_to_ros_topics_end_to_end(monkeypatch):
    """SimVehicle -> cf_server -> UDP -> drone_bridge -> PoseStamped, and a command back the other way."""
    db, String = _load_bridge(monkeypatch)
    srv_sock, bridge_sock = _udp(), _udp()
    monkeypatch.setattr(db, "LISTEN", bridge_sock.getsockname())
    monkeypatch.setattr(db, "SERVER", srv_sock.getsockname())
    bridge_sock.close()   # the node binds this port itself
    node = db.DroneBridge()

    vehicle = SimVehicle().open()
    try:
        server = CfServer(vehicle, srv_sock, db.LISTEN, EventLog())
        vehicle.s["yaw"] = 90.0
        time.sleep(0.05)
        for _ in range(4):
            server.step()
            time.sleep(0.01)
        node.receive()
        poses = node.pubs["/group3/cf/pose"]
        assert len(poses) == 4
        p = poses[-1]
        assert p.header.frame_id == "world" and p.header.stamp == "stamp"
        assert abs(p.pose.position.z) < 0.01
        assert abs(p.pose.orientation.z - math.sin(math.radians(45))) < 0.01
        assert node.pubs["/group3/cf/state"][-1].data.startswith("LANDED,4.")

        assert node.command == "Status"   # never orders anything by default
        node.subs["/group3/cf/cmd"](String(data="Goto,1.0,0.0,0.8"))
        node.send()
        time.sleep(0.05)
        server.step()
        assert server.command == "Goto,1.0,0.0,0.8" and server.state == "LANDED"
        assert sorted(p for p, _ in node.timers) == [0.02, 0.2]
    finally:
        vehicle.close()
        node.sock.close()
        srv_sock.close()
