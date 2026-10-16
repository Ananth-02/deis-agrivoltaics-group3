import time

from agri_loop.link.messages import DroneBeacon, Heartbeat
from agri_loop.link.udp import UdpPort


def _wait(port, n, timeout=1.0):
    got, t_end = [], time.time() + timeout
    while len(got) < n and time.time() < t_end:
        got += port.poll()
        time.sleep(0.005)
    return got


def test_udp_roundtrip_on_localhost():
    rx = UdpPort(("127.0.0.1", 0), ("127.0.0.1", 9))
    port = rx.sock.getsockname()[1]
    tx = UdpPort(("127.0.0.1", 0), ("127.0.0.1", port))
    tx.send(DroneBeacon(1, time.time(), 1, 0.5, 0.0, 0.6, 0.15, 0.0, 0.0, 0.0, 1))
    tx.send(Heartbeat(2, time.time()))
    got = _wait(rx, 2)
    assert [type(m).__name__ for m, _ in got] == ["DroneBeacon", "Heartbeat"]
    assert abs(got[0][0].x - 0.5) < 1e-6
    rx.close()
    tx.close()


def test_receive_side_loss_injector_drops_everything_at_one():
    rx = UdpPort(("127.0.0.1", 0), ("127.0.0.1", 9), rx_loss=1.0)
    tx = UdpPort(("127.0.0.1", 0), ("127.0.0.1", rx.sock.getsockname()[1]))
    for i in range(5):
        tx.send(Heartbeat(i, time.time()))
    assert _wait(rx, 1, timeout=0.3) == []
    rx.close()
    tx.close()
