"""Real UDP transport for the same components (RPi, laptop, localhost).

Usage pattern: one UdpPort per node, `send(msg)` encodes and sends, `poll()`
returns decoded messages that arrived. Timestamps are taken from time.time()
on the receiving host, so run all nodes on one host (or chrony-synchronised
hosts) when measuring latency.
"""
from __future__ import annotations

import random
import socket
import time
from typing import List, Tuple

from .messages import decode, encode


class UdpPort:
    def __init__(self, bind: Tuple[str, int], peer: Tuple[str, int], rx_loss: float = 0.0, seed: int = 0):
        self.rx_loss = rx_loss          # software loss injector on receive (any OS; use tc netem on Linux if possible)
        self._rng = random.Random(seed)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(bind)
        self.sock.setblocking(False)
        self.peer = peer

    def send(self, msg) -> None:
        self.sock.sendto(encode(msg), self.peer)

    def poll(self) -> List[Tuple[object, float]]:
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except ConnectionResetError:  # Windows: ICMP port unreachable from a peer that is not up yet
                continue
            if self.rx_loss and self._rng.random() < self.rx_loss:
                continue
            try:
                out.append((decode(data), time.time()))
            except ValueError:
                pass
        return out

    def close(self) -> None:
        self.sock.close()
