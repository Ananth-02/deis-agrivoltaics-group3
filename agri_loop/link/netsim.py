"""Virtual-time network used by the C4 harness.

Software stand-in for `tc netem`: independent loss, fixed delay plus uniform
jitter. Time is virtual, so a 10 minute run takes a few seconds. The same
components also run over real UDP (see udp.py) for the real measurements.
"""
from __future__ import annotations

import heapq
import random
from typing import Callable, List, Tuple


class SimNet:
    def __init__(self, loss: float = 0.0, delay_s: float = 0.010, jitter_s: float = 0.005, seed: int = 0):
        self.loss = loss
        self.delay = delay_s
        self.jitter = jitter_s
        self.rng = random.Random(seed)
        self._q: List[Tuple[float, int, bytes, Callable[[bytes, float], None]]] = []
        self._n = 0
        self.sent = 0
        self.dropped = 0

    def send(self, data: bytes, now: float, deliver: Callable[[bytes, float], None]) -> None:
        self.sent += 1
        if self.rng.random() < self.loss:
            self.dropped += 1
            return
        t = now + self.delay + self.rng.uniform(0, self.jitter)
        self._n += 1
        heapq.heappush(self._q, (t, self._n, data, deliver))

    def deliver_until(self, now: float) -> None:
        while self._q and self._q[0][0] <= now:
            t, _, data, deliver = heapq.heappop(self._q)
            deliver(data, t)
