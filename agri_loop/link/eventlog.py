"""Append-only event log shared by all components; written as JSON lines."""
from __future__ import annotations

import json
from typing import Any, Dict, List


class EventLog:
    def __init__(self) -> None:
        self.events: List[Dict[str, Any]] = []

    def add(self, ev: str, t: float, **kw: Any) -> None:
        d = {"ev": ev, "t": round(float(t), 6)}
        d.update(kw)
        self.events.append(d)

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            for d in self.events:
                f.write(json.dumps(d) + "\n")

    @staticmethod
    def load(path: str) -> List[Dict[str, Any]]:
        with open(path, "r", encoding="utf-8") as f:
            return [json.loads(l) for l in f if l.strip()]
