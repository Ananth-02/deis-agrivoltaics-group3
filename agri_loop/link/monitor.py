"""Link statistics computed from an event log (simulated or real)."""
from __future__ import annotations

import statistics
from typing import Any, Dict, List, Optional


def _pct(xs: List[float], q: float) -> Optional[float]:
    if not xs:
        return None
    xs = sorted(xs)
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def stream_stats(events: List[Dict[str, Any]], msg: str) -> Dict[str, Any]:
    """Packet delivery ratio and one-way latency for one message stream.

    Uses `send` and `recv` entries. Retransmissions (retx=True) are ignored,
    so for events the ratio is for first transmissions only.
    """
    sent: Dict[int, float] = {}
    recv: Dict[int, float] = {}
    for e in events:
        if e.get("msg") != msg:
            continue
        if e["ev"] == "send" and not e.get("retx"):
            sent.setdefault(e["seq"], e["t"])
        elif e["ev"] == "recv":
            recv.setdefault(e["seq"], e["t"])
    got = [s for s in sent if s in recv]
    lat = [(recv[s] - sent[s]) * 1000.0 for s in got]
    return {
        "msg": msg,
        "sent": len(sent),
        "received": len(got),
        "pdr": (len(got) / len(sent)) if sent else None,
        "lat_ms_median": _pct(lat, 0.5),
        "lat_ms_p95": _pct(lat, 0.95),
        "lat_ms_max": max(lat) if lat else None,
        "lat_ms": lat,
    }


def event_stats(events: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Event to angle-reached latency (as seen by the gateway) and failures."""
    lat = [e["latency"] * 1000.0 for e in events if e["ev"] == "angle_reached"]
    return {
        "events_reached": len(lat),
        "events_failed": sum(1 for e in events if e["ev"] == "event_failed"),
        "stale_rejected": sum(1 for e in events if e["ev"] == "reject_stale"),
        "retransmissions": sum(1 for e in events if e["ev"] == "send" and e.get("retx")),
        "faults": sum(1 for e in events if e["ev"] == "fault"),
        "latency_ms_median": _pct(lat, 0.5),
        "latency_ms_p95": _pct(lat, 0.95),
        "latency_ms_max": max(lat) if lat else None,
        "latency_ms": lat,
    }
