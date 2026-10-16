"""Message set for the drone <-> panel loop.

Modelled on C-ITS: DroneBeacon ~ CAM (periodic awareness), PanelEvent ~ DENM
(event, must be delivered), PanelState = panel status, Heartbeat = watchdog.
Every message carries a sequence number and the sender's timestamp (seconds,
sender clock) so delivery ratio and latency can be computed from logs.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, asdict

# mission types (what the drone is doing, decides the panel's target angle)
MISSION_NONE = 0
MISSION_CROP_OCCLUSION = 1
MISSION_CROP_SHADOW = 2
MISSION_PANEL_VIEW = 3

# panel events
EVENT_APPROACH = 0
EVENT_CLEARED = 1
EVENT_ABORT = 2

# panel FSM states
FSM_IDLE_TRACK = 0
FSM_ARMED = 1
FSM_ACTUATING = 2
FSM_HOLD = 3
FSM_RESTORE = 4
FSM_FAULT = 5
FSM_NAMES = {0: "IDLE_TRACK", 1: "ARMED", 2: "ACTUATING", 3: "HOLD", 4: "RESTORE", 5: "FAULT"}

T_BEACON, T_EVENT, T_STATE, T_HEARTBEAT = 1, 2, 3, 4


@dataclass
class DroneBeacon:
    seq: int
    t_send: float
    station_id: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    heading_deg: float
    mission_type: int

    TYPE = T_BEACON


@dataclass
class PanelEvent:
    seq: int
    t_send: float
    panel_id: int
    event_type: int
    mission_type: int
    target_angle_deg: float

    TYPE = T_EVENT


@dataclass
class PanelState:
    seq: int
    t_send: float
    panel_id: int
    fsm_state: int
    angle_deg: float
    target_deg: float
    current_a: float
    ack_seq: int

    TYPE = T_STATE


@dataclass
class Heartbeat:
    seq: int
    t_send: float

    TYPE = T_HEARTBEAT


# ---- binary codec (little endian): 1 byte type, then fixed layout ----
_FMT = {
    T_BEACON: ("<BIdHfffffffB", DroneBeacon),
    T_EVENT: ("<BIdBBBf", PanelEvent),
    T_STATE: ("<BIdBBfffI", PanelState),
    T_HEARTBEAT: ("<BId", Heartbeat),
}


def encode(msg) -> bytes:
    fmt, _ = _FMT[msg.TYPE]
    vals = list(asdict(msg).values())
    return struct.pack(fmt, msg.TYPE, *vals)


def decode(data: bytes):
    if not data:
        raise ValueError("empty packet")
    t = data[0]
    if t not in _FMT:
        raise ValueError(f"unknown message type {t}")
    fmt, cls = _FMT[t]
    if len(data) != struct.calcsize(fmt):
        raise ValueError(f"bad length {len(data)} for type {t}")
    vals = struct.unpack(fmt, data)[1:]
    return cls(*vals)


def type_name(msg) -> str:
    return type(msg).__name__
