"""AI deck 1.1 image stream over Wi-Fi (CPX on TCP), recorded to disk.

Wire format, as sent by the Bitcraze wifi-img-streamer example
(aideck-gap8-examples, examples/other/wifi-img-streamer):

    every CPX packet:  uint16 length, uint8 routing, uint8 function   (little endian)
                       then length - 2 payload bytes
    first packet of a frame carries the image header:
                       uint8 magic (0xBC), uint16 width, uint16 height,
                       uint8 depth, uint8 format (0 raw, 1 JPEG), uint32 size
    following packets carry `size` bytes of image data in chunks.

Width and height are read from the header, never assumed: 324x244 (QVGA build)
and 324x324 (native build) both occur. The AI deck 1.1 camera (HM01B0) is
monochrome, so raw frames are saved as 8-bit grey unless debayer=True.

Output (hand-off format agreed with the C5 vision pipeline):
    <out_dir>/index.csv   idx,t,file        t = host epoch seconds at frame header
    <out_dir>/meta.csv    idx,t,t_done,width,height,depth,format,bytes
    <out_dir>/000000.png ...
"""
from __future__ import annotations

import csv
import os
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import cv2
import numpy as np

CPX_HDR = struct.Struct("<HBB")
IMG_HDR = struct.Struct("<BHHBBI")
IMG_MAGIC = 0xBC
FMT_RAW = 0
FMT_JPEG = 1
DEFAULT_IP = "192.168.4.1"   # AI deck in access point mode
DEFAULT_PORT = 5000


@dataclass
class Frame:
    t: float          # host time when the image header arrived
    t_done: float     # host time when the last byte arrived
    width: int
    height: int
    depth: int
    fmt: int
    data: bytes


def read_frame(rx: Callable[[int], bytes], clock: Callable[[], float] = time.time) -> Frame:
    """Read packets until one complete frame is assembled. `rx(n)` must return exactly n bytes."""
    while True:
        length, _routing, _function = CPX_HDR.unpack(rx(CPX_HDR.size))
        payload = rx(length - 2)
        if len(payload) != IMG_HDR.size or payload[0] != IMG_MAGIC:
            continue  # not a frame header: resynchronise on the next packet
        t = clock()
        _magic, width, height, depth, fmt, size = IMG_HDR.unpack(payload)
        buf = bytearray()
        while len(buf) < size:
            length, _dst, _src = CPX_HDR.unpack(rx(CPX_HDR.size))
            buf.extend(rx(length - 2))
        return Frame(t, clock(), width, height, depth, fmt, bytes(buf[:size]))


def decode_frame(fr: Frame, debayer: bool = False, flip: str = "none") -> np.ndarray:
    """flip: 'none', 'h', 'v' or 'hv'. A 45 degree mirror in front of the camera needs one flip."""
    img = _decode(fr, debayer)
    if flip == "h":
        return cv2.flip(img, 1)
    if flip == "v":
        return cv2.flip(img, 0)
    if flip == "hv":
        return cv2.flip(img, -1)
    return img


def _decode(fr: Frame, debayer: bool) -> np.ndarray:
    if fr.fmt == FMT_JPEG:
        img = cv2.imdecode(np.frombuffer(fr.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError("JPEG decode failed")
        return img
    img = np.frombuffer(fr.data, np.uint8)[: fr.width * fr.height].reshape(fr.height, fr.width)
    if debayer:
        return cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
    return img


def socket_rx(sock: socket.socket) -> Callable[[int], bytes]:
    def rx(n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("AI deck stream closed")
            buf.extend(chunk)
        return bytes(buf)
    return rx


@dataclass
class StreamStats:
    frames: int = 0
    decode_errors: int = 0
    dropped: int = 0          # frames the writer could not keep up with
    t_first: Optional[float] = None
    t_last: Optional[float] = None
    width: int = 0
    height: int = 0
    fmt: int = -1
    error: str = ""
    times: List[float] = field(default_factory=list)

    @property
    def fps(self) -> float:
        if self.frames < 2 or self.t_first is None or self.t_last is None or self.t_last <= self.t_first:
            return 0.0
        return (self.frames - 1) / (self.t_last - self.t_first)


class FrameRecorder:
    """Receives frames on one thread and writes PNGs on another, so disk writes never stall the socket."""

    def __init__(self, out_dir: str, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 debayer: bool = False, save: bool = True, timeout_s: float = 5.0, flip: str = "none") -> None:
        self.out_dir = out_dir
        self.ip, self.port = ip, port
        self.debayer, self.save, self.timeout_s, self.flip = debayer, save, timeout_s, flip
        self.stats = StreamStats()
        self._stop = threading.Event()
        self._q: "queue.Queue[Optional[tuple]]" = queue.Queue(maxsize=600)
        self._rx_thread: Optional[threading.Thread] = None
        self._wr_thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self.connected = threading.Event()

    def start(self) -> "FrameRecorder":
        os.makedirs(self.out_dir, exist_ok=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, name="aideck-rx", daemon=True)
        self._wr_thread = threading.Thread(target=self._wr_loop, name="aideck-wr", daemon=True)
        self._wr_thread.start()
        self._rx_thread.start()
        return self

    def stop(self) -> StreamStats:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        if self._rx_thread:
            self._rx_thread.join(timeout=3)
        self._q.put(None)
        if self._wr_thread:
            self._wr_thread.join(timeout=30)
        return self.stats

    def _rx_loop(self) -> None:
        try:
            self._sock = socket.create_connection((self.ip, self.port), timeout=self.timeout_s)
            self._sock.settimeout(self.timeout_s)
            self.connected.set()
            rx = socket_rx(self._sock)
            idx = 0
            while not self._stop.is_set():
                fr = read_frame(rx)
                st = self.stats
                if st.t_first is None:
                    st.t_first = fr.t
                st.t_last = fr.t
                st.frames += 1
                st.times.append(fr.t)
                st.width, st.height, st.fmt = fr.width, fr.height, fr.fmt
                try:
                    self._q.put_nowait((idx, fr))
                except queue.Full:
                    st.dropped += 1
                idx += 1
        except Exception as e:  # noqa: BLE001 - reported in stats, flight code decides what to do
            if not self._stop.is_set():
                self.stats.error = f"{type(e).__name__}: {e}"

    def _wr_loop(self) -> None:
        with open(os.path.join(self.out_dir, "index.csv"), "w", newline="") as fi, \
                open(os.path.join(self.out_dir, "meta.csv"), "w", newline="") as fm:
            wi, wm = csv.writer(fi), csv.writer(fm)
            wi.writerow(["idx", "t", "file"])
            wm.writerow(["idx", "t", "t_done", "width", "height", "depth", "format", "bytes"])
            while True:
                item = self._q.get()
                if item is None:
                    break
                idx, fr = item
                name = f"{idx:06d}.png"
                if self.save:
                    try:
                        cv2.imwrite(os.path.join(self.out_dir, name), decode_frame(fr, self.debayer, self.flip))
                    except Exception:  # noqa: BLE001
                        self.stats.decode_errors += 1
                        continue
                wi.writerow([idx, f"{fr.t:.6f}", name])
                wm.writerow([idx, f"{fr.t:.6f}", f"{fr.t_done:.6f}", fr.width, fr.height, fr.depth, fr.fmt, len(fr.data)])


def encode_packets(img: np.ndarray, fmt: int = FMT_RAW, chunk: int = 1020) -> bytes:
    """Build the CPX byte stream for one frame (used by the simulator and the tests)."""
    if fmt == FMT_JPEG:
        ok, enc = cv2.imencode(".jpg", img)
        data = enc.tobytes()
    else:
        data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    h, w = img.shape[:2]
    out = bytearray()
    hdr = IMG_HDR.pack(IMG_MAGIC, w, h, 1, fmt, len(data))
    out += CPX_HDR.pack(len(hdr) + 2, 0x09, 0x00) + hdr
    for i in range(0, len(data), chunk):
        part = data[i:i + chunk]
        out += CPX_HDR.pack(len(part) + 2, 0x09, 0x00) + part
    return bytes(out)
