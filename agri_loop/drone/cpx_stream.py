"""AI deck 1.1 image stream over Wi-Fi (CPX on TCP), recorded to disk.

Two streamer firmwares are supported, chosen with `protocol`.

"bitcraze": the Bitcraze wifi-img-streamer example
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

"nanocockpit": the IDSIA NanoCockpit streamer (github.com/idsia-robotics/nanocockpit,
src/gap/examples/streamer with the src/nina ESP32 firmware), checked against its
source on 2026-11-05 (tcp.py, streamer.py, streamer.h, uart_protocol.h):

    every CPX packet:  uint16 length (payload only), uint8 routing, uint8 function|version
                       then length payload bytes; function 0x06 is the streamer
    BUFFER_BEGIN:      uint8 0x10, uint8 type (0x01 image), uint32 size, uint32 crc32
                       (0 = none), 2 pad bytes, then the first part of the buffer
    BUFFER_DATA:       uint8 0x11, 3 pad bytes, then the next part of the buffer
    image buffer:      148-byte metadata (version 10: width, height, bpp, format,
                       frame id, GAP8 timestamp, forwarded drone state, ...) then pixels

The last part is padded to a multiple of 4 bytes, so only `size` bytes are kept.
The deck names itself aideck.local over mDNS (160x160 grey frames, 30 fps requested).

Output (hand-off format agreed with the C5 vision pipeline):
    <out_dir>/index.csv   idx,t,file        t = host epoch seconds at frame header
    <out_dir>/meta.csv    idx,t,t_done,width,height,depth,format,bytes,deck_id,deck_t_us
                          (deck_id and deck_t_us are -1 for the Bitcraze streamer)
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
import zlib
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import cv2
import numpy as np

CPX_HDR = struct.Struct("<HBB")
IMG_HDR = struct.Struct("<BHHBBI")
IMG_MAGIC = 0xBC
FMT_RAW = 0
FMT_JPEG = 1
DEFAULT_IP = "192.168.4.1"   # Bitcraze streamer in access point mode
DEFAULT_PORT = 5000
PROTO_BITCRAZE = "bitcraze"
PROTO_NANOCOCKPIT = "nanocockpit"
PROTOCOLS = (PROTO_BITCRAZE, PROTO_NANOCOCKPIT)
NC_HOST = "aideck.local"     # NanoCockpit ESP32 mDNS name (menuconfig MDNS_HOSTNAME)

NC_CPX_HDR = struct.Struct("<HBB")
NC_F_STREAMER = 0x06
NC_CMD_BEGIN, NC_CMD_DATA = 0x10, 0x11
NC_TYPE_IMAGE = 0x01
NC_BEGIN = struct.Struct("<BBII2x")
NC_DATA = struct.Struct("<B3x")
NC_META = struct.Struct("<BHHBBBII")   # first 16 bytes of streamer_metadata_t
NC_META_SIZE = 148
NC_META_VERSION = 10
NC_MTU = 4088                          # 4092-byte packets minus the 4-byte header


def default_protocol() -> str:
    """AGRI_AIDECK_PROTO picks the streamer firmware without editing code (default bitcraze)."""
    proto = os.environ.get("AGRI_AIDECK_PROTO", PROTO_BITCRAZE).strip().lower()
    if proto not in PROTOCOLS:
        raise ValueError(f"AGRI_AIDECK_PROTO={proto!r}, expected one of {', '.join(PROTOCOLS)}")
    return proto


def default_host(protocol: str) -> str:
    """AGRI_AIDECK_IP if set, else the usual address for that firmware."""
    env = os.environ.get("AGRI_AIDECK_IP", "").strip()
    if env:
        return env
    return NC_HOST if protocol == PROTO_NANOCOCKPIT else DEFAULT_IP


@dataclass
class Frame:
    t: float          # host time when the image header arrived
    t_done: float     # host time when the last byte arrived
    width: int
    height: int
    depth: int
    fmt: int
    data: bytes
    deck_id: int = -1     # NanoCockpit: camera hardware frame counter (wraps at 256)
    deck_t_us: int = -1   # NanoCockpit: GAP8 end-of-frame time, microseconds


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


def read_frame_nanocockpit(rx: Callable[[int], bytes], clock: Callable[[], float] = time.time) -> Frame:
    """Read NanoCockpit streamer packets until one complete image is assembled."""
    buf: Optional[bytearray] = None
    remaining, crc, t = 0, 0, 0.0
    while True:
        length, _routing, function = NC_CPX_HDR.unpack(rx(NC_CPX_HDR.size))
        if length > NC_MTU:
            raise ValueError(f"CPX length {length} is over {NC_MTU}: stream out of step")
        payload = rx(length)
        if (function & 0x3F) != NC_F_STREAMER or not payload:
            continue
        if payload[0] == NC_CMD_BEGIN and len(payload) >= NC_BEGIN.size:
            _cmd, kind, size, crc = NC_BEGIN.unpack_from(payload)
            if kind != NC_TYPE_IMAGE:
                buf = None
                continue
            t, buf, remaining = clock(), bytearray(), size
            body = payload[NC_BEGIN.size:]
        elif payload[0] == NC_CMD_DATA and buf is not None:
            body = payload[NC_DATA.size:]
        else:
            buf = None  # out of sequence: wait for the next BUFFER_BEGIN
            continue
        take = min(len(body), remaining)
        buf += body[:take]
        remaining -= take
        if remaining:
            continue
        data, buf = bytes(buf), None
        if crc and zlib.crc32(data) != crc:
            continue  # corrupted buffer: dropped, as the NanoCockpit client does
        if len(data) < NC_META_SIZE:
            continue
        version, width, height, bpp, fmt, frame_id, t_us, _t_state = NC_META.unpack_from(data)
        if version != NC_META_VERSION:
            raise ValueError(f"NanoCockpit metadata v{version}, this reader knows v{NC_META_VERSION}")
        if bpp != 1 or fmt != 0:
            raise ValueError(f"NanoCockpit frame format {fmt} with {bpp} bytes per pixel, expected 8-bit grey")
        pixels = data[NC_META_SIZE:NC_META_SIZE + width * height]
        return Frame(t, clock(), width, height, 1, FMT_RAW, pixels, deck_id=frame_id, deck_t_us=t_us)


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
    error: str = ""              # last error seen, kept even after a successful reconnect
    reconnects: int = 0
    gaps: List[tuple] = field(default_factory=list)   # (t_lost, t_back) per drop
    times: List[float] = field(default_factory=list)

    @property
    def fps(self) -> float:
        if self.frames < 2 or self.t_first is None or self.t_last is None or self.t_last <= self.t_first:
            return 0.0
        return (self.frames - 1) / (self.t_last - self.t_first)


class FrameRecorder:
    """Receives frames on one thread and writes PNGs on another, so disk writes never stall the socket."""

    def __init__(self, out_dir: str, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 debayer: bool = False, save: bool = True, timeout_s: float = 5.0, flip: str = "none",
                 retry_s: float = 1.0, protocol: str = PROTO_BITCRAZE) -> None:
        if protocol not in PROTOCOLS:
            raise ValueError(f"unknown AI deck protocol {protocol!r}")
        self.protocol = protocol
        self._read = read_frame_nanocockpit if protocol == PROTO_NANOCOCKPIT else read_frame
        self.out_dir = out_dir
        self.ip, self.port = ip, port
        self.debayer, self.save, self.timeout_s, self.flip = debayer, save, timeout_s, flip
        self.retry_s = retry_s
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
        """Receive frames; after a dropped connection, keep reconnecting until stop() is called."""
        idx = 0
        t_lost: Optional[float] = None
        while not self._stop.is_set():
            try:
                self._sock = socket.create_connection((self.ip, self.port), timeout=self.timeout_s)
                self._sock.settimeout(self.timeout_s)
                if t_lost is not None:
                    self.stats.reconnects += 1
                    self.stats.gaps.append((t_lost, time.time()))
                    t_lost = None
                self.connected.set()
                rx = socket_rx(self._sock)
                while not self._stop.is_set():
                    fr = self._read(rx)
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
            except Exception as e:  # noqa: BLE001 - reported in stats, the run carries on
                if self._stop.is_set():
                    break
                self.stats.error = f"{type(e).__name__}: {e}"
                if t_lost is None:
                    t_lost = time.time()
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                self._stop.wait(self.retry_s)
        if self._sock is not None:  # a reconnect may have opened a socket after stop() closed the old one
            try:
                self._sock.close()
            except OSError:
                pass

    def _wr_loop(self) -> None:
        with open(os.path.join(self.out_dir, "index.csv"), "w", newline="") as fi, \
                open(os.path.join(self.out_dir, "meta.csv"), "w", newline="") as fm:
            wi, wm = csv.writer(fi), csv.writer(fm)
            wi.writerow(["idx", "t", "file"])
            wm.writerow(["idx", "t", "t_done", "width", "height", "depth", "format", "bytes",
                         "deck_id", "deck_t_us"])
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
                wm.writerow([idx, f"{fr.t:.6f}", f"{fr.t_done:.6f}", fr.width, fr.height, fr.depth, fr.fmt, len(fr.data),
                             fr.deck_id, fr.deck_t_us])


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


def encode_nanocockpit_packets(img: np.ndarray, frame_id: int = 0, t_us: int = 0, checksum: bool = False,
                               mtu: int = NC_MTU) -> bytes:
    """Build the NanoCockpit CPX byte stream for one grey frame, padded like the GAP8 does."""
    h, w = img.shape[:2]
    meta = bytearray(NC_META_SIZE)
    NC_META.pack_into(meta, 0, NC_META_VERSION, w, h, 1, 0, frame_id & 0xFF, t_us & 0xFFFFFFFF, 0)
    data = bytes(meta) + np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    crc = (zlib.crc32(data) or 0xFFFFFFFF) if checksum else 0
    routing = 0x03 | (0x04 << 3) | (1 << 6)      # to the Wi-Fi host, from the GAP8, last packet
    out = bytearray()
    pos = 0
    while pos < len(data):
        head = NC_BEGIN.pack(NC_CMD_BEGIN, NC_TYPE_IMAGE, len(data), crc) if pos == 0 else NC_DATA.pack(NC_CMD_DATA)
        part = data[pos:pos + mtu - len(head)]
        pos += len(part)
        if pos >= len(data) and len(part) % 4:
            part += bytes(4 - len(part) % 4)
        body = head + part
        out += NC_CPX_HDR.pack(len(body), routing, NC_F_STREAMER) + body
    return bytes(out)
