"""Frame-record tunnel codec (Vortex CANopen design §7.2).

Python twin of hf-core ``handlers/common/canopen/HfCanFrameRecord.h``. The
pipe bus (host CI), the Wi-Fi TLS tunnel and ESP-NOW all carry these
datagrams. All fields are little-endian::

    datagram: u8 ver=1, u8 flags, u16 seq, u32 ack/epoch, records...
    record:   u32 id | EXT (bit 31) | RTR (bit 30), u8 flags (FDF/BRS/ESI), u8 len, data[len]

On a byte stream each datagram is preceded by a u16 length.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterable, Iterator, List, Optional, Tuple

VERSION = 1
HEADER = struct.Struct("<BBHI")
RECORD = struct.Struct("<IBB")
MAX_DATAGRAM = 4096

ID_EXT = 0x80000000
ID_RTR = 0x40000000

FLAG_EXT = 0x01
FLAG_RTR = 0x02
FLAG_FDF = 0x04
FLAG_BRS = 0x08
FLAG_ESI = 0x10
RECORD_FLAGS = FLAG_FDF | FLAG_BRS | FLAG_ESI

DGRAM_KEEPALIVE = 0x01

_FD_LENGTHS = frozenset(range(9)) | {12, 16, 20, 24, 32, 48, 64}


class DecodeError(ValueError):
    """The datagram is not a valid frame-record datagram."""


@dataclass
class Frame:
    """One CAN / CAN FD frame (the CanFrame of the C++ side)."""

    id: int
    data: bytes = b""
    flags: int = 0

    @property
    def is_extended(self) -> bool:
        return bool(self.flags & FLAG_EXT)

    @property
    def is_remote(self) -> bool:
        return bool(self.flags & FLAG_RTR)

    @property
    def is_fd(self) -> bool:
        return bool(self.flags & FLAG_FDF)


@dataclass
class Header:
    version: int = VERSION
    flags: int = 0
    seq: int = 0
    epoch: int = 0


def well_formed(f: Frame) -> bool:
    """Same structural rules as hf::can::IsWellFormed."""
    if f.flags & ~(FLAG_EXT | FLAG_RTR | RECORD_FLAGS):
        return False
    if f.id > (0x1FFFFFFF if f.is_extended else 0x7FF) or f.id < 0:
        return False
    n = len(f.data)
    if f.is_fd:
        return not f.is_remote and n in _FD_LENGTHS
    if f.flags & (FLAG_BRS | FLAG_ESI):
        return False
    return n <= 8


def encode_record(f: Frame) -> bytes:
    if not well_formed(f):
        raise ValueError(f"malformed frame {f!r}")
    raw = f.id | (ID_EXT if f.is_extended else 0) | (ID_RTR if f.is_remote else 0)
    return RECORD.pack(raw, f.flags & RECORD_FLAGS, len(f.data)) + bytes(f.data)


def encode_datagram(frames: Iterable[Frame], seq: int = 0, epoch: int = 0, flags: int = 0) -> bytes:
    out = bytearray(HEADER.pack(VERSION, flags, seq & 0xFFFF, epoch & 0xFFFFFFFF))
    for f in frames:
        out += encode_record(f)
    if len(out) > MAX_DATAGRAM:
        raise ValueError("datagram too long")
    return bytes(out)


def decode_datagram(buf: bytes) -> Tuple[Header, List[Frame]]:
    """Decode one datagram; raises DecodeError and returns nothing on any defect."""
    if len(buf) < HEADER.size:
        raise DecodeError("truncated header")
    ver, flags, seq, epoch = HEADER.unpack_from(buf, 0)
    if ver != VERSION:
        raise DecodeError(f"version {ver}")
    frames: List[Frame] = []
    off = HEADER.size
    while off < len(buf):
        if len(buf) - off < RECORD.size:
            raise DecodeError("truncated record")
        raw, rflags, n = RECORD.unpack_from(buf, off)
        if rflags & ~RECORD_FLAGS:
            raise DecodeError("unknown record flags")
        if n > 64:
            raise DecodeError("record length")
        if len(buf) - off - RECORD.size < n:
            raise DecodeError("truncated data")
        flags_ = rflags | (FLAG_EXT if raw & ID_EXT else 0) | (FLAG_RTR if raw & ID_RTR else 0)
        f = Frame(raw & ~(ID_EXT | ID_RTR), bytes(buf[off + RECORD.size: off + RECORD.size + n]), flags_)
        if not well_formed(f):
            raise DecodeError("malformed frame")
        frames.append(f)
        off += RECORD.size + n
    return Header(ver, flags, seq, epoch), frames


def frame_stream(datagram: bytes) -> bytes:
    """Stream form: u16 length + datagram."""
    return struct.pack("<H", len(datagram)) + datagram


@dataclass
class StreamDecoder:
    """Splits a byte stream into datagrams (u16 length prefix)."""

    _buf: bytearray = field(default_factory=bytearray)

    def feed(self, data: bytes) -> Iterator[bytes]:
        self._buf += data
        while len(self._buf) >= 2:
            (n,) = struct.unpack_from("<H", self._buf, 0)
            if n < HEADER.size or n > MAX_DATAGRAM:
                raise DecodeError(f"stream datagram length {n}")
            if len(self._buf) < 2 + n:
                break
            dgram = bytes(self._buf[2:2 + n])
            del self._buf[:2 + n]
            yield dgram
