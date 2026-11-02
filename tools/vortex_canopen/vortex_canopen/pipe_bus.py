"""python-can bus for the host pipe bus (hf-core PipeHub / PipePort).

The hub is a UNIX stream socket that forwards every datagram of frame
records (``frame_record``) to every other participant, like a CAN bus. The
host node binary (``vortex_node_host --port pipe:<path>``) hosts the hub
when nobody else does. Use it directly::

    bus = PipeBus(channel="/tmp/vortex.sock")
    network = canopen.Network(bus); network.connect()

or through python-can's plugin entry point (``interface="vortex_pipe"``)
once the package is installed. The same codec carries the Wi-Fi tunnel.
"""

from __future__ import annotations

import os
import select
import socket
import threading
import time
from collections import deque
from typing import Deque, Optional, Tuple

import can

from . import frame_record as fr


def _to_frame(msg: can.Message) -> fr.Frame:
    flags = 0
    if msg.is_extended_id:
        flags |= fr.FLAG_EXT
    if msg.is_remote_frame:
        flags |= fr.FLAG_RTR
    if msg.is_fd:
        flags |= fr.FLAG_FDF
        if msg.bitrate_switch:
            flags |= fr.FLAG_BRS
        if msg.error_state_indicator:
            flags |= fr.FLAG_ESI
    data = b"" if msg.is_remote_frame else bytes(msg.data[: msg.dlc])
    return fr.Frame(msg.arbitration_id, data, flags)


def _to_message(f: fr.Frame, channel: str) -> can.Message:
    return can.Message(
        timestamp=time.time(),
        arbitration_id=f.id,
        is_extended_id=f.is_extended,
        is_remote_frame=f.is_remote,
        is_fd=f.is_fd,
        bitrate_switch=bool(f.flags & fr.FLAG_BRS),
        error_state_indicator=bool(f.flags & fr.FLAG_ESI),
        dlc=len(f.data),
        data=f.data,
        channel=channel,
        is_rx=True,
    )


class PipeBus(can.BusABC):
    """A participant on the pipe bus at ``channel`` (the hub socket path)."""

    def __init__(self, channel: str, connect_timeout: float = 5.0, **kwargs):
        self.channel_info = f"vortex pipe {channel}"
        self._path = channel
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = time.monotonic() + connect_timeout
        while True:
            try:
                self._sock.connect(channel)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() > deadline:
                    raise can.CanInitializationError(f"no pipe hub at {channel}")
                time.sleep(0.02)
        self._sock.setblocking(False)
        self._decoder = fr.StreamDecoder()
        self._pending: Deque[can.Message] = deque()
        self._tx_lock = threading.Lock()
        self._seq = 0
        self._epoch = int.from_bytes(os.urandom(4), "little")
        super().__init__(channel=channel, **kwargs)

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        dgram = fr.encode_datagram([_to_frame(msg)], seq=self._seq, epoch=self._epoch)
        data = fr.frame_stream(dgram)
        with self._tx_lock:
            self._seq = (self._seq + 1) & 0xFFFF
            deadline = None if timeout is None else time.monotonic() + timeout
            view = memoryview(data)
            while view:
                try:
                    n = self._sock.send(view)
                    view = view[n:]
                except BlockingIOError:
                    wait = None if deadline is None else max(0.0, deadline - time.monotonic())
                    if wait == 0.0:
                        raise can.CanOperationError("pipe bus send timed out")
                    select.select([], [self._sock], [], wait)

    def _recv_internal(self, timeout: Optional[float]) -> Tuple[Optional[can.Message], bool]:
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self._pending:
            wait = None if deadline is None else max(0.0, deadline - time.monotonic())
            r, _, _ = select.select([self._sock], [], [], wait)
            if not r:
                return None, False
            try:
                chunk = self._sock.recv(65536)
            except BlockingIOError:
                continue
            if not chunk:
                raise can.CanOperationError("pipe hub closed the connection")
            for dgram in self._decoder.feed(chunk):
                try:
                    _, frames = fr.decode_datagram(dgram)
                except fr.DecodeError:
                    continue
                for f in frames:
                    self._pending.append(_to_message(f, self._path))
        return self._pending.popleft(), False

    def shutdown(self) -> None:
        super().shutdown()
        try:
            self._sock.close()
        except OSError:
            pass
