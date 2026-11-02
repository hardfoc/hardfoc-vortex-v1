"""Fixtures for the host-node conformance-style suites (design §9.4).

The suites run python-canopen against ``vortex_node_host`` (host/node), the
firmware's CANopen node built for the host. The bus is

- ``vcan0`` (SocketCAN) when ``VORTEX_BUS=vcan0``, or with ``VORTEX_BUS=auto``
  (the default) when the interface exists, or
- the pipe bus (hf-core PipeHub, frame-record codec) otherwise: the node
  process hosts the hub and the tests join it with
  ``vortex_canopen.pipe_bus.PipeBus``.

``VORTEX_NODE_HOST`` overrides the binary path
(default ``build/node-host/vortex_node_host``).

Every test starts from a node that was just reset (NMT reset node: OD
defaults, boot-up, pre-operational), so tests do not depend on each other.
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

import can
import canopen
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "vortex_canopen"))

from vortex_canopen import eds_path  # noqa: E402
from vortex_canopen.pipe_bus import PipeBus  # noqa: E402

NODE_ID = 5
HB_COB = 0x700
NMT_PREOP = 0x7F
NMT_OPERATIONAL = 0x05
NMT_STOPPED = 0x04


def node_binary() -> Path:
    return Path(os.environ.get("VORTEX_NODE_HOST", REPO / "build" / "node-host" / "vortex_node_host"))


def bus_kind() -> str:
    want = os.environ.get("VORTEX_BUS", "auto")
    if want == "auto":
        return "vcan0" if Path("/sys/class/net/vcan0").exists() else "pipe"
    return want


@dataclass
class BusSpec:
    kind: str                # "vcan0" (any SocketCAN name) or "pipe"
    pipe_path: str = ""

    @property
    def port_arg(self) -> str:
        return f"pipe:{self.pipe_path}" if self.kind == "pipe" else self.kind

    def open(self) -> can.BusABC:
        if self.kind == "pipe":
            return PipeBus(channel=self.pipe_path)
        return can.Bus(interface="socketcan", channel=self.kind)


class NodeProcess:
    """One vortex_node_host process."""

    def __init__(self, spec: BusSpec, node_id: int, extra: Optional[List[str]] = None):
        self.args = [str(node_binary()), "--node-id", str(node_id), "--port", spec.port_arg] + (extra or [])
        self.proc: Optional[subprocess.Popen] = None
        self.lines: "queue.Queue[str]" = queue.Queue()

    def start(self, timeout: float = 10.0) -> "NodeProcess":
        if not Path(self.args[0]).exists():
            pytest.exit(f"{self.args[0]} not found: build host/node first (see host/node/CMakeLists.txt)", 2)
        self.proc = subprocess.Popen(self.args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self._pump, daemon=True).start()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                if self.proc.poll() is not None:
                    break
                continue
            if "ready" in line:
                return self
        raise RuntimeError(f"vortex_node_host did not start: {' '.join(self.args)}")

    def _pump(self) -> None:
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            sys.stderr.write(f"[node] {line}")
            self.lines.put(line)

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None


class FrameLog(can.Listener):
    """Every frame seen on the bus, with a monotonic receive time."""

    def __init__(self):
        self._q: "queue.Queue[tuple[float, can.Message]]" = queue.Queue()

    def on_message_received(self, msg: can.Message) -> None:
        self._q.put((time.monotonic(), msg))

    def clear(self) -> None:
        while not self._q.empty():
            self._q.get_nowait()

    def wait_for(self, match: Callable[[can.Message], bool], timeout: float = 1.0) -> Optional[can.Message]:
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            try:
                _, msg = self._q.get(timeout=left)
            except queue.Empty:
                return None
            if match(msg):
                return msg

    def collect(self, match: Callable[[can.Message], bool], duration: float) -> List[tuple]:
        out = []
        deadline = time.monotonic() + duration
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return out
            try:
                t, msg = self._q.get(timeout=left)
            except queue.Empty:
                return out
            if match(msg):
                out.append((t, msg))


@dataclass
class Bench:
    spec: BusSpec
    network: canopen.Network
    node: canopen.RemoteNode
    log: FrameLog
    procs: List[NodeProcess] = field(default_factory=list)

    def send(self, cob_id: int, data: bytes = b"", remote: bool = False) -> None:
        self.network.send_message(cob_id, data, remote=remote)

    def nmt(self, command: int, node_id: int = NODE_ID) -> None:
        self.send(0x000, bytes([command, node_id]))

    def wait_bootup(self, node_id: int = NODE_ID, timeout: float = 2.0) -> Optional[can.Message]:
        return self.log.wait_for(lambda m: m.arbitration_id == HB_COB + node_id and bytes(m.data) == b"\x00", timeout)

    def wait_state(self, state: int, node_id: int = NODE_ID, timeout: float = 1.0) -> bool:
        return self.log.wait_for(
            lambda m: m.arbitration_id == HB_COB + node_id and len(m.data) == 1 and m.data[0] == state, timeout
        ) is not None

    def reset_node(self, node_id: int = NODE_ID) -> None:
        self.log.clear()
        self.nmt(0x81, node_id)
        assert self.wait_bootup(node_id), "no boot-up after NMT reset node"


@pytest.fixture(scope="session")
def bus_spec():
    kind = bus_kind()
    if kind == "pipe":
        d = tempfile.mkdtemp(prefix="vx")
        return BusSpec("pipe", os.path.join(d, "bus.sock"))
    return BusSpec(kind)


@pytest.fixture(scope="session")
def node_process(bus_spec):
    p = NodeProcess(bus_spec, NODE_ID).start()
    yield p
    p.stop()


@pytest.fixture(scope="session")
def session_bench(bus_spec, node_process):
    network = canopen.Network(bus_spec.open())
    network.connect()
    log = FrameLog()
    network.notifier.add_listener(log)
    node = network.add_node(NODE_ID, eds_path())
    node.sdo.RESPONSE_TIMEOUT = 1.0
    bench = Bench(bus_spec, network, node, log, [node_process])
    yield bench
    network.disconnect()


@pytest.fixture
def bench(session_bench):
    """The session node, freshly reset (OD defaults, pre-operational)."""
    session_bench.reset_node()
    session_bench.log.clear()
    return session_bench


def pytest_report_header(config):
    return f"vortex node host: {node_binary()} on {bus_kind()}"
