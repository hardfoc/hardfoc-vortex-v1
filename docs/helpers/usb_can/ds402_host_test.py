#!/usr/bin/env python3
"""
Host-side CANopen (NMT + expedited SDO) probe for USB-CAN adapters via python-can.

Does not implement a full DS402 state machine; use for bench / bus bring-up alongside
hf-utils-canopen documentation (see docs/canopen-and-motor-path.md).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from canopen_frames import (
    build_control_word,
    build_mode_of_operation,
    build_nmt_start_remote_node,
    build_status_word_request,
    build_target_velocity,
    parse_expedited_sdo_upload_response,
)


def _open_bus(interface: str, channel: str, bitrate: int):
    import can

    return can.Bus(interface=interface, channel=channel, bitrate=bitrate)


def _send(bus, arbitration_id: int, data: bytes) -> None:
    import can

    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    bus.send(msg)


def _recv_sdo_response(bus, node_id: int, timeout_s: float) -> Optional[bytes]:
    import can

    cob = 0x580 + (node_id & 0x7F)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        m = bus.recv(timeout=max(0.001, deadline - time.monotonic()))
        if m is None:
            continue
        if m.is_extended_id:
            continue
        if m.arbitration_id == cob:
            return bytes(m.data)
    return None


def cmd_nmt_start(bus, node_id: int) -> None:
    f = build_nmt_start_remote_node(node_id)
    _send(bus, f.arbitration_id, f.data)
    print(f"Sent NMT Start remote node, node={node_id}")


def cmd_read_status(bus, node_id: int, timeout_s: float) -> None:
    f = build_status_word_request(node_id)
    _send(bus, f.arbitration_id, f.data)
    data = _recv_sdo_response(bus, node_id, timeout_s)
    if not data:
        print("No SDO response (timeout).")
        return
    parsed = parse_expedited_sdo_upload_response(data)
    if not parsed:
        print(f"SDO response (raw): {data.hex()}")
        return
    index, payload = parsed
    if index != 0x6041:
        print(f"Unexpected index {index:#x}, payload {payload.hex()}")
        return
    if len(payload) >= 2:
        sw = payload[0] | (payload[1] << 8)
        print(f"Statusword 0x6041 = 0x{sw:04x} (masked DS402 state bits: 0x{sw & 0x6F:02x})")
    else:
        print(f"Statusword payload: {payload.hex()}")


def cmd_ds402_bringup_velocity(bus, node_id: int, timeout_s: float) -> None:
    """
    Minimal sequence: shutdown -> switch on -> enable op, profile velocity, small target.
    Drive must accept CiA402; scaling is device-specific.
    """
    steps = [
        ("shutdown", build_control_word(node_id, 0x0006)),
        ("switch_on", build_control_word(node_id, 0x0007)),
        ("enable_op", build_control_word(node_id, 0x000F)),
        ("mode_pv", build_mode_of_operation(node_id, 3)),
        ("target_vel", build_target_velocity(node_id, 100)),
    ]
    for name, fr in steps:
        _send(bus, fr.arbitration_id, fr.data)
        time.sleep(0.05)
        r = _recv_sdo_response(bus, node_id, timeout_s)
        if r:
            print(f"  {name}: SDO ack {r.hex()}")
        else:
            print(f"  {name}: (no SDO reply within {timeout_s}s)")
    cmd_read_status(bus, node_id, timeout_s)


def main() -> int:
    parser = argparse.ArgumentParser(description="USB-CAN CANopen / DS402-style host probe")
    parser.add_argument("--interface", default="socketcan", help="python-can interface name")
    parser.add_argument("--channel", default="can0", help="e.g. can0 or COM5 for slcan")
    parser.add_argument("--bitrate", type=int, default=500_000, help="CAN nominal bitrate")
    parser.add_argument("--node-id", type=int, default=1)
    parser.add_argument(
        "--action",
        choices=("nmt_start", "read_status", "ds402_velocity_demo"),
        default="read_status",
    )
    parser.add_argument("--timeout", type=float, default=0.5, help="SDO response wait (seconds)")
    args = parser.parse_args()

    try:
        import can  # noqa: F401
    except ImportError:
        print("Install python-can: pip install -r requirements.txt", file=sys.stderr)
        return 2

    bus = _open_bus(args.interface, args.channel, args.bitrate)
    try:
        if args.action == "nmt_start":
            cmd_nmt_start(bus, args.node_id)
        elif args.action == "read_status":
            cmd_read_status(bus, args.node_id, args.timeout)
        else:
            cmd_nmt_start(bus, args.node_id)
            time.sleep(0.1)
            cmd_ds402_bringup_velocity(bus, args.node_id, args.timeout)
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
