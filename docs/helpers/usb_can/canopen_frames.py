"""
CANopen frame builders aligned with hf-utils-canopen (CanOpenUtils.cpp / CanOpenMotorUtils.cpp).

COB-IDs: NMT 0x000, SDO client->server 0x600+node, SDO server->client 0x580+node.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass
class CanFrame:
    """Minimal frame; maps to CanOpen::CanFrame in C++."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False
    is_remote_frame: bool = False


def build_nmt_start_remote_node(node_id: int) -> CanFrame:
    """Same as CanOpen::BuildNmt(nodeId, NmtCommand::StartNode)."""
    return CanFrame(0x000, bytes([0x01, node_id & 0xFF]))


def build_sdo_download(node_id: int, index: int, sub_index: int, value: int, size: int) -> CanFrame:
    """Expedited SDO download; matches CanOpen::BuildSdoDownload."""
    if size == 1:
        cmd = 0x2F
    elif size == 2:
        cmd = 0x2B
    else:
        cmd = 0x23
        size = 4
    data = bytearray(8)
    data[0] = cmd
    data[1] = index & 0xFF
    data[2] = (index >> 8) & 0xFF
    data[3] = sub_index & 0xFF
    data[4] = value & 0xFF
    data[5] = (value >> 8) & 0xFF
    data[6] = (value >> 16) & 0xFF
    data[7] = (value >> 24) & 0xFF
    if size < 4:
        for i in range(4 + size, 8):
            data[i] = 0x00
    return CanFrame(0x600 + (node_id & 0x7F), bytes(data))


def build_sdo_upload_request(node_id: int, index: int, sub_index: int) -> CanFrame:
    """Matches CanOpen::BuildSdoUpload."""
    data = bytes(
        [
            0x40,
            index & 0xFF,
            (index >> 8) & 0xFF,
            sub_index & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )
    return CanFrame(0x600 + (node_id & 0x7F), data)


def build_control_word(node_id: int, control_word: int) -> CanFrame:
    return build_sdo_download(node_id, 0x6040, 0x00, control_word & 0xFFFF, 2)


def build_mode_of_operation(node_id: int, mode: int) -> CanFrame:
    return build_sdo_download(node_id, 0x6060, 0x00, mode & 0xFF, 1)


def build_target_velocity(node_id: int, velocity: int) -> CanFrame:
    v = velocity & 0xFFFFFFFF
    return build_sdo_download(node_id, 0x60FF, 0x00, v, 4)


def build_target_position(node_id: int, position: int) -> CanFrame:
    p = position & 0xFFFFFFFF
    return build_sdo_download(node_id, 0x607A, 0x00, p, 4)


def build_status_word_request(node_id: int) -> CanFrame:
    return build_sdo_upload_request(node_id, 0x6041, 0x00)


def parse_expedited_sdo_upload_response(data: bytes) -> tuple[int, bytes] | None:
    """
    Parse expedited SDO upload response (server -> client).
    Returns (index, payload_bytes) or None if not a simple expedited response.
    """
    if len(data) < 8:
        return None
    cmd = data[0]
    # 0x4B = 2 bytes, 0x43 = 4 bytes expedited upload response
    if cmd not in (0x4F, 0x4B, 0x43):
        return None
    index = data[1] | (data[2] << 8)
    if cmd == 0x4F:
        payload = data[4:5]
    elif cmd == 0x4B:
        payload = data[4:6]
    elif cmd == 0x43:
        payload = data[4:8]
    else:
        return None
    return index, payload


def frames_to_python_can(frames: List[CanFrame]):
    """Lazy import can.Message for callers that have python-can."""
    import can  # pylint: disable=import-outside-toplevel

    return [
        can.Message(
            arbitration_id=f.arbitration_id,
            data=f.data,
            is_extended_id=f.is_extended_id,
            is_remote_frame=f.is_remote_frame,
        )
        for f in frames
    ]
