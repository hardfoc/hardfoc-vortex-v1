"""
Offline checks: CANopen frame bytes match hf-utils-canopen C++ builders.
Run: python test_frame_encoding.py
"""

from __future__ import annotations

from canopen_frames import (
    build_control_word,
    build_mode_of_operation,
    build_nmt_start_remote_node,
    build_sdo_download,
    build_sdo_upload_request,
    build_status_word_request,
    build_target_position,
    build_target_velocity,
)


def _eq_frame(label: str, f, expect_id: int, expect_data: bytes) -> None:
    assert f.arbitration_id == expect_id, f"{label} id {f.arbitration_id:x} != {expect_id:x}"
    assert f.data == expect_data, f"{label} data {f.data.hex()} != {expect_data.hex()}"


def main() -> None:
    _eq_frame("nmt", build_nmt_start_remote_node(5), 0x000, bytes([0x01, 0x05]))

    # 4-byte download 0x1234 sub 0x56 value 0x78563412 -> cmd 0x23
    f = build_sdo_download(3, 0x1234, 0x56, 0x78563412, 4)
    _eq_frame(
        "sdo4",
        f,
        0x603,
        bytes([0x23, 0x34, 0x12, 0x56, 0x12, 0x34, 0x56, 0x78]),
    )

    f2 = build_sdo_download(1, 0x6040, 0x00, 0x0006, 2)  # controlword shutdown
    _eq_frame("cw", f2, 0x601, bytes([0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]))

    f3 = build_sdo_upload_request(2, 0x6041, 0x00)
    _eq_frame("up", f3, 0x602, bytes([0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]))

    _eq_frame(
        "status_req",
        build_status_word_request(7),
        0x607,
        bytes([0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]),
    )

    mv = build_mode_of_operation(1, 3)  # profile velocity
    _eq_frame("mode", mv, 0x601, bytes([0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]))

    tv = build_target_velocity(1, -1000 & 0xFFFFFFFF)
    _eq_frame("tv", tv, 0x601, bytes([0x23, 0xFF, 0x60, 0x00, 0x18, 0xFC, 0xFF, 0xFF]))

    tp = build_target_position(1, 1000)
    _eq_frame("tp", tp, 0x601, bytes([0x23, 0x7A, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00]))

    cw = build_control_word(1, 0x000F)  # enable operation
    _eq_frame("enable_op", cw, 0x601, bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]))

    print("test_frame_encoding: all assertions passed.")


if __name__ == "__main__":
    main()
