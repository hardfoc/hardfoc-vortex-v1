"""CiA 301 SDO server 1 (design §9.4 test_sdo).

Expedited (1/2/3/4 bytes), segmented and block transfer (upload and
download, CRC on, plus a corrupted CRC that the server must abort), the
abort codes the stack produces today, and the router keeping SDO server 2
(tunnel-only virtual COB-IDs) off the CAN-like port.
"""

from __future__ import annotations

import struct
import time

import pytest
from canopen.sdo.exceptions import SdoAbortedError

from conftest import NODE_ID

SDO_RX = 0x600 + NODE_ID
SDO_TX = 0x580 + NODE_ID


def raw_request(bench, data: bytes, timeout: float = 1.0):
    bench.log.clear()
    bench.send(SDO_RX, data.ljust(8, b"\x00"))
    return bench.log.wait_for(lambda m: m.arbitration_id == SDO_TX, timeout)


def abort_code(msg) -> int:
    assert msg is not None and msg.data[0] == 0x80, f"expected an abort, got {msg}"
    return struct.unpack_from("<I", bytes(msg.data), 4)[0]


# ── expedited ───────────────────────────────────────────────────────────────

@pytest.mark.parametrize(
    "index, sub, value, size",
    [
        (0x6060, 0, -7, 1),            # I8
        (0x1029, 1, 2, 1),             # U8
        (0x1017, 0, 1234, 2),          # U16
        (0x605A, 0, -3, 2),            # I16
        (0x1006, 0, 0xDEADBEEF, 4),    # U32
        (0x607A, 0, -123456789, 4),    # I32
    ],
    ids=["i8", "u8", "u16", "i16", "u32", "i32"],
)
def test_expedited_download_and_upload(bench, index, sub, value, size):
    var = bench.node.sdo[index] if sub == 0 else bench.node.sdo[index][sub]
    var.raw = value
    assert var.raw == value
    assert len(bench.node.sdo.upload(index, sub)) == size


def test_expedited_three_bytes(bench):
    # A 3-character visible string travels in one expedited frame each way.
    bench.node.sdo.download(0x6403, 0, b"abc")
    msg = raw_request(bench, bytes([0x40, 0x03, 0x64, 0x00]))
    assert msg is not None
    assert msg.data[0] == 0x47  # expedited, size indicated, 4 - 1 = 3 bytes
    assert bytes(msg.data[4:7]) == b"abc"
    assert bench.node.sdo.upload(0x6403, 0) == b"abc"


def test_expedited_wire_format_of_a_u16_read(bench):
    msg = raw_request(bench, bytes([0x40, 0x17, 0x10, 0x00]))
    assert bytes(msg.data[:4]) == bytes([0x4B, 0x17, 0x10, 0x00])
    assert struct.unpack_from("<H", bytes(msg.data), 4)[0] == 100


# ── segmented ───────────────────────────────────────────────────────────────

def test_segmented_upload(bench):
    assert bench.node.sdo.upload(0x1008, 0) == b"HardFOC Vortex V1"


def test_segmented_download(bench):
    text = b"segmented-motor-maker-0123456789"  # 32 bytes, 5 segments
    bench.node.sdo.download(0x6404, 0, text)
    assert bench.node.sdo.upload(0x6404, 0).rstrip(b"\x00") == text


def test_segmented_toggle_error_aborts(bench):
    # Initiate a segmented download of 10 bytes, then send the first segment
    # with the toggle bit set: the server aborts with 0x05030000.
    msg = raw_request(bench, bytes([0x21, 0x04, 0x64, 0x00, 10, 0, 0, 0]))
    assert msg is not None and msg.data[0] == 0x60
    msg = raw_request(bench, bytes([0x10]) + b"0123456")
    assert abort_code(msg) == 0x05030000


# ── block ───────────────────────────────────────────────────────────────────

def test_block_upload_with_crc(bench):
    with bench.node.sdo.open(0x1008, 0, "rb", block_transfer=True, request_crc_support=True) as f:
        assert f.read() == b"HardFOC Vortex V1"


def test_block_download_with_crc(bench):
    text = b"block-transfer-with-crc-16-ccitt"
    with bench.node.sdo.open(0x6403, 0, "wb", size=len(text), block_transfer=True, request_crc_support=True) as f:
        f.write(text)
    assert bench.node.sdo.upload(0x6403, 0).rstrip(b"\x00") == text
    with bench.node.sdo.open(0x6403, 0, "rb", block_transfer=True) as f:
        assert f.read().rstrip(b"\x00") == text


def test_block_download_bad_crc_aborts(bench):
    """Raw block download whose end frame carries a wrong CRC: abort 0x05040004."""
    data = b"0123456789ABCD"  # 14 bytes: two segments
    # initiate: ccs=6, cc (CRC) = bit 2, s = bit 1, cs = 0
    msg = raw_request(bench, bytes([0xC6, 0x03, 0x64, 0x00]) + struct.pack("<I", len(data)))
    assert msg is not None and (msg.data[0] & 0xE3) == 0xA0, f"block initiate refused: {msg}"
    blksize = msg.data[4]
    assert blksize >= 2
    bench.log.clear()
    bench.send(SDO_RX, bytes([0x01]) + data[0:7])
    bench.send(SDO_RX, bytes([0x82]) + data[7:14])  # last segment of the block, seqno 2
    ack = bench.log.wait_for(lambda m: m.arbitration_id == SDO_TX, 1.0)
    assert ack is not None and ack.data[0] == 0xA2 and ack.data[1] == 2
    # end: ccs=6, n = 0 unused bytes in the last segment, cs=1; CRC deliberately wrong
    msg = raw_request(bench, bytes([0xC1, 0xAD, 0xDE]))
    assert abort_code(msg) == 0x05040004


# ── aborts ──────────────────────────────────────────────────────────────────

def test_abort_object_does_not_exist(bench):
    msg = raw_request(bench, bytes([0x40, 0xFF, 0x2F, 0x00]))
    assert abort_code(msg) == 0x06020000


def test_abort_sub_index_does_not_exist(bench):
    msg = raw_request(bench, bytes([0x40, 0x17, 0x10, 0x01]))
    assert abort_code(msg) == 0x06090011


def test_abort_write_read_only(bench):
    with pytest.raises(SdoAbortedError) as e:
        bench.node.sdo[0x1000].raw = 1
    assert e.value.code == 0x06010002


def test_abort_read_write_only(bench):
    with pytest.raises(SdoAbortedError) as e:
        _ = bench.node.sdo[0x4002].raw
    assert e.value.code == 0x06010001


def test_abort_length_too_high(bench):
    # 4 bytes into the U16 0x1017
    msg = raw_request(bench, bytes([0x23, 0x17, 0x10, 0x00, 1, 0, 0, 0]))
    assert abort_code(msg) in (0x06070012, 0x06070010)


def test_abort_length_too_low(bench):
    # 1 byte into the U32 0x1006
    msg = raw_request(bench, bytes([0x2F, 0x06, 0x10, 0x00, 1]))
    assert abort_code(msg) in (0x06070013, 0x06070010)


def test_abort_unknown_command_specifier(bench):
    msg = raw_request(bench, bytes([0xE0, 0x00, 0x10, 0x00]))
    assert abort_code(msg) == 0x05040001


def test_values_unchanged_after_aborts(bench):
    raw_request(bench, bytes([0x23, 0x17, 0x10, 0x00, 1, 0, 0, 0]))
    assert bench.node.sdo[0x1017].raw == 100


# ── router: SDO server 2 is tunnel-only ─────────────────────────────────────

def test_sdo_server_2_uses_the_virtual_cob_ids(bench):
    assert bench.node.sdo[0x1201][1].raw == 0x6C0 + NODE_ID
    assert bench.node.sdo[0x1201][2].raw == 0x5C0 + NODE_ID


def test_sdo_server_2_is_not_reachable_from_the_can_port(bench):
    drops_before = bench.node.sdo[0x4202][0x0A].raw  # router drops, host port
    bench.log.clear()
    bench.send(0x6C0 + NODE_ID, bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]))
    reply = bench.log.wait_for(lambda m: m.arbitration_id in (0x5C0 + NODE_ID, 0x580 + NODE_ID), 0.4)
    assert reply is None
    time.sleep(0.25)  # 0x4202 refreshes every 100 ms
    assert bench.node.sdo[0x4202][0x0A].raw == drops_before + 1
