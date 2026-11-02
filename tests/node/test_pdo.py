"""CiA 301 PDO (design §9.4 test_pdo).

Default TPDO1 (event timer), PDOs only in Operational, the CiA 301 mapping
procedure, invalid mapping aborts, synchronous TPDO, inhibit time, RPDO
into the OD, RPDO length error and RPDO timeout EMCYs.
"""

from __future__ import annotations

import statistics
import struct
import time

import pytest
from canopen.sdo.exceptions import SdoAbortedError

from conftest import NMT_OPERATIONAL, NMT_PREOP, NODE_ID

TPDO1 = 0x180 + NODE_ID
TPDO2 = 0x280 + NODE_ID
RPDO1 = 0x200 + NODE_ID
EMCY = 0x080 + NODE_ID
SYNC = 0x080
DISABLE = 0x80000000


def start(bench):
    bench.nmt(0x01)
    assert bench.wait_state(NMT_OPERATIONAL)
    bench.log.clear()


def frames(bench, cob, duration):
    return bench.log.collect(lambda m: m.arbitration_id == cob, duration)


def mapping(index, sub, bits):
    return (index << 16) | (sub << 8) | bits


def remap_tpdo1(bench, entries, transmission_type=254, event_ms=0, inhibit_100us=0):
    sdo = bench.node.sdo
    sdo[0x1800][1].raw = DISABLE | TPDO1 | 0x40000000
    sdo[0x1A00][0].raw = 0
    for i, e in enumerate(entries, start=1):
        sdo[0x1A00][i].raw = e
    sdo[0x1A00][0].raw = len(entries)
    sdo[0x1800][2].raw = transmission_type
    sdo[0x1800][3].raw = inhibit_100us
    sdo[0x1800][5].raw = event_ms
    sdo[0x1800][1].raw = TPDO1 | 0x40000000


def emcy_with(bench, code, timeout=1.0):
    return bench.log.wait_for(
        lambda m: m.arbitration_id == EMCY and len(m.data) == 8 and struct.unpack_from("<H", bytes(m.data))[0] == code,
        timeout,
    )


def test_no_pdos_in_pre_operational(bench):
    assert frames(bench, TPDO1, 0.35) == []


def test_default_tpdo1_event_timer(bench):
    # 0x1800: type 254, event timer 100 ms; 0x1A00: statusword + modes display (3 bytes)
    start(bench)
    got = frames(bench, TPDO1, 1.05)
    assert len(got) >= 8
    assert all(m.dlc == 3 for _, m in got)
    gaps = [b[0] - a[0] for a, b in zip(got, got[1:])]
    assert 0.085 <= statistics.mean(gaps) <= 0.115


def test_tpdos_stop_in_stopped_state(bench):
    start(bench)
    assert frames(bench, TPDO1, 0.25)
    bench.nmt(0x02)
    time.sleep(0.05)
    bench.log.clear()
    assert frames(bench, TPDO1, 0.3) == []


def test_mapping_procedure_and_content(bench):
    # 0x1200:01 (ro, constant 0x605) and 0x1280:01 (rw) are TPDO-mappable U32s.
    remap_tpdo1(bench, [mapping(0x1200, 1, 32), mapping(0x1280, 1, 32)], event_ms=20)
    assert bench.node.sdo[0x1A00][0].raw == 2
    assert bench.node.sdo[0x1A00][1].raw == 0x12000120
    bench.node.sdo[0x1280][1].raw = 0x12345678
    start(bench)
    msg = bench.log.wait_for(lambda m: m.arbitration_id == TPDO1, 0.5)
    assert msg is not None and msg.dlc == 8
    assert struct.unpack("<II", bytes(msg.data)) == (0x600 + NODE_ID, 0x12345678)
    bench.node.sdo[0x1280][1].raw = 0x0BADF00D
    time.sleep(0.05)
    bench.log.clear()
    msg = bench.log.wait_for(lambda m: m.arbitration_id == TPDO1, 0.5)
    assert struct.unpack("<II", bytes(msg.data))[1] == 0x0BADF00D


def test_mapping_rejected_while_pdo_valid(bench):
    with pytest.raises(SdoAbortedError):
        bench.node.sdo[0x1A00][0].raw = 0  # TPDO1 COB-ID still valid


def test_mapping_of_a_non_mappable_object_aborts(bench):
    bench.node.sdo[0x1800][1].raw = DISABLE | TPDO1
    bench.node.sdo[0x1A00][0].raw = 0
    with pytest.raises(SdoAbortedError) as e:
        bench.node.sdo[0x1A00][1].raw = mapping(0x1017, 0, 16)
    assert e.value.code == 0x06040041


def test_mapping_longer_than_8_bytes_aborts(bench):
    bench.node.sdo[0x1800][1].raw = DISABLE | TPDO1
    bench.node.sdo[0x1A00][0].raw = 0
    for i in range(1, 4):
        bench.node.sdo[0x1A00][i].raw = mapping(0x1200, 1, 32)
    with pytest.raises(SdoAbortedError) as e:
        bench.node.sdo[0x1A00][0].raw = 3  # 12 bytes on a classic build
    assert e.value.code == 0x06040042


def test_synchronous_tpdo_follows_sync(bench):
    # TPDO2: type 1 (every SYNC), statusword + position actual (6 bytes)
    assert bench.node.sdo[0x1801][2].raw == 1
    start(bench)
    assert frames(bench, TPDO2, 0.3) == []
    for _ in range(5):
        bench.send(SYNC)
        msg = bench.log.wait_for(lambda m: m.arbitration_id == TPDO2, 0.5)
        assert msg is not None and msg.dlc == 6


def test_inhibit_time_limits_the_rate(bench):
    # event timer 10 ms, inhibit 50 ms: frames no closer than ~50 ms
    remap_tpdo1(bench, [mapping(0x1200, 1, 32)], event_ms=10, inhibit_100us=500)
    start(bench)
    got = frames(bench, TPDO1, 0.8)
    gaps = [b[0] - a[0] for a, b in zip(got, got[1:])]
    assert len(gaps) >= 5
    assert min(gaps) >= 0.04
    assert statistics.mean(gaps) <= 0.075


def test_rpdo_writes_the_od(bench):
    # RPDO1: controlword (U16) + modes of operation (I8), type 255
    start(bench)
    bench.send(RPDO1, struct.pack("<Hb", 0x000F, -3))
    time.sleep(0.05)
    assert bench.node.sdo[0x6040].raw == 0x000F
    assert bench.node.sdo[0x6060].raw == -3


def test_rpdo_ignored_in_pre_operational(bench):
    bench.send(RPDO1, struct.pack("<Hb", 0x0006, 1))
    time.sleep(0.05)
    assert bench.node.sdo[0x6040].raw == 0


def test_rpdo_length_error_emcy(bench):
    start(bench)
    bench.send(RPDO1, b"\x06\x00")  # 2 of 3 bytes
    msg = emcy_with(bench, 0x8210)
    assert msg is not None  # a "communication info" error: 0x1001 stays clear, NMT state unchanged
    bench.send(RPDO1, struct.pack("<Hb", 0x0006, 1))
    assert emcy_with(bench, 0x0000) is not None  # error reset once a correct RPDO arrives


def test_rpdo_timeout_emcy(bench):
    bench.node.sdo[0x1400][5].raw = 100  # event timer = RPDO timeout monitor
    start(bench)
    bench.send(RPDO1, struct.pack("<Hb", 0x0006, 1))
    msg = emcy_with(bench, 0x8250, timeout=1.0)
    assert msg is not None
    assert msg.data[2] & 0x10  # error register: communication (a critical communication error)
    # 0x1029:01 = 0: a communication error takes the node to pre-operational
    assert bench.wait_state(NMT_PREOP)
