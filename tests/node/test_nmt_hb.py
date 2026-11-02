"""CiA 301 NMT and heartbeat (design §9.4 test_nmt_hb).

Boot-up message, NMT state machine and its addressing, heartbeat producer
period (0x1017), 0x1F80 NMT startup, reset communication and reset node.
"""

from __future__ import annotations

import statistics

import pytest

from conftest import HB_COB, NMT_OPERATIONAL, NMT_PREOP, NMT_STOPPED, NODE_ID

START, STOP, PREOP, RESET_NODE, RESET_COMM = 0x01, 0x02, 0x80, 0x81, 0x82


def hb_times(bench, duration):
    frames = bench.log.collect(lambda m: m.arbitration_id == HB_COB + NODE_ID, duration)
    return [t for t, _ in frames], [bytes(m.data) for _, m in frames]


def test_bootup_message_after_reset_communication(bench):
    bench.nmt(RESET_COMM)
    msg = bench.wait_bootup()
    assert msg is not None
    assert msg.dlc == 1 and not msg.is_remote_frame


def test_starts_pre_operational_and_waits_for_nmt_start(bench):
    # 0x1F80 bit 2 = 0 (default): stay pre-operational until an NMT master starts the node.
    assert bench.node.sdo[0x1F80].raw == 0
    _, states = hb_times(bench, 0.5)
    assert states and set(states) == {bytes([NMT_PREOP])}


@pytest.mark.parametrize(
    "command, state",
    [(START, NMT_OPERATIONAL), (STOP, NMT_STOPPED), (PREOP, NMT_PREOP)],
    ids=["start", "stop", "pre-operational"],
)
def test_nmt_command_changes_state(bench, command, state):
    bench.nmt(command)
    assert bench.wait_state(state)


def test_nmt_state_sequence_and_sdo_only_when_not_stopped(bench):
    bench.nmt(START)
    assert bench.wait_state(NMT_OPERATIONAL)
    assert bench.node.sdo[0x1000].raw == 0x00020192
    bench.nmt(STOP)
    assert bench.wait_state(NMT_STOPPED)
    bench.node.sdo.RESPONSE_TIMEOUT = 0.3
    try:
        with pytest.raises(Exception):
            _ = bench.node.sdo[0x1000].raw  # SDO is not available in Stopped
    finally:
        bench.node.sdo.RESPONSE_TIMEOUT = 1.0
    bench.nmt(PREOP)
    assert bench.wait_state(NMT_PREOP)
    assert bench.node.sdo[0x1000].raw == 0x00020192


def test_nmt_broadcast_reaches_the_node(bench):
    bench.nmt(START, node_id=0)
    assert bench.wait_state(NMT_OPERATIONAL)


def test_nmt_for_another_node_is_ignored(bench):
    bench.nmt(START, node_id=NODE_ID + 1)
    _, states = hb_times(bench, 0.35)
    assert states and set(states) == {bytes([NMT_PREOP])}


def test_heartbeat_period_default_100_ms(bench):
    assert bench.node.sdo[0x1017].raw == 100
    times, _ = hb_times(bench, 1.25)
    assert len(times) >= 10
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert 0.085 <= statistics.mean(gaps) <= 0.115
    assert min(gaps) >= 0.06


def test_heartbeat_period_follows_0x1017(bench):
    bench.node.sdo[0x1017].raw = 40
    bench.log.clear()
    times, _ = hb_times(bench, 0.8)
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(gaps) >= 10
    assert 0.03 <= statistics.mean(gaps) <= 0.05


def test_heartbeat_off_with_zero(bench):
    bench.node.sdo[0x1017].raw = 0
    bench.log.clear()
    times, _ = hb_times(bench, 0.4)
    assert len(times) <= 1  # at most one frame already in flight


def test_reset_communication_keeps_application_values(bench):
    bench.node.sdo[0x6060].raw = 3
    bench.log.clear()
    bench.nmt(RESET_COMM)
    assert bench.wait_bootup()
    assert bench.node.sdo[0x6060].raw == 3


def test_reset_node_restores_od_defaults(bench):
    bench.node.sdo[0x1017].raw = 250
    bench.node.sdo[0x6060].raw = 3
    bench.log.clear()
    bench.nmt(RESET_NODE)
    assert bench.wait_bootup()
    assert bench.node.sdo[0x1017].raw == 100
    assert bench.node.sdo[0x6060].raw == 0


def test_nmt_startup_self_start(bench):
    # 0x1F80 bit 2 = 1: the node enters Operational after boot-up without an NMT master.
    bench.node.sdo[0x1F80].raw = 0x04
    bench.log.clear()
    bench.nmt(RESET_COMM)
    assert bench.wait_bootup()
    assert bench.wait_state(NMT_OPERATIONAL)
