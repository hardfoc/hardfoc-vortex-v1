"""CiA 305 LSS slave (design §9.4 test_lss).

A second, LSS-unconfigured node (node-ID 0xFF, its own serial number) joins
the bus next to the session node. The suite finds it with fastscan,
assigns a node-ID and stores it, checks that the new ID takes effect and
survives a restart (the sim NVS file stands in for the `lss` NVS record),
and inquires identity and node-ID of the configured node through selective
switching. (CANopenNode's LSS slave has no "identify remote slave"
service, 0x46..0x4B, so that is not tested.)
"""

from __future__ import annotations

import os
import tempfile
import time

import pytest
from canopen import lss as lss_mod
from canopen.lss import LssError

from conftest import HB_COB, NODE_ID, NodeProcess

VENDOR = 0x00000000
PRODUCT = 0x00560001
REVISION = 0x00010000
SERIAL_B = 0x00C0FFEE
NEW_ID = 10


@pytest.fixture
def unconfigured(bench, bus_spec):
    nvs = os.path.join(tempfile.mkdtemp(prefix="vxlss"), "nvs.txt")
    proc = NodeProcess(bus_spec, 0xFF, ["--serial", str(SERIAL_B), "--nvs", nvs]).start()
    time.sleep(0.1)
    yield proc, nvs
    proc.stop()
    bench.network.lss.send_switch_state_global(bench.network.lss.WAITING_STATE)


def test_fastscan_configure_store_and_activate_node_id(bench, unconfigured):
    proc, nvs = unconfigured
    lss = bench.network.lss
    # Fastscan (python-canopen walks all 128 address bits, ~13 s) answers only
    # for the unconfigured node, and leaves it in LSS configuration state.
    found, address = lss.fast_scan()
    assert found
    assert address == [VENDOR, PRODUCT, REVISION, SERIAL_B]
    lss.configure_node_id(NEW_ID)
    lss.store_configuration()
    bench.log.clear()
    # Back to waiting: CANopenNode applies the pending node-ID with a reset communication.
    lss.send_switch_state_global(lss.WAITING_STATE)
    assert bench.wait_bootup(NEW_ID, timeout=2.0)
    node_b = bench.network.add_node(NEW_ID, bench.node.object_dictionary)
    try:
        assert node_b.sdo[0x1018][4].raw == SERIAL_B
        assert node_b.sdo[0x1201][1].raw == 0x6C0 + NEW_ID
    finally:
        del bench.network[NEW_ID]

    # Restart with the same sim NVS: the stored node-ID wins over --node-id 255.
    proc.stop()
    bench.log.clear()
    proc.start()
    assert bench.wait_bootup(NEW_ID, timeout=3.0)
    assert open(nvs).read().startswith("lss=0a")


def test_inquire_identity_and_node_id_of_the_configured_node(bench):
    lss = bench.network.lss
    lss.send_switch_state_selective(VENDOR, PRODUCT, REVISION, 0)
    try:
        assert lss.inquire_node_id() == NODE_ID
        assert lss.inquire_lss_address(lss_mod.CS_INQUIRE_VENDOR_ID) == VENDOR
        assert lss.inquire_lss_address(lss_mod.CS_INQUIRE_PRODUCT_CODE) == PRODUCT
        assert lss.inquire_lss_address(lss_mod.CS_INQUIRE_REVISION_NUMBER) == REVISION
        assert lss.inquire_lss_address(lss_mod.CS_INQUIRE_SERIAL_NUMBER) == 0
    finally:
        lss.send_switch_state_global(lss.WAITING_STATE)
    # Selecting and inquiring does not disturb the node.
    assert bench.node.sdo[0x1000].raw == 0x00020192


def test_bit_timing_table(bench):
    lss = bench.network.lss
    lss.send_switch_state_selective(VENDOR, PRODUCT, REVISION, 0)
    try:
        lss.configure_bit_timing(2)  # 500 kbit/s
        with pytest.raises(LssError):
            lss.configure_bit_timing(5)  # reserved entry of the CiA 305 table
    finally:
        lss.send_switch_state_global(lss.WAITING_STATE)


def test_configure_node_id_rejects_invalid_ids(bench):
    lss = bench.network.lss
    lss.send_switch_state_selective(VENDOR, PRODUCT, REVISION, 0)
    try:
        with pytest.raises(LssError):
            lss.configure_node_id(0x80)
    finally:
        lss.send_switch_state_global(lss.WAITING_STATE)
    assert bench.node.sdo[0x1000].raw == 0x00020192


@pytest.mark.parametrize("path", ["/etc/passwd", "../outside/nvs.txt", "/tmp/../etc/nvs.txt"])
def test_nvs_path_outside_the_allowed_bases_is_refused(bus_spec, path):
    import subprocess
    from conftest import node_binary
    r = subprocess.run([str(node_binary()), "--node-id", "9", "--port", bus_spec.port_arg, "--nvs", path],
                       capture_output=True, text=True, timeout=10)
    assert r.returncode == 2
    assert "refused" in r.stderr
