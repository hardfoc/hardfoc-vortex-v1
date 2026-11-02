# CANopen on Vortex V1

Vortex V1 is a CANopen **device**: a CiA 301 node with the CiA 402 drive
profile. It does not act as a master for other drives.

## What is here

| File | Role |
|---|---|
| `CoNode.{hpp,cpp}` | One CANopenNode instance over the object dictionary generated from `od/vortex_od.yaml`, its ports and the frame router. Communication reset, LSS, NMT reset node, CAN statistics (0x4202). Platform-neutral: the firmware and the host node binary build the same file. |
| `FrameRouter.{hpp,cpp}` | Maps CANopenNode's RX/TX buffers to the consumers and producers named in `contracts/router_policy.hpp` and applies those rules per port through the CO driver's policy hooks. CAN only for now; SDO server 2 (0x1201, virtual COB-IDs 0x6C0/0x5C0 + node-ID) is never reachable from or sent to the CAN port. |
| `CoSvc.hpp`, `CoSvcFreeRtos.cpp`, `CoSvcStd.cpp` | The `co_svc` service thread (design §4.4): woken by received frames and by CANopenNode's timer-next. FreeRTOS task at priority 17 on the firmware, `std::thread` on the host. |
| `CanopenDevice.{hpp,cpp}` | Firmware entry points: CoNode over `hf::can::EspTwaiPort` on the HAL's TWAI controller. |
| `config/hf_co_app_config.h` | Stack configuration (`CO_CONFIG_*`): SDO block transfer with CRC, configurable EMCY, LSS fastscan, timer-next. |

The CO driver and the port bindings live in hf-core
(`handlers/common/canopen/`): `CO_driver_hardfoc.c` (non-blocking TX with
pending-frame retry, ring-buffered RX dispatch in task context, error status
and bus-off recovery), `EspTwaiPort`, `LinuxSocketCanPort` and `PipePort`.

Enable it with **`idf.py menuconfig`**: *HardFOC Vortex HAL → Build the
CANopenNode stack*, then *HardFOC Vortex main (app) → CAN RX drain worker →
CANopen device*. The default worker is **None**: the firmware brings the
board up and leaves the CAN bus alone. The node-ID is
`CONFIG_VORTEX_CANOPEN_NODE_ID` until LSS assigns one. NMT startup follows
0x1F80: bit 2 = 0 (default) waits for an NMT start.

## Host node and CI suites

`host/node` builds the same node for Linux or macOS as `vortex_node_host`,
on SocketCAN (`--port vcan0`) or on the pipe bus (`--port pipe:<path>`, a
UNIX-socket hub carrying the §7.2 frame records). `tests/node` runs
python-canopen against it:

```sh
git submodule update --init hal/hf-hal-vortex-v1
git -C hal/hf-hal-vortex-v1 submodule update --init lib/core third_party/CANopenNode
cmake -S host/node -B build/node-host -G Ninja && cmake --build build/node-host
python -m pip install -r tests/node/requirements.txt
python -m pytest tests/node                 # VORTEX_BUS=auto: vcan0 if present, else the pipe bus
```

| Suite | Covers |
|---|---|
| `test_nmt_hb.py` | boot-up, NMT states and addressing, heartbeat period (0x1017), 0x1F80, reset communication / node |
| `test_sdo.py` | expedited 1–4 bytes, segmented, block upload / download with CRC (and a corrupted CRC), abort codes, SDO server 2 kept off the CAN port |
| `test_pdo.py` | mapping procedure and aborts, event timer, inhibit time, SYNC TPDO, RPDO into the OD, RPDO length and timeout EMCYs |
| `test_lss.py` | fastscan, configure / store / activate node-ID (persists across a restart), inquire identity and node-ID, bit-timing table |
| `test_frame_record.py` | the Python frame-record codec against the same golden bytes as hf-core's C++ codec |

## Not here yet

Parameter storage (0x1010 / 0x1011) and the NVS LSS record (step 7), the
CiA 402 axis and `drive_rt` (step 8), the fault funnel and safety arbiter
(step 9), the wireless ports. The full plan, object dictionary layout,
threads, timing, wireless access rules and CAN FD path are in
`docs/design/canopen-node.md`.
