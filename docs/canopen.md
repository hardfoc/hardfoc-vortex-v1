# CANopen

The CANopen side of Vortex V1: the CANopen device on the CAN port, the object dictionary and how it is generated, and
the tools that read it.

Vortex V1 is a CANopen **device** (slave): a CiA 301 node that will carry the CiA 402 drive profile. It is not a
master for other drives. The [CANopen node design](design/canopen-node.md) is the plan; this page describes the code.

## What runs today

| Piece | State | Where |
| --- | --- | --- |
| CANopenNode v4 | Compiled into the HAL only with `CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE` | HAL `third_party/CANopenNode` |
| CO driver and ports | hf-core's generic driver `CO_driver_hardfoc.c` and the `EspTwaiPort` binding | HAL `lib/core/handlers/common/canopen/` |
| CANopen device | `CoNode`, served by the `co_svc` task; started by boot step `can_worker` when menuconfig selects **CANopen device** | [`main/apps/node/canopen/CoNode.cpp`](../main/apps/node/canopen/CoNode.cpp), [`CanopenDevice.cpp`](../main/apps/node/canopen/CanopenDevice.cpp) |
| Object dictionary in the build | The Vortex dictionary generated from `od/vortex_od.yaml` (`HF_CORE_CANOPENNODE_OD_DIR`) | `od/canopennode/` |
| Stack configuration | SDO block transfer with CRC, configurable EMCY, LSS fastscan, timer-next | [`main/apps/node/canopen/config/hf_co_app_config.h`](../main/apps/node/canopen/config/hf_co_app_config.h) |
| Host node | The same `CoNode` on SocketCAN or the pipe bus, tested by python-canopen in CI | `host/node`, `tests/node` |

How a frame travels:

- **Receive.** The TWAI RX interrupt hands each frame to a lock-free ring in the CO driver and wakes `co_svc`.
  `co_svc` dispatches the frame to CANopenNode in task context, through the frame router.
- **Transmit.** Frames go out with `EspCan::TrySendMessage()`, which never blocks. A full TX queue leaves the frame
  pending in its CANopenNode buffer, and `CO_CANmodule_process()` sends it again.
- **Errors.** TEC, REC and bus-off become CANopen EMCY messages, and a bus-off controller gets a recovery request.
- **Router.** `FrameRouter` applies `contracts/router_policy.hpp`. SDO server 2 (0x1201, virtual COB-IDs
  0x6C0/0x5C0 + node-ID) is never reachable from the CAN port.

!!! note "Not here yet"
    Parameter storage (0x1010 / 0x1011) and the NVS LSS record are the next step of the
    [design](design/canopen-node.md#102-ordered-prs). After that come the CiA 402 axis and `drive_rt`, then the
    fault funnel. Until then, a reset forgets an LSS-assigned node-ID, and the node runs with a 100 Hz tick
    ([Threads](threads.md#tasks)).

### Start sequence

`StartCanopenDevice(EspCan&, bit rate)` creates the port, the node and the service task. `CoNode::Start()` then
runs, in order:

| Step | Call | Values |
| --- | --- | --- |
| 1 | `CO_new()` | Allocates the stack on the heap |
| 2 | `CO_CANinit()` | The CO driver instance with the `EspTwaiPort` in the CAN slot; bit rate from `CommChannelsManager::GetCanBitRate()` (500 kbit/s) |
| 3 | `CO_LSSinit()` | LSS address from 0x1018; pending node-ID and bit rate |
| 4 | `CO_CANopenInit()` | Node ID `CONFIG_VORTEX_CANOPEN_NODE_ID` (or the LSS-assigned one); first heartbeat 500 ms; SDO server timeout 1000 ms; NMT startup from 0x1F80 (bit 2 = 0, the default, waits for an NMT start) |
| 5 | `CO_CANopenInitPDO()` | PDOs from the dictionary |
| 6 | `CO_CANsetNormalMode()`, task `co_svc` | 6144-byte stack, priority 17; woken by received frames and by CANopenNode's timer-next, at least one tick per wait |

`CO_ERROR_NODE_ID_UNCONFIGURED_LSS` is accepted in steps 4 and 5, so an unconfigured node (ID 255) can be set up with
LSS. NMT reset communication repeats steps 2 to 6. NMT reset node restarts the chip.

## The object dictionary

One YAML file, [`od/vortex_od.yaml`](../od/vortex_od.yaml), is the source of every artefact. `tools/odgen` writes
them and CI fails when one is stale:

| Artefact | Path | Used by |
| --- | --- | --- |
| CANopenNode dictionary | `od/canopennode/OD.c`, `OD.h` | the firmware and the host node |
| EDS (CiA 306) | `od/vortex_v1.eds`, also packaged in `tools/vortex_canopen` | CANopen masters and tools |
| C++ identifiers | `contracts/vortex_od_ids.hpp` | contracts and host tests |
| Python identifiers | `tools/vortex_canopen/vortex_canopen/od_ids.py` | host tools |
| Reference page | `docs/od-reference.md` | the [Object dictionary](od-reference.md) page of this site |

| Device field | Value |
| --- | --- |
| Product name | HardFOC Vortex V1 |
| Vendor ID | `0x00000000` (not registered with CiA yet) |
| Product code | `0x00560001` |
| Revision (`0x1018:03`) | 1.0 |

`od/vortex_od.lock.json` records the dictionary hash for each revision: changing the dictionary without bumping
`device.revision` fails the check.

```bash
python3 -m pip install -r tools/odgen/requirements.txt
python3 tools/odgen            # regenerate every artefact
python3 tools/odgen --check    # what CI runs
```

## Host tools

| Tool | What it is |
| --- | --- |
| `tools/vortex_canopen` | Python package with the generated identifiers and the matching EDS (`vortex_canopen.eds_path()`), the frame-record codec and the pipe-bus python-can interface (`vortex_pipe`); the base of the planned `vortex-canopen` command line |
| `host/node` (`vortex_node_host`) | The CANopen node built for Linux or macOS, on `vcan0` or the pipe bus; see [Testing and CI](testing.md) |
| [USB-CAN scripts](tools/usb-can/index.md) | Small python-can scripts that build NMT and expedited SDO frames by hand, for bench checks |

Next: [Configuration](configuration.md).
