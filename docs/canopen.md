# CANopen

The CANopen side of Vortex V1: what runs on the CAN port today, the object dictionary and how it is generated, and
the tools that read it.

Vortex V1 is a CANopen **device** (slave): a CiA 301 node that will carry the CiA 402 drive profile. It is not a
master for other drives. The [CANopen node design](design/canopen-node.md) is the plan; this page describes the code.

## What runs today

| Piece | State | Where |
| --- | --- | --- |
| CANopenNode v4 | Compiled into the HAL only with `CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE` | HAL `third_party/CANopenNode` |
| Slave runtime | Started by boot step `can_worker` when menuconfig selects **CANopenNode slave** | `main/apps/node/canopen/CanopenSlaveRuntime.cpp` |
| Object dictionary in the build | The upstream CANopenNode example (`example/OD.c`), not the Vortex dictionary yet | HAL CMake |
| Vortex object dictionary | Generated and checked in CI; not linked into the firmware yet | `od/` |

!!! warning "The slave runtime is a bring-up shim"
    It cannot receive frames on hardware: it polls `BaseCan::ReceiveMessage()`, which the ESP32 CAN wrapper does
    not implement yet. It does not persist parameters, and its loop never sleeps at the current tick rate
    ([Threads](threads.md#tasks)). The [design's inventory](design/canopen-node.md#3-what-exists-today-verified)
    lists every gap with its source line.

### Start sequence

`StartCanopenSlaveRuntime(BaseCan&)` runs, in order:

| Step | Call | Values |
| --- | --- | --- |
| 1 | `hf_co_driver_register_tx()` | TX goes to `BaseCan::SendMessage()` with a 20 ms timeout; classic frames, 8 bytes at most |
| 2 | `CO_new()` | Allocates the stack on the heap; logs the bytes used |
| 3 | `CO_CANinit()` | 500 kbit/s |
| 4 | `CO_CANopenInit()` | Node ID `CONFIG_VORTEX_CANOPEN_NODE_ID`; first heartbeat 500 ms; SDO server timeout 1000 ms, client 500 ms; NMT starts to Operational |
| 5 | `CO_CANopenInitPDO()` | PDOs from the dictionary |
| 6 | `CO_CANsetNormalMode()`, task `canopen_slave` | 8192-byte stack, priority 5, no core affinity |

`CO_ERROR_NODE_ID_UNCONFIGURED_LSS` is accepted in steps 4 and 5, so an unconfigured node can still be set up with
LSS. The task loop drains received standard frames into CANopenNode, then runs SYNC, RPDO, `CO_process()` and TPDO.

## The object dictionary

One YAML file, [`od/vortex_od.yaml`](../od/vortex_od.yaml), is the source of every artefact. `tools/odgen` writes
them and CI fails when one is stale:

| Artefact | Path | Used by |
| --- | --- | --- |
| CANopenNode dictionary | `od/canopennode/OD.c`, `OD.h` | the firmware, once the runtime is rebuilt |
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
| `tools/vortex_canopen` | Python package with the generated identifiers and the matching EDS (`vortex_canopen.eds_path()`); the base of the planned `vortex-canopen` command line |
| [USB-CAN scripts](tools/usb-can/index.md) | Small python-can scripts that build NMT and expedited SDO frames by hand, for bench checks |

Next: [Configuration](configuration.md).
