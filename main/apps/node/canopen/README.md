# Vortex `main` CANopen application layer

## What is implemented today

| Piece | Role |
|--------|------|
| **`CommChannelsManager::GetCanOpenBus()`** | Single owned **`BaseCan*`** after HAL init (TWAI). |
| **`CanOpenBaseCanLink` / `HfUtilsCanOpenTransport`** (hf-core) | **`CanOpen::CanFrame`** ↔ **`hf_can_message_t`** on **`BaseCan`**. |
| **`hf-utils-canopen`** | Frame builders (`BuildNmt`, `BuildSdoDownload`, `BuildSdoUpload`, motor helpers). |
| **`CanOpenExtras.h`** | **Inverse** helpers: **`ParseSdoResponse`**, **`ParseEmergency`**, **`ParseHeartbeat`**, **`DecodeStatusWord`**. |
| **`hf-vortex-driver` `VortexProtocol.hpp`** | **Canonical CiA 301 / 402 index constants** (`hf::vortex::kIdx*`) for firmware and tooling. |
| **`CANOpenBLDCThread`** | **CiA402 master/client** (default boot worker via **menuconfig**): NMT start; **periodic SDO poll** of **0x6041**, **0x6064**, **0x606C**, **0x6061**; **DS402 controlword ramp** to *Operation enabled*; **modes** PP / PV / profile torque; **profile** objects **0x6081–0x6084**; **homing** **0x6098 / 0x6099 / 0x609A** + start; EMCY / HB / expedited SDO RX; best-effort TPDO-shaped RX. |
| **`VortexCanHostThread`** | Optional boot worker: **`VortexBaseCanTransport`** + **`VortexHostSession`** (NMT start, RX drain). Select under **`idf.py menuconfig` → HardFOC Vortex main (app) → CAN RX drain worker** instead of BLDC when you want the portable **hf-vortex-driver** path without the CiA402 state machine. **Mutually exclusive** with **`CANOpenBLDCThread`** on the same bus. |
| **`CanOpenHostServiceThread`** | Optional **RX drain + callback** pattern — **do not run two threads that both `Read()` the same `BaseCan`** unless the HAL provides fan-out or a shared queue. |

This stack path is a **CiA402-oriented CAN master / client**, **not** a certified **CANopen slave node** (no full object dictionary server, no PDO mapper, no CANopenNode).

### CiA402 master-client coverage (summary)

| DS402 area | In firmware now | Notes |
|-------------|------------------|--------|
| NMT start remote node | Yes | `setupCANOpenCommunication()` |
| Statusword decode → drive state | Yes | `DecodeStatusWord` + SDO **0x6041** |
| Controlword ramp Shutdown → Switch on → Enable op | Yes | `EnableMotor()` + `handleStateMachine()` (rate-limited SDOs) |
| Fault reset | Yes | `ResetFault()` |
| Modes of operation **6060** | Yes | `writeModeOfOperation()` |
| Profile position **607A** + **new set-point** bit | Yes | `SetPositionMode()` |
| Profile velocity **60FF** | Yes | `SetVelocityMode()` |
| Profile torque **6071** | Yes | `SetTorqueMode()` |
| Profile limits **6081–6085** | Partial | `SetProfileParameters` + per-mode writers; **607F** max profile velocity optional per drive |
| Homing **6098–609A** | Yes | Default method **18** (override per drive OD) |
| RPDO decode (typical layout) | Best effort | Assumes fixed byte layout; real drives map PDOs in the EDS |
| **6061** display | Yes | Polled on SDO round-robin |
| **6077** torque actual | Yes | Parsed when SDO response index matches |
| Vendor / TMC9660 extras over CAN | Constants only | **`VortexProtocol`** `0x2100:*` placeholders — map in drive firmware OD if you expose SPI bridge / FOC status |
| **VortexHostSession** + **VortexBaseCanTransport** | Boot via **`VortexCanHostThread`** (menuconfig) or custom wiring | Same wire bytes as `Build*` helpers; **`VortexCanHostThread`** is the reference integration in `main` |

## Single RX consumer rule

**`BaseCan::ReceiveMessage`** delivers each frame to **one** reader. If **`CANOpenBLDCThread`** and **`CanOpenHostServiceThread`** both run `Read()` on the same bus, frames will be split unpredictably.

**Options:**

1. **Default:** **`idf.py menuconfig` → HardFOC Vortex main (app) → CAN RX drain worker** is **`CANOpenBLDCThread`**; only that worker (or the alternative **`VortexCanHostThread`**, or **none**) owns RX. Keep **`CanOpenHostServiceThread`** stopped unless it replaces the boot worker.
2. **Future:** One **`CanOpenIoThread`** in the HAL or app that pushes frames into a **FreeRTOS queue**, with BLDC and monitor tasks as consumers.
3. **Slave / full node (optional):** Enable **`CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE`** and initialize **`hal/hf-hal-vortex-v1/third_party/CANopenNode`**; replace **`CO_driver_blank`** with a **`CO_driver`** on **`BaseCan`** and a **single** stack RX path (do not parallel **`CANOpenBLDCThread`** `Read()` on the same **`BaseCan`**).

## Object dictionary indices

Application and threads should use **`hf::vortex::kIdx*`** from **`vortex/VortexProtocol.hpp`**.  
For legacy call sites, **`CanOpen::ObjectDictionary::*`** in **`EnhancedCANOpenUtils.h`** are **aliases** to the same constants.

## NMT command alignment

**`CanOpen::NmtCommand`** and **`hf::vortex::NmtCommand`** use the same numeric CiA 301 values; prefer **`CanOpen::BuildNmt`** in firmware to stay aligned with **`hf-utils-canopen`** tests.

## Related docs

- Repository: **`docs/canopen-and-motor-path.md`**
- **Slave product plan (Vortex = node, Flux/PC = masters):** **`docs/planning/canopen-vortex-slave-od-plan.md`**
- HAL: **`COMM_CHANNELS_MANAGER_README.md`** (bus accessors)
- **CANopenNode slave (optional):** `hal/hf-hal-vortex-v1/third_party/README.md` + HAL **`Kconfig`** — submodule under HAL, compiled via hf-core CMake (not `components/canopennode`)
