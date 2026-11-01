# Vortex `main` CANopen application layer

## What is implemented today

| Piece | Role |
|--------|------|
| **`CommChannelsManager::GetCanOpenBus()`** | Single owned **`BaseCan*`** after HAL init (TWAI). |
| **`CanOpenBaseCanLink` / `HfUtilsCanOpenTransport`** (hf-core) | **`CanOpen::CanFrame`** ↔ **`hf_can_message_t`** on **`BaseCan`**. |
| **`hf-utils-canopen`** | Frame builders (`BuildNmt`, `BuildSdoDownload`, `BuildSdoUpload`, motor helpers). |
| **`CanOpenExtras.h`** | **Inverse** helpers: **`ParseSdoResponse`**, **`ParseEmergency`**, **`ParseHeartbeat`**, **`DecodeStatusWord`**. |
| **`hf-vortex-driver` `VortexProtocol.hpp`** | **Canonical CiA 301 / 402 index constants** (`hf::vortex::kIdx*`) for firmware and tooling. |

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
| **VortexHostSession** | Host-side tooling in `hf-vortex-driver`; no firmware boot worker uses it yet | The CANopen node redesign replaces the boot workers |

## Single RX consumer rule

**`BaseCan::ReceiveMessage`** delivers each frame to **one** reader. If **`CANOpenBLDCThread`** and **`CanOpenHostServiceThread`** both run `Read()` on the same bus, frames will be split unpredictably.

**Options:**

1. **Default:** **`idf.py menuconfig` → HardFOC Vortex main (app) → CAN RX drain worker** is **`CANOpenBLDCThread`**; only that worker (or the CANopenNode slave, or **none**) owns RX. Keep **`CanOpenHostServiceThread`** stopped unless it replaces the boot worker.
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
