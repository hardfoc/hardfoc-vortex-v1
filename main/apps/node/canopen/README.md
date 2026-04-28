# Vortex `main` CANopen application layer

## What is implemented today

| Piece | Role |
|--------|------|
| **`CommChannelsManager::GetCanOpenBus()`** | Single owned **`BaseCan*`** after HAL init (TWAI). |
| **`CanOpenBaseCanLink` / `HfUtilsCanOpenTransport`** (hf-core) | **`CanOpen::CanFrame`** ↔ **`hf_can_message_t`** on **`BaseCan`**. |
| **`hf-utils-canopen`** | Frame builders (`BuildNmt`, `BuildSdoDownload`, `BuildSdoUpload`, motor helpers). |
| **`CanOpenExtras.h`** | **Inverse** helpers: **`ParseSdoResponse`**, **`ParseEmergency`**, **`ParseHeartbeat`**, **`DecodeStatusWord`**. |
| **`hf-vortex-driver` `VortexProtocol.hpp`** | **Canonical CiA 301 / 402 index constants** (`hf::vortex::kIdx*`) for firmware and tooling. |
| **`CANOpenBLDCThread`** | **Motor client**: NMT start, periodic statusword SDO upload, controlword SDO download, EMCY / HB / SDO RX decode, optional PDO-shaped parsing (best-effort). |
| **`CanOpenHostServiceThread`** | Optional **RX drain + callback** pattern — **do not run two threads that both `Read()` the same `BaseCan`** unless the HAL provides fan-out or a shared queue. |

This stack path is a **CiA402-oriented CAN master / client**, **not** a certified **CANopen slave node** (no full object dictionary server, no PDO mapper, no CANopenNode).

## Single RX consumer rule

**`BaseCan::ReceiveMessage`** delivers each frame to **one** reader. If **`CANOpenBLDCThread`** and **`CanOpenHostServiceThread`** both run `Read()` on the same bus, frames will be split unpredictably.

**Options:**

1. **Default (current):** Only **`CANOpenBLDCThread`** owns RX for the motor node; keep **`CanOpenHostServiceThread`** stopped or use it **instead of** the BLDC thread in bring-up builds.
2. **Future:** One **`CanOpenIoThread`** in the HAL or app that pushes frames into a **FreeRTOS queue**, with BLDC and monitor tasks as consumers.
3. **Slave / full node (optional):** Enable **`CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE`** and initialize **`hal/hf-hal-vortex-v1/third_party/CANopenNode`**; replace **`CO_driver_blank`** with a **`CO_driver`** on **`BaseCan`** and a **single** stack RX path (do not parallel **`CANOpenBLDCThread`** `Read()` on the same **`BaseCan`**).

## Object dictionary indices

Application and threads should use **`hf::vortex::kIdx*`** from **`vortex/VortexProtocol.hpp`**.  
For legacy call sites, **`CanOpen::ObjectDictionary::*`** in **`EnhancedCANOpenUtils.h`** are **aliases** to the same constants.

## NMT command alignment

**`CanOpen::NmtCommand`** and **`hf::vortex::NmtCommand`** use the same numeric CiA 301 values; prefer **`CanOpen::BuildNmt`** in firmware to stay aligned with **`hf-utils-canopen`** tests.

## Related docs

- Repository: **`docs/canopen-and-motor-path.md`**
- HAL: **`COMM_CHANNELS_MANAGER_README.md`** (bus accessors)
- **CANopenNode slave (optional):** `hal/hf-hal-vortex-v1/third_party/README.md` + HAL **`Kconfig`** — submodule under HAL, compiled via hf-core CMake (not `components/canopennode`)
