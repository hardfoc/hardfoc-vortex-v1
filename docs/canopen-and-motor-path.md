# CANopen path on Vortex: hf-utils-canopen, transport, TMC9660, and first-pass DS402

This note ties together what exists in this repository today, how complete it is versus **CANopen CiA 301 / DS402**, where **TMC9660** fits, and how a **PC + USB-CAN** can exercise the same frames the firmware sends.

## 1. `hf-utils-canopen` (HAL: `lib/core/hf-core-utils/hf-utils-canopen`)

**Role:** Small, dependency-free **frame builders** plus a shared **`CanOpen::CanFrame`** struct. It is **not** a full CANopen stack (no object dictionary, no SDO client state machine, no NMT slave, no SYNC/PDO scheduler).

**Supported well (subset of CiA 301 / DSP402):**

| Area | Support |
|------|---------|
| **NMT master** | `BuildNmt(nodeId, cmd)` — COB-ID `0x000`, 2-byte payload (command + node). Same pattern as `CANOpenBLDCThread::setupCANOpenCommunication()` (manual bytes `0x01`, node id). |
| **SDO client → server (expedited)** | `BuildSdoDownload` for 1 / 2 / 4 bytes (`0x2F` / `0x2B` / `0x23`). COB-ID `0x600 + nodeId`. |
| **SDO upload request** | `BuildSdoUpload` — byte0 `0x40`. |
| **Heartbeat producer frame** | `BuildHeartbeat` — COB-ID `0x700 + nodeId` (builder only; no stack to consume slaves). |

**DS402-oriented helpers** (`CanOpenMotorUtils.*`): controlword `0x6040`, modes `0x6060`, target velocity `0x60FF`, target position `0x607A`, statusword request `0x6041`, actual position/velocity requests, torque, profile accel/decel, max profile velocity — all as **expedited SDO** frames.

**Gaps vs “full” CANopen:**

- No **segmented / block** SDO transfer.
- No **PDO** map configuration or cyclic PDO exchange (only what you encode by hand).
- No **LSS**, **EMCY** consumer, **SYNC** producer, **NMT** state parsing on the host beyond what your app code does.
- **Drive state** enums in headers are simplified masks vs full CiA402 **statusword** bit decoding (your thread should mask `0x6F` or full table per drive vendor).

For many BLDC drives speaking **CiA402 over CANopen**, **NMT + SDO** is enough for bring-up; production motion often adds **Rx/Tx PDOs** for rate.

## 2. Transport: `HfUtilsCanOpenTransport` + `BaseCan` (not CRTP)

**Location:** `hal/hf-hal-vortex-v1/lib/core/handlers/common/canopen/` — **`HfUtilsCanOpenTransport.hpp`**, **`HfUtilsCanOpenBridge.hpp`**, **`CanOpenBaseCanLink.hpp`** (header-only **hf-core** layer; **`hf-utils-canopen`** stays free of **`BaseCan`**).

**Mechanism:**

- `HfUtilsCanOpenTransport` holds a reference to **`BaseCan&`** (virtual interface: `SendMessage` / `ReceiveMessage` / … on `hf_can_message_t`).
- `send()` / `receive()` convert **`CanOpen::CanFrame` ↔ `hf_can_message_t`** and call **`BaseCan`**.
- **`CanOpenBaseCanLink`** wraps `HfUtilsCanOpenTransport` with **`Open`/`Close`/`Write`/`Read`** for **`CANOpenBLDCThread`**.

So the **upper layer** (`main`, managers) gets CANopen bytes on the bus by:

1. Obtaining the board **`BaseCan*`** from **`CommChannelsManager::GetInstance().GetCanOpenBus()`** (after HAL init; alias of **`GetCanBus()`** on Vortex V1).
2. Passing that reference into **`HfUtilsCanOpenTransport`** or **`CanOpenBaseCanLink`**.
3. Building frames with **`CanOpen::Build*`** helpers and writing them on the link.

There is **no CRTP** on the CAN path; CRTP appears in **TMC9660’s TMCL** stack (**SPI/UART** in `hf-tmc9660-driver`), which is a **different protocol** from CANopen.

## 3. TMC9660 driver vs “Vortex commands” on CAN

**`hf-tmc9660-driver`** is built around **TMCL / register** access over **SPI or UART** (`tmc9660_comm_interface.hpp` — CRTP transports there).

**CANopen on the Vortex board** is expected to talk to a **motor drive that speaks CiA402 on CAN** (external BLDC inverter, or a bridge node), **not** the raw TMCL packet format on the TWAI bus. The TMC9660 on the Vortex PCB is still the primary FOC device over SPI/UART from the ESP32; **CANopen BLDC** in `main` is a **parallel** integration path for drives that expose a CANopen interface.

The **`hf-vortex-driver`** package (git submodule under **`hf-core-drivers/external/hf-vortex-driver`**, remote `https://github.com/N3b3x/hf-vortex-driver.git` until mirrored under `hardfoc`) defines **`hf::vortex::VortexHostSession`**, **`VortexCanTransport`**, and **`VortexProtocol`** for Vortex-oriented CAN framing; wire **`VortexCanTransport::send`** to **`BaseCan`** (same pattern as **`HfUtilsCanOpenTransport`**) in application or manager code. On **Vortex HAL** builds, **`HF_CORE_ENABLE_VORTEX_DRIVER`** is set **ON** so **`VortexHostSession.cpp`** is compiled into the HAL component; on **Flux HAL** it defaults **OFF** (enable in `CMakeLists.txt` if you need the protocol on Flux).

## 4. What `CANOpenBLDCThread` actually does today

- Opens **`CanOpenBaseCanLink`** → initializes **`BaseCan`**.
- Sends **NMT Start** via **`CanOpen::BuildNmt`** to the configured **node id** (default **1** in `VortexSystemThread.cpp`).
- Uses **`hf::vortex::VortexProtocol`** indices for SDO access (e.g. **0x6040** / **0x6041**).
- Parses RX with **`CanOpenExtras`** (**`ParseSdoResponse`**, **`ParseEmergency`**, **`ParseHeartbeat`**, **`DecodeStatusWord`**).
- Runs a **10 ms** `Step()`: drains RX with non-blocking read, state machine, status updates, periodic SDO-style traffic (see `CANOpenBLDCThread.cpp` for the exact sequence).

App-layer layout and the **single-RX-consumer** rule for **`CanOpenHostServiceThread`**: **`main/apps/node/canopen/README.md`**.

Treat the thread as a **starting point** for DS402 bring-up, not a certified DS402 master implementation.

### CANopenNode (CiA 301 slave, optional)

Upstream **[CANopenNode](https://github.com/CANopenNode/CANopenNode)** is built **into the Vortex HAL component** when **`CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE`** is enabled (**`idf.py menuconfig` → HardFOC Vortex HAL**). Sources and **`example/OD.*`** / **`CO_driver_blank`** are wired from **`hal/hf-hal-vortex-v1/lib/core/cmake/hf_core_build_settings.cmake`** using the vendor tree **`hal/hf-hal-vortex-v1/third_party/CANopenNode`** (git submodule — see **`hal/hf-hal-vortex-v1/third_party/README.md`**). Replacing the blank driver with a **TWAI** / **`BaseCan`** port should keep a **single RX owner** on that bus (see **`main/apps/node/canopen/README.md`**).

## 5. Typical BLDC CiA402 bring-up (first pass)

Order is vendor-dependent; a common pattern:

1. **NMT** `Start remote node` (or `Reset communication` then pre-op / start as required).
2. **SDO** read **statusword** `0x6041` until drive leaves fault / not ready (may need **fault reset** pulse on **controlword** `0x6040`).
3. **SDO** set **modes of operation** `0x6060` (e.g. profile velocity `3`, profile position `1`, CSP `8`, …).
4. **SDO** write targets (`0x60FF`, `0x607A`, …) and **controlword** sequence **Shutdown → Switch on → Enable operation** per DS402.

Always cross-check **object dictionary** and **scaling** (rpm vs counts/s, position units) on the specific drive / TMC CANopen documentation.

## 6. PC-side testing (USB-CAN)

See **[helpers/usb_can/README.md](helpers/usb_can/README.md)** for:

- **`requirements.txt`** (`python-can`).
- **`ds402_host_test.py`** — encodes the same COB-IDs and expedited SDO bytes as `hf-utils-canopen`, optional NMT + status/control loop.
- **`test_frame_encoding.py`** — offline asserts (no adapter).

Use the same **bitrate** (e.g. **500 kbit/s**) and **node id** as the Vortex `CANOpenBLDCThread` / drive. When the ESP32 is on the bus, you can use the script as a **second master** only for monitoring or coordinated tests — two masters controlling the same node require care.

## 7. Relation to “Flux alone” testing

With **only a USB-CAN adapter on a PC** (no ESP32), you can:

- Talk to a **commercial CANopen servo / BLDC** that shares the bus, or
- Pair with a **CAN bus simulator** / second interface in loopback wiring.

With **ESP32 running Flux/Vortex firmware** and **USB-CAN on the PC**, you can:

- **Sniff** traffic, or
- **Send** NMT/SDO frames the firmware also understands, for **integration tests** (avoid conflicting controlword writes while the motor thread is enabled unless that is intentional).
