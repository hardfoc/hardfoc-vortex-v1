# Node workers and `utils/`

## `WS2812TestThread`

- **File**: `apps/node/sub_threads/ws2812/WS2812TestThread.{h,cpp}`
- **Base**: `BaseThread` (RTOS abstraction from HAL core utils).
- **Role**: Exercise the **`hf-ws2812-rmt-driver`** with **`WS2812Strip`**, **`WS2812Animator`**, cycling **`WS2812Animator::Effect`** entries on a timer.
- **Lifecycle**: Started from **`VortexSystemThread`** boot step `ws2812` (`EnsureInitialized` + `Start`); stopped in `TeardownWs2812` with **`Stop()`**.

Default internal stack size in header: **4096** bytes (see class definition).

## `CANOpenBLDCThread`

- **File**: `apps/node/sub_threads/canopen_bldc/CANOpenBLDCThread.{h,cpp}` (+ **`EnhancedCANOpenUtils`**).
- **Base**: `BaseThread`.
- **Construction**: **`CANOpenBLDCThread(uint8_t nodeId, BaseCan& canBus)`** — uses HAL **`BaseCan`** from **`CommChannelsManager::GetCanOpenBus()`** (same TWAI instance as **`GetCanBus()`**).
- **Transport**: Uses **`CanOpenBaseCanLink`** (see below) internally for **`Open` / `Write` / `Read` / `Close`** style access to **`HfUtilsCanOpenTransport`**.
- **Public API** (excerpt): **`EnableMotor`**, **`DisableMotor`**, **`SetVelocityMode`**, **`SetPositionMode`**, **`GetMotorStatus`**, **`IsThreadRunning`**, CiA402-related state helpers.

**Default node ID** in the boot table: **1** (`kDefaultMotorNodeId` in `VortexSystemThread.cpp`).

If CAN is missing or init fails, the orchestrator **skips** this worker (`recoverable: true`) and continues boot.

## `CanOpenHostServiceThread`

- **File**: `apps/node/sub_threads/canopen_host/CanOpenHostServiceThread.{h,cpp}`
- **Role**: Each **`Step()`**, drain up to **`max_per_step_`** frames from **`BaseCan`** via **`HfUtilsCanOpenTransport`** and invoke an optional **`std::function<void(const CanOpen::CanFrame&)>`** callback.
- **Use case**: Bring-up logging, SDO/TPDO sniffing, bridging — **not** a full **CANopenNode** stack.

**Important**: This translation unit is **compiled** as part of `main` (`CMakeLists.txt` lists the `.cpp`), but the **current boot table does not instantiate or start** `CanOpenHostServiceThread`. To use it, construct it with a **`BaseCan&`**, set **`setFrameHandler`**, then **`EnsureInitialized` / `Start`** from an appropriate init step (and add teardown).

## `CanOpenBaseCanLink` (`lib/core/handlers/common/canopen/CanOpenBaseCanLink.hpp` in the HAL’s **hf-core** tree)

Small **facade** over **`HfUtilsCanOpenTransport`**:

- **`Open()`** → `transport_.can().EnsureInitialized()`
- **`Close()`** → `EnsureDeinitialized()`
- **`Write(CanFrame)`** / **`Read(CanFrame, timeoutMs)`** → send/receive

Used by **`CANOpenBLDCThread`** so motor logic stays on **`CanOpen::CanFrame`** while the bus remains the HAL **`BaseCan`** instance.

## `vortex_demos.h` (`utils/`)

Header-only **declarations** for a legacy-style “comprehensive demo” API (`RunComprehensiveVortexDemo`, per-subsystem `Demo*` functions). **No `.cpp` in this repository implements these symbols** in the current `SRCS` list, so linking is unaffected unless you add an implementation.

Prefer extending **`VortexSupervisor.cpp`** or dedicated modules if you add new demos.

## Thread interaction summary

| Component | Started by | Observed by |
|-----------|------------|-------------|
| `VortexSystemThread` | `app_main` | `app_main` (boot wait), supervisor (`MonitorWorkerThreads`) |
| `WS2812TestThread` | Boot table | Supervisor status logs |
| `CANOpenBLDCThread` | Boot table (if CAN OK) | Supervisor status + motor demo branch |
| `CanOpenHostServiceThread` | *Not started by default* | N/A until you wire it |
