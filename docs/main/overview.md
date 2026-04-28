# `main/` overview

## Design intent

The firmware follows a **Flux-style** split:

- **`app_main`** stays thin: no large control loops inline.
- **One orchestrator thread** (`VortexSystemThread`) owns ordered bring-up (HAL, then workers) and a light periodic `Step()`.
- **Product logic and demos** run on the **ESP-IDF main task** after orchestrator `Setup()` completes (`RunVortexSupervisorLoop`).

There is **no `middleware/`** layer; cross-cutting concerns live in the HAL (managers, handlers) or in small app helpers under `utils/`.

## Directory tree

```
main/
├── app_main.cpp
├── CMakeLists.txt
├── apps/
│   ├── system/
│   │   ├── api/
│   │   │   ├── VortexSystemStateMachine.h
│   │   │   ├── VortexSystemStateMachine.cpp
│   │   └── sub_threads/
│   │       ├── VortexSystemThread.h
│   │       └── VortexSystemThread.cpp
│   ├── supervisor/
│   │   ├── VortexSupervisor.h
│   │   └── VortexSupervisor.cpp
│   └── node/
│       ├── canopen/              # README + `VortexCanOpenApp.hpp` barrel (client stack)
│       └── sub_threads/
│           ├── ws2812/
│           ├── canopen_bldc/
│           └── canopen_host/
└── utils/
    └── vortex_demos.h          # Declarations only (optional / legacy demos)
```

## Responsibility matrix

| Area | Responsibility |
|------|----------------|
| **`app_main.cpp`** | Logger init; start `VortexSystemThread`; poll until setup complete; verify `BootSucceeded()`; enter supervisor loop. On fatal errors, log and `esp_restart()`. |
| **`apps/system/`** | **State machine**: `Boot` → `HalInit` → `ThreadsUp` → `Ready` / `Fault` (updated by init steps and teardown). **Orchestrator**: runs the boot table, owns `WS2812TestThread` and `CANOpenBLDCThread` instance. |
| **`apps/supervisor/`** | Post-boot **blocking loop** on the main task: `PerformHealthCheck`, status logs, GPIO/ADC/LED/temp/motor/IMU/encoder demonstrations, optional CANopen motion snippets. |
| **`apps/node/sub_threads/`** | **Workers** derived from `BaseThread` (HAL `hf-utils-rtos-wrap`): WS2812 strip test, CANopen CiA402-style BLDC control, optional host-side frame drain helper. |
| **`utils/`** | Headers and small glue **not** registered as separate IDF components (e.g. demo declarations). CANopen **`BaseCan`** framing lives in **hf-core** `handlers/common/canopen/`. |

## Tasks vs files (runtime)

| FreeRTOS task | Typical owner |
|----------------|---------------|
| IDF **main** task | `app_main` → then `RunVortexSupervisorLoop` (blocking `while(true)`). |
| **VortexSys** (`BaseThread`) | `VortexSystemThread`: runs `Setup()` once, then `Step()` every ~100 ms. |
| **WS2812** worker | `WS2812TestThread` (started from boot table). |
| **CANopen BLDC** worker | `CANOpenBLDCThread` (started from boot table when CAN is available). |

## Namespaces and types

- **`vortex_app::VortexSystemThread`** — orchestrator singleton (`GetInstance()`).
- **`vortex_app::VortexSystemStateMachine`** — global boot/runtime state; used for logging and transitions during init.
- **`Vortex`** — HAL singleton (`Vortex::GetInstance()`); see HAL docs.

## Where to change behavior

| Goal | Likely file |
|------|-------------|
| Add a boot step (new driver thread) | `VortexSystemThread.cpp` — extend `kInitSequence[]` and add `Setup*` / `Teardown*` functions. |
| Change demo cadence or remove demos | `VortexSupervisor.cpp` — loop constants and branches. |
| Adjust CANopen node id or creation policy | `VortexSystemThread.cpp` (`kDefaultMotorNodeId`, `SetupCanOpenBldc`). |
| New app-only CANopen / protocol helper | `utils/` + wire from relevant thread. |
