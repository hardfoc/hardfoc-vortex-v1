# `main/CMakeLists.txt` — ESP-IDF component

This file registers the **`main`** component: sources, dependencies, public include directories, and compile flags.

## `idf_component_register`

### `SRCS` (explicit list)

| Source | Purpose |
|--------|---------|
| `app_main.cpp` | IDF entry |
| `apps/supervisor/VortexSupervisor.cpp` | Supervisor loop and demos |
| `apps/system/api/VortexSystemStateMachine.cpp` | Lifecycle state storage |
| `apps/system/sub_threads/VortexSystemThread.cpp` | Orchestrator + boot table |
| `apps/node/sub_threads/ws2812/WS2812TestThread.cpp` | WS2812 test animations |
| `apps/node/sub_threads/canopen_bldc/CANOpenBLDCThread.cpp` | CANopen motor worker |
| `apps/node/sub_threads/canopen_bldc/EnhancedCANOpenUtils.cpp` | CANopen helpers |
| `apps/node/sub_threads/canopen_host/CanOpenHostServiceThread.cpp` | Host-side CAN drain thread (compiled; see [workers-and-utils.md](workers-and-utils.md)) |

### `REQUIRES` (public link dependencies)

Includes **`freertos`**, **`esp_system`**, **`esp_timer`**, several **`esp_driver_*`** components, **`log`**, **`hf-hal-vortex-v1`**, **`hf-ws2812-rmt-driver`**.

Anything listed here is part of the component’s **public** dependency graph for downstream components (normally only relevant if other components depend on `main`, which is uncommon).

### `PRIV_REQUIRES`

Private IDF pieces: **`spi_flash`**, **`nvs_flash`**, **`esp_partition`**, **`hal`**, **`soc`**, **`esp_hw_support`**, **`esp_rom`**, **`esp_netif`**, **`esp_event`**, **`esp_wifi`**, **`esp_pm`**, **`esp_ota`**, **`esp_http_client`**, **`esp_https_ota`**.

These satisfy symbols the HAL or IDF may pull in without advertising them as public API of `main`.

### `INCLUDE_DIRS`

The component exposes these directories to **all translation units in `main`** (and to dependents if any):

- `.` (root of `main/`)
- `apps/supervisor`
- `apps/system/api`
- `apps/system/sub_threads`
- `apps/node/sub_threads/ws2812`
- `apps/node/sub_threads/canopen_bldc`
- `apps/node/sub_threads/canopen_host`
- `utils`

Headers must live under one of these paths (or be reachable via includes from HAL) to avoid include path surprises.

## Feature and definition flags

```cmake
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
```

Compiler options (excerpt): **`-Wall`**, **`-Wextra`**, **`-Wno-unused-parameter`**, **`-Wno-missing-field-initializers`**, **`-O2`**, **`-g`**.

Preprocessor definitions on `main`:

| Define | Role |
|--------|------|
| `HARDFOC_MAIN_APPLICATION=1` | Identifies this image as the main application |
| `HARDFOC_VORTEX_API_MAIN=1` | API / HAL gates for main-app code paths |
| `ESP_IDF_VERSION_MAJOR=5` / `MINOR=5` | Version alignment with project expectations |
| `LOG_LOCAL_LEVEL=ESP_LOG_INFO` | Default log verbosity for this component |

## Adding a new `.cpp` file

1. Place the file under the appropriate `apps/` or `utils/` subtree.
2. Add its path to **`SRCS`**.
3. If it includes headers from a **new** directory, add that directory to **`INCLUDE_DIRS`**.
4. If it needs a new IDF component (e.g. `esp_driver_twai`), add **`esp_driver_twai`** to **`REQUIRES`** or **`PRIV_REQUIRES`** as appropriate.

## Relation to root `CMakeLists.txt`

The **repository root** `CMakeLists.txt` sets **`EXTRA_COMPONENT_DIRS`** so IDF finds **`hf-hal-vortex-v1`** and **`hf-ws2812-rmt-driver`**. The **`main`** component does not repeat those paths; it only **requires** those component names by string.
