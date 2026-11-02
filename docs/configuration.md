# Configuration

Every switch that changes what this firmware does: the menuconfig options, the committed `sdkconfig` values and the
compile definitions.

## menuconfig

`idf.py menuconfig` shows two menus that belong to Vortex.

**HardFOC Vortex main (app)** (`main/Kconfig.projbuild`):

| Option | Values | Default | Effect |
| --- | --- | --- | --- |
| `VORTEX_APP_CAN_BOOT_WORKER` (choice) | `VORTEX_APP_CAN_BOOT_CANOPEN_SLAVE`, `VORTEX_APP_CAN_BOOT_NONE` | `NONE` | Which task drains the CANopen bus at boot. **CANopenNode slave** also compiles `CanopenSlaveRuntime.cpp` |
| `VORTEX_CANOPEN_NODE_ID` | 1 to 127 | 1 | Node ID passed to `CO_CANopenInit()` and `CO_CANopenInitPDO()` |

**HardFOC Vortex HAL** (in the HAL submodule):

| Option | Default | Effect |
| --- | --- | --- |
| `HARDFOC_HAL_CANOPENNODE_SLAVE` | off | Compiles CANopenNode into the HAL; the app's CANopenNode slave choice depends on it |

To try the slave: enable `HARDFOC_HAL_CANOPENNODE_SLAVE`, then pick **CANopenNode slave** under
**CAN RX drain worker at boot**, and set the node ID. Read the [CANopen](canopen.md#what-runs-today) limits first.

## Committed sdkconfig

The repository commits a full `sdkconfig`. The values the firmware depends on:

| Setting | Value | Why it matters |
| --- | --- | --- |
| `CONFIG_IDF_TARGET` | `esp32c6` | The only target |
| `CONFIG_FREERTOS_UNICORE` | `y` | One core for every task |
| `CONFIG_FREERTOS_HZ` | 100 | 10 ms tick; see [Threads](threads.md) |
| `CONFIG_ESP_MAIN_TASK_STACK_SIZE` | 3584 | Stack of the supervisor loop |
| `CONFIG_ESP_TASK_WDT_TIMEOUT_S` | 5 | Task watchdog, idle task checked |
| `CONFIG_ESP_CONSOLE_UART_BAUDRATE` | 115200 | Console on UART0, USB Serial/JTAG secondary |
| `CONFIG_LOG_DEFAULT_LEVEL` | 3 (info) | |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `2MB` | |
| `CONFIG_PARTITION_TABLE_SINGLE_APP` | `y` | No OTA slot; see [Boot and updates](boot-and-ota.md#flash-layout) |
| `CONFIG_COMPILER_OPTIMIZATION_DEBUG` | `y` | `-Og` for ESP-IDF; `main` adds `-O2` |

## Compile definitions

| Define | Set in | Scope |
| --- | --- | --- |
| `HARDFOC_VORTEX_V1=1`, `HARDFOC_VORTEX_API=1` | root `CMakeLists.txt` | every component |
| `ESP_IDF_VERSION_MAJOR=5`, `ESP_IDF_VERSION_MINOR=5` | root `CMakeLists.txt` | every component |
| `HARDFOC_MAIN_APPLICATION=1`, `HARDFOC_VORTEX_API_MAIN=1` | `main/CMakeLists.txt` | `main` only |
| `LOG_LOCAL_LEVEL=ESP_LOG_INFO` | `main/CMakeLists.txt` | `main` only |

`main` builds as C++17 with `-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -O2 -g`. The
project version is `2.0.0` (`PROJECT_VER` in the root `CMakeLists.txt`).

## Adding a source file to main

1. Put it under `main/apps/` or `main/utils/`.
2. Add it to `MAIN_SRCS` in `main/CMakeLists.txt` (inside an `if(CONFIG_...)` block if a menuconfig option gates it).
3. Add a new header directory to `INCLUDE_DIRS`.
4. Add a new ESP-IDF component to `REQUIRES` or `PRIV_REQUIRES`.

Next: [Troubleshooting](troubleshooting.md).
