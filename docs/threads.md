# Threads

Every FreeRTOS task the firmware creates, with its stack, priority, period and owner, read from the sources.

The ESP32-C6 runs FreeRTOS on one core (`CONFIG_FREERTOS_UNICORE=y`) with a 100 Hz tick (`CONFIG_FREERTOS_HZ=100`),
so every delay is a multiple of 10 ms.

## Tasks

| Task | Created by | Stack (bytes) | Priority | Period | Source |
| --- | --- | --- | --- | --- | --- |
| ESP-IDF main | ESP-IDF | 3584 (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) | ESP-IDF default (1) | 100 ms supervisor tick | `main/app_main.cpp`, `main/apps/supervisor/VortexSupervisor.cpp` |
| `VortexSys` | `app_main` | 16384, static | 5 | `Step()` every 100 ms | `main/apps/system/sub_threads/VortexSystemThread.h` |
| `LedDemo` | boot step `ws2812` | 4096, static | 2 | `Step()` every 20 ms, next animation every 5 s | `main/apps/node/sub_threads/ws2812/WS2812TestThread.h` |
| `canopen_slave` | boot step `can_worker`, only with the CANopenNode slave selected | 8192, heap | 5 | loop with `vTaskDelay(pdMS_TO_TICKS(1))` | `main/apps/node/canopen/CanopenSlaveRuntime.cpp` |

What the HAL managers lock, and from which tasks they may be called, is on the
[HAL threading page](https://hardfoc.github.io/hf-hal-vortex-v1/latest/architecture/threading/).

!!! warning "The CANopen loop never sleeps"
    At a 100 Hz tick, `pdMS_TO_TICKS(1)` is zero ticks, so `vTaskDelay()` in the `canopen_slave` loop only yields
    to tasks of equal priority. Read from the code, not measured: with the CANopenNode slave selected, the
    priority 5 loop keeps `LedDemo`, the main task and the idle task off the CPU whenever `VortexSys` is
    blocked, and the task watchdog checks the idle task. The [CANopen node design](design/canopen-node.md#44-threads-priorities-and-timing-on-the-single-core-c6)
    replaces this loop with a timer-driven schedule and `CONFIG_FREERTOS_HZ=1000`. The default build (CAN worker
    **None**) does not start this task.

## Who talks to whom

| Producer | Consumer | Through |
| --- | --- | --- |
| `app_main` | `VortexSys` | `IsSetupComplete()` and `BootSucceeded()`, polled by `TestLogicWithTimeout()` for up to 120 s |
| Supervisor | `LedDemo` | `IsThreadRunning()`, logged every 10 s |
| Supervisor, `LedDemo` | status LED | `LedManager`: both write it (see [Troubleshooting](troubleshooting.md#the-status-led-flickers-between-patterns)) |
| `canopen_slave` | CAN bus | `BaseCan::SendMessage()` (20 ms timeout) and `ReceiveMessage()` (no wait); the only RX consumer |

## Watchdog

The task watchdog is on with a 5 s timeout and watches the idle task (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=5`,
`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y`). No firmware task subscribes to it.

Next: [Boot and updates](boot-and-ota.md).
