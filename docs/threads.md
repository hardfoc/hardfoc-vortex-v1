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
| `co_svc` | boot step `can_worker`, only with the CANopen device selected | 6144, heap | 17 | woken by received frames; otherwise CANopenNode's timer-next, at least one tick | `main/apps/node/canopen/CoSvcFreeRtos.cpp` |

What the HAL managers lock, and from which tasks they may be called, is on the
[HAL threading page](https://hardfoc.github.io/hf-hal-vortex-v1/latest/architecture/threading/).

!!! note "co_svc at a 100 Hz tick"
    `co_svc` sleeps until a frame arrives or CANopenNode's next deadline, and never less than one tick. At the
    current 100 Hz tick a 1 ms deadline therefore becomes 10 ms; event timers and heartbeats below that are
    coarser than configured. The [CANopen node design](design/canopen-node.md#44-threads-priorities-and-timing-on-the-single-core-c6)
    moves to `CONFIG_FREERTOS_HZ=1000`. The default build (CAN worker **None**) does not start this task.

## Who talks to whom

| Producer | Consumer | Through |
| --- | --- | --- |
| `app_main` | `VortexSys` | `IsSetupComplete()` and `BootSucceeded()`, polled by `TestLogicWithTimeout()` for up to 120 s |
| Supervisor | `LedDemo` | `IsThreadRunning()`, logged every 10 s |
| Supervisor, `LedDemo` | status LED | `LedManager`: both write it (see [Troubleshooting](troubleshooting.md#the-status-led-flickers-between-patterns)) |
| TWAI RX interrupt | `co_svc` | the CO driver's lock-free RX ring and a task notification |
| `co_svc` | CAN bus | `EspCan::TrySendMessage()` (never blocks); the only RX consumer |

## Watchdog

The task watchdog is on with a 5 s timeout and watches the idle task (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=5`,
`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y`). No firmware task subscribes to it.

Next: [Boot and updates](boot-and-ota.md).
