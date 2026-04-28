# `main/` component — documentation

The `main/` directory is the **ESP-IDF application component**: one binary, one `app_main`, and a small layered layout (system orchestration, supervisor on the IDF main task, node worker threads, utilities).

## Reading order

1. **[overview.md](overview.md)** — What each folder does and how it maps to FreeRTOS tasks.
2. **[boot-and-orchestration.md](boot-and-orchestration.md)** — Cold boot from `app_main` through `VortexSystemThread` and the init table.
3. **[cmake-component.md](cmake-component.md)** — How the component is registered for CMake / IDF, dependencies, and compile options.
4. **[supervisor-loop.md](supervisor-loop.md)** — Post-boot behavior on the main task: health checks and periodic demos.
5. **[workers-and-utils.md](workers-and-utils.md)** — WS2812 test thread, CANopen BLDC thread, host service thread, and **`CanOpenBaseCanLink`** (hf-core `handlers/common/canopen/`).

## Source map (quick)

| Path | Role |
|------|------|
| `app_main.cpp` | IDF entry: logger, start orchestrator, wait for boot, run supervisor |
| `apps/system/sub_threads/` | `VortexSystemThread` — boot table, periodic tick |
| `apps/system/api/` | `VortexSystemStateMachine` — coarse lifecycle flags |
| `apps/supervisor/` | `RunVortexSupervisorLoop` — demos + health on main task |
| `apps/node/sub_threads/` | Device-oriented `BaseThread` workers |
| `utils/` | App-only helpers (e.g. CANopen transport on `BaseCan`) |

For the **Vortex singleton API** (GPIO, ADC, motors, …), see the HAL: `hal/hf-hal-vortex-v1/lib/api/`.
