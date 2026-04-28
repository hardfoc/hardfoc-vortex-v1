# HardFOC Vortex V1 — documentation index

This folder holds **project-level** documentation for the firmware repository. The repository root intentionally keeps a **single** markdown overview: [README.md](../README.md).

## Firmware application (`main/`)

The ESP-IDF `main` component is documented end-to-end here:

| Document | Contents |
|----------|----------|
| [main/README.md](main/README.md) | Navigation and reading order |
| [main/overview.md](main/overview.md) | Directory tree, roles, naming |
| [main/boot-and-orchestration.md](main/boot-and-orchestration.md) | `app_main`, `VortexSystemThread`, boot table, state machine |
| [main/cmake-component.md](main/cmake-component.md) | `main/CMakeLists.txt`: sources, `REQUIRES`, include dirs, flags |
| [main/supervisor-loop.md](main/supervisor-loop.md) | `RunVortexSupervisorLoop`, demos, timing |
| [main/workers-and-utils.md](main/workers-and-utils.md) | Node threads, CANopen link, optional demos header |

## Build and environment

| Document | Contents |
|----------|----------|
| [build-guide.md](build-guide.md) | ESP-IDF setup, clone, build, flash, troubleshooting |

## CANopen stack and host testing

| Document / path | Contents |
|-----------------|----------|
| [canopen-and-motor-path.md](canopen-and-motor-path.md) | `hf-utils-canopen` vs CiA301/DS402, `BaseCan` transport, TMC9660 vs CAN path |
| [helpers/usb_can/README.md](helpers/usb_can/README.md) | Python + USB-CAN scripts (`ds402_host_test.py`, offline frame tests) |

## HAL (submodule)

Hardware abstraction, handlers, and deep API docs live in the **HAL submodule** (not duplicated here):

- [HAL documentation index](../hal/hf-hal-vortex-v1/DOCUMENTATION_INDEX.md)
- [Vortex API overview](../hal/hf-hal-vortex-v1/lib/api/README.md)
- [Layered architecture (handbook)](../hal/hf-hal-vortex-v1/docs/hf-development-handbook/process/layered-architecture.md)
