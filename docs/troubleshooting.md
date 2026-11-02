# Troubleshooting

Symptoms, their causes and the fix, starting with the cheapest check.

| Symptom | Cause | Fix |
| --- | --- | --- |
| CMake stops with `Vortex HAL not found at: .../hal/hf-hal-vortex-v1` | The HAL submodule is not checked out | Run the submodule commands in [Getting started](getting-started.md#steps) |
| Submodule update fails on `docs/hf-development-handbook` | That HAL submodule is private | `git -C hal/hf-hal-vortex-v1 config submodule.docs/hf-development-handbook.update none`, then update again |
| Build errors after switching ESP-IDF version | Stale `build/` or a shell from another ESP-IDF | `. $IDF_PATH/export.sh` for v5.5, then `idf.py fullclean build` |
| `Filename too long` or `$GIT_DIR too big` on Windows | Nested submodules under a long path | `git config --global core.longpaths true` and clone to a short path such as `C:\src`, or use WSL2 |
| `FATAL: VortexSystemThread boot sequence failed` | The HAL did not initialise | Scroll up to the first `VortexSys` error; the HAL logs which manager failed |
| `FATAL: ... Setup did not complete in time` | A boot step blocked for 120 s | Find the last `[n/3]` line; that step is stuck |
| `CAN bus unavailable (skip CAN worker)` | The CANopenNode slave is selected but the HAL has no CAN bus | Check the HAL CAN initialisation; boot continues without CANopen |
| Task watchdog resets with the CANopenNode slave selected | The `canopen_slave` loop never sleeps at a 100 Hz tick ([Threads](threads.md#tasks)) | Keep the CAN worker on **None** until the runtime is rebuilt |
| No CANopen frames answered | The slave runtime cannot receive on hardware yet ([CANopen](canopen.md#what-runs-today)) | Known limitation |
| `System health check failed!` every 5 s, LED on `STATUS_WARN` | A HAL component reports unhealthy | Read the system diagnostics printed at start-up: failed components and warnings are listed |

## The status LED flickers between patterns

Three writers share the WS2812: the `LedDemo` thread changes animation every 5 s, the supervisor toggles
`STATUS_OK` / `STATUS_BOOT` every 2 s and sets the health status every 5 s, and the LED demo every 20 s steps through
four status animations. The LED is not a reliable status indicator in this build; read the console.

## Read the log first

Every task logs with its own tag: `MAIN` (`app_main`), `VortexSys` (boot), `VortexMain` (supervisor), `LedDemo`,
`CanopenSlave`. Filter on one tag in `idf.py monitor` output to follow one task.
