# Getting started

Build the firmware with the ESP-IDF version CI uses, flash it to a Vortex V1 board and read the boot log.

!!! warning "The board drives a motor"
    Bring a new board up on a current-limited bench supply with the motor unloaded. The firmware does not
    command motion on its own, but the HAL demos talk to the TMC9660 every 20 s.

## Requirements

| Item | Version or value | Where it is set |
| --- | --- | --- |
| ESP-IDF | v5.5 (CI builds with the `release-v5.5` image of `espressif/idf`) | `.github/workflows/build-and-analyze.yml` |
| Target | `esp32c6` | `sdkconfig` (`CONFIG_IDF_TARGET`), workflow `IDF_TARGET` |
| CMake | 3.16 or newer (ships with ESP-IDF) | `CMakeLists.txt` |
| Git | with submodule support | the HAL is a submodule |
| Board | Vortex V1, USB-C for flash and console | |

The root `CMakeLists.txt` also defines `ESP_IDF_VERSION_MAJOR=5` and `ESP_IDF_VERSION_MINOR=5`, so build with a
5.5 release.

## Steps

<div class="hf-steps" markdown>

1. **Install ESP-IDF v5.5** for the ESP32-C6 and open a shell with its environment:

    ```bash
    mkdir -p ~/esp && cd ~/esp
    git clone -b release/v5.5 --recursive https://github.com/espressif/esp-idf.git
    cd esp-idf && ./install.sh esp32c6 && . ./export.sh
    ```

    The ESP-IDF extension for VS Code installs the same thing through its setup wizard.

2. **Clone the firmware with its submodules.** The HAL (`hal/hf-hal-vortex-v1`) brings hf-core, the chip drivers
   and CANopenNode with it:

    ```bash
    git clone https://github.com/hardfoc/hardfoc-vortex-v1.git && cd hardfoc-vortex-v1
    git submodule update --init hal/hf-hal-vortex-v1
    git -C hal/hf-hal-vortex-v1 submodule init
    git -C hal/hf-hal-vortex-v1 config submodule.docs/hf-development-handbook.update none
    git submodule update --init --recursive
    ```

    The HAL's `docs/hf-development-handbook` submodule is private; the third line skips it, as CI does. With access
    to it, `git clone --recursive` is enough.

3. **Build.** The repository commits its `sdkconfig`, so the target is already `esp32c6`:

    ```bash
    idf.py build
    ```

    `idf.py menuconfig` opens the options described in [Configuration](configuration.md).

4. **Flash and open the console** (115200 baud on UART0, with USB Serial/JTAG as the secondary console):

    ```bash
    idf.py -p PORT flash monitor
    ```

</div>

## What the boot log shows

| Tag | Lines | Meaning |
| --- | --- | --- |
| `MAIN` | `Spawning VortexSystemThread...` | `app_main` started the orchestrator |
| `VortexSys` | `Boot: 3 init steps`, then `[1/3] vortex_hal`, `[2/3] ws2812`, `[3/3] can_worker` | the boot table runs |
| `VortexSys` | `CAN worker disabled (menuconfig: none)` | the default: no CANopen task |
| `VortexSys` | `Boot complete — state=...` | every non-recoverable step passed |
| `MAIN` | `Boot complete. Main task entering supervisor loop.` | the supervisor takes over |
| `VortexMain` | banner, ESP-IDF version, chip information, system diagnostics | printed once |
| `VortexMain` | `System Status - Uptime: ...` every 10 s, demos every 20 s | steady state |

A `FATAL: ... rebooting in 2000 ms` line from `MAIN` means a boot step failed; see
[Troubleshooting](troubleshooting.md).

## Other useful targets

| Command | What it does |
| --- | --- |
| `idf.py -DCMAKE_BUILD_TYPE=Debug build` | The Debug build CI also runs |
| `idf.py size-components` | Flash and RAM use per component (CI keeps it as an artifact) |
| `idf.py fullclean` | Remove `build/` after an ESP-IDF or submodule change |

Next: [Architecture](architecture.md).
