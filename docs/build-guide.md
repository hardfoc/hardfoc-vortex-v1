# HardFOC Vortex V1 — Build guide

## Prerequisites

### Required software

- **ESP-IDF v5.5.0 or newer** (latest stable recommended)
- **Python 3.8+** (ESP-IDF tools)
- **Git** with submodule support
- **CMake 3.16+** (bundled with ESP-IDF in most setups)

### Supported platforms

- **Windows 10/11** (WSL2 recommended for some workflows)
- **macOS 10.15+** (Intel and Apple Silicon)
- **Linux** (Ubuntu 20.04+, Debian 11+, etc.)

### Hardware

- **HardFOC Vortex V1** board
- **USB-C** for flash and serial monitor
- **Power supply** when not powered over USB

## Quick start

### 1. Install ESP-IDF v5.5+

#### Option A: clone ESP-IDF (Linux / macOS example)

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.0 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
source ./export.sh
```

#### Option B: ESP-IDF VS Code extension

Install the **ESP-IDF** extension and follow its setup wizard.

### 2. Clone this project

```bash
git clone --recursive https://github.com/hardfoc/hardfoc-vortex-v1-demo.git
cd hardfoc-vortex-v1-demo

# If you forgot --recursive:
git submodule update --init --recursive
```

### 3. Target and configure

```bash
idf.py set-target esp32c6
idf.py menuconfig   # optional
```

### 4. Build

```bash
idf.py fullclean build
# or
idf.py build
```

### 5. Flash and monitor

```bash
idf.py flash
idf.py monitor
# or
idf.py flash monitor
```

## Project layout (build-relevant)

```
hardfoc-vortex-v1/
├── main/                          # ESP-IDF `main` component (firmware app)
├── hal/hf-hal-vortex-v1/          # Vortex HAL (git submodule)
├── docs/                          # Project documentation (this folder)
├── CMakeLists.txt                 # Root: EXTRA_COMPONENT_DIRS → hal/…
├── sdkconfig
└── README.md                      # Root readme only; details live under docs/
```

### Key build settings

| Setting | Value |
|--------|--------|
| C standard | C11 (IDF default where applicable) |
| C++ standard | C++17 (`main` component) |
| Typical build type | Release / `-O2` in `main` |
| Target | ESP32-C6 |

### Component dependencies (main)

- **hf-hal-vortex-v1** — HAL under `hal/hf-hal-vortex-v1/`
- **hf-ws2812-rmt-driver** — extra component (see root `CMakeLists.txt`)
- IDF components: **freertos**, **esp_driver_***, **log**, etc. (see `main/CMakeLists.txt`)

### Root compile definitions (excerpt)

```cmake
HARDFOC_VORTEX_V1=1
HARDFOC_VORTEX_API=1
ESP_IDF_VERSION_MAJOR=5
ESP_IDF_VERSION_MINOR=5
```

## Useful `idf.py` targets

```bash
idf.py clean
idf.py fullclean
idf.py build
idf.py build --only-components main
idf.py -v build
idf.py flash
idf.py monitor
idf.py flash monitor
idf.py size
idf.py size-components
idf.py compiledb
```

## Troubleshooting

### Submodule missing

```text
Error: No such file or directory: 'hal/hf-hal-vortex-v1/...'
```

Run:

```bash
git submodule update --init --recursive
```

### Wrong ESP-IDF version

Use ESP-IDF **v5.5+**, re-`source export.sh` (or `export.ps1`) after switching versions.

### Component `hf-hal-vortex-v1` not found

Confirm root `CMakeLists.txt` sets `EXTRA_COMPONENT_DIRS` to the HAL and external paths (see repository `CMakeLists.txt`).

### Nested submodules on Windows (`Filename too long`, `$GIT_DIR too big`)

Nested clones store git metadata under **`.git/modules/...`** with very long paths. **`git config core.longpaths true`** (repo or `--global`) is still worth setting—it fixes many **checked-out file** paths—but it **does not** always fix **`$GIT_DIR` too big** or temp-file paths under `.git/modules`, especially under **OneDrive** + deep folders.

**What works reliably**

1. Set long paths (once per machine is typical):  
   `git config --global core.longpaths true`  
   Optionally enable the OS long-path policy (Windows 10 1607+): set registry `HKLM\SYSTEM\CurrentControlSet\Control\FileSystem\LongPathsEnabled` to `1`, or **Group Policy** → *Enable Win32 long paths*.
2. Keep the monorepo on a **short local path**, e.g. **`C:\src\hardfoc-all-solutions`** (avoid long `OneDrive\Documents\GitHub\...` chains for submodule-heavy work).
3. From that root, run the sync script (see parent repo **`scripts/sync-submodules.ps1`**) or manually:  
   `git submodule sync --recursive` then `git submodule update --init --recursive` inside **`hardfoc-flux-v1`** / **`hardfoc-vortex-v1`** as needed.
4. **WSL2** clone of the same repos is another good option; paths stay short inside the Linux filesystem.

**Note:** A `subst` drive letter to the same OneDrive folder often **does not** help—Git may still canonicalize to the long `C:\Users\...` path for submodule `GIT_DIR`.

### C++17 errors

Ensure `main/CMakeLists.txt` uses `cxx_std_17` / `CMAKE_CXX_STANDARD` 17 for the `main` component.

## Testing after flash

1. **Serial**: `idf.py monitor` — expect orchestrator logs (`VortexSys`), then supervisor (`VortexMain`).
2. **Health**: periodic diagnostics and component demos (see [main supervisor loop](main/supervisor-loop.md)).

## Further reading

- [Documentation index](DOCUMENTATION_INDEX.md)
- [Main component documentation](main/README.md)
- [HAL API](../hal/hf-hal-vortex-v1/lib/api/README.md)
- [HAL documentation index](../hal/hf-hal-vortex-v1/DOCUMENTATION_INDEX.md)
- [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/)
