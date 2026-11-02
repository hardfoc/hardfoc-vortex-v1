# USB-CAN helpers (host PC)

Small **Python 3** utilities to exercise **CANopen CiA 301 / DS402-style** frames on a **USB-to-CAN** adapter. They mirror the frame layout used in **`hf-utils-canopen`** (`hal/hf-hal-vortex-v1/lib/core/hf-core-utils/hf-utils-canopen`) so you can:

- Validate encoding **without hardware** (`test_frame_encoding.py`).
- **Probe / command** a drive on the bus (`ds402_host_test.py`) with the same bitrate as the Vortex board (typically **500 kbit/s**).

These scripts are **not** part of the ESP-IDF build; they live under **`docs/helpers/`** to mark them as optional host tooling.

## Install

```bash
cd docs/helpers/usb_can
python -m venv .venv
.venv\Scripts\activate   # Windows
# source .venv/bin/activate  # Linux/macOS
pip install -r requirements.txt
```

## Hardware / drivers

You need a **`python-can`** backend for your adapter, for example:

| OS | Common backend | Notes |
|----|----------------|--------|
| Linux | `socketcan` | `channel=can0` |
| Windows | `slcan`, `pcan`, `usb2can`, … | Depends on manufacturer DLL / COM bridge |

See [python-can bus interfaces](https://python-can.readthedocs.io/en/stable/interfaces.html).

## Usage examples

**Offline tests (no adapter):**

```bash
python test_frame_encoding.py
```

**Live bus (SocketCAN example):**

```bash
python ds402_host_test.py --interface socketcan --channel can0 --bitrate 500000 --node-id 1 --action nmt_start
python ds402_host_test.py --interface socketcan --channel can0 --bitrate 500000 --node-id 1 --action read_status
```

**Windows SLCAN (example COM port):**

```bash
python ds402_host_test.py --interface slcan --channel COM5 --bitrate 500000 --node-id 1 --action read_status
```

(`--bitrate` may be ignored for some slcan bridges; set your adapter’s terminal baud to match.)

## Safety

Motor drives can **move unexpectedly** when receiving valid NMT/SDO sequences. Use **bench supply**, **no load**, and **known node id** until behavior is verified.

## See also

- [canopen-and-motor-path.md](../../canopen-and-motor-path.md) — architecture and standards coverage
- Firmware: `main/apps/node/sub_threads/canopen_bldc/CANOpenBLDCThread.cpp`
- HAL helpers: `hal/hf-hal-vortex-v1/lib/core/hf-core-utils/hf-utils-canopen/`

There is no separate **`hf-vortex-driver`** package in this tree; CANopen + `BaseCan` integration is documented above. If a dedicated driver repo is added later, these scripts can be copied or symlinked into its `examples/` folder.
