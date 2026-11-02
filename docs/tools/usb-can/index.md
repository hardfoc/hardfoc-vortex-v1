# USB-CAN tools

Python scripts that send CANopen NMT and expedited SDO frames from a PC through a USB-CAN adapter, for bench checks.

The scripts build frames by hand with python-can, with the same COB-IDs as hf-core's `hf-utils-canopen`
(`canopen_frames.py`): NMT `0x000`, SDO request `0x600 + node`, SDO response `0x580 + node`. They are not part of
the firmware build. The [CANopen node design](../../design/canopen-node.md#92-python-package-vortex-canopen) plans
to replace them with the `vortex-canopen` package, which reads the generated EDS.

!!! danger "A valid sequence moves a drive"
    `ds402_velocity_demo` walks a CiA 402 drive to Operation Enabled. Use a current-limited supply, no load and a
    known node ID.

## Install

```bash
cd docs/tools/usb-can
python3 -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
python3 -m pip install -r requirements.txt
```

`requirements.txt` installs python-can. The adapter needs a [python-can interface](https://python-can.readthedocs.io/en/stable/interfaces.html):
`socketcan` on Linux (`--channel can0`); `slcan`, `pcan` or `usb2can` on Windows, depending on the adapter.

## Scripts

| Script | What it does |
| --- | --- |
| `test_frame_encoding.py` | Checks the frame builders offline; no adapter needed |
| `ds402_host_test.py --action nmt_start` | Sends NMT Start to the node |
| `ds402_host_test.py --action read_status` | Reads the statusword `0x6041` by SDO (the default action) |
| `ds402_host_test.py --action ds402_velocity_demo` | NMT Start, the CiA 402 enable sequence (`0x6040` = 6, 7, 15), profile velocity mode and a target velocity of 100 |

| Option | Default | Meaning |
| --- | --- | --- |
| `--interface` | `socketcan` | python-can interface name |
| `--channel` | `can0` | Interface channel, for example `COM5` for slcan |
| `--bitrate` | `500000` | Nominal bit rate; the firmware uses 500 kbit/s |
| `--node-id` | `1` | Target node; the firmware default is 1 |
| `--timeout` | `0.5` | Seconds to wait for each SDO reply |

```bash
python3 test_frame_encoding.py
python3 ds402_host_test.py --interface socketcan --channel can0 --node-id 1 --action read_status
```

With the Vortex slave runtime as it is today, expect no SDO reply: the firmware cannot receive frames yet
([CANopen](../../canopen.md#what-runs-today)).
