# Testing and CI

What runs on every pull request, how to run the same checks on your machine, and what is still tested only on the
bench.

## Host tests (contracts)

The headers in `contracts/` build and run on a PC with sanitizers (`tests/host`, preset `host-debug`: Debug, ASan
and UBSan, Ninja):

```bash
cd tests/host
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug
```

| Test | Covers |
| --- | --- |
| `test_cia402_rules.cpp` | The CiA 402 rule table (`contracts/cia402_rules.hpp`) |
| `test_fault_policy.cpp` | The fault policy table (`contracts/vortex_fault_policy.hpp`) |
| `test_router_policy.cpp` | Router policy and authority rules (`contracts/router_policy.hpp`) |
| `test_od_store_codec.cpp` | The parameter store record codec (`contracts/od_store_codec.hpp`) |
| `test_od_ids.cpp` | The generated identifiers (`contracts/vortex_od_ids.hpp`) |

The build also compiles every contracts header on its own (`vortex_contracts_standalone`), so a header that misses
an include fails.

## Object dictionary checks

```bash
git submodule update --init hal/hf-hal-vortex-v1
git -C hal/hf-hal-vortex-v1 submodule update --init third_party/CANopenNode
python3 -m pip install -r tools/odgen/requirements.txt
python3 tools/odgen --check
VORTEX_REQUIRE_CANOPENNODE=1 python3 -m pytest -q tools/odgen/tests
```

| Test | Covers |
| --- | --- |
| `test_cli.py` | `--check` passes on the tree, fails on any drift, and enforces the revision bump |
| `test_schema.py` | The YAML schema rules |
| `test_eds.py` | python-canopen loads the EDS; every object matches the YAML; export and re-import keep every object |
| `test_canopennode_layout.py` | The CANopenNode example EDS through odgen reproduces the example `OD.c` / `OD.h` |
| `test_od_c_build.py` | The generated `OD.c` builds with CANopenNode v4 and `CO_CANopenInit` accepts it |

odgen supports Python 3.10 and newer; CI uses 3.10.

## CI workflows

| Workflow | Jobs | Runs on |
| --- | --- | --- |
| `host.yml` (**CI • Host tests**) | `contracts-host`, `od-drift`, `node-host`, `CI gate` (green only when all are) | every pull request, push to `main` |
| `build-and-analyze.yml` | ESP-IDF build in `release-v5.5`, Release and Debug, plus a Release build with the CANopen device on (`ci/sdkconfig.canopen`), with size report and firmware artifacts; cppcheck on `main/`; CodeQL; actionlint | pull requests to `main`, push to `main` |
| `docs.yml` (**Docs**) | Strict site build, link check, Lighthouse, screenshots; publish to GitHub Pages from `main` | pull requests to `main`, push to `main` |

## The CANopen node on the host

`host/node` builds `vortex_node_host`: the firmware's `CoNode`, `FrameRouter`, generated `OD.c` and CANopenNode,
with hf-core's `LinuxSocketCanPort` (`--port vcan0`) or `PipePort` (`--port pipe:<path>`, a UNIX-socket hub
carrying frame records). `tests/node` runs python-canopen against it
([design §9.4](design/canopen-node.md#94-host-ci-same-node-simulated-hardware)):

```bash
git -C hal/hf-hal-vortex-v1 submodule update --init lib/core third_party/CANopenNode
cmake -S host/node -B build/node-host -G Ninja && cmake --build build/node-host
python3 -m pip install -r tests/node/requirements.txt
python3 -m pytest tests/node          # vcan0 if present, else the pipe bus (VORTEX_BUS=vcan0|pipe)
```

| Suite | Checks |
| --- | --- |
| `test_nmt_hb.py` | boot-up, NMT states and addressing, heartbeat period (0x1017), 0x1F80, reset communication and node |
| `test_sdo.py` | expedited 1–4 bytes, segmented, block transfer with CRC (and a corrupted CRC), abort codes, SDO server 2 kept off the CAN port |
| `test_pdo.py` | mapping procedure and aborts, event timer, inhibit time, SYNC TPDO, RPDO into the OD, RPDO length and timeout EMCYs |
| `test_lss.py` | fastscan, node-ID assignment and store across a restart, inquiries, bit-timing table, `--nvs` path validation |
| `test_frame_record.py` | the Python frame-record codec against the golden bytes of hf-core's C++ codec |

CI job `node-host` builds the binary with ASan and UBSan and runs the suites on the pipe bus, then again on
`vcan0` when the runner can load `vcan`.

## On the bench

Nothing runs on hardware in CI. The [CANopen node design](design/canopen-node.md#93-bench-test-plan-usb-can) has the
USB-CAN bench plan.

Next: [Design notes](design/index.md).
