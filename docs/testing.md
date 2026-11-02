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
| `host.yml` (**CI • Host tests**) | `contracts-host`, `od-drift`, `CI gate` (green only when both are) | every pull request, push to `main` |
| `build-and-analyze.yml` | ESP-IDF build in `release-v5.5`, Release and Debug, with size report and firmware artifacts; cppcheck on `main/`; CodeQL; actionlint | pull requests to `main`, push to `main` |
| `docs.yml` (**Docs**) | Strict site build, link check, Lighthouse, screenshots; publish to GitHub Pages from `main` | pull requests to `main`, push to `main` |

## On the bench

Nothing runs on hardware in CI. The [CANopen node design](design/canopen-node.md#93-bench-test-plan-usb-can) has the
USB-CAN bench plan, and [section 9.4](design/canopen-node.md#94-host-ci-same-node-simulated-hardware) the plan to run
the same node against a simulated motor in host CI.

Next: [Design notes](design/index.md).
