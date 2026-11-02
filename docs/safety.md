# Safety

What stops the drive from doing harm today, what is only designed, and the rules to follow on the bench.

!!! danger "Nothing here is safety-rated"
    Vortex V1 has no certified safety function (no STO, no SS1). The [CANopen node design](design/canopen-node.md#22-non-goals-this-design)
    is safety-aware, but lists CiA 402-4 functional safety as a non-goal.

## What the firmware does today

| Mechanism | Where | Effect |
| --- | --- | --- |
| No motion from the firmware | `main/apps/supervisor/VortexSupervisor.cpp` | The supervisor reads the motor controller, IMU and encoder; it never enables or moves the motor on a timer. Motion is reserved for the CiA 402 state machine of the CANopen device |
| HAL must initialise | boot step `vortex_hal`, not recoverable | If the HAL fails, the completed steps are torn down and the board reboots after 2 s |
| Health check every 5 s | `Vortex::PerformHealthCheck()` | Failure sets the status LED to `STATUS_WARN` and logs `System health check failed!` |
| Task watchdog | `CONFIG_ESP_TASK_WDT_TIMEOUT_S=5`, idle task checked | A starved idle task resets the chip |
| TMC9660 protection | the TMC9660 itself | The chip has its own overcurrent, supply and temperature protection; nothing in `main/` changes its settings |
| CAN off by default | menuconfig CAN worker **None** | The firmware neither sends nor answers on CAN unless you select the CANopenNode slave |

## Rules already written and tested

The rules the CANopen device will enforce exist as header-only tables in `contracts/`, with host tests that run in
CI. The firmware does not use them yet.

| Contract | Header | Host test |
| --- | --- | --- |
| CiA 402 power drive state machine, transitions 0 to 16 | `contracts/cia402_rules.hpp` | `tests/host/test_cia402_rules.cpp` |
| Fault policy: cause to EMCY code, error register and reaction, with escalation | `contracts/vortex_fault_policy.hpp` | `tests/host/test_fault_policy.cpp` |
| Frame routing and wireless authority rules | `contracts/router_policy.hpp` | `tests/host/test_router_policy.cpp` |
| Parameter store record format | `contracts/od_store_codec.hpp` | `tests/host/test_od_store_codec.cpp` |

## What the design adds

The [CANopen node design](design/canopen-node.md) describes the rest; the sections to read:

| Topic | Section |
| --- | --- |
| Liveness requirements for motion | [5.5](design/canopen-node.md#55-liveness-requirements-for-motion) |
| Safe defaults and limit enforcement | [6.2](design/canopen-node.md#62-safe-defaults-and-limit-enforcement) |
| Fault funnel and EMCY mapping | [6.4](design/canopen-node.md#64-fault-funnel-and-emcy-mapping) |
| Safety arbiter and liveness-gated watchdog (500 ms, fed only while critical threads are live) | [6.5](design/canopen-node.md#65-safety-arbiter-and-liveness-gated-watchdog) |
| Wireless safety rules | [7.4](design/canopen-node.md#74-safety-rules-never-bypass) |

## On the bench

- Use a current-limited supply and an unloaded motor until the configuration is proven.
- Know the node ID before you send NMT or SDO frames: a valid sequence enables a CiA 402 drive.
- With the CANopenNode slave selected, read the [threads warning](threads.md#tasks) first.
- The bench documents linked from the [HAL hardware page](https://hardfoc.github.io/hf-hal-vortex-v1/latest/hardware/)
  cover the first spin of a motor on the TMC9660.

Next: [CANopen](canopen.md).
