# CANopen on Vortex V1

Vortex V1 is a CANopen **device**: a CiA 301 node with the CiA 402 drive
profile. It does not act as a master for other drives.

## What is here today

| File | Role |
|---|---|
| `CanopenSlaveRuntime.{hpp,cpp}` | Starts CANopenNode on the HAL CAN bus when **`idf.py menuconfig` → HardFOC Vortex main (app) → CAN RX drain worker** is set to the CANopenNode slave. Requires `CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE` in the HAL menu. |

The default boot worker is **None**: the firmware brings the board up and
leaves the CAN bus alone.

## Known limitations

The current runtime is a bring-up shim. It compiles the upstream example
object dictionary, does not persist parameters, and depends on
`BaseCan::ReceiveMessage`, which the ESP32 CAN wrapper does not implement
yet, so it cannot receive frames on hardware.

## Where this is going

The CANopen device is being rebuilt in PR-sized steps:

- one YAML object dictionary that generates the firmware dictionary, the EDS
  file and the tooling constants, with a drift check in CI;
- the CiA 402 state machine and fault policy as host-tested rule tables;
- a non-blocking CAN driver binding with an RX ring, FD-sized frames and
  bus-off handling;
- a `vortex-canopen` Python tool for USB-CAN adapters, and host CI that runs
  the same node on a virtual CAN bus against a simulated motor.

The full design, including the object dictionary layout, threads, timing,
wireless access rules and the CAN FD path, is tracked with the HardFOC
modernisation plan.
