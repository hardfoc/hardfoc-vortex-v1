# Hardware

The board this firmware runs on, in one table, and where its wiring is documented.

The [Vortex V1 HAL documentation](https://hardfoc.github.io/hf-hal-vortex-v1/latest/hardware/) is the reference for
the board: block diagram, every pin, the buses and the managers that own each chip. This firmware never uses a pin
number; it calls the HAL by name.

| Chip | Function | Bus to the ESP32-C6 |
| --- | --- | --- |
| ESP32-C6 | Runs this firmware | - |
| TMC9660 | Motor controller with gate drivers and its own current, velocity and position loops | UART (default) or SPI |
| AS5047U | 14-bit magnetic angle encoder (slot; not fitted by default) | SPI |
| BNO08x | 9-axis IMU | I2C |
| PCAL95555 | 16-bit I/O expander for TMC9660 control lines, IMU control, CAN standby, power good | I2C |
| NTC thermistor | Board temperature, read through the TMC9660 | via the TMC9660 |
| CAN transceiver | The CANopen port | TWAI |
| WS2812 | Status LED | RMT |

| HAL page | What you find |
| --- | --- |
| [Hardware](https://hardfoc.github.io/hf-hal-vortex-v1/latest/hardware/) | Chips, buses and wiring notes |
| [Pin map](https://hardfoc.github.io/hf-hal-vortex-v1/latest/hardware/pin-map/) | Every functional pin name used by the supervisor demos |
| [Getting started](https://hardfoc.github.io/hf-hal-vortex-v1/latest/getting-started/) | The HAL's own examples, to test one chip at a time |

Next: [Architecture](architecture.md).
