# Vortex V1 object dictionary

<!-- Source: od/vortex_od.yaml, revision 1.0, OD hash 0xFF85661C. -->
<!-- Do not edit: change the YAML and run `python3 tools/odgen`. -->

| | |
|---|---|
| Device | HardFOC Vortex V1 |
| Vendor ID | `0x00000000` |
| Product code | `0x00560001` |
| Revision (0x1018:03) | 1.0 = `0x00010000` |
| OD hash (0x5FFE) | `0xFF85661C` |
| Objects | 130 |

Access: `ro`, `rw`, `wo`, `const`. PDO: `t` TPDO-mappable, `r` RPDO-mappable, `tr` both. Persist: `comm` (0x1010:02), `app` (0x1010:03), `cal` (0x1010:04). `$NODEID+x` defaults add the node-ID at boot. *(extension)*: served by an OD extension.

## Areas

| First | Last | Area | Owner |
|---|---|---|---|
| `0x1000` | `0x1FFF` | Communication profile | CiA 301 |
| `0x2000` | `0x20FF` | Board identity | board |
| `0x2100` | `0x21FF` | Host encoder | sensors |
| `0x2200` | `0x22FF` | IMU | sensors |
| `0x2300` | `0x23FF` | Temperatures and power | board |
| `0x2400` | `0x26FF` | GPIO, expansion, LED, wireless | board |
| `0x2700` | `0x27FF` | Control authority | safety |
| `0x2800` | `0x2FFF` | Parameter store | storage |
| `0x3000` | `0x33FF` | Axis 1 manufacturer block | drive (0x3030.. axis diagnostics and TMC9660 telemetry) |
| `0x4000` | `0x40FF` | Fault journal | safety |
| `0x4100` | `0x41FF` | Service bridge | drive |
| `0x4200` | `0x42FF` | Diagnostics | platform |
| `0x5F00` | `0x5FFF` | OD meta | odgen |
| `0x6000` | `0x67FF` | CiA 402 axis 1 | CiA 402 |

## Communication profile

### 0x1000 Device type

*var, persist `none`*

Profile 402 (0x0192), additional information 0x0002 (servo drive). Verify against CiA 402-2.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Device type | U32 | const |  | `0x00020192` |  |  | Profile 402 (0x0192), additional information 0x0002 (servo drive). Verify against CiA 402-2. |

### 0x1001 Error register

*var, persist `none`*

bit 0 generic, 1 current, 2 voltage, 3 temperature, 4 communication, 5 device profile, 7 manufacturer.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Error register | U8 | ro | t | `0x00` |  |  | bit 0 generic, 1 current, 2 voltage, 3 temperature, 4 communication, 5 device profile, 7 manufacturer. |

### 0x1003 Pre-defined error field

*array, persist `none`, extension `CO_EM`*

Error history (16 entries) kept by CO_EM. Writing 0 to sub 0 clears it.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of errors | U8 | rw |  | (extension) |  |  |  |
| 01 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 02 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 03 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 04 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 05 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 06 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 07 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 08 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 09 | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0A | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0B | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0C | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0D | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0E | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 0F | Standard error field | U32 | ro |  | (extension) |  |  |  |
| 10 | Standard error field | U32 | ro |  | (extension) |  |  |  |

### 0x1005 COB-ID SYNC message

*var, persist `comm`*

SYNC consumer.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | COB-ID SYNC message | U32 | rw |  | `0x00000080` |  |  | SYNC consumer. |

### 0x1006 Communication cycle period

*var, persist `comm`*

SYNC timeout is 1.5× this period. 0x60C2 mirrors it for interpolation.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Communication cycle period | U32 | rw |  | `0x00000000` |  | µs | SYNC timeout is 1.5× this period. 0x60C2 mirrors it for interpolation. |

### 0x1007 Synchronous window length

*var, persist `comm`*

Synchronous TPDOs are not sent after the window.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Synchronous window length | U32 | rw |  | `0x00000000` |  | µs | Synchronous TPDOs are not sent after the window. |

### 0x1008 Manufacturer device name

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Manufacturer device name | VS[17] | const |  | "HardFOC Vortex V1" |  |  |  |

### 0x1009 Manufacturer hardware version

*var, persist `none`*

Board revision, written at boot.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Manufacturer hardware version | VS[16] | const |  | "V1" |  |  | Board revision, written at boot. |

### 0x100A Manufacturer software version

*var, persist `none`*

Semantic version and git SHA, written at boot.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Manufacturer software version | VS[48] | const |  | "" |  |  | Semantic version and git SHA, written at boot. |

### 0x1010 Store parameters

*array, persist `none`, extension `CO_storage`*

Write 0x65766173 ("save"). Refused in Operation Enabled with abort 0x08000022 (flash stalls the cache).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x04` |  |  |  |
| 01 | Save all parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 02 | Save communication parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 03 | Save application parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 04 | Save calibration parameters | U32 | rw |  | `0x00000001` |  |  |  |

### 0x1011 Restore default parameters

*array, persist `none`, extension `CO_storage`*

Write 0x64616F6C ("load"). Defaults apply after the next reset.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x04` |  |  |  |
| 01 | Restore all default parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 02 | Restore communication default parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 03 | Restore application default parameters | U32 | rw |  | `0x00000001` |  |  |  |
| 04 | Restore calibration default parameters | U32 | rw |  | `0x00000001` |  |  |  |

### 0x1012 COB-ID time stamp object

*var, persist `comm`*

TIME consumer. Sets the fault-journal wall clock.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | COB-ID time stamp object | U32 | rw |  | `0x00000100` |  |  | TIME consumer. Sets the fault-journal wall clock. |

### 0x1014 COB-ID EMCY

*var, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | COB-ID EMCY | U32 | rw |  | `$NODEID+0x80` |  |  |  |

### 0x1015 Inhibit time EMCY

*var, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Inhibit time EMCY | U16 | rw |  | `0x000A` |  | 100 µs |  |

### 0x1016 Consumer heartbeat time

*array, persist `comm`*

Bits 16–23 node-ID, bits 0–15 time in ms. A heartbeat ≤ 500 ms is one of the liveness sources required for motion (§5.5).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x04` |  |  |  |
| 01 | Consumer heartbeat time | U32 | rw |  | `0x00000000` |  |  |  |
| 02 | Consumer heartbeat time | U32 | rw |  | `0x00000000` |  |  |  |
| 03 | Consumer heartbeat time | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Consumer heartbeat time | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1017 Producer heartbeat time

*var, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Producer heartbeat time | U16 | rw |  | `0x0064` |  | ms |  |

### 0x1018 Identity

*record, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Vendor-ID | U32 | const |  | `0x00000000` |  |  | Q1 placeholder |
| 02 | Product code | U32 | const |  | `0x00560001` |  |  |  |
| 03 | Revision number | U32 | const |  | `$REVISION` = `0x00010000` |  |  | major << 16 \| minor; bumped on every OD change |
| 04 | Serial number | U32 | const |  | `0x00000000` |  |  | low 32 bits of the eFuse MAC, written at boot (LSS address) |

### 0x1019 Synchronous counter overflow value

*var, persist `comm`*

0 = SYNC without counter.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Synchronous counter overflow value | U8 | rw |  | `0x00` |  |  | 0 = SYNC without counter. |

### 0x1029 Error behavior

*array, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Communication error | U8 | rw |  | `0x00` |  |  | 0 = pre-operational (default) |
| 02 | Profile error | U8 | rw |  | `0x01` |  |  | 1 = no change |

### 0x1200 SDO server parameter

*record, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x02` |  |  |  |
| 01 | COB-ID client to server (rx) | U32 | ro | t | `$NODEID+0x600` |  |  |  |
| 02 | COB-ID server to client (tx) | U32 | ro | t | `$NODEID+0x580` |  |  |  |

### 0x1201 SDO server parameter 2

*record, persist `comm`*

Tunnel-only SDO server for the Wi-Fi / ESP-NOW ports (§4.3). These COB-IDs never go on the physical bus; the router delivers only tunnel frames to this server.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x02` |  |  |  |
| 01 | COB-ID client to server (rx) | U32 | rw |  | `$NODEID+0x6C0` |  |  |  |
| 02 | COB-ID server to client (tx) | U32 | rw |  | `$NODEID+0x5C0` |  |  |  |

### 0x1280 SDO client parameter

*record, persist `none`*

Local SDO client of the CiA 309-3 gateway (CO_CONFIG_SDO_CLI_LOCAL). Unused when the gateway is compiled out.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x03` |  |  |  |
| 01 | COB-ID client to server (tx) | U32 | rw | tr | `0x80000000` |  |  |  |
| 02 | COB-ID server to client (rx) | U32 | rw | tr | `0x80000000` |  |  |  |
| 03 | Node-ID of the SDO server | U8 | rw |  | `0x01` |  |  |  |

### 0x1400 RPDO communication parameter

*record, persist `comm`*

all modes, state and mode changes

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | COB-ID used by RPDO | U32 | rw |  | `$NODEID+0x200` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFF` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |

### 0x1401 RPDO communication parameter

*record, persist `comm`*

pp (async 255 allowed) / csp

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | COB-ID used by RPDO | U32 | rw |  | `$NODEID+0x300` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0x01` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |

### 0x1402 RPDO communication parameter

*record, persist `comm`*

pv / csv

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | COB-ID used by RPDO | U32 | rw |  | `$NODEID+0x400` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0x01` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |

### 0x1403 RPDO communication parameter

*record, persist `comm`*

tq / cst

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | COB-ID used by RPDO | U32 | rw |  | `$NODEID+0x500` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0x01` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |

### 0x1600 RPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60400010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x60600008` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1601 RPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60400010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x607A0020` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1602 RPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60400010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x60FF0020` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1603 RPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60400010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x60710010` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1800 TPDO communication parameter

*record, persist `comm`*

state

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0x40000180` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0064` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0064` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1801 TPDO communication parameter

*record, persist `comm`*

position modes

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0x40000280` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0x01` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1802 TPDO communication parameter

*record, persist `comm`*

velocity modes

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0x40000380` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0x01` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0000` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1803 TPDO communication parameter

*record, persist `comm`*

torque, error, temperature, supply

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0x40000480` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x0064` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1804 TPDO communication parameter

*record, persist `comm`*

IMU quaternion

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0xC00001C0` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x000A` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1805 TPDO communication parameter

*record, persist `comm`*

accel

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0xC00002C0` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x000A` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1806 TPDO communication parameter

*record, persist `comm`*

gyro

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0xC00003C0` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x000A` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1807 TPDO communication parameter

*record, persist `comm`*

host encoder

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x06` |  |  |  |
| 01 | COB-ID used by TPDO | U32 | rw |  | `$NODEID+0xC00004C0` |  |  |  |
| 02 | Transmission type | U8 | rw |  | `0xFE` |  |  |  |
| 03 | Inhibit time | U16 | rw |  | `0x0000` |  |  |  |
| 05 | Event timer | U16 | rw |  | `0x000A` |  |  |  |
| 06 | SYNC start value | U8 | rw |  | `0x00` |  |  |  |

### 0x1A00 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60410010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x60610008` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A01 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60410010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x60640020` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A02 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x02` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60410010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x606C0020` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x00000000` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A03 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x04` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x60770010` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x603F0010` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x23000110` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x23010110` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A04 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x04` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x22100110` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x22100210` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x22100310` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x22100410` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A05 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x04` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x22110110` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x22110210` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x22110310` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x22110410` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A06 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x05` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x22120110` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x22120210` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x22120310` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x22120408` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x22120508` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1A07 TPDO mapping parameter

*record, persist `comm`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Number of mapped application objects in PDO | U8 | rw |  | `0x03` |  |  |  |
| 01 | Application object 1 | U32 | rw |  | `0x21010110` |  |  |  |
| 02 | Application object 2 | U32 | rw |  | `0x21010310` |  |  |  |
| 03 | Application object 3 | U32 | rw |  | `0x21010710` |  |  |  |
| 04 | Application object 4 | U32 | rw |  | `0x00000000` |  |  |  |
| 05 | Application object 5 | U32 | rw |  | `0x00000000` |  |  |  |
| 06 | Application object 6 | U32 | rw |  | `0x00000000` |  |  |  |
| 07 | Application object 7 | U32 | rw |  | `0x00000000` |  |  |  |
| 08 | Application object 8 | U32 | rw |  | `0x00000000` |  |  |  |

### 0x1F80 NMT startup

*var, persist `comm`*

bit 2 = 0 waits for NMT start (default); bit 2 = 1 self-starts for stand-alone demos.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | NMT startup | U32 | rw |  | `0x00000000` |  |  | bit 2 = 0 waits for NMT start (default); bit 2 = 1 self-starts for stand-alone demos. |

## Board identity

### 0x2000 Firmware build

*record, persist `none`*

Written at boot from the HF_HAL_GIT_SHA_* macros and the IDF version.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x09` |  |  |  |
| 01 | Application version | VS[32] | const |  | "" |  |  |  |
| 02 | HAL git SHA | VS[40] | const |  | "" |  |  |  |
| 03 | Core git SHA | VS[40] | const |  | "" |  |  |  |
| 04 | TMC9660 driver git SHA | VS[40] | const |  | "" |  |  |  |
| 05 | AS5047U driver git SHA | VS[40] | const |  | "" |  |  |  |
| 06 | BNO08x driver git SHA | VS[40] | const |  | "" |  |  |  |
| 07 | CANopenNode git SHA | VS[40] | const |  | "" |  |  |  |
| 08 | Build time | U32 | const |  | `0x00000000` |  | s | Unix time |
| 09 | IDF version | VS[32] | const |  | "" |  |  |  |

### 0x2001 Board info

*record, persist `none`, extension `BoardInfo`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | Hardware revision | U8 | ro |  | `0x00` |  |  |  |
| 02 | MAC address | OS[6] | ro |  | `000000000000` |  |  |  |
| 03 | Reset reason | U32 | ro |  | `0x00000000` |  |  | esp_reset_reason_t |
| 04 | Uptime | U32 | ro |  | `0x00000000` |  | s |  |
| 05 | Free heap | U32 | ro |  | `0x00000000` |  | B |  |

### 0x2002 Component versions

*record, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | TMC9660 firmware id | U32 | ro |  | `0x00000000` |  |  |  |
| 02 | TMC9660 bootloader id | U32 | ro |  | `0x00000000` |  |  |  |
| 03 | BNO08x SH-2 version | VS[24] | ro |  | "" |  |  |  |
| 04 | AS5047U id | U16 | ro |  | `0x0000` |  |  |  |

## Host encoder

### 0x2101 Encoder status

*record, persist `none`, extension `EncoderSnapshot`, tags `telemetry`*

Host AS5047U on SPI2 (CS GPIO20). Snapshot published by drive_rt.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x08` |  |  |  |
| 01 | Angle raw | U16 | ro | t | `0x0000` |  | counts | 14-bit |
| 02 | Angle compensated | U16 | ro | t | `0x0000` |  | counts |  |
| 03 | Velocity raw | I16 | ro | t | `0` |  | counts/sample |  |
| 04 | Multi-turn position | I32 | ro | t | `0` |  | counts |  |
| 05 | AGC | U8 | ro | t | `0x00` |  |  |  |
| 06 | Magnitude | U16 | ro | t | `0x0000` |  |  |  |
| 07 | Error flags | U16 | ro | t | `0x0000` |  |  |  |
| 08 | Communication errors | U32 | ro | t | `0x00000000` |  |  |  |

## IMU

### 0x2210 IMU packed quaternion

*record, persist `none`, extension `ImuSnapshot`, tags `telemetry`*

Rotation vector, Q14 fixed point (value / 16384). One 8-byte TPDO.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Quaternion i | I16 | ro | t | `0` |  |  |  |
| 02 | Quaternion j | I16 | ro | t | `0` |  |  |  |
| 03 | Quaternion k | I16 | ro | t | `0` |  |  |  |
| 04 | Quaternion real | I16 | ro | t | `0` |  |  |  |

### 0x2211 IMU packed acceleration

*record, persist `none`, extension `ImuSnapshot`, tags `telemetry`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Acceleration x | I16 | ro | t | `0` |  | m/s² |  |
| 02 | Acceleration y | I16 | ro | t | `0` |  | m/s² |  |
| 03 | Acceleration z | I16 | ro | t | `0` |  | m/s² |  |
| 04 | Sample counter | U16 | ro | t | `0x0000` |  |  |  |

### 0x2212 IMU packed gyro

*record, persist `none`, extension `ImuSnapshot`, tags `telemetry`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | Angular rate x | I16 | ro | t | `0` |  | rad/s |  |
| 02 | Angular rate y | I16 | ro | t | `0` |  | rad/s |  |
| 03 | Angular rate z | I16 | ro | t | `0` |  | rad/s |  |
| 04 | Calibration status | U8 | ro | t | `0x00` |  |  |  |
| 05 | Accuracy | U8 | ro | t | `0x00` |  |  |  |

## Temperatures and power

### 0x2300 Temperatures

*record, persist `none`, extension `TemperatureSnapshot`, tags `telemetry`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Power stage NTC | I16 | ro | t | `0` |  | °C | TMC AIN3 |
| 02 | TMC9660 chip | I16 | ro | t | `0` |  | °C |  |
| 03 | ESP32 die | I16 | ro | t | `0` |  | °C |  |
| 04 | Motor | I16 | ro | t | `0` |  | °C | if fitted |

### 0x2301 Power

*record, persist `none`, extension `PowerSnapshot`, tags `telemetry`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x03` |  |  |  |
| 01 | Supply voltage | U16 | ro | t | `0x0000` |  | mV |  |
| 02 | Driver voltage | U16 | ro | t | `0x0000` |  | mV |  |
| 03 | Power good | BOOL | ro | t | `0x00` |  |  | PCAL95555 P6 |

## Control authority

### 0x2700 Control authority

*record, persist `none`, extension `Authority`*

Owner codes 0 none, 1 CAN, 2 Wi-Fi, 3 ESP-NOW. Authority changes only in Switched On or below; CAN can always revoke (§7.4).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x02` |  |  |  |
| 01 | Current owner | U8 | ro | t | `0x00` | … 0x03 |  |  |
| 02 | Request | U8 | rw |  | `0x00` | … 0x03 |  |  |

### 0x2701 Authority policy

*record, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Allowed enable transports | U8 | rw |  | `0x01` | … 0x07 |  | bit 0 CAN, bit 1 Wi-Fi, bit 2 ESP-NOW |
| 02 | Wireless deadman | U16 | rw |  | `0x00C8` | 0x0014 … | ms |  |
| 03 | Require master liveness | BOOL | rw |  | `0x01` |  |  | §5.5 gate on transition 4 |
| 04 | Wireless service | BOOL | rw |  | `0x00` |  |  | wireless may write 0x1010, 0x4100, 0x2601 (§7.4 rule 5) |

## Parameter store

### 0x2800 Parameter store meta

*record, persist `none`, extension `StoreMeta`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x05` |  |  |  |
| 01 | Store count | U32 | ro |  | `0x00000000` |  |  |  |
| 02 | Last store result | U32 | ro |  | `0x00000000` |  |  | 0 ok, else an SDO abort code |
| 03 | OD hash at last store | U32 | ro |  | `0x00000000` |  |  |  |
| 04 | Records applied at restore | U32 | ro |  | `0x00000000` |  |  |  |
| 05 | Records dropped at restore | U32 | ro |  | `0x00000000` |  |  | unknown, wrong group, type, length or range |

## Fault journal

### 0x4000 Fault journal header

*record, persist `none`, extension `FaultJournal`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x04` |  |  |  |
| 01 | Entry count | U16 | ro |  | `0x0000` |  |  |  |
| 02 | Head | U16 | ro |  | `0x0000` |  |  |  |
| 03 | Capacity | U16 | ro |  | `0x0040` |  |  |  |
| 04 | Overflow count | U32 | ro |  | `0x00000000` |  |  |  |

### 0x4001 Fault journal

*var, persist `none`, extension `FaultJournal`*

32-byte entries, read with segmented or block SDO (§5.3 layout).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Fault journal | DOM | ro |  | — |  |  | 32-byte entries, read with segmented or block SDO (§5.3 layout). |

### 0x4002 Fault journal clear

*var, persist `none`, extension `FaultJournal`*

Write 0x00726C63 ("clr", the byte order of the 0x1010 / 0x1011 signatures).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Fault journal clear | U32 | wo |  | `0x00000000` |  |  | Write 0x00726C63 ("clr", the byte order of the 0x1010 / 0x1011 signatures). |

### 0x4003 Active causes

*record, persist `none`, extension `FaultBook`, tags `telemetry`*

Bit n = cause id n of contracts/vortex_fault_policy.hpp.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x02` |  |  |  |
| 01 | Active causes | U32 | ro | t | `0x00000000` |  |  |  |
| 02 | Escalated causes | U32 | ro | t | `0x00000000` |  |  |  |

## Diagnostics

### 0x4200 Thread stats

*record, persist `none`, extension `ThreadStats`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x0A` |  |  |  |
| 01 | drive_rt last step | U32 | ro |  | `0x00000000` |  | µs |  |
| 02 | drive_rt max step | U32 | ro |  | `0x00000000` |  | µs |  |
| 03 | drive_rt overruns | U32 | ro |  | `0x00000000` |  |  |  |
| 04 | co_svc last step | U32 | ro |  | `0x00000000` |  | µs |  |
| 05 | co_svc max step | U32 | ro |  | `0x00000000` |  | µs |  |
| 06 | co_svc overruns | U32 | ro |  | `0x00000000` |  |  |  |
| 07 | i2c_io last step | U32 | ro |  | `0x00000000` |  | µs |  |
| 08 | i2c_io max step | U32 | ro |  | `0x00000000` |  | µs |  |
| 09 | i2c_io overruns | U32 | ro |  | `0x00000000` |  |  |  |
| 0A | Liveness stale mask | U32 | ro |  | `0x00000000` |  |  |  |

### 0x4201 Timing histogram

*record, persist `none`, extension `ThreadStats`*

Counts per bucket since boot. Jitter is SYNC to RT step start; WCET is the RT step time.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x10` |  |  |  |
| 01 | Jitter up to 10 us | U32 | ro |  | `0x00000000` |  |  |  |
| 02 | Jitter up to 20 us | U32 | ro |  | `0x00000000` |  |  |  |
| 03 | Jitter up to 50 us | U32 | ro |  | `0x00000000` |  |  |  |
| 04 | Jitter up to 100 us | U32 | ro |  | `0x00000000` |  |  |  |
| 05 | Jitter up to 200 us | U32 | ro |  | `0x00000000` |  |  |  |
| 06 | Jitter up to 500 us | U32 | ro |  | `0x00000000` |  |  |  |
| 07 | Jitter up to 1000 us | U32 | ro |  | `0x00000000` |  |  |  |
| 08 | Jitter above 1000 us | U32 | ro |  | `0x00000000` |  |  |  |
| 09 | Step up to 100 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0A | Step up to 150 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0B | Step up to 200 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0C | Step up to 250 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0D | Step up to 300 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0E | Step up to 500 us | U32 | ro |  | `0x00000000` |  |  |  |
| 0F | Step up to 1000 us | U32 | ro |  | `0x00000000` |  |  |  |
| 10 | Step above 1000 us | U32 | ro |  | `0x00000000` |  |  |  |

### 0x4202 CAN stats

*record, persist `none`, extension `CanStats`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x0A` |  |  |  |
| 01 | TEC | U16 | ro |  | `0x0000` |  |  |  |
| 02 | REC | U16 | ro |  | `0x0000` |  |  |  |
| 03 | Bus state | U8 | ro |  | `0x00` |  |  | contracts BusState |
| 04 | Bus-off count | U32 | ro |  | `0x00000000` |  |  |  |
| 05 | RX overruns | U32 | ro |  | `0x00000000` |  |  |  |
| 06 | TX queue full | U32 | ro |  | `0x00000000` |  |  |  |
| 07 | Router drops CAN | U32 | ro |  | `0x00000000` |  |  |  |
| 08 | Router drops Wi-Fi | U32 | ro |  | `0x00000000` |  |  |  |
| 09 | Router drops ESP-NOW | U32 | ro |  | `0x00000000` |  |  |  |
| 0A | Router drops host | U32 | ro |  | `0x00000000` |  |  |  |

## OD meta

### 0x5FFE OD layout hash

*var, persist `none`*

CRC32 of the generated OD description (docs excluded). `vortex-canopen info` compares it with the packaged EDS.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | OD layout hash | U32 | const |  | `$OD_HASH` = `0xFF85661C` |  |  | CRC32 of the generated OD description (docs excluded). `vortex-canopen info` compares it with the packaged EDS. |

### 0x5FFF Manufacturer OD version

*var, persist `none`*

major << 8 | minor.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Manufacturer OD version | U16 | const |  | `$OD_VERSION` = `0x0100` |  |  | major << 8 \| minor. |

## CiA 402 axis 1

### 0x6007 Abort connection option code

*var, persist `app`*

0 none, 1 fault, 2 disable voltage, 3 quick stop

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Abort connection option code | I16 | rw |  | `1` | 0 … 3 |  | 0 none, 1 fault, 2 disable voltage, 3 quick stop |

### 0x603F Error code

*var, persist `none`*

last EMCY code of this axis

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Error code | U16 | ro | t | `0x0000` |  |  | last EMCY code of this axis |

### 0x6040 Controlword

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Controlword | U16 | rw | r | `0x0000` |  |  |  |

### 0x6041 Statusword

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Statusword | U16 | ro | t | `0x0000` |  |  |  |

### 0x605A Quick stop option code

*var, persist `app`*

2 = quick-stop ramp then Switch On Disabled

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Quick stop option code | I16 | rw |  | `2` | 0 … 8 |  | 2 = quick-stop ramp then Switch On Disabled |

### 0x605B Shutdown option code

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Shutdown option code | I16 | rw |  | `0` | 0 … 1 |  |  |

### 0x605C Disable operation option code

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Disable operation option code | I16 | rw |  | `1` | 0 … 1 |  |  |

### 0x605D Halt option code

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Halt option code | I16 | rw |  | `1` | 1 … 4 |  |  |

### 0x605E Fault reaction option code

*var, persist `app`*

2 (quick-stop ramp) only where the bridge is healthy

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Fault reaction option code | I16 | rw |  | `0` | 0 … 4 |  | 2 (quick-stop ramp) only where the bridge is healthy |

### 0x6060 Modes of operation

*var, persist `none`*

changes only in Switched On or below, or in Operation Enabled at zero velocity

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Modes of operation | I8 | rw | r | `0` |  |  | changes only in Switched On or below, or in Operation Enabled at zero velocity |

### 0x6061 Modes of operation display

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Modes of operation display | I8 | ro | t | `0` |  |  |  |

### 0x6062 Position demand value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Position demand value | I32 | ro | t | `0` |  | inc |  |

### 0x6064 Position actual value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Position actual value | I32 | ro | t | `0` |  | inc |  |

### 0x6065 Following error window

*var, persist `app`*

0xFFFFFFFF = off

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Following error window | U32 | rw |  | `0xFFFFFFFF` |  | inc | 0xFFFFFFFF = off |

### 0x6066 Following error time out

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Following error time out | U16 | rw |  | `0x0000` |  | ms |  |

### 0x6067 Position window

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Position window | U32 | rw |  | `0xFFFFFFFF` |  | inc |  |

### 0x6068 Position window time

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Position window time | U16 | rw |  | `0x0000` |  | ms |  |

### 0x606B Velocity demand value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Velocity demand value | I32 | ro | t | `0` |  | inc/s |  |

### 0x606C Velocity actual value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Velocity actual value | I32 | ro | t | `0` |  | inc/s |  |

### 0x606D Velocity window

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Velocity window | U16 | rw |  | `0x0000` |  | inc/s |  |

### 0x606E Velocity window time

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Velocity window time | U16 | rw |  | `0x0000` |  | ms |  |

### 0x6071 Target torque

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Target torque | I16 | rw | r | `0` |  | ‰ of 0x6076 |  |

### 0x6072 Max torque

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Max torque | U16 | rw |  | `0x03E8` |  | ‰ |  |

### 0x6073 Max current

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Max current | U16 | rw |  | `0x03E8` |  | ‰ |  |

### 0x6075 Motor rated current

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motor rated current | U32 | rw |  | `0x00000000` |  | mA |  |

### 0x6076 Motor rated torque

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motor rated torque | U32 | rw |  | `0x00000000` |  | mNm |  |

### 0x6077 Torque actual value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Torque actual value | I16 | ro | t | `0` |  | ‰ |  |

### 0x6078 Current actual value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Current actual value | I16 | ro | t | `0` |  | ‰ |  |

### 0x6079 DC link circuit voltage

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | DC link circuit voltage | U32 | ro | t | `0x00000000` |  | mV |  |

### 0x607A Target position

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Target position | I32 | rw | r | `0` |  | inc |  |

### 0x607C Home offset

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Home offset | I32 | rw |  | `0` |  | inc |  |

### 0x607D Software position limit

*array, persist `app`*

Both 0 = no limit. Violation → EMCY 0x8612, quick stop.

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Min position limit | I32 | rw |  | `0` |  | inc |  |
| 02 | Max position limit | I32 | rw |  | `0` |  | inc |  |

### 0x607E Polarity

*var, persist `app`*

bit 7 position, bit 6 velocity (MotorMechanics::invert_output)

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Polarity | U8 | rw |  | `0x00` |  |  | bit 7 position, bit 6 velocity (MotorMechanics::invert_output) |

### 0x607F Max profile velocity

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Max profile velocity | U32 | rw |  | `0x00000000` |  | inc/s |  |

### 0x6080 Max motor speed

*var, persist `app`*

required for tq / cst (§6.2)

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Max motor speed | U32 | rw |  | `0x00000000` |  | rpm | required for tq / cst (§6.2) |

### 0x6081 Profile velocity

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Profile velocity | U32 | rw |  | `0x00000000` |  | inc/s |  |

### 0x6083 Profile acceleration

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Profile acceleration | U32 | rw |  | `0x00000000` |  | inc/s² |  |

### 0x6084 Profile deceleration

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Profile deceleration | U32 | rw |  | `0x00000000` |  | inc/s² |  |

### 0x6085 Quick stop deceleration

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Quick stop deceleration | U32 | rw |  | `0x00000000` |  | inc/s² |  |

### 0x6086 Motion profile type

*var, persist `app`*

0 linear

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motion profile type | I16 | rw |  | `0` | 0 … 0 |  | 0 linear |

### 0x6087 Torque slope

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Torque slope | U32 | rw |  | `0x00000000` |  | ‰/s |  |

### 0x608F Position encoder resolution

*array, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Encoder increments | U32 | rw |  | `0x00000001` |  |  |  |
| 02 | Motor revolutions | U32 | rw |  | `0x00000001` |  |  |  |

### 0x6091 Gear ratio

*array, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Motor revolutions | U32 | rw |  | `0x00000001` |  |  |  |
| 02 | Shaft revolutions | U32 | rw |  | `0x00000001` |  |  |  |

### 0x6092 Feed constant

*array, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Feed | U32 | rw |  | `0x00000001` |  |  |  |
| 02 | Shaft revolutions | U32 | rw |  | `0x00000001` |  |  |  |

### 0x6098 Homing method

*var, persist `app`*

17–30 switches, 33/34 index, 35/37 current position; others abort 0x06090030

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Homing method | I8 | rw |  | `0` |  |  | 17–30 switches, 33/34 index, 35/37 current position; others abort 0x06090030 |

### 0x6099 Homing speeds

*array, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Speed during search for switch | U32 | rw |  | `0x00000000` |  | inc/s |  |
| 02 | Speed during search for zero | U32 | rw |  | `0x00000000` |  | inc/s |  |

### 0x609A Homing acceleration

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Homing acceleration | U32 | rw |  | `0x00000000` |  | inc/s² |  |

### 0x60B0 Position offset

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Position offset | I32 | rw | r | `0` |  | inc |  |

### 0x60B1 Velocity offset

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Velocity offset | I32 | rw | r | `0` |  | inc/s |  |

### 0x60B2 Torque offset

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Torque offset | I16 | rw | r | `0` |  | ‰ |  |

### 0x60C2 Interpolation time period

*record, persist `none`*

Mirrors 0x1006 (value × 10^index s).

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | const |  | `0x02` |  |  |  |
| 01 | Interpolation time period value | U8 | rw |  | `0x01` |  |  |  |
| 02 | Interpolation time index | I8 | rw |  | `-3` | -6 … -3 |  |  |

### 0x60E0 Positive torque limit value

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Positive torque limit value | U16 | rw |  | `0x03E8` |  | ‰ |  |

### 0x60E1 Negative torque limit value

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Negative torque limit value | U16 | rw |  | `0x03E8` |  | ‰ |  |

### 0x60E4 Additional position actual value

*array, persist `none`, extension `EncoderSnapshot`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x01` |  |  |  |
| 01 | Host encoder position | I32 | ro | t | `0` |  | counts | AS5047U multi-turn counts |

### 0x60F4 Following error actual value

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Following error actual value | I32 | ro | t | `0` |  | inc |  |

### 0x60FD Digital inputs

*var, persist `none`*

bit 0 negative limit, 1 positive limit, 2 home (TMC REF); bits 16+ board spare inputs

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Digital inputs | U32 | ro | t | `0x00000000` |  |  | bit 0 negative limit, 1 positive limit, 2 home (TMC REF); bits 16+ board spare inputs |

### 0x60FE Digital outputs

*array, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Highest sub-index supported | U8 | ro |  | `0x02` |  |  |  |
| 01 | Physical outputs | U32 | rw | r | `0x00000000` |  |  |  |
| 02 | Bit mask | U32 | rw | r | `0x00000000` |  |  |  |

### 0x60FF Target velocity

*var, persist `none`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Target velocity | I32 | rw | r | `0` |  | inc/s |  |

### 0x6402 Motor type

*var, persist `app`*

from the catalog row (CiA 402-2 table; verify values)

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motor type | U16 | rw |  | `0x0000` |  |  | from the catalog row (CiA 402-2 table; verify values) |

### 0x6403 Motor catalogue number

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motor catalogue number | VS[32] | rw |  | "" |  |  |  |

### 0x6404 Motor manufacturer

*var, persist `app`*

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Motor manufacturer | VS[32] | rw |  | "" |  |  |  |

### 0x6502 Supported drive modes

*var, persist `none`*

pp, pv, tq, hm, csp, csv, cst

| Sub | Name | Type | Access | PDO | Default | Range | Units | Notes |
|---|---|---|---|---|---|---|---|---|
| 00 | Supported drive modes | U32 | const |  | `0x000003AD` |  |  | pp, pv, tq, hm, csp, csv, cst |
