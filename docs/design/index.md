# Design notes

The design documents for the Vortex V1 firmware: plans written before the code, each with its status at the top.

A design is a plan; once a step lands, the code is the source of truth and the concept pages of this site describe
it. Both documents are drafts for owner review.

| Document | What it covers |
| --- | --- |
| [CANopen node](canopen-node.md) | The CiA 301 / 305 / 402 device: object dictionary, transports (CAN, Wi-Fi, ESP-NOW), threads and timing, safety and authority rules, storage, tooling, host CI and the CAN FD path, with the ordered implementation steps |
| [Drive observability](drive-observability.md) | Everything the TMC9660 exposes (telemetry, status flags, configuration, commands), how the node streams and captures it, the health tree, the configuration image and the `vortex-canopen` dashboard |

| Where the design is today | Page |
| --- | --- |
| Contracts package (step 3) and object dictionary generator (step 4) are in the repository | [Testing and CI](../testing.md), [CANopen](../canopen.md#the-object-dictionary) |
| The slave runtime is still the bring-up shim the design replaces | [CANopen](../canopen.md#what-runs-today) |
