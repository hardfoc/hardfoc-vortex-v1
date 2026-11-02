# Design notes

Design documents for the Vortex V1 firmware. Each one states its status at
the top; a design is a plan, and the code is the source of truth once a step
lands.

| Document | What it covers |
|---|---|
| [CANopen node](canopen-node.md) | The CiA 301/305/402 device: object dictionary, transports (CAN, Wi-Fi, ESP-NOW), threads and timing, safety and authority rules, storage, tooling, host CI and the CAN FD path, with the ordered implementation steps. |
| [Drive observability](drive-observability.md) | Everything the TMC9660 exposes (telemetry, status flags, configuration, commands), how the node streams and captures it, the health tree, configuration image and the `vortex-canopen` dashboard. |
