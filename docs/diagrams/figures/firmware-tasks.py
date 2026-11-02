"""The Vortex V1 firmware at run time: the tasks main/ starts, the HAL they share and the chips below it.

Reads: From top to bottom. The ESP-IDF main task runs app_main, which starts the VortexSys orchestrator task and
waits for its boot table, then becomes the supervisor loop. VortexSys brings up the HAL, starts the LedDemo
thread (WS2812TestThread) and, when menuconfig selects the CANopen device, the co_svc task that serves CoNode (CANopenNode
over the generated object dictionary, on the TWAI port). Every task reaches the hardware through the
Vortex HAL singleton, which owns the managers, the hf-core handlers and the chip drivers. Stack sizes and
priorities are the constants in the sources listed below.
"""

from hardfoc_mkdocs_theme.figkit import Fig

SOURCES = [
    "main/app_main.cpp",
    "main/apps/system/sub_threads/VortexSystemThread.h",
    "main/apps/system/sub_threads/VortexSystemThread.cpp",
    "main/apps/node/sub_threads/ws2812/WS2812TestThread.h",
    "main/apps/node/canopen/CanopenDevice.cpp",
    "main/apps/node/canopen/CoSvcFreeRtos.cpp",
    "main/apps/supervisor/VortexSupervisor.cpp",
]


def build(_data):
    f = Fig(
        "firmware-tasks",
        "Four tasks over one HAL",
        alt="Task diagram of the Vortex V1 firmware: the ESP-IDF main task runs app_main and then the supervisor "
        "loop; it starts the VortexSys orchestrator, which starts the LedDemo (WS2812 test) thread and, optionally, the "
        "CANopen device task (co_svc, serving CoNode); all of them use the Vortex HAL, which drives the TMC9660, AS5047U, BNO08x, "
        "PCAL95555, the CAN transceiver and the WS2812 LED.",
        width=1200,
        kicker="Architecture",
        subtitle="main/ starts the tasks; the HAL owns every chip.",
        profile="overview",
        legend=[("brand", "this repository"), ("teal", "optional"), ("slate", "HAL and hardware")],
    )
    W = f.body_w
    g = 24
    half = (W - g) / 2
    main = f.card(0, 0, half, "ESP-IDF main task", ["app_main, then the supervisor", "loop (100 ms tick)"],
                  tone="brand", accent=True)
    orch = f.card(half + g, 0, half, "VortexSys", ["boot table, Step() every 100 ms", "16384 B stack, priority 5"],
                  tone="brand", accent=True)
    y = main.b + 48
    led = f.card(0, y, half, "LedDemo", ["WS2812TestThread: LED animations", "4096 B stack, priority 2"], tone="brand")
    co = f.card(half + g, y, half, "co_svc", ["CoNode (CANopen device), when", "menuconfig selects it; 6144 B, priority 17"],
                tone="teal", dashed=True)
    hal = f.card(0, led.b + 48, W, "Vortex HAL (hal/hf-hal-vortex-v1)",
                 ["Vortex::GetInstance(): managers, hf-core handlers, chip drivers"], tone="slate", accent=True)
    chips = f.card(0, hal.b + 48, W, "Vortex V1 board",
                   ["ESP32-C6, TMC9660, AS5047U, BNO08x, PCAL95555, CAN, WS2812"], tone="slate")
    f.connect(main, orch, "right", "left", tone="rail")
    f.connect(orch, led, "bottom", "top", at=0.3, tone="rail")
    f.connect(orch, co, "bottom", "top", tone="rail")
    f.connect(led, hal, "bottom", "top", tone="rail")
    f.connect(co, hal, "bottom", "top", tone="rail")
    f.connect(hal, chips, "bottom", "top", tone="rail")
    return f
