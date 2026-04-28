# Supervisor loop (`VortexSupervisor`)

After boot, **`RunVortexSupervisorLoop(Vortex& vortex, VortexSystemThread& orch)`** runs on the **IDF main task** and never returns under normal operation.

## Startup banner

Before the loop:

1. ASCII **banner** via `Logger::LogBanner`.
2. **ESP-IDF version** and compile **date/time**.
3. **`PrintChipInfo()`** — chip model, cores, features, revision, flash size, free heap.
4. **`vortex.GetSystemDiagnostics`** → **`PrintSystemDiagnostics`** if successful — colored summary of subsystem health flags.
5. **`vortex.leds.SetStatus(LedAnimation::STATUS_OK)`**.
6. **`vortex.GetSystemVersion()`** logged.

## Main loop tick

Base delay: **`MAIN_LOOP_DELAY_MS = 100`** ms per iteration.

A counter **`main_loop_count`** increments each iteration. Derived behavior uses **modulo** on that counter (so intervals are multiples of 100 ms unless noted).

### Health check

Every **`SYSTEM_HEALTH_CHECK_INTERVAL = 50`** iterations → **5 s**:

- Calls **`vortex.PerformHealthCheck()`**.
- On failure: **`STATUS_WARN`** LED; on success: **`STATUS_OK`**.

### Status log + worker snapshot

Every **`STATUS_LOG_INTERVAL = 100`** iterations → **10 s**:

- Uptime and free heap.
- **`MonitorWorkerThreads(orch)`** — WS2812 thread running flag; CANopen BLDC running + **`GetMotorStatus()`** if available.

### Component demos (batch A)

Every **`DEMO_INTERVAL = 200`** iterations → **20 s**:

- **`DemonstrateGpio`**
- **`DemonstrateAdc`**
- **`DemonstrateLeds`**
- **`DemonstrateTemperature`**

### Component demos (batch B)

When **`main_loop_count % DEMO_INTERVAL == DEMO_INTERVAL / 2`** (offset by **10 s** from batch A):

- **`DemonstrateMotorController`**
- **`DemonstrateImu`**
- **`DemonstrateEncoders`**

### CANopen motion snippet

When **`main_loop_count % MOTOR_DEMO_INTERVAL == 150`** with **`MOTOR_DEMO_INTERVAL = 300`** (30 s period, sub-phase at +15 s):

- If **`orch.CanOpenBldc()`** is non-null and running: **`EnableMotor`**, delay 1 s, **`SetVelocityMode(100)`**, 3 s, **`SetPositionMode(1000)`**, 3 s, **`DisableMotor`**.

### LED blink toggle

Every **`LED_BLINK_INTERVAL = 20`** iterations → **2 s**:

- Toggles between **`STATUS_OK`** and **`STATUS_INIT`** on the HAL LED manager (in addition to health-driven status).

## HAL usage in demos

Demos call **`Vortex`** sub-managers by name (e.g. **`vortex.gpio.Set("GPIO_EXT_GPIO_CS_1", ...)`**, **`vortex.adc.ReadChannelV(...)`**, **`vortex.motors.handler(0)`**, **`vortex.imu.GetBno08xHandler(0)`**, **`vortex.encoders.ReadAngle`**, **`vortex.leds.SetStatus`**, **`vortex.temp.ReadTemperatureCelsius`**).

Channel and GPIO **string IDs** must match HAL / board configuration (see HAL GPIO and ADC documentation).

## Modifying behavior for production

- Remove or guard demo blocks with **Kconfig** or a **build flag** so release images do not drive motors periodically.
- Shorten or replace **`vTaskDelay`**-based cadence with event-driven work if you add CLI / network commands.
