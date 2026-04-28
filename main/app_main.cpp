/**
 * @file app_main.cpp
 * @brief ESP-IDF entry — thin supervisor over `VortexSystemThread` (Flux-style).
 *
 * @details
 * - `app_main` initializes logging, starts the system orchestrator task, waits for
 *   its `Setup()` table (HAL → WS2812 → CANopen) to finish, then runs the product
 *   supervisor loop on the IDF main task.
 * - Demos and periodic health live in `apps/supervisor/VortexSupervisor.cpp`.
 */

#include "VortexSystemThread.h"
#include "VortexSupervisor.h"

#include "Vortex.h"
#include "handlers/logger/Logger.h"
#include "Utility.h"

#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr const char* kTag = "MAIN";
constexpr uint16_t kPreRebootDelayMs = 2000U;

[[noreturn]] void ResetWithDiagnostic(const char* reason) noexcept {
  Logger::GetInstance().Error(kTag, "FATAL: %s — rebooting in %u ms", reason,
                              static_cast<unsigned>(kPreRebootDelayMs));
  vTaskDelay(pdMS_TO_TICKS(kPreRebootDelayMs));
  esp_restart();
  while (true) {
  }
}

}  // namespace

extern "C" void app_main(void) {
  Logger::GetInstance().Initialize();

  auto& orch = vortex_app::VortexSystemThread::GetInstance();
  Logger::GetInstance().Info(kTag, "Spawning VortexSystemThread...");
  if (!orch.EnsureInitialized()) {
    ResetWithDiagnostic("VortexSystemThread EnsureInitialized failed");
  }

  if (!orch.Start()) {
    ResetWithDiagnostic("VortexSystemThread Start failed");
  }

  constexpr uint32_t kBootWaitMs = vortex_app::ComputeVortexBootTimeoutMs();
  if (!TestLogicWithTimeout([&]() { return orch.IsSetupComplete(); }, true, kBootWaitMs, 10, nullptr)) {
    ResetWithDiagnostic("VortexSystemThread Setup did not complete in time");
  }

  if (!orch.BootSucceeded()) {
    ResetWithDiagnostic("VortexSystemThread boot sequence failed");
  }

  Logger::GetInstance().Info(kTag, "Boot complete. Main task entering supervisor loop.");
  auto& vortex = Vortex::GetInstance();
  RunVortexSupervisorLoop(vortex, orch);
}
