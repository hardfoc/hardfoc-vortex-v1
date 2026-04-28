#pragma once

#include "BaseThread.h"
#include "VortexSystemStateMachine.h"

#include "WS2812TestThread.h"

#include <cstddef>
#include <cstdint>
#include <memory>

class CANOpenBLDCThread;
class VortexCanHostThread;

namespace vortex_app {

/**
 * @brief Single orchestrator task: table-driven HAL + worker threads (Flux-style, no middleware).
 * @details `app_main` stays thin; boot sequencing and periodic governance live here.
 */
class VortexSystemThread : public BaseThread {
public:
  static VortexSystemThread& GetInstance() noexcept;

  VortexSystemStateMachine& StateMachine() noexcept { return VortexSystemStateMachine::GetInstance(); }

  [[nodiscard]] bool BootSucceeded() const noexcept { return boot_succeeded_; }

  [[nodiscard]] int LastCompletedInitStep() const noexcept { return last_completed_step_; }

  using SetupFn = bool (*)(VortexSystemStateMachine& sm);
  using TeardownFn = bool (*)(VortexSystemStateMachine& sm);

  struct InitStep {
    const char* name;
    SetupFn setup;
    TeardownFn teardown;
    bool recoverable;
  };

  static std::size_t InitSequenceLength() noexcept;
  static const InitStep* InitSequenceAt(std::size_t i) noexcept;

  WS2812TestThread& Ws2812Test() noexcept { return ws2812_thread_; }
  CANOpenBLDCThread* CanOpenBldc() noexcept { return canopen_bldc_.get(); }
  VortexCanHostThread* VortexCanHost() noexcept { return vortex_can_host_.get(); }

  /** @brief Used by boot table functions in `VortexSystemThread.cpp` only. */
  std::unique_ptr<CANOpenBLDCThread>& MutableCanOpenBldc() noexcept { return canopen_bldc_; }
  std::unique_ptr<VortexCanHostThread>& MutableVortexCanHost() noexcept { return vortex_can_host_; }

protected:
  bool Initialize() noexcept override;
  bool Setup() noexcept override;
  uint32_t Step() noexcept override;
  bool Cleanup() noexcept override;
  bool ResetVariables() noexcept override;

private:
  VortexSystemThread() noexcept;

  void TeardownThrough(int last_completed_index) noexcept;

  static constexpr std::size_t kThreadStackBytes = 16384U;
  static constexpr OS_Uint kThreadPriority = 5U;
  static constexpr uint32_t kDispatchPeriodMs = 100U;

  int last_completed_step_{-1};
  uint32_t tick_{0};
  bool boot_succeeded_{false};

  WS2812TestThread ws2812_thread_;
  std::unique_ptr<CANOpenBLDCThread> canopen_bldc_;
  std::unique_ptr<VortexCanHostThread> vortex_can_host_;

  uint8_t thread_stack_[kThreadStackBytes]{};
};

constexpr uint32_t ComputeVortexBootTimeoutMs() noexcept { return 120000U; }

}  // namespace vortex_app
