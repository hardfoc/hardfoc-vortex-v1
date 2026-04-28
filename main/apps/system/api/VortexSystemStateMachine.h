#pragma once

#include <atomic>
#include <cstdint>

namespace vortex_app {

enum class VortexSystemState : std::uint8_t {
  Boot = 0,
  HalInit,
  ThreadsUp,
  Ready,
  Fault,
};

enum class VortexChangeReason : std::uint8_t {
  Init = 0,
  Runtime,
  Error,
  Cleanup,
};

/**
 * @brief Lightweight global lifecycle state for the motor-controller node.
 * @details Flux uses a richer machine; Vortex only needs boot vs ready vs fault.
 */
class VortexSystemStateMachine {
public:
  static VortexSystemStateMachine& GetInstance() noexcept;

  [[nodiscard]] VortexSystemState State() const noexcept {
    return state_.load(std::memory_order_acquire);
  }

  bool TryTransition(VortexSystemState to, VortexChangeReason reason) noexcept;

  [[nodiscard]] const char* StateName() const noexcept;

private:
  VortexSystemStateMachine() noexcept = default;

  std::atomic<VortexSystemState> state_{VortexSystemState::Boot};
  std::atomic<VortexChangeReason> last_reason_{VortexChangeReason::Init};
};

}  // namespace vortex_app
