#pragma once
/**
 * @file SimAxisPort.hpp
 * @brief Skeleton of the simulated TMC9660 axis (design §9.4).
 *
 * @details Step 8 replaces this with the full model (R, L, Kt, J, b, load,
 *          the TMC current / velocity / position loops and ramp, fault flags,
 *          the ~1 kHz target sampling and the SPI heartbeat) behind the same
 *          AxisPort facade the firmware uses, and pytest drives faults over a
 *          JSON control socket. Today it is a first-order velocity model so
 *          the host binary has a place to grow it.
 */
#include <cstdint>

namespace vortex::sim {

class SimAxisPort {
 public:
  struct State {
    int32_t position{0};     ///< increments
    int32_t velocity{0};     ///< increments / s
    int16_t current_ma{0};
    bool enabled{false};
  };

  void SetEnabled(bool on) noexcept { state_.enabled = on; }
  void SetTargetVelocity(int32_t v) noexcept { target_velocity_ = v; }

  /** @brief Advance the model by @p dt_us (velocity follows the target with a 50 ms time constant). */
  void Step(uint32_t dt_us) noexcept {
    const int32_t target = state_.enabled ? target_velocity_ : 0;
    const int64_t dv = static_cast<int64_t>(target - state_.velocity) * dt_us / 50000;
    state_.velocity += static_cast<int32_t>(dv);
    state_.position += static_cast<int32_t>(static_cast<int64_t>(state_.velocity) * dt_us / 1000000);
  }

  const State& Get() const noexcept { return state_; }

 private:
  State state_{};
  int32_t target_velocity_{0};
};

}  // namespace vortex::sim
