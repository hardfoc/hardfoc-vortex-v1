#pragma once

#include <cstddef>
#include <cstdint>

#include "BaseThread.h"
#include "managers/LedManager.h"

/**
 * @brief LED demo thread for the onboard WS2812.
 *
 * Cycles the HAL LedManager through its animations so a bench user can see
 * the LED path working. The LED itself is owned by LedManager; this thread
 * never touches the RMT driver directly.
 */
class WS2812TestThread : public BaseThread {
public:
  /**
   * @param updateIntervalMs Step period; also the animation update rate.
   * @param effectDurationMs How long each animation runs before the next.
   */
  explicit WS2812TestThread(uint32_t updateIntervalMs = 20, uint32_t effectDurationMs = 5000) noexcept;
  ~WS2812TestThread() noexcept override = default;

  WS2812TestThread(const WS2812TestThread&) = delete;
  WS2812TestThread& operator=(const WS2812TestThread&) = delete;

  /** @brief Brightness in percent (0-100) applied at the next Setup or immediately if running. */
  void SetBrightnessPercent(uint8_t percent) noexcept;

  /** @brief Animation currently shown. */
  [[nodiscard]] LedAnimation CurrentAnimation() const noexcept { return current_; }

protected:
  bool Initialize() noexcept override;
  bool Setup() noexcept override;
  uint32_t Step() noexcept override;
  bool Cleanup() noexcept override;
  bool ResetVariables() noexcept override;

private:
  void StartCurrent() noexcept;

  static constexpr std::size_t kStackBytes = 4096U;
  static constexpr OS_Uint kPriority = 2U;

  uint32_t update_interval_ms_;
  uint32_t effect_duration_ms_;
  uint8_t brightness_percent_{50};
  std::size_t effect_index_{0};
  LedAnimation current_{LedAnimation::OFF};
  uint32_t effect_started_ms_{0};

  uint8_t stack_[kStackBytes]{};
};
