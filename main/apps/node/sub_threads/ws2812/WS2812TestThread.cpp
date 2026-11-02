#include "WS2812TestThread.h"

#include "OsUtility.h"
#include "handlers/logger/Logger.h"

namespace {

constexpr const char* TAG = "LedDemo";

struct DemoStep {
  LedAnimation animation;
  LedColor color;
};

constexpr DemoStep kDemo[] = {
    {LedAnimation::SOLID, LedColors::WHITE},   {LedAnimation::RAINBOW, LedColors::WHITE},
    {LedAnimation::BREATH, LedColors::BLUE},   {LedAnimation::BLINK, LedColors::ORANGE},
    {LedAnimation::STATUS_OK, LedColors::GREEN},
};
constexpr std::size_t kDemoCount = sizeof(kDemo) / sizeof(kDemo[0]);

}  // namespace

WS2812TestThread::WS2812TestThread(uint32_t updateIntervalMs, uint32_t effectDurationMs) noexcept
    : BaseThread("LedDemo"), update_interval_ms_(updateIntervalMs), effect_duration_ms_(effectDurationMs) {}

bool WS2812TestThread::Initialize() noexcept {
  return CreateBaseThread(stack_, sizeof(stack_), kPriority, kPriority, 0, OS_AUTO_START);
}

bool WS2812TestThread::Setup() noexcept {
  auto& leds = LedManager::GetInstance();
  if (!leds.EnsureInitialized()) {
    Logger::GetInstance().Error(TAG, "LedManager not available");
    return false;
  }
  if (leds.SetBrightnessPercent(brightness_percent_) != LedError::SUCCESS) {
    Logger::GetInstance().Warn(TAG, "Could not set brightness to %u%%", static_cast<unsigned>(brightness_percent_));
  }
  StartCurrent();
  return true;
}

uint32_t WS2812TestThread::Step() noexcept {
  auto& leds = LedManager::GetInstance();
  if (os_get_elapsed_time_msec() - effect_started_ms_ >= effect_duration_ms_) {
    effect_index_ = (effect_index_ + 1U) % kDemoCount;
    StartCurrent();
  }
  (void)leds.UpdateAnimation();
  return update_interval_ms_;
}

bool WS2812TestThread::Cleanup() noexcept {
  auto& leds = LedManager::GetInstance();
  (void)leds.StopAnimation();
  (void)leds.TurnOff();
  return true;
}

bool WS2812TestThread::ResetVariables() noexcept {
  effect_index_ = 0;
  current_ = LedAnimation::OFF;
  effect_started_ms_ = 0;
  return true;
}

void WS2812TestThread::SetBrightnessPercent(uint8_t percent) noexcept {
  brightness_percent_ = percent > 100U ? 100U : percent;
  if (IsThreadRunning()) {
    (void)LedManager::GetInstance().SetBrightnessPercent(brightness_percent_);
  }
}

void WS2812TestThread::StartCurrent() noexcept {
  const DemoStep& step = kDemo[effect_index_];
  current_ = step.animation;
  effect_started_ms_ = os_get_elapsed_time_msec();
  if (LedManager::GetInstance().StartAnimation(step.animation, step.color) != LedError::SUCCESS) {
    Logger::GetInstance().Warn(TAG, "Animation %s failed to start", LedAnimationToString(step.animation));
  } else {
    Logger::GetInstance().Info(TAG, "Animation: %s", LedAnimationToString(step.animation));
  }
}
