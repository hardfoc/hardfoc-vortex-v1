/**
 * @file CoSvcFreeRtos.cpp
 * @brief co_svc on the firmware: a FreeRTOS task woken by task notifications.
 *
 * @details Priority 17 (design §4.4: below lwIP at 18, above the wireless
 *          gateway). The RX ISR path (EspTwaiPort → CO driver ring → wake
 *          hook) notifies the task; otherwise it sleeps until CANopenNode's
 *          timer-next, at least one tick. NMT reset node restarts the chip.
 */
#include <atomic>

#include "CoSvc.hpp"

#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace vortex::node {
namespace {
constexpr UBaseType_t kPriority = 17;
constexpr uint32_t kStackBytes = 6144;
}  // namespace

struct CoSvc::Impl {
  TaskHandle_t task{nullptr};
  std::atomic<bool> run{false};
  std::atomic<bool> finished{true};
  std::atomic<uint32_t> passes{0};
  std::atomic<uint32_t> max_pass_us{0};
  CoNode* node{nullptr};
};

CoSvc::CoSvc(CoNode& node) noexcept : node_(node), impl_(new Impl) { impl_->node = &node; }

CoSvc::~CoSvc() {
  Stop();
  delete impl_;
}

void CoSvc::Wake(void* ctx) {
  auto* impl = static_cast<Impl*>(ctx);
  TaskHandle_t task = impl->task;
  if (task == nullptr) {
    return;
  }
  if (xPortInIsrContext()) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
    portYIELD_FROM_ISR(woken);
  } else {
    xTaskNotifyGive(task);
  }
}

static void CoSvcTask(void* arg) {
  auto* impl = static_cast<CoSvc::Impl*>(arg);
  int64_t last = esp_timer_get_time();
  while (impl->run.load()) {
    const int64_t t0 = esp_timer_get_time();
    const auto dt = static_cast<uint32_t>(t0 - last);
    last = t0;
    uint32_t next_us = 1000U;
    const CoNode::Step step = impl->node->Process(dt, next_us);
    const auto pass = static_cast<uint32_t>(esp_timer_get_time() - t0);
    impl->passes.fetch_add(1, std::memory_order_relaxed);
    if (pass > impl->max_pass_us.load(std::memory_order_relaxed)) {
      impl->max_pass_us.store(pass, std::memory_order_relaxed);
    }
    if (step == CoNode::Step::ResetApp) {
      esp_restart();
    }
    if (step == CoNode::Step::Quit) {
      break;
    }
    TickType_t ticks = pdMS_TO_TICKS((next_us + 999U) / 1000U);
    if (ticks == 0) {
      ticks = 1;  // never spin: with a 100 Hz tick a 1 ms wait would be 0 ticks
    }
    (void)ulTaskNotifyTake(pdTRUE, ticks);
  }
  impl->task = nullptr;
  impl->finished.store(true);
  vTaskDelete(nullptr);
}

bool CoSvc::Start() noexcept {
  if (impl_->run.exchange(true)) {
    return true;
  }
  impl_->finished.store(false);
  node_.SetWake(&CoSvc::Wake, impl_);
  if (xTaskCreate(&CoSvcTask, "co_svc", kStackBytes, impl_, kPriority, &impl_->task) != pdPASS) {
    impl_->run.store(false);
    impl_->finished.store(true);
    impl_->task = nullptr;
    node_.SetWake(nullptr, nullptr);
    return false;
  }
  return true;
}

void CoSvc::Stop() noexcept {
  if (impl_ == nullptr || !impl_->run.exchange(false)) {
    return;
  }
  Wake(impl_);
  for (int i = 0; i < 500 && !impl_->finished.load(); ++i) {
    vTaskDelay(1);
  }
  node_.SetWake(nullptr, nullptr);
}

bool CoSvc::Running() const noexcept { return impl_->run.load(); }
uint32_t CoSvc::Passes() const noexcept { return impl_->passes.load(); }
uint32_t CoSvc::MaxPassUs() const noexcept { return impl_->max_pass_us.load(); }

}  // namespace vortex::node
