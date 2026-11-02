/**
 * @file CoSvcStd.cpp
 * @brief co_svc on a host: std::thread + condition variable (host node binary).
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "CoSvc.hpp"

namespace vortex::node {

struct CoSvc::Impl {
  std::thread thread;
  std::mutex mu;
  std::condition_variable cv;
  bool woken{false};
  std::atomic<bool> run{false};
  std::atomic<uint32_t> passes{0};
  std::atomic<uint32_t> max_pass_us{0};
};

CoSvc::CoSvc(CoNode& node) noexcept : node_(node), impl_(new Impl) {}

CoSvc::~CoSvc() {
  Stop();
  delete impl_;
}

void CoSvc::Wake(void* ctx) {
  auto* impl = static_cast<Impl*>(ctx);
  {
    std::lock_guard<std::mutex> lk(impl->mu);
    impl->woken = true;
  }
  impl->cv.notify_one();
}

bool CoSvc::Start() noexcept {
  if (impl_->run.exchange(true)) {
    return true;
  }
  node_.SetWake(&CoSvc::Wake, impl_);
  impl_->thread = std::thread([this] {
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    while (impl_->run.load()) {
      const auto t0 = Clock::now();
      const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t0 - last).count();
      last = t0;
      uint32_t next_us = 1000U;
      const CoNode::Step step = node_.Process(static_cast<uint32_t>(dt), next_us);
      const auto pass =
          static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count());
      impl_->passes.fetch_add(1, std::memory_order_relaxed);
      if (pass > impl_->max_pass_us.load(std::memory_order_relaxed)) {
        impl_->max_pass_us.store(pass, std::memory_order_relaxed);
      }
      if (step == CoNode::Step::ResetApp) {
        (void)node_.ResetApplication();
        continue;
      }
      if (step == CoNode::Step::Quit) {
        break;
      }
      if (next_us < 100U) {
        next_us = 100U;
      } else if (next_us > 10000U) {
        next_us = 10000U;
      }
      std::unique_lock<std::mutex> lk(impl_->mu);
      impl_->cv.wait_for(lk, std::chrono::microseconds(next_us), [this] { return impl_->woken || !impl_->run.load(); });
      impl_->woken = false;
    }
    impl_->run.store(false);
  });
  return true;
}

void CoSvc::Stop() noexcept {
  if (impl_ == nullptr) {
    return;
  }
  impl_->run.store(false);
  Wake(impl_);
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
  node_.SetWake(nullptr, nullptr);
}

bool CoSvc::Running() const noexcept { return impl_->run.load(); }
uint32_t CoSvc::Passes() const noexcept { return impl_->passes.load(); }
uint32_t CoSvc::MaxPassUs() const noexcept { return impl_->max_pass_us.load(); }

}  // namespace vortex::node
