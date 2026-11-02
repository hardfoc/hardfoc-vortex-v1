#pragma once
/**
 * @file CoSvc.hpp
 * @brief The `co_svc` service thread (design §4.4): runs CoNode::Process()
 *        whenever a frame arrives or CANopenNode's timer-next expires.
 *
 * @details Two implementations with this interface:
 *          - CoSvcFreeRtos.cpp (firmware): a FreeRTOS task at priority 17,
 *            woken by a task notification from the RX ISR path;
 *          - CoSvcStd.cpp (host node binary): a std::thread woken through a
 *            condition variable.
 *          NMT reset node restarts the chip on the firmware and restores the
 *          OD defaults (CoNode::ResetApplication) on the host.
 */
#include <cstdint>

#include "CoNode.hpp"

namespace vortex::node {

class CoSvc {
 public:
  explicit CoSvc(CoNode& node) noexcept;
  ~CoSvc();
  CoSvc(const CoSvc&) = delete;
  CoSvc& operator=(const CoSvc&) = delete;

  /** @brief Install the wake hook on the node and start the thread (call after CoNode::Start()). */
  bool Start() noexcept;
  /** @brief Ask the thread to stop and wait for it. */
  void Stop() noexcept;
  bool Running() const noexcept;

  /** @brief hf_co_wake_fn: wake the thread (ISR-safe on the firmware). */
  static void Wake(void* ctx);

  /** @brief Service passes run so far, and the longest one (µs). */
  uint32_t Passes() const noexcept;
  uint32_t MaxPassUs() const noexcept;

  struct Impl;  ///< platform state (defined by the implementation file)

 private:
  CoNode& node_;
  Impl* impl_{nullptr};
};

}  // namespace vortex::node
