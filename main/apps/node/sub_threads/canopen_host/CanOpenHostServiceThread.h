#pragma once

#include "BaseThread.h"
#include "CanFrame.h"
#include "HfUtilsCanOpenTransport.hpp"
#include "base/BaseCan.h"

#include <cstdint>
#include <functional>

/**
 * @brief Drains `BaseCan` RX (non-blocking) each step and dispatches to a handler.
 * @details Complements `CANOpenBLDCThread`: use for generic SDO/TPDO sniffing, bridge logging,
 *          or early bring-up. **Do not run alongside `CANOpenBLDCThread` on the same `BaseCan`**
 *          unless only one thread calls `Read()` — see `apps/node/canopen/README.md`.
 *          Does not run CANopenNode — application sets `onFrame`.
 */
class CanOpenHostServiceThread : public BaseThread {
public:
  explicit CanOpenHostServiceThread(BaseCan& can);

  void setFrameHandler(std::function<void(const CanOpen::CanFrame&)> h) { on_frame_ = std::move(h); }
  void setMaxFramesPerStep(unsigned n) { max_per_step_ = n; }

protected:
  bool Initialize() noexcept override;
  bool Setup() noexcept override;
  uint32_t Step() noexcept override;
  bool Cleanup() noexcept override;
  bool ResetVariables() noexcept override;

private:
  HfUtilsCanOpenTransport transport_;
  std::function<void(const CanOpen::CanFrame&)> on_frame_{};
  unsigned max_per_step_{32};
};
