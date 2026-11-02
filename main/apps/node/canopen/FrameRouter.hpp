#pragma once
/**
 * @file FrameRouter.hpp
 * @brief Per-port RX/TX policy between the one CANopenNode instance and its
 *        ports (design §4.3), applied through the CO driver's policy hooks.
 *
 * @details hf-core's CO_driver_hardfoc.c asks the router, per frame, whether
 *          a received frame may reach the consumer that matched (RX buffer
 *          index → `RxTarget`) and whether a produced frame may leave on a
 *          port (TX buffer index → `TxOrigin`). The decisions themselves are
 *          the pure rules in `contracts/router_policy.hpp`; this class only
 *          maps CANopenNode's buffers to contract targets / origins (once
 *          per communication reset, by comparing the consumer and producer
 *          objects) and keeps per-port drop counters (0x4202:07..0A).
 *
 *          Step 6 wires the CAN port only (and the host sim port, which the
 *          contracts treat as CAN). The tunnel ports (subscriptions, rate
 *          limits, 0x2701 authority) arrive with the wireless steps.
 */
#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

#include "contracts/router_policy.hpp"
#include "hf_co_driver.h"

extern "C" {
#include "CANopen.h"
}

namespace vortex::node {

class FrameRouter {
 public:
  using Port = contracts::router::Port;
  using RxTarget = contracts::router::RxTarget;
  using TxOrigin = contracts::router::TxOrigin;

  /** RX buffers whose consumer the contracts do not name (always allowed on CAN-like ports). */
  static constexpr uint8_t kRxOther = 0xFFU;
  /** TX buffers whose producer the contracts do not name (always allowed on CAN-like ports). */
  static constexpr uint8_t kTxOther = 0xFFU;

  /** @brief Declare the port in driver slot @p slot (slot = static_cast<uint8_t>(role)). */
  void SetPort(Port role, const contracts::PortCaps& caps) noexcept;

  /** @brief Rebuild the buffer → target / origin tables after CO_CANopenInit / InitPDO. */
  void Classify(const CO_t& co);

  /** @brief Install the hooks on @p drv (ctx = this). */
  void Install(hf_co_driver_t* drv) noexcept;

  /** @brief Frames the router refused on @p role since boot. */
  uint32_t Drops(Port role) const noexcept;

  /** @brief Contract target of RX buffer @p index (kRxOther when not named). */
  uint8_t TargetOf(uint16_t index) const noexcept;
  /** @brief Contract origin of TX buffer @p index (kTxOther when not named). */
  uint8_t OriginOf(uint16_t index) const noexcept;

  contracts::router::AuthorityPolicy& Policy() noexcept { return policy_; }

 private:
  static bool RxHook(void* ctx, uint8_t port, uint16_t rx_index, const void* consumer, const hf_co_frame_t* frame);
  static bool TxHook(void* ctx, uint8_t port, uint16_t tx_index, const hf_co_frame_t* frame);
  static contracts::CanFrame ToContract(const hf_co_frame_t& f) noexcept;

  struct Slot {
    bool used{false};
    contracts::PortCaps caps{};
  };
  std::array<Slot, contracts::router::kPortCount> slots_{};
  std::array<std::atomic<uint32_t>, contracts::router::kPortCount> drops_{};
  std::vector<uint8_t> rx_target_;
  std::vector<uint8_t> tx_origin_;
  contracts::router::AuthorityPolicy policy_{};
};

}  // namespace vortex::node
