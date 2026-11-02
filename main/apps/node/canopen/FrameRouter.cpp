/**
 * @file FrameRouter.cpp
 * @brief CANopenNode buffer classification and the CO driver policy hooks.
 */
#include "FrameRouter.hpp"

#include <cstring>
#include <utility>

extern "C" {
#include "OD.h"
}

namespace vortex::node {

namespace router = contracts::router;

void FrameRouter::SetPort(Port role, const contracts::PortCaps& caps) noexcept {
  const auto i = static_cast<uint8_t>(role);
  if (i < slots_.size()) {
    slots_[i].used = true;
    slots_[i].caps = caps;
  }
}

void FrameRouter::Install(hf_co_driver_t* drv) noexcept { hf_co_driver_set_policy(drv, &RxHook, &TxHook, this); }

uint32_t FrameRouter::Drops(Port role) const noexcept {
  const auto i = static_cast<uint8_t>(role);
  return i < drops_.size() ? drops_[i].load(std::memory_order_relaxed) : 0U;
}

uint8_t FrameRouter::TargetOf(uint16_t index) const noexcept {
  return index < rx_target_.size() ? rx_target_[index] : kRxOther;
}

uint8_t FrameRouter::OriginOf(uint16_t index) const noexcept {
  return index < tx_origin_.size() ? tx_origin_[index] : kTxOther;
}

void FrameRouter::Classify(const CO_t& co) {
  CO_CANmodule_t* m = co.CANmodule;
  CO_LOCK_CAN_SEND(m);
  rx_target_.assign(m->rxSize, kRxOther);
  tx_origin_.assign(m->txSize, kTxOther);

  auto rx_is = [](const void* object, RxTarget t) { return std::make_pair(object, static_cast<uint8_t>(t)); };
  std::vector<std::pair<const void*, uint8_t>> consumers;
  consumers.push_back(rx_is(co.NMT, RxTarget::Nmt));
#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) != 0
  consumers.push_back(rx_is(co.SYNC, RxTarget::Sync));
#endif
#if ((CO_CONFIG_TIME) & CO_CONFIG_TIME_ENABLE) != 0
  consumers.push_back(rx_is(co.TIME, RxTarget::Time));
#endif
  consumers.push_back(rx_is(co.em, RxTarget::EmcyConsumer));
#if ((CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE) != 0
  for (uint8_t i = 0; i < OD_CNT_ARR_1016; ++i) {
    consumers.push_back(rx_is(&co.HBconsMonitoredNodes[i], RxTarget::HbConsumer));
  }
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) != 0
  for (uint16_t i = 0; i < OD_CNT_RPDO; ++i) {
    consumers.push_back(rx_is(&co.RPDO[i], RxTarget::Rpdo));
  }
#endif
  consumers.push_back(rx_is(&co.SDOserver[0], RxTarget::Sdo1Server));
#if OD_CNT_SDO_SRV > 1
  consumers.push_back(rx_is(&co.SDOserver[1], RxTarget::Sdo2Server));
#endif
#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) != 0
  consumers.push_back(rx_is(co.LSSslave, RxTarget::LssSlave));
#endif
  for (uint16_t i = 0; i < m->rxSize; ++i) {
    for (const auto& c : consumers) {
      if (c.first != nullptr && m->rxArray[i].object == c.first) {
        rx_target_[i] = c.second;
        break;
      }
    }
  }

  auto tx_is = [](const CO_CANtx_t* b, TxOrigin o) { return std::make_pair(b, static_cast<uint8_t>(o)); };
  std::vector<std::pair<const CO_CANtx_t*, uint8_t>> producers;
  producers.push_back(tx_is(co.NMT->HB_TXbuff, TxOrigin::NmtHeartbeat));
#if ((CO_CONFIG_EM) & CO_CONFIG_EM_PRODUCER) != 0
  producers.push_back(tx_is(co.em->CANtxBuff, TxOrigin::Emcy));
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) != 0
  for (uint16_t i = 0; i < OD_CNT_TPDO; ++i) {
    producers.push_back(tx_is(co.TPDO[i].CANtxBuff, TxOrigin::Tpdo));
  }
#endif
  producers.push_back(tx_is(co.SDOserver[0].CANtxBuff, TxOrigin::Sdo1Response));
#if OD_CNT_SDO_SRV > 1
  producers.push_back(tx_is(co.SDOserver[1].CANtxBuff, TxOrigin::Sdo2Response));
#endif
#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) != 0
  producers.push_back(tx_is(co.LSSslave->TXbuff, TxOrigin::LssSlave));
#endif
#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_PRODUCER) != 0
  producers.push_back(tx_is(co.SYNC->CANtxBuff, TxOrigin::SyncProducer));
#endif
  for (uint16_t i = 0; i < m->txSize; ++i) {
    for (const auto& p : producers) {
      if (p.first == &m->txArray[i]) {
        tx_origin_[i] = p.second;
        break;
      }
    }
  }
  CO_UNLOCK_CAN_SEND(m);
}

contracts::CanFrame FrameRouter::ToContract(const hf_co_frame_t& f) noexcept {
  contracts::CanFrame c;
  c.id = f.id;
  c.len = f.len;
  c.flags = f.flags;
  std::memcpy(c.data, f.data, f.len);
  c.ts_us = f.ts_us;
  return c;
}

bool FrameRouter::RxHook(void* ctx, uint8_t port, uint16_t rx_index, const void* consumer, const hf_co_frame_t* frame) {
  (void)consumer;
  auto* self = static_cast<FrameRouter*>(ctx);
  if (port >= self->slots_.size() || !self->slots_[port].used) {
    return false;
  }
  const auto role = static_cast<Port>(port);
  const uint8_t t = self->TargetOf(rx_index);
  bool allow;
  if (t == kRxOther) {
    allow = !router::IsTunnel(role);
  } else {
    allow = router::DecideRx(role, static_cast<RxTarget>(t), ToContract(*frame), self->slots_[port].caps,
                             self->policy_) == router::Verdict::Allow;
  }
  if (!allow) {
    self->drops_[port].fetch_add(1, std::memory_order_relaxed);
  }
  return allow;
}

bool FrameRouter::TxHook(void* ctx, uint8_t port, uint16_t tx_index, const hf_co_frame_t* frame) {
  auto* self = static_cast<FrameRouter*>(ctx);
  if (port >= self->slots_.size() || !self->slots_[port].used) {
    return false;
  }
  const auto role = static_cast<Port>(port);
  const uint8_t o = self->OriginOf(tx_index);
  bool allow;
  if (o == kTxOther) {
    allow = !router::IsTunnel(role);
  } else {
    // Tunnel mirrors (subscriptions, rate limits) come with the wireless ports.
    allow = router::DecideTx(role, static_cast<TxOrigin>(o), ToContract(*frame), self->slots_[port].caps, false,
                             nullptr, 0U) == router::Verdict::Allow;
  }
  if (!allow) {
    self->drops_[port].fetch_add(1, std::memory_order_relaxed);
  }
  return allow;
}

}  // namespace vortex::node
