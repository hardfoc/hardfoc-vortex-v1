/**
 * @file CoNode.cpp
 * @brief CANopenNode instance over the generated OD (see CoNode.hpp).
 */
#include "CoNode.hpp"

#include <cstring>

extern "C" {
#include "CANopen.h"
#include "OD.h"
}

namespace vortex::node {
namespace {

namespace router = contracts::router;

// Error register bits that take the node out of Operational (CiA 301 0x1029:01 = 0).
constexpr uint16_t kNmtControlBase = CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION;
// 0x1F80 bit 2: start Operational without an NMT master (stand-alone demos).
constexpr uint32_t kNmtStartupSelfStart = 0x04U;
constexpr uint32_t kDiagPeriodUs = 100000U;

// OD defaults, captured before the stack first touches the OD, for the host's NMT reset node.
struct OdDefaults {
  bool captured{false};
  OD_RAM_t ram;
  OD_PERSIST_COMM_t comm;
  OD_PERSIST_APP_t app;
};
OdDefaults& Defaults() {
  static OdDefaults d;
  return d;
}

// SDO server 2 (0x1201) COB-ID bases: tunnel-only virtual IDs, client→server
// 0x6C0+id and server→client 0x5C0+id (§4.3). CANopenNode adds the node-ID
// only to SDO server 1, so CoNode applies it, as the EDS's $NODEID does.
constexpr uint32_t kSdo2RxBase = router::kCobSdo2RxBase;
constexpr uint32_t kSdo2TxBase = router::kCobSdo2TxBase;

// cob = base + previous id (or the bare base): move it to base + id; keep anything else.
uint32_t WithNodeId(uint32_t cob, uint32_t base, uint8_t previous, uint8_t id) {
  const uint32_t can_id = cob & 0x7FFU;
  if (can_id == base || can_id == base + previous) {
    return (cob & ~0x7FFU) | (base + id);
  }
  return cob;
}

bool IsCia305BitRate(uint16_t kbps) {
  switch (kbps) {
    case 1000: case 800: case 500: case 250: case 125: case 50: case 20: case 10:
      return true;
    default:
      return false;
  }
}

}  // namespace

CoNode::CoNode(const CoNodeConfig& config) noexcept : cfg_(config) {
  drv_ = hf_co_driver_acquire();
  pending_node_id_ = cfg_.node_id;
  pending_bitrate_ = cfg_.bitrate_kbps;
  OdDefaults& d = Defaults();
  if (!d.captured) {
    std::memcpy(&d.ram, &OD_RAM, sizeof(OD_RAM));
    std::memcpy(&d.comm, &OD_PERSIST_COMM, sizeof(OD_PERSIST_COMM));
    std::memcpy(&d.app, &OD_PERSIST_APP, sizeof(OD_PERSIST_APP));
    d.captured = true;
  }
}

CoNode::~CoNode() {
  Stop();
  if (drv_ != nullptr) {
    hf_co_driver_release(drv_);
    drv_ = nullptr;
  }
}

bool CoNode::AddPort(router::Port role, hf::can::ICanPort& port, bool error_source) noexcept {
  const auto slot = static_cast<uint8_t>(role);
  if (drv_ == nullptr || co_ != nullptr || slot >= router::kPortCount) {
    return false;
  }
  const hf::can::PortCaps hc = port.Caps();
  contracts::PortCaps caps;
  caps.fd = hc.fd;
  caps.max_len = hc.max_len;
  caps.nominal_bps = hc.nominal_bps;
  caps.data_bps = hc.data_bps;
  caps.hw_filters = hc.hw_filters;
  router_.SetPort(role, caps);
  return bindings_[slot].Attach(drv_, slot, port, error_source);
}

void CoNode::SetWake(hf_co_wake_fn wake, void* ctx) noexcept { hf_co_driver_set_wake(drv_, wake, ctx); }

bool CoNode::Start() noexcept {
  if (drv_ == nullptr || co_ != nullptr) {
    return false;
  }
  router_.Install(drv_);
  uint32_t heap = 0;
  co_ = CO_new(nullptr, &heap);
  if (co_ == nullptr) {
    return false;
  }
  if (lss_store_ != nullptr) {
    uint8_t id = 0;
    uint16_t kbps = 0;
    if (lss_store_->Load(id, kbps)) {
      if ((id >= 1U && id <= 127U) || id == CO_LSS_NODE_ID_ASSIGNMENT) {
        pending_node_id_ = id;
      }
      if (IsCia305BitRate(kbps)) {
        pending_bitrate_ = kbps;
      }
    }
  }
  if (!CommunicationReset()) {
    CO_delete(co_);
    co_ = nullptr;
    return false;
  }
  return true;
}

void CoNode::Stop() noexcept {
  for (auto& b : bindings_) {
    b.Detach();
  }
  if (co_ != nullptr) {
    CO_CANsetConfigurationMode(drv_);
    CO_CANmodule_disable(co_->CANmodule);
    CO_delete(co_);
    co_ = nullptr;
  }
}

bool CoNode::CommunicationReset() noexcept {
  co_->CANmodule->CANnormal = false;
  CO_CANsetConfigurationMode(drv_);
  CO_CANmodule_disable(co_->CANmodule);
  if (CO_CANinit(co_, drv_, pending_bitrate_) != CO_ERROR_NO) {
    return false;
  }

  OD_RAM.x1018_identity.serialNumber = cfg_.serial_number;
  CO_LSS_address_t lss{};
  lss.identity.vendorID = OD_RAM.x1018_identity.vendor_ID;
  lss.identity.productCode = OD_RAM.x1018_identity.productCode;
  lss.identity.revisionNumber = OD_RAM.x1018_identity.revisionNumber;
  lss.identity.serialNumber = OD_RAM.x1018_identity.serialNumber;
  if (CO_LSSinit(co_, &lss, &pending_node_id_, &pending_bitrate_) != CO_ERROR_NO) {
    return false;
  }
  active_node_id_ = pending_node_id_;

  if (active_node_id_ >= 1U && active_node_id_ <= 127U) {
    auto& p = OD_PERSIST_COMM.x1201_SDOServerParameter2;
    p.COB_IDClientToServerRx = WithNodeId(p.COB_IDClientToServerRx, kSdo2RxBase, sdo2_node_id_, active_node_id_);
    p.COB_IDServerToClientTx = WithNodeId(p.COB_IDServerToClientTx, kSdo2TxBase, sdo2_node_id_, active_node_id_);
    sdo2_node_id_ = active_node_id_;
  }

  uint16_t nmt_control = kNmtControlBase;
  if ((OD_PERSIST_COMM.x1F80_NMTStartup & kNmtStartupSelfStart) != 0U) {
    nmt_control |= CO_NMT_STARTUP_TO_OPERATIONAL;
  }
  uint32_t err_info = 0;
  CO_ReturnError_t err =
      CO_CANopenInit(co_, nullptr, nullptr, OD, nullptr, nmt_control, cfg_.first_hb_ms, cfg_.sdo_server_timeout_ms,
                     cfg_.sdo_client_timeout_ms, false, active_node_id_, &err_info);
  if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
    return false;
  }
  err = CO_CANopenInitPDO(co_, co_->em, OD, active_node_id_, &err_info);
  if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
    return false;
  }
  CO_LSSslave_initCfgStoreCall(co_->LSSslave, this, &CoNode::LssStore);
  CO_LSSslave_initCkBitRateCall(co_->LSSslave, this, &CoNode::LssCheckBitRate);
  CO_LSSslave_initActBitRateCall(co_->LSSslave, this, &CoNode::LssActivateBitRate);

  router_.Classify(*co_);
  CO_CANsetNormalMode(co_->CANmodule);
  return true;
}

CoNode::Step CoNode::Process(uint32_t dt_us, uint32_t& next_us) noexcept {
  if (co_ == nullptr) {
    return Step::Quit;
  }
  uint32_t timer_next = 10000U;
  const CO_NMT_reset_cmd_t reset = CO_process(co_, false, dt_us, &timer_next);

  CO_LOCK_OD(co_->CANmodule);
  if (!co_->nodeIdUnconfigured && co_->CANmodule->CANnormal) {
    bool_t sync_was = false;
#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) != 0
    sync_was = CO_process_SYNC(co_, dt_us, &timer_next);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) != 0
    CO_process_RPDO(co_, sync_was, dt_us, &timer_next);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) != 0
    CO_process_TPDO(co_, sync_was, dt_us, &timer_next);
#endif
    (void)sync_was;
  }
  CO_UNLOCK_OD(co_->CANmodule);

  diag_elapsed_us_ += dt_us;
  if (diag_elapsed_us_ >= kDiagPeriodUs) {
    diag_elapsed_us_ = 0;
    UpdateDiagnostics();
  }

  // Frames still pending on a port are retried from CO_CANmodule_process().
  if (hf_co_driver_tx_pending(drv_) != 0U && timer_next > 1000U) {
    timer_next = 1000U;
  }
  next_us = timer_next;

  switch (reset) {
    case CO_RESET_COMM:
      return CommunicationReset() ? Step::Run : Step::Quit;
    case CO_RESET_APP:
      return Step::ResetApp;
    case CO_RESET_QUIT:
      return Step::Quit;
    default:
      return Step::Run;
  }
}

bool CoNode::ResetApplication() noexcept {
  if (co_ == nullptr) {
    return false;
  }
  const OdDefaults& d = Defaults();
  CO_LOCK_OD(co_->CANmodule);
  std::memcpy(&OD_RAM, &d.ram, sizeof(OD_RAM));
  std::memcpy(&OD_PERSIST_COMM, &d.comm, sizeof(OD_PERSIST_COMM));
  std::memcpy(&OD_PERSIST_APP, &d.app, sizeof(OD_PERSIST_APP));
  CO_UNLOCK_OD(co_->CANmodule);
  sdo2_node_id_ = 0;
  // A power cycle forgets an LSS ID that was configured but not stored.
  pending_node_id_ = cfg_.node_id;
  pending_bitrate_ = cfg_.bitrate_kbps;
  if (lss_store_ != nullptr) {
    uint8_t id = 0;
    uint16_t kbps = 0;
    if (lss_store_->Load(id, kbps)) {
      if ((id >= 1U && id <= 127U) || id == CO_LSS_NODE_ID_ASSIGNMENT) {
        pending_node_id_ = id;
      }
      if (IsCia305BitRate(kbps)) {
        pending_bitrate_ = kbps;
      }
    }
  }
  return CommunicationReset();
}

void CoNode::UpdateDiagnostics() noexcept {
  // 0x4202 CAN statistics: the physical port (CAN), else the host sim port.
  const hf::can::ICanPort* port = bindings_[static_cast<uint8_t>(router::Port::Can)].Port();
  uint8_t slot = static_cast<uint8_t>(router::Port::Can);
  if (port == nullptr) {
    slot = static_cast<uint8_t>(router::Port::HostSim);
    port = bindings_[slot].Port();
  }
  hf::can::PortStatus st;
  if (port != nullptr) {
    st = port->Status();
  }
  hf_co_port_stats_t ds;
  hf_co_driver_get_stats(drv_, slot, &ds);

  CO_LOCK_OD(co_->CANmodule);
  auto& s = OD_RAM.x4202_CANStats;
  s.TEC = st.tec;
  s.REC = st.rec;
  s.busState = static_cast<uint8_t>(st.state);
  s.bus_offCount = st.bus_off_count;
  s.RXOverruns = st.rx_overruns + ds.rx_ring_overruns;
  s.TXQueueFull = st.tx_queue_full;
  s.routerDropsCAN = router_.Drops(router::Port::Can);
  s.routerDropsWi_Fi = router_.Drops(router::Port::WifiTunnel);
  s.routerDropsESP_NOW = router_.Drops(router::Port::EspNowTunnel);
  s.routerDropsHost = router_.Drops(router::Port::HostSim);
  CO_UNLOCK_OD(co_->CANmodule);
}

bool_t CoNode::LssStore(void* object, uint8_t id, uint16_t bit_rate) {
  auto* self = static_cast<CoNode*>(object);
  return (self->lss_store_ != nullptr && self->lss_store_->Store(id, bit_rate)) ? 1 : 0;
}

bool_t CoNode::LssCheckBitRate(void* object, uint16_t bit_rate) {
  (void)object;
  return IsCia305BitRate(bit_rate) ? 1 : 0;
}

void CoNode::LssActivateBitRate(void* object, uint16_t delay_ms) {
  // The new bit rate (pending_bitrate_) applies at the next communication
  // reset; re-initialising the controller after the switch delay is step 7.
  (void)object;
  (void)delay_ms;
}

}  // namespace vortex::node
