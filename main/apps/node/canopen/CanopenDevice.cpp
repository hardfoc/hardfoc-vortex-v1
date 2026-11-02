/**
 * @file CanopenDevice.cpp
 * @brief CoNode + EspTwaiPort + co_svc on the firmware.
 *
 * @details The TWAI RX ISR hands frames to the CO driver's lock-free ring
 *          (EspTwaiPort → hf_co_driver_rx_push) and notifies `co_svc`, which
 *          dispatches them to CANopenNode in task context. Frames go out
 *          through EspCan::TrySendMessage; a full TX queue leaves the frame
 *          pending in its CANopenNode buffer and CO_CANmodule_process()
 *          re-sends it. This replaces the interim runtime's
 *          `retry_pending_tx()` and its RX polling loop.
 */
#include "CanopenDevice.hpp"

#include <sdkconfig.h>

#include <new>

#include "CoNode.hpp"
#include "CoSvc.hpp"
#include "EspTwaiPort.h"
#include "handlers/logger/Logger.h"
#include "mcu/esp32/EspCan.h"

namespace vortex_app {
namespace {

constexpr const char* TAG = "CanopenDevice";

hf::can::EspTwaiPort* g_port = nullptr;
vortex::node::CoNode* g_node = nullptr;
vortex::node::CoSvc* g_svc = nullptr;

void Teardown() noexcept {
  if (g_svc != nullptr) {
    g_svc->Stop();
    delete g_svc;
    g_svc = nullptr;
  }
  if (g_node != nullptr) {
    g_node->Stop();
    delete g_node;
    g_node = nullptr;
  }
  delete g_port;
  g_port = nullptr;
}

}  // namespace

bool StartCanopenDevice(EspCan& can, uint32_t bitrate_bps) noexcept {
  if (g_svc != nullptr) {
    return true;
  }
  vortex::node::CoNodeConfig cfg;
  cfg.node_id = static_cast<uint8_t>(CONFIG_VORTEX_CANOPEN_NODE_ID);
  cfg.bitrate_kbps = static_cast<uint16_t>(bitrate_bps / 1000U);
  // 0x1018:04 / LSS serial: the low 32 bits of the eFuse MAC come with the
  // board-identity objects (design §5.1); 0 until then.
  cfg.serial_number = 0;

  g_port = new (std::nothrow) hf::can::EspTwaiPort(can, bitrate_bps);
  g_node = new (std::nothrow) vortex::node::CoNode(cfg);
  if (g_port == nullptr || g_node == nullptr) {
    Logger::GetInstance().Error(TAG, "out of memory");
    Teardown();
    return false;
  }
  if (!g_node->AddPort(vortex::contracts::router::Port::Can, *g_port, true) || !g_node->Start()) {
    Logger::GetInstance().Error(TAG, "CANopen initialisation failed");
    Teardown();
    return false;
  }
  g_svc = new (std::nothrow) vortex::node::CoSvc(*g_node);
  if (g_svc == nullptr || !g_svc->Start()) {
    Logger::GetInstance().Error(TAG, "co_svc task did not start");
    Teardown();
    return false;
  }
  Logger::GetInstance().Info(TAG, "CANopen device running: node %u, %u kbit/s",
                             static_cast<unsigned>(g_node->ActiveNodeId()),
                             static_cast<unsigned>(g_node->BitRateKbps()));
  return true;
}

void StopCanopenDevice() noexcept { Teardown(); }

bool IsCanopenDeviceRunning() noexcept { return g_svc != nullptr && g_svc->Running(); }

}  // namespace vortex_app
