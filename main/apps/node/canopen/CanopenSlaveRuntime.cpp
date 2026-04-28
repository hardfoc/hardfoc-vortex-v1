/**
 * @file CanopenSlaveRuntime.cpp
 * @brief CANopenNode v4 slave: CO_new + polling RX on BaseCan + CO_process / PDO.
 */
#include "CanopenSlaveRuntime.hpp"

#include "base/BaseCan.h"
#include "handlers/logger/Logger.h"

#include "hf_co_driver_iface.h"

#include <sdkconfig.h>

#include <cinttypes>
#include <cstring>

#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include "CANopen.h"
#include "OD.h"
}

namespace vortex_app {
namespace {

constexpr const char* TAG = "CanopenSlave";

#define NMT_CONTROL                                                                                                    \
  (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
constexpr uint16_t kFirstHbMs = 500U;
constexpr uint16_t kSdoSrvTimeoutMs = 1000U;
constexpr uint16_t kSdoCliTimeoutMs = 500U;

BaseCan* g_bus = nullptr;
CO_t* g_co = nullptr;
TaskHandle_t g_task = nullptr;
volatile bool g_run = false;
volatile bool g_slave_task_finished = false;

extern "C" int co_tx_bridge(uint32_t std_id, uint8_t dlc, const uint8_t* data, void* ctx) {
  auto* bus = static_cast<BaseCan*>(ctx);
  if (bus == nullptr || dlc > 8U) {
    return -1;
  }
  hf_can_message_t m{};
  m.id = std_id;
  m.dlc = dlc;
  m.is_extended = false;
  m.is_rtr = false;
  if (data != nullptr) {
    (void)std::memcpy(m.data, data, dlc);
  }
  const hf_can_err_t e = bus->SendMessage(m, 20U);
  return (e == hf_can_err_t::CAN_SUCCESS) ? 0 : -1;
}

static void canopen_task(void* /*arg*/) {
  Logger::GetInstance().Info(TAG, "CANopen slave task running");
  g_slave_task_finished = false;
  uint32_t last_us = static_cast<uint32_t>(esp_timer_get_time());
  while (g_run && g_co != nullptr && g_bus != nullptr) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t dt_us = now - last_us;
    if (dt_us == 0U) {
      dt_us = 1000U;
    }
    last_us = now;

    hf_can_message_t m{};
    while (g_bus->ReceiveMessage(m, 0U) == hf_can_err_t::CAN_SUCCESS) {
      if (!m.is_extended && !m.is_rtr) {
        const uint16_t id = static_cast<uint16_t>(m.id & 0x7FFU);
        hf_co_driver_process_rx(g_co->CANmodule, id, m.dlc, m.data);
      }
    }

#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) != 0
    const bool_t syncWas = CO_process_SYNC(g_co, dt_us, nullptr);
#else
    const bool_t syncWas = false;
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) != 0
    CO_process_RPDO(g_co, syncWas, dt_us, nullptr);
#endif
    (void)CO_process(g_co, false, dt_us, nullptr);
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) != 0
    CO_process_TPDO(g_co, syncWas, dt_us, nullptr);
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
  }
  Logger::GetInstance().Info(TAG, "CANopen slave task exit");
  g_task = nullptr;
  g_slave_task_finished = true;
  vTaskDelete(nullptr);
}

}  // namespace

bool StartCanopenSlaveRuntime(BaseCan& can) noexcept {
  if (g_task != nullptr) {
    return true;
  }
  g_slave_task_finished = false;
  g_bus = &can;
  hf_co_driver_register_tx(co_tx_bridge, g_bus);

  uint32_t heap_used = 0;
  g_co = CO_new(nullptr, &heap_used);
  if (g_co == nullptr) {
    Logger::GetInstance().Error(TAG, "CO_new failed (heap)");
    g_bus = nullptr;
    return false;
  }
  Logger::GetInstance().Info(TAG, "CANopen heap used: %" PRIu32 " bytes", heap_used);

  void* const can_ptr = nullptr;
  CO_CANsetConfigurationMode(can_ptr);
  CO_CANmodule_disable(g_co->CANmodule);

  constexpr uint16_t kBitrateKbps = 500U;
  CO_ReturnError_t err = CO_CANinit(g_co, can_ptr, kBitrateKbps);
  if (err != CO_ERROR_NO) {
    Logger::GetInstance().Error(TAG, "CO_CANinit failed: %d", static_cast<int>(err));
    CO_delete(g_co);
    g_co = nullptr;
    g_bus = nullptr;
    return false;
  }

  const uint8_t node_id = static_cast<uint8_t>(CONFIG_VORTEX_CANOPEN_NODE_ID);
  uint32_t err_info = 0;
  err = CO_CANopenInit(g_co, nullptr, nullptr, OD, nullptr, NMT_CONTROL, kFirstHbMs, kSdoSrvTimeoutMs, kSdoCliTimeoutMs,
                       false, node_id, &err_info);
  if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
    Logger::GetInstance().Error(TAG, "CO_CANopenInit failed: %d info 0x%" PRIX32, static_cast<int>(err), err_info);
    CO_delete(g_co);
    g_co = nullptr;
    g_bus = nullptr;
    return false;
  }

  err = CO_CANopenInitPDO(g_co, g_co->em, OD, node_id, &err_info);
  if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
    Logger::GetInstance().Error(TAG, "CO_CANopenInitPDO failed: %d info 0x%" PRIX32, static_cast<int>(err), err_info);
    CO_delete(g_co);
    g_co = nullptr;
    g_bus = nullptr;
    return false;
  }

  CO_CANsetNormalMode(g_co->CANmodule);

  g_run = true;
  const BaseType_t ok =
      xTaskCreatePinnedToCore(canopen_task, "canopen_slave", 8192, nullptr, 5, &g_task, tskNO_AFFINITY);
  if (ok != pdPASS) {
    g_run = false;
    Logger::GetInstance().Error(TAG, "xTaskCreatePinnedToCore failed");
    CO_delete(g_co);
    g_co = nullptr;
    g_bus = nullptr;
    g_task = nullptr;
    return false;
  }
  return true;
}

void StopCanopenSlaveRuntime() noexcept {
  g_run = false;
  for (int i = 0; i < 250 && !g_slave_task_finished && g_task != nullptr; ++i) {
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (g_co != nullptr) {
    CO_CANmodule_disable(g_co->CANmodule);
    CO_delete(g_co);
    g_co = nullptr;
  }
  g_bus = nullptr;
  hf_co_driver_register_tx(nullptr, nullptr);
  g_slave_task_finished = false;
}

bool IsCanopenSlaveRuntimeRunning() noexcept { return g_task != nullptr && g_run; }

}  // namespace vortex_app
