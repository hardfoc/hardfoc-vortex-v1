#pragma once
/**
 * @file CoNode.hpp
 * @brief The Vortex CANopen device: one CANopenNode instance over the
 *        generated object dictionary, its ports and the frame router
 *        (design §4.3, §10.1 "CanopenSlaveRuntime → CoNode").
 *
 * @details Platform-neutral: the firmware (EspTwaiPort, FreeRTOS `co_svc`
 *          task) and the Linux host binary (LinuxSocketCanPort / PipePort,
 *          a std::thread `co_svc`) run this same class over the same OD.c.
 *
 *          Life cycle:
 *          1. construct with a CoNodeConfig, AddPort() each transport;
 *          2. Start(): CO_new and the first communication reset
 *             (CO_CANinit → CO_LSSinit → CO_CANopenInit → CO_CANopenInitPDO,
 *             tolerating CO_ERROR_NODE_ID_UNCONFIGURED_LSS, as the interim
 *             runtime did);
 *          3. the service thread calls Process() whenever it is woken (a
 *             frame arrived, see SetWake()) or the time CO_process()
 *             reported has passed. Process() handles NMT reset
 *             communication itself and returns ResetApp / Quit for the
 *             thread to act on.
 *
 *          Not here yet (later design steps): 0x1010/0x1011 storage and the
 *          NVS LSS record (step 7), the CiA 402 axis and `drive_rt`
 *          (step 8), the fault funnel (step 9), the wireless ports.
 */
#include <cstdint>

#include "FrameRouter.hpp"
#include "HfCanPort.h"
#include "HfCoPortBinding.h"
#include "contracts/router_policy.hpp"
#include "hf_co_driver.h"

namespace vortex::node {

/** @brief Where the LSS-configured node-ID and bit rate persist (step 7: NVS record `lss`). */
class ILssStore {
 public:
  virtual ~ILssStore() = default;
  /** @brief Stored values, or false when nothing is stored. */
  virtual bool Load(uint8_t& node_id, uint16_t& bitrate_kbps) = 0;
  /** @brief Persist the values (LSS "store configuration"). */
  virtual bool Store(uint8_t node_id, uint16_t bitrate_kbps) = 0;
};

struct CoNodeConfig {
  uint8_t node_id{1};               ///< 1..127, or 0xFF to start LSS-unconfigured
  uint16_t bitrate_kbps{500};       ///< CiA 305 bit rate the port runs at
  uint32_t serial_number{0};        ///< 0x1018:04, the LSS address serial
  uint16_t first_hb_ms{500};
  uint16_t sdo_server_timeout_ms{1000};
  uint16_t sdo_client_timeout_ms{500};
};

class CoNode {
 public:
  enum class Step : uint8_t {
    Run = 0,   ///< keep calling Process()
    ResetApp,  ///< NMT reset node: restart the device (firmware) or ResetApplication() (host)
    Quit,      ///< stop the service thread
  };

  explicit CoNode(const CoNodeConfig& config) noexcept;
  ~CoNode();
  CoNode(const CoNode&) = delete;
  CoNode& operator=(const CoNode&) = delete;

  /** @brief Attach @p port in the router slot of @p role (before Start()). */
  bool AddPort(contracts::router::Port role, hf::can::ICanPort& port, bool error_source) noexcept;
  /** @brief Optional LSS persistence (before Start()). */
  void SetLssStore(ILssStore* store) noexcept { lss_store_ = store; }
  /** @brief Wake hook for the service thread; ISR-safe (called on every RX frame and TX-done). */
  void SetWake(hf_co_wake_fn wake, void* ctx) noexcept;

  /** @brief Allocate the stack and run the first communication reset. */
  bool Start() noexcept;
  /** @brief Detach the ports and free the stack. */
  void Stop() noexcept;

  /**
   * @brief One service pass: CO_process (RX dispatch, NMT, HB, EMCY, SDO, LSS),
   *        then SYNC / RPDO / TPDO under the OD lock.
   * @param dt_us time since the previous call
   * @param next_us out: longest time until the next call is needed
   */
  Step Process(uint32_t dt_us, uint32_t& next_us) noexcept;

  /** @brief Restore the OD defaults and reset communication (host emulation of NMT reset node). */
  bool ResetApplication() noexcept;

  bool Running() const noexcept { return co_ != nullptr; }
  uint8_t ActiveNodeId() const noexcept { return active_node_id_; }
  uint16_t BitRateKbps() const noexcept { return pending_bitrate_; }
  CO_t* Co() noexcept { return co_; }
  FrameRouter& Router() noexcept { return router_; }
  hf_co_driver_t* Driver() noexcept { return drv_; }

 private:
  bool CommunicationReset() noexcept;
  void UpdateDiagnostics() noexcept;
  static bool_t LssStore(void* object, uint8_t id, uint16_t bit_rate);
  static bool_t LssCheckBitRate(void* object, uint16_t bit_rate);
  static void LssActivateBitRate(void* object, uint16_t delay_ms);

  CoNodeConfig cfg_;
  CO_t* co_{nullptr};
  hf_co_driver_t* drv_{nullptr};
  FrameRouter router_;
  hf::can::CoPortBinding bindings_[contracts::router::kPortCount];
  ILssStore* lss_store_{nullptr};
  uint8_t pending_node_id_{0};
  uint8_t active_node_id_{0};
  uint16_t pending_bitrate_{0};
  uint8_t sdo2_node_id_{0};  ///< node-ID last applied to the 0x1201 COB-IDs
  uint32_t diag_elapsed_us_{0};
};

}  // namespace vortex::node
