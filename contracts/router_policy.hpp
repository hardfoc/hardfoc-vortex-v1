#pragma once
/**
 * @file router_policy.hpp
 * @brief FrameRouter per-port policy and wireless authority rules
 *        (design §4.3, §7.4).
 *
 * @details One CANopenNode instance serves N ports. The router decides, per
 *          frame, whether a received frame may reach a stack consumer and
 *          whether a stack producer's frame may leave on a port.
 *
 *          Decisions are keyed on the *stack object* that consumes or
 *          produced the frame (`RxTarget`, `TxOrigin`), not on the raw
 *          COB-ID. The driver knows both: the RX buffer that matched and the
 *          TX buffer that was sent. This matters because the tunnel-only
 *          SDO server 2 COB-IDs (client→server 0x6C0+id, server→client
 *          0x5C0+id) are not free on a physical bus: 0x6C0+id is the
 *          heartbeat COB-ID of node id−0x40 for node-IDs ≥ 0x41, and 0x5C0+id
 *          is an SDO-server COB-ID of another node for every node-ID. A
 *          COB-ID drop rule on the CAN port would therefore discard real
 *          heartbeats. Instead a CAN frame is never classified as an SDO
 *          server 2 request (`PredefinedTarget`), so a CAN node still cannot
 *          pose as the wireless client.
 *
 *          | Port | RX accepted | TX sent |
 *          |---|---|---|
 *          | CAN (0) | every consumer except SDO server 2 | everything except SDO server 2 responses |
 *          | Wi-Fi / ESP-NOW tunnel | SDO server 2; NMT pre-op / reset node; other NMT and RPDO only if the port may enable (0x2701:01) | SDO server 2 responses; subscribed HB / EMCY / TPDO mirrors, rate-limited (default 100 Hz each) |
 *          | Host sim (3) | as CAN | as CAN |
 *
 *          Pure and HAL-free (host test: tests/host/test_router_policy.cpp).
 */
#include <cstdint>

#include "contracts/can_frame.hpp"

namespace vortex::contracts::router {

/** @brief Ports under the one CO_CANmodule. */
enum class Port : uint8_t {
  Can = 0,           ///< physical bus (EspTwaiPort, later StmFdcanPort)
  WifiTunnel = 1,    ///< TLS tunnel
  EspNowTunnel = 2,  ///< ESP-NOW tunnel
  HostSim = 3,       ///< LinuxSocketCanPort / PipePort (CI)
};
inline constexpr uint8_t kPortCount = 4U;

constexpr bool IsTunnel(Port p) noexcept { return p == Port::WifiTunnel || p == Port::EspNowTunnel; }

/** @brief 0x2701:01 bit of @p p (bit 0 CAN, bit 1 Wi-Fi, bit 2 ESP-NOW; host sim counts as CAN). */
constexpr uint8_t PortBit(Port p) noexcept {
  return p == Port::HostSim ? static_cast<uint8_t>(0x01U) : static_cast<uint8_t>(1U << static_cast<uint8_t>(p));
}

/** @brief 0x2700:01 owner code of @p p (0 none, 1 CAN, 2 Wi-Fi, 3 ESP-NOW). */
constexpr uint8_t OwnerCode(Port p) noexcept {
  return p == Port::HostSim ? static_cast<uint8_t>(1U) : static_cast<uint8_t>(static_cast<uint8_t>(p) + 1U);
}

/** @brief Authority policy (0x2701). */
struct AuthorityPolicy {
  uint8_t enable_ports{0x01U};       ///< 0x2701:01 ports allowed to enable motion (default CAN only)
  uint16_t wireless_deadman_ms{200}; ///< 0x2701:02
  bool require_master_liveness{true};///< 0x2701:03
  bool wireless_service{false};      ///< 0x2701:04 wireless may write service objects (§7.4 rule 5)
};

/** @brief Stack object a received frame would be delivered to. */
enum class RxTarget : uint8_t {
  Unhandled = 0,  ///< no consumer: dropped on every port
  Nmt,
  Sync,
  Time,
  EmcyConsumer,
  HbConsumer,
  Rpdo,
  Sdo1Server,
  Sdo2Server,
  LssSlave,
};
inline constexpr uint8_t kRxTargetCount = 10U;

/** @brief Stack object that produced a frame. */
enum class TxOrigin : uint8_t {
  NmtHeartbeat = 0,  ///< boot-up and heartbeat producer
  Emcy,
  Tpdo,
  Sdo1Response,
  Sdo2Response,
  SdoClient,         ///< local SDO client (309-3 gateway): never on a wire
  LssSlave,
  SyncProducer,
};
inline constexpr uint8_t kTxOriginCount = 8U;

/** Predefined connection set (CiA 301) and the tunnel-only SDO server 2 IDs (§4.3). */
inline constexpr uint32_t kCobNmt = 0x000U;
inline constexpr uint32_t kCobSync = 0x080U;
inline constexpr uint32_t kCobEmcyBase = 0x080U;
inline constexpr uint32_t kCobTime = 0x100U;
inline constexpr uint32_t kCobSdoTxBase = 0x580U;
inline constexpr uint32_t kCobSdoRxBase = 0x600U;
inline constexpr uint32_t kCobSdo2RxBase = 0x6C0U;  ///< client→server, tunnel only
inline constexpr uint32_t kCobSdo2TxBase = 0x5C0U;  ///< server→client, tunnel only
inline constexpr uint32_t kCobHeartbeatBase = 0x700U;
inline constexpr uint32_t kCobLssMasterToSlave = 0x7E5U;
inline constexpr uint32_t kCobLssSlaveToMaster = 0x7E4U;

/** NMT command specifiers (CiA 301). */
namespace nmt {
inline constexpr uint8_t kStart = 0x01U;
inline constexpr uint8_t kStop = 0x02U;
inline constexpr uint8_t kEnterPreOperational = 0x80U;
inline constexpr uint8_t kResetNode = 0x81U;
inline constexpr uint8_t kResetCommunication = 0x82U;
}  // namespace nmt

/**
 * @brief Consumer of @p f under the predefined connection set.
 * @details Used by tests and tools; the firmware driver uses the RX buffer
 *          that matched (runtime-configured COB-IDs). The SDO server 2 ID is
 *          recognised only on tunnel ports; on CAN the same ID keeps its
 *          physical meaning (heartbeat, SDO of another node).
 */
constexpr RxTarget PredefinedTarget(Port p, const CanFrame& f, uint8_t node_id) noexcept {
  if (f.IsExtended() || f.IsRemote()) return RxTarget::Unhandled;
  const uint32_t id = f.id;
  if (IsTunnel(p) && id == kCobSdo2RxBase + node_id) return RxTarget::Sdo2Server;
  if (id == kCobNmt) return RxTarget::Nmt;
  if (id == kCobSync) return RxTarget::Sync;
  if (id > kCobEmcyBase && id <= kCobEmcyBase + 0x7FU) return RxTarget::EmcyConsumer;
  if (id == kCobTime) return RxTarget::Time;
  if (id == 0x200U + node_id || id == 0x300U + node_id || id == 0x400U + node_id || id == 0x500U + node_id) {
    return RxTarget::Rpdo;
  }
  if (id == kCobSdoRxBase + node_id) return RxTarget::Sdo1Server;
  if (id > kCobHeartbeatBase && id <= kCobHeartbeatBase + 0x7FU && id != kCobHeartbeatBase + node_id) {
    return RxTarget::HbConsumer;
  }
  if (id == kCobLssMasterToSlave) return RxTarget::LssSlave;
  return RxTarget::Unhandled;
}

/** @brief Router verdict. */
enum class Verdict : uint8_t {
  Allow = 0,
  DenyPolicy,         ///< the port may not use this consumer / producer
  DenyLength,         ///< malformed, or longer than the port carries (Caps().max_len)
  DenyNotSubscribed,  ///< a mirror the tunnel did not subscribe to
  DenyRateLimited,    ///< a mirror above its per-port rate
  DenyUnhandled,      ///< no consumer
};

/** @brief May a tunnel send this NMT command? Pre-op / reset node always; start, stop, reset comm need policy. */
constexpr bool TunnelNmtAllowed(Port p, uint8_t command, const AuthorityPolicy& pol) noexcept {
  if (command == nmt::kEnterPreOperational || command == nmt::kResetNode) return true;
  if (command == nmt::kStart || command == nmt::kStop || command == nmt::kResetCommunication) {
    return (pol.enable_ports & PortBit(p)) != 0U;
  }
  return false;  // unknown specifier
}

/**
 * @brief RX decision.
 * @param p Port the frame arrived on.
 * @param target Consumer it would reach (RX buffer that matched, or PredefinedTarget).
 * @param f The frame.
 * @param caps The arrival port's capabilities.
 * @param pol 0x2701.
 */
constexpr Verdict DecideRx(Port p, RxTarget target, const CanFrame& f, const PortCaps& caps,
                           const AuthorityPolicy& pol) noexcept {
  if (!FitsPort(f, caps)) return Verdict::DenyLength;
  if (target == RxTarget::Unhandled) return Verdict::DenyUnhandled;
  if (!IsTunnel(p)) return target == RxTarget::Sdo2Server ? Verdict::DenyPolicy : Verdict::Allow;
  switch (target) {
    case RxTarget::Sdo2Server: return Verdict::Allow;
    case RxTarget::Nmt:
      return (f.len >= 1U && TunnelNmtAllowed(p, f.data[0], pol)) ? Verdict::Allow : Verdict::DenyPolicy;
    case RxTarget::Rpdo: return (pol.enable_ports & PortBit(p)) != 0U ? Verdict::Allow : Verdict::DenyPolicy;
    default: return Verdict::DenyPolicy;
  }
}

/** @brief Minimum interval between mirrored frames of one kind on one port. */
inline constexpr uint32_t kDefaultMirrorIntervalUs = 10000U;  ///< 100 Hz

/** @brief Interval rate limiter for one (port, mirror) pair. Wrap-safe on a u32 µs clock. */
struct RateLimiter {
  uint32_t min_interval_us{kDefaultMirrorIntervalUs};
  uint32_t last_us{0};
  bool primed{false};

  constexpr bool Allow(uint32_t now_us) noexcept {
    if (primed && (now_us - last_us) < min_interval_us) return false;
    primed = true;
    last_us = now_us;
    return true;
  }
};

/**
 * @brief TX decision.
 * @param p Destination port.
 * @param origin Producer of the frame.
 * @param f The frame.
 * @param caps Destination capabilities.
 * @param subscribed Tunnel only: the peer subscribed to this mirror (HB / EMCY / this TPDO).
 * @param limiter Tunnel only: the (port, mirror) limiter, or nullptr for no limit.
 * @param now_us Clock for the limiter.
 */
constexpr Verdict DecideTx(Port p, TxOrigin origin, const CanFrame& f, const PortCaps& caps, bool subscribed,
                           RateLimiter* limiter, uint32_t now_us) noexcept {
  if (!FitsPort(f, caps)) return Verdict::DenyLength;
  if (origin == TxOrigin::SdoClient) return Verdict::DenyPolicy;
  if (!IsTunnel(p)) return origin == TxOrigin::Sdo2Response ? Verdict::DenyPolicy : Verdict::Allow;
  switch (origin) {
    case TxOrigin::Sdo2Response: return Verdict::Allow;
    case TxOrigin::NmtHeartbeat:
    case TxOrigin::Emcy:
    case TxOrigin::Tpdo:
      if (!subscribed) return Verdict::DenyNotSubscribed;
      if (limiter != nullptr && !limiter->Allow(now_us)) return Verdict::DenyRateLimited;
      return Verdict::Allow;
    default: return Verdict::DenyPolicy;
  }
}

// ---------------------------------------------------------------------------
// Authority rules (§7.4)
// ---------------------------------------------------------------------------

/** @brief Objects a tunnel may write only with 0x2701:04 "wireless service" (§7.4 rule 5). */
constexpr bool IsServiceObject(uint16_t index) noexcept {
  return index == 0x1010U || index == 0x4100U || index == 0x2601U;
}

/** @brief May an SDO write from @p p to @p index proceed (router-level rule; access checks follow)? */
constexpr bool SdoWriteAllowed(Port p, uint16_t index, const AuthorityPolicy& pol) noexcept {
  if (!IsTunnel(p)) return true;
  return !IsServiceObject(index) || pol.wireless_service;
}

/** @brief May @p p request control authority (0x2700:02)? CAN always; a tunnel only if 0x2701:01 lets it enable. */
constexpr bool MayRequestAuthority(Port p, const AuthorityPolicy& pol) noexcept {
  return !IsTunnel(p) || (pol.enable_ports & PortBit(p)) != 0U;
}

/**
 * @brief Authority arbitration (§7.4 rule 3).
 * @param owner Current owner code (0 none).
 * @param requester Requesting port.
 * @param drive_function_off The axis is in SwitchedOn or below.
 * @return New owner code. CAN always wins (it can revoke); a tunnel gets
 *         authority only when free, allowed by policy, and with the drive off.
 */
constexpr uint8_t ArbitrateAuthority(uint8_t owner, Port requester, bool drive_function_off,
                                     const AuthorityPolicy& pol) noexcept {
  const uint8_t req = OwnerCode(requester);
  if (owner == req) return owner;
  if (!IsTunnel(requester)) return req;  // CAN wins
  if (!drive_function_off || !MayRequestAuthority(requester, pol)) return owner;
  return owner == 0U ? req : owner;
}

}  // namespace vortex::contracts::router
