#pragma once
/**
 * @file vortex_fault_policy.hpp
 * @brief Fault policy table: cause → EMCY code, error register, reaction
 *        (design §6.4), plus the escalation rule.
 *
 * @details One row per cause. The `SafetyArbiter` looks a cause up here and
 *          does three things from the same row: `CO_errorReport` (EMCY code,
 *          0x1001 bits; EMCY bytes 3–7 carry {cause id, raw flags u32}), the
 *          CiA 402 reaction, and the journal entry. Where the design table
 *          names two sources or two codes in one row, the row is split into
 *          one cause per code so that every cause maps to exactly one EMCY
 *          code (R8).
 *
 *          Escalation follows `pw_shared/system/fault_state.hpp`: the same
 *          cause raised `kEscalateCount` times within `kEscalateWindowMs`
 *          becomes escalated. A non-escalated cause clears on fault reset
 *          once its condition is gone; an escalated cause additionally needs
 *          its owner's `Cleared` predicate. Fault reset (CiA 402 transition
 *          15) succeeds only when no latched cause is left.
 *
 *          Pure and HAL-free (host test: tests/host/test_fault_policy.cpp).
 */
#include <cstdint>

namespace vortex::contracts::fault {

/** @brief 0x1001 error register bits (CiA 301). */
namespace er {
inline constexpr uint8_t kGeneric = 0x01U;
inline constexpr uint8_t kCurrent = 0x02U;
inline constexpr uint8_t kVoltage = 0x04U;
inline constexpr uint8_t kTemperature = 0x08U;
inline constexpr uint8_t kCommunication = 0x10U;
inline constexpr uint8_t kProfile = 0x20U;
inline constexpr uint8_t kManufacturer = 0x80U;
}  // namespace er

/** @brief What the arbiter does for a cause (design §6.4 "Class → reaction"). */
enum class Reaction : uint8_t {
  Warn = 0,         ///< statusword bit 7 + EMCY; no state change; not latched
  QuickStop,        ///< inject the quick stop command (0x605A ramp)
  AbortConnection,  ///< apply 0x6007 (default: fault)
  RefuseSwitchOn,   ///< block transition 3 until the parameters are valid
  Fault,            ///< CiA 402 fault: reaction per 0x605E
  DisableNow,       ///< bridge off at once (TMC SYSTEM_OFF, DRV_EN low), then Fault; 0x605E downgraded to coast
  Reset,            ///< journal first, bridge off, then reset the MCU
};

/** @brief Conditions that modify a row's reaction. */
namespace cond {
inline constexpr uint8_t kNone = 0x00U;
/** Fault only if the sensor is the active position source, else Warn. */
inline constexpr uint8_t kWarnUnlessPositionSource = 0x01U;
/** Warn, but 0x6007 reaction if it happened in OperationEnabled. */
inline constexpr uint8_t kAbortIfOperationEnabled = 0x02U;
/** 0x6007 reaction only in cyclic modes (csp/csv/cst), else Warn. */
inline constexpr uint8_t kAbortOnlyCyclic = 0x04U;
}  // namespace cond

/** @brief Fault causes. The numeric value is the cause id sent in EMCY byte 3 and stored in the journal. */
enum class Cause : uint8_t {
  GateShortCircuit = 0,
  GateOvercurrent,
  I2tLimit1,
  I2tLimit2,
  SupplyOvervoltage,
  SupplyUndervoltage,
  PowerGoodLost,
  SupplyVoltageWarning,
  ChipOvertemperature,
  ChipTemperatureWarning,
  NtcOvertemperature,
  HallInvalid,
  HostEncoderFault,
  ImuLost,
  TmcCommsFault,
  TmcHeartbeatStopped,
  FollowingError,
  SoftwareLimit,
  CanRxOverflow,
  CanErrorPassive,
  HeartbeatConsumerLoss,
  BusOffRecovered,
  PdoLengthError,
  PdoLengthExceeded,
  SyncLengthError,
  SyncTimeout,
  RpdoTimeout,
  ParameterInvalid,
  StoreCrcMismatch,
  RtStall,
  SvcStall,
  COUNT
};
inline constexpr uint8_t kCauseCount = static_cast<uint8_t>(Cause::COUNT);
static_assert(kCauseCount <= 32U, "0x4003 carries cause masks as U32");

/** @brief Where a cause is detected (documentation and journal decoding). */
enum class Source : uint8_t { TmcGateDriver, TmcGeneral, TmcSupply, PcalPowerGood, TemperatureManager, HostEncoder, Imu, DriveRt, Host, CanPort, CoHbConsumer, CoPdo, CoSync, Storage, LivenessMonitor };

/** @brief One row of the policy table. */
struct Policy {
  Cause cause;
  const char* name;
  Source source;
  uint16_t emcy;      ///< CiA 301 / 402 EMCY error code
  uint8_t er;         ///< 0x1001 bits set while the cause is active
  Reaction reaction;
  uint8_t conditions; ///< cond bits
};

/**
 * @brief The policy table (design §6.4). Index = cause id.
 *
 * | Cause | Source | EMCY | ER bits | Reaction |
 * |---|---|---|---|---|
 * | gate short circuit | TMC gate driver | 0x2320 | 0,1 | disable now |
 * | gate overcurrent (phase / low / high side) | TMC gate driver | 0x2310 | 0,1 | disable now |
 * | I²t limit 1 / 2 | TMC general flags 16 / 17 | 0x2310 | 0,1 | fault |
 * | supply overvoltage | TMC OV / 0x2301 | 0x3210 | 0,2 | disable now |
 * | supply undervoltage / power-good lost | TMC UV / PCAL P6 | 0x3220 | 0,2 | fault |
 * | UV / OV warning | TMC flags 19 / 20 | 0x3200 | 2 | warn |
 * | chip temperature exceeded | TMC flag 14 | 0x4210 | 0,3 | fault |
 * | chip temperature warning | TMC flag 24 | 0x4210 | 3 | warn |
 * | NTC exceeded | TemperatureManager | 0x4310 | 0,3 | fault |
 * | Hall invalid | TMC flag 5 | 0x7305 | 0,5 | fault |
 * | host encoder diag / plausibility | AS5047U | 0x7300 | 0,5 | fault if position source, else warn |
 * | IMU lost | BNO08x | 0xFF10 | 7 | warn |
 * | TMC comms | drive_rt | 0x7510 | 0,7 | disable now |
 * | TMC watchdog / heartbeat stopped | TMC flags 9 / 26 | 0x6010 | 0,7 | fault |
 * | following error | host | 0x8611 | 0,5 | fault |
 * | software limit | host | 0x8612 | 5 | quick stop |
 * | CAN RX overflow | port | 0x8110 | 4 | warn |
 * | error passive | port | 0x8120 | 4 | warn |
 * | heartbeat consumer loss | CO_HBconsumer | 0x8130 | 4 | 0x6007 |
 * | recovered from bus-off | port | 0x8140 | 4 | warn (0x6007 if in OE) |
 * | PDO length error / exceeded | CO_PDO | 0x8210 / 0x8220 | 4 | warn |
 * | SYNC length / timeout | CO_SYNC | 0x8240 / 0x8100 | 4 | 0x6007 in cyclic modes |
 * | RPDO timeout | CO_PDO | 0x8250 | 4 | 0x6007 |
 * | parameter invalid / store CRC mismatch | storage | 0x6320 | 0,7 | refuse transition 3 |
 * | RT / service thread stall | LivenessMonitor | 0x6100 | 0,7 | reset |
 */
inline constexpr Policy kPolicy[kCauseCount] = {
    {Cause::GateShortCircuit, "gate_short_circuit", Source::TmcGateDriver, 0x2320U, er::kGeneric | er::kCurrent, Reaction::DisableNow, cond::kNone},
    {Cause::GateOvercurrent, "gate_overcurrent", Source::TmcGateDriver, 0x2310U, er::kGeneric | er::kCurrent, Reaction::DisableNow, cond::kNone},
    {Cause::I2tLimit1, "i2t_limit_1", Source::TmcGeneral, 0x2310U, er::kGeneric | er::kCurrent, Reaction::Fault, cond::kNone},
    {Cause::I2tLimit2, "i2t_limit_2", Source::TmcGeneral, 0x2310U, er::kGeneric | er::kCurrent, Reaction::Fault, cond::kNone},
    {Cause::SupplyOvervoltage, "supply_overvoltage", Source::TmcSupply, 0x3210U, er::kGeneric | er::kVoltage, Reaction::DisableNow, cond::kNone},
    {Cause::SupplyUndervoltage, "supply_undervoltage", Source::TmcSupply, 0x3220U, er::kGeneric | er::kVoltage, Reaction::Fault, cond::kNone},
    {Cause::PowerGoodLost, "power_good_lost", Source::PcalPowerGood, 0x3220U, er::kGeneric | er::kVoltage, Reaction::Fault, cond::kNone},
    {Cause::SupplyVoltageWarning, "supply_voltage_warning", Source::TmcGeneral, 0x3200U, er::kVoltage, Reaction::Warn, cond::kNone},
    {Cause::ChipOvertemperature, "chip_overtemperature", Source::TmcGeneral, 0x4210U, er::kGeneric | er::kTemperature, Reaction::Fault, cond::kNone},
    {Cause::ChipTemperatureWarning, "chip_temperature_warning", Source::TmcGeneral, 0x4210U, er::kTemperature, Reaction::Warn, cond::kNone},
    {Cause::NtcOvertemperature, "ntc_overtemperature", Source::TemperatureManager, 0x4310U, er::kGeneric | er::kTemperature, Reaction::Fault, cond::kNone},
    {Cause::HallInvalid, "hall_invalid", Source::TmcGeneral, 0x7305U, er::kGeneric | er::kProfile, Reaction::Fault, cond::kNone},
    {Cause::HostEncoderFault, "host_encoder_fault", Source::HostEncoder, 0x7300U, er::kGeneric | er::kProfile, Reaction::Fault, cond::kWarnUnlessPositionSource},
    {Cause::ImuLost, "imu_lost", Source::Imu, 0xFF10U, er::kManufacturer, Reaction::Warn, cond::kNone},
    {Cause::TmcCommsFault, "tmc_comms_fault", Source::DriveRt, 0x7510U, er::kGeneric | er::kManufacturer, Reaction::DisableNow, cond::kNone},
    {Cause::TmcHeartbeatStopped, "tmc_heartbeat_stopped", Source::TmcGeneral, 0x6010U, er::kGeneric | er::kManufacturer, Reaction::Fault, cond::kNone},
    {Cause::FollowingError, "following_error", Source::Host, 0x8611U, er::kGeneric | er::kProfile, Reaction::Fault, cond::kNone},
    {Cause::SoftwareLimit, "software_limit", Source::Host, 0x8612U, er::kProfile, Reaction::QuickStop, cond::kNone},
    {Cause::CanRxOverflow, "can_rx_overflow", Source::CanPort, 0x8110U, er::kCommunication, Reaction::Warn, cond::kNone},
    {Cause::CanErrorPassive, "can_error_passive", Source::CanPort, 0x8120U, er::kCommunication, Reaction::Warn, cond::kNone},
    {Cause::HeartbeatConsumerLoss, "heartbeat_consumer_loss", Source::CoHbConsumer, 0x8130U, er::kCommunication, Reaction::AbortConnection, cond::kNone},
    {Cause::BusOffRecovered, "bus_off_recovered", Source::CanPort, 0x8140U, er::kCommunication, Reaction::Warn, cond::kAbortIfOperationEnabled},
    {Cause::PdoLengthError, "pdo_length_error", Source::CoPdo, 0x8210U, er::kCommunication, Reaction::Warn, cond::kNone},
    {Cause::PdoLengthExceeded, "pdo_length_exceeded", Source::CoPdo, 0x8220U, er::kCommunication, Reaction::Warn, cond::kNone},
    {Cause::SyncLengthError, "sync_length_error", Source::CoSync, 0x8240U, er::kCommunication, Reaction::AbortConnection, cond::kAbortOnlyCyclic},
    {Cause::SyncTimeout, "sync_timeout", Source::CoSync, 0x8100U, er::kCommunication, Reaction::AbortConnection, cond::kAbortOnlyCyclic},
    {Cause::RpdoTimeout, "rpdo_timeout", Source::CoPdo, 0x8250U, er::kCommunication, Reaction::AbortConnection, cond::kNone},
    {Cause::ParameterInvalid, "parameter_invalid", Source::Storage, 0x6320U, er::kGeneric | er::kManufacturer, Reaction::RefuseSwitchOn, cond::kNone},
    {Cause::StoreCrcMismatch, "store_crc_mismatch", Source::Storage, 0x6320U, er::kGeneric | er::kManufacturer, Reaction::RefuseSwitchOn, cond::kNone},
    {Cause::RtStall, "rt_stall", Source::LivenessMonitor, 0x6100U, er::kGeneric | er::kManufacturer, Reaction::Reset, cond::kNone},
    {Cause::SvcStall, "svc_stall", Source::LivenessMonitor, 0x6100U, er::kGeneric | er::kManufacturer, Reaction::Reset, cond::kNone},
};

constexpr const Policy& PolicyOf(Cause c) noexcept { return kPolicy[static_cast<uint8_t>(c)]; }
constexpr uint32_t CauseBit(Cause c) noexcept { return static_cast<uint32_t>(1U) << static_cast<uint8_t>(c); }

/** @brief Context that the conditional rows depend on. */
struct Context {
  bool operation_enabled{false};      ///< the axis was in OperationEnabled when the cause was raised
  bool cyclic_mode{false};            ///< the active mode is csp / csv / cst
  bool sensor_is_position_source{false};  ///< the faulted sensor feeds the position loop
};

/** @brief Reaction of @p c after its conditions are applied. */
constexpr Reaction EffectiveReaction(Cause c, const Context& ctx) noexcept {
  const Policy& p = PolicyOf(c);
  if ((p.conditions & cond::kWarnUnlessPositionSource) != 0U && !ctx.sensor_is_position_source) return Reaction::Warn;
  if ((p.conditions & cond::kAbortIfOperationEnabled) != 0U && ctx.operation_enabled) return Reaction::AbortConnection;
  if ((p.conditions & cond::kAbortOnlyCyclic) != 0U && !ctx.cyclic_mode) return Reaction::Warn;
  return p.reaction;
}

/** @brief Does @p r put CiA 402 into Fault (directly, not via 0x6007)? */
constexpr bool IsFaultReaction(Reaction r) noexcept {
  return r == Reaction::Fault || r == Reaction::DisableNow || r == Reaction::Reset;
}

/** @brief Does @p r fault the axis, given whether the 0x6007 effect is "fault"? */
constexpr bool FaultsAxis(Reaction r, bool abort_connection_is_fault) noexcept {
  return IsFaultReaction(r) || (r == Reaction::AbortConnection && abort_connection_is_fault);
}

/** @brief Is the bridge healthy enough for a 0x605E ramp? ("disable now" downgrades to coast, §6.4.) */
constexpr bool BridgeHealthy(Reaction r) noexcept { return r != Reaction::DisableNow && r != Reaction::Reset; }

/** @brief Does @p r latch until fault reset? */
constexpr bool Latches(Reaction r) noexcept { return r != Reaction::Warn; }

/** @brief EMCY manufacturer bytes 3–7: {cause id, raw flags u32 little-endian}. */
constexpr void PackEmcyManufacturerBytes(Cause c, uint32_t raw_flags, uint8_t out[5]) noexcept {
  out[0] = static_cast<uint8_t>(c);
  out[1] = static_cast<uint8_t>(raw_flags & 0xFFU);
  out[2] = static_cast<uint8_t>((raw_flags >> 8) & 0xFFU);
  out[3] = static_cast<uint8_t>((raw_flags >> 16) & 0xFFU);
  out[4] = static_cast<uint8_t>((raw_flags >> 24) & 0xFFU);
}

/** Escalation (pw fault_state.hpp values). */
inline constexpr uint8_t kEscalateCount = 3U;
inline constexpr uint32_t kEscalateWindowMs = 10U * 60U * 1000U;

/** @brief What one raise did. */
struct RaiseResult {
  Reaction reaction{Reaction::Warn};  ///< effective reaction
  bool first{false};                  ///< the cause was not latched / reported before
  bool escalated{false};              ///< this raise crossed the repeat threshold
};

/**
 * @brief The axis fault record (owned by the SafetyArbiter; one owner thread).
 * @details Feeds 0x4003 (active / escalated masks), the CiA 402 `fault_active`
 *          input and 0x1001.
 */
class FaultBook {
 public:
  /** @brief Record cause @p c at @p now_ms under @p ctx. */
  constexpr RaiseResult Raise(Cause c, uint32_t now_ms, const Context& ctx) noexcept {
    RaiseResult r{};
    const uint8_t i = static_cast<uint8_t>(c);
    if (i >= kCauseCount) return r;
    const uint32_t bit = CauseBit(c);
    r.reaction = EffectiveReaction(c, ctx);
    hist_[i][head_[i] % kEscalateCount] = now_ms;
    head_[i] = static_cast<uint8_t>(head_[i] + 1U);
    if (count_[i] < kEscalateCount) ++count_[i];
    const bool repeat =
        count_[i] >= kEscalateCount && (now_ms - hist_[i][head_[i] % kEscalateCount]) <= kEscalateWindowMs;
    if (!Latches(r.reaction)) {
      r.first = (warn_ & bit) == 0U;
      warn_ |= bit;
      return r;
    }
    r.first = (active_ & bit) == 0U;
    active_ |= bit;
    if (repeat && r.reaction != Reaction::Reset && (escalated_ & bit) == 0U) {
      escalated_ |= bit;
      r.escalated = true;
    }
    return r;
  }

  /**
   * @brief The condition of @p c is gone.
   * @details Warnings stop being reported. Latched causes whose reaction does
   *          not fault the axis (quick stop, refuse switch-on, a 0x6007 effect
   *          that is not "fault") release unless escalated. Fault causes stay
   *          until `FaultReset`.
   */
  constexpr void ConditionGone(Cause c, const Context& ctx, bool abort_connection_is_fault) noexcept {
    const uint32_t bit = CauseBit(c);
    warn_ &= ~bit;
    if ((escalated_ & bit) != 0U) return;
    if (!FaultsAxis(EffectiveReaction(c, ctx), abort_connection_is_fault)) active_ &= ~bit;
  }

  /**
   * @brief Fault reset request (CiA 402 transition 15 is gated on the result).
   * @param present_mask Causes whose condition is still present.
   * @param cleared_mask Causes whose owner's `Cleared` predicate holds (needed for escalated causes).
   * @return true when no latched cause is left.
   */
  constexpr bool FaultReset(uint32_t present_mask, uint32_t cleared_mask) noexcept {
    for (uint8_t i = 0; i < kCauseCount; ++i) {
      const uint32_t bit = static_cast<uint32_t>(1U) << i;
      if ((active_ & bit) == 0U) continue;
      if ((present_mask & bit) != 0U) continue;
      if ((escalated_ & bit) != 0U && (cleared_mask & bit) == 0U) continue;
      active_ &= ~bit;
      escalated_ &= ~bit;
    }
    warn_ &= present_mask;
    return active_ == 0U;
  }

  /** @brief Would a fault reset succeed now? (no state change) */
  constexpr bool FaultResetAllowed(uint32_t present_mask, uint32_t cleared_mask) const noexcept {
    FaultBook copy = *this;
    return copy.FaultReset(present_mask, cleared_mask);
  }

  constexpr uint32_t ActiveMask() const noexcept { return active_; }
  constexpr uint32_t EscalatedMask() const noexcept { return escalated_; }
  constexpr uint32_t WarningMask() const noexcept { return warn_; }
  constexpr bool IsActive(Cause c) const noexcept { return ((active_ | warn_) & CauseBit(c)) != 0U; }

  /** @brief 0x1001 value: OR of the ER bits of every active or reported cause. */
  constexpr uint8_t ErrorRegister() const noexcept {
    uint8_t out = 0U;
    for (uint8_t i = 0; i < kCauseCount; ++i) {
      if (((active_ | warn_) & (static_cast<uint32_t>(1U) << i)) != 0U) out = static_cast<uint8_t>(out | kPolicy[i].er);
    }
    return out;
  }

  /**
   * @brief CiA 402 input `fault_active`: any latched cause that faults the axis.
   * @param abort_connection_is_fault 0x6007 effect is "fault" (option code 1, the default).
   */
  constexpr bool FaultActive(const Context& ctx, bool abort_connection_is_fault) const noexcept {
    for (uint8_t i = 0; i < kCauseCount; ++i) {
      if ((active_ & (static_cast<uint32_t>(1U) << i)) == 0U) continue;
      if (FaultsAxis(EffectiveReaction(static_cast<Cause>(i), ctx), abort_connection_is_fault)) return true;
    }
    return false;
  }

 private:
  uint32_t active_{0};
  uint32_t escalated_{0};
  uint32_t warn_{0};
  uint32_t hist_[kCauseCount][kEscalateCount]{};
  uint8_t head_[kCauseCount]{};
  uint8_t count_[kCauseCount]{};
};

}  // namespace vortex::contracts::fault
