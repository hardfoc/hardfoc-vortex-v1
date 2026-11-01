#pragma once
/**
 * @file cia402_rules.hpp
 * @brief CiA 402 power drive state machine as one rule table (design §5.6).
 *
 * @details Pure and HAL-free. `DriveAxis` calls `Decide()` once per
 *          `drive_rt` cycle with the decoded controlword (level, not edge,
 *          except fault reset) and the gate inputs, then performs the
 *          returned `Action`. Host tests walk every transition 0–16 from
 *          every state (tests/host/test_cia402_rules.cpp).
 *
 *          @verbatim
 *          Start ─0─► NotReadyToSwitchOn ─1─► SwitchOnDisabled ◄──────────────┐
 *                SwitchOnDisabled ─2─► ReadyToSwitchOn ─3─► SwitchedOn ─4─► OperationEnabled
 *                ReadyToSwitchOn ─7─► SwitchOnDisabled     SwitchedOn ─6─► ReadyToSwitchOn
 *                SwitchedOn ─10─► SwitchOnDisabled        OperationEnabled ─5─► SwitchedOn
 *                OperationEnabled ─8─► ReadyToSwitchOn    OperationEnabled ─9─► SwitchOnDisabled
 *                OperationEnabled ─11─► QuickStopActive ─12─► SwitchOnDisabled
 *                QuickStopActive ─16─► OperationEnabled (quick stop option code 5..8)
 *                any state but FRA/Fault ─13─► FaultReactionActive ─14─► Fault ─15─► SwitchOnDisabled
 *          @endverbatim
 *
 *          Gates (§5.5, §6.2, §7.4): transitions that move *away* from safety
 *          (2, 3, 4, 15, 16) need the controlword source to hold authority;
 *          3 needs a valid configuration; 4 and 16 need a master liveness
 *          source; 15 needs every active cause cleared. Transitions towards
 *          safety are never gated.
 */
#include <cstdint>

namespace vortex::contracts::cia402 {

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------

/** @brief Power drive system states (CiA 402-2 §6.3). `Start` is the power-on pseudo state. */
enum class State : uint8_t {
  Start = 0,
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
};
inline constexpr uint8_t kStateCount = 9U;

constexpr uint16_t StateBit(State s) noexcept { return static_cast<uint16_t>(1U << static_cast<uint8_t>(s)); }

constexpr const char* StateName(State s) noexcept {
  switch (s) {
    case State::Start: return "Start";
    case State::NotReadyToSwitchOn: return "NotReadyToSwitchOn";
    case State::SwitchOnDisabled: return "SwitchOnDisabled";
    case State::ReadyToSwitchOn: return "ReadyToSwitchOn";
    case State::SwitchedOn: return "SwitchedOn";
    case State::OperationEnabled: return "OperationEnabled";
    case State::QuickStopActive: return "QuickStopActive";
    case State::FaultReactionActive: return "FaultReactionActive";
    case State::Fault: return "Fault";
  }
  return "?";
}

/** @brief Is the power stage (bridge) allowed to drive the motor in @p s? */
constexpr bool DriveFunctionEnabled(State s) noexcept {
  return s == State::OperationEnabled || s == State::QuickStopActive || s == State::FaultReactionActive;
}

// ---------------------------------------------------------------------------
// Controlword (0x6040)
// ---------------------------------------------------------------------------

/** @brief Device control commands (CiA 402-2 controlword table). */
enum class Command : uint8_t {
  None = 0,         ///< no command (bit 7 held high after a fault reset)
  Shutdown,         ///< 0xxx x110
  SwitchOn,         ///< 0xxx 0111 (also "disable operation" from OperationEnabled)
  EnableOperation,  ///< 0xxx 1111 (also "switch on + enable operation")
  DisableVoltage,   ///< 0xxx xx0x
  QuickStop,        ///< 0xxx x01x
  FaultReset,       ///< bit 7 rising edge
};
inline constexpr uint8_t kCommandCount = 7U;

constexpr uint8_t CommandBit(Command c) noexcept { return static_cast<uint8_t>(1U << static_cast<uint8_t>(c)); }

/** @brief Controlword bits. Bits 4–6 and 9 are mode specific; 8 is halt. */
namespace cw {
inline constexpr uint16_t kSwitchOn = 0x0001U;
inline constexpr uint16_t kEnableVoltage = 0x0002U;
inline constexpr uint16_t kQuickStop = 0x0004U;  ///< active low: 0 = quick stop
inline constexpr uint16_t kEnableOperation = 0x0008U;
inline constexpr uint16_t kFaultReset = 0x0080U;
inline constexpr uint16_t kHalt = 0x0100U;

/** Decode masks and values (bit 7 is part of every mask: commands need it low). */
inline constexpr uint16_t kMaskDisableVoltage = 0x0082U, kValDisableVoltage = 0x0000U;
inline constexpr uint16_t kMaskQuickStop = 0x0086U, kValQuickStop = 0x0002U;
inline constexpr uint16_t kMaskShutdown = 0x0087U, kValShutdown = 0x0006U;
inline constexpr uint16_t kMaskSwitchOn = 0x008FU, kValSwitchOn = 0x0007U;
inline constexpr uint16_t kMaskEnableOperation = 0x008FU, kValEnableOperation = 0x000FU;
}  // namespace cw

/**
 * @brief Decode a controlword.
 * @param word Controlword now.
 * @param previous Controlword of the previous cycle (for the fault-reset edge).
 * @details Fault reset is the 0→1 edge of bit 7. While bit 7 stays high no
 *          other command is decoded (every command pattern has bit 7 low).
 */
constexpr Command DecodeControlword(uint16_t word, uint16_t previous) noexcept {
  if ((word & cw::kFaultReset) != 0U) {
    return (previous & cw::kFaultReset) == 0U ? Command::FaultReset : Command::None;
  }
  if ((word & cw::kMaskDisableVoltage) == cw::kValDisableVoltage) return Command::DisableVoltage;
  if ((word & cw::kMaskQuickStop) == cw::kValQuickStop) return Command::QuickStop;
  if ((word & cw::kMaskShutdown) == cw::kValShutdown) return Command::Shutdown;
  if ((word & cw::kMaskSwitchOn) == cw::kValSwitchOn) return Command::SwitchOn;
  return Command::EnableOperation;  // (word & 0x8F) == 0x0F: the only pattern left
}

/** @brief Canonical controlword for @p c (bits 4–15 zero). `None` encodes as 0x0080 (bit 7 held). */
constexpr uint16_t EncodeControlword(Command c) noexcept {
  switch (c) {
    case Command::Shutdown: return cw::kValShutdown;
    case Command::SwitchOn: return cw::kValSwitchOn;
    case Command::EnableOperation: return cw::kValEnableOperation;
    case Command::DisableVoltage: return cw::kValDisableVoltage;
    case Command::QuickStop: return cw::kValQuickStop;
    case Command::FaultReset: return cw::kFaultReset;
    case Command::None: return cw::kFaultReset;
  }
  return 0U;
}

/** @brief Halt bit (8): stop with the 0x605D ramp, stay in OperationEnabled. */
constexpr bool IsHalt(uint16_t word) noexcept { return (word & cw::kHalt) != 0U; }

// ---------------------------------------------------------------------------
// Option codes (0x605A–0x605E, 0x6007)
// ---------------------------------------------------------------------------

namespace option {

/** @brief How the axis stops. */
enum class StopRamp : uint8_t {
  Coast = 0,     ///< disable the drive function now (bridge off)
  SlowDown,      ///< slow-down ramp (0x6084 profile deceleration)
  QuickStopRamp, ///< quick-stop ramp (0x6085)
  CurrentLimit,  ///< ramp at the current limit
  VoltageLimit,  ///< ramp at the voltage limit
};

/** Defaults (design §5.2). */
inline constexpr int16_t kQuickStopDefault = 2;          ///< 0x605A: quick-stop ramp → SwitchOnDisabled
inline constexpr int16_t kShutdownDefault = 0;           ///< 0x605B: disable drive function
inline constexpr int16_t kDisableOperationDefault = 1;   ///< 0x605C: slow-down ramp, then disable
inline constexpr int16_t kHaltDefault = 1;               ///< 0x605D: slow-down ramp
inline constexpr int16_t kFaultReactionDefault = 0;      ///< 0x605E: disable drive, coast
inline constexpr int16_t kAbortConnectionDefault = 1;    ///< 0x6007: fault signal

/** 0x605A quick stop option code: 0..8 supported. */
constexpr bool IsValidQuickStop(int16_t c) noexcept { return c >= 0 && c <= 8; }
/** 0x605A 5..8: stay in QuickStopActive after stopping (transition 16 allowed). */
constexpr bool QuickStopStays(int16_t c) noexcept { return c >= 5 && c <= 8; }
constexpr StopRamp QuickStopRampOf(int16_t c) noexcept {
  switch (c) {
    case 1: case 5: return StopRamp::SlowDown;
    case 2: case 6: return StopRamp::QuickStopRamp;
    case 3: case 7: return StopRamp::CurrentLimit;
    case 4: case 8: return StopRamp::VoltageLimit;
    default: return StopRamp::Coast;
  }
}

/** 0x605B shutdown option code: 0 disable drive, 1 slow-down ramp then disable. */
constexpr bool IsValidShutdown(int16_t c) noexcept { return c == 0 || c == 1; }
constexpr StopRamp ShutdownRampOf(int16_t c) noexcept { return c == 1 ? StopRamp::SlowDown : StopRamp::Coast; }

/** 0x605C disable operation option code: 0 disable drive, 1 slow-down ramp then disable. */
constexpr bool IsValidDisableOperation(int16_t c) noexcept { return c == 0 || c == 1; }
constexpr StopRamp DisableOperationRampOf(int16_t c) noexcept {
  return c == 1 ? StopRamp::SlowDown : StopRamp::Coast;
}

/** 0x605D halt option code: 1 slow-down, 2 quick-stop, 3 current limit, 4 voltage limit. */
constexpr bool IsValidHalt(int16_t c) noexcept { return c >= 1 && c <= 4; }
constexpr StopRamp HaltRampOf(int16_t c) noexcept {
  return IsValidHalt(c) ? static_cast<StopRamp>(c) : StopRamp::SlowDown;
}

/** 0x605E fault reaction option code: 0 coast, 1 slow-down, 2 quick-stop, 3 current, 4 voltage limit. */
constexpr bool IsValidFaultReaction(int16_t c) noexcept { return c >= 0 && c <= 4; }
/**
 * @brief Ramp used on a fault.
 * @param bridge_healthy false for "disable now" causes (§6.4): the bridge itself
 *        is faulted, so any ramp is downgraded to coasting.
 */
constexpr StopRamp FaultReactionRampOf(int16_t c, bool bridge_healthy) noexcept {
  if (!bridge_healthy || !IsValidFaultReaction(c)) return StopRamp::Coast;
  return static_cast<StopRamp>(c);
}

/** @brief Effect of 0x6007 abort connection option code on loss of a liveness source. */
enum class AbortEffect : uint8_t {
  None = 0,        ///< 0: no action
  Fault,           ///< 1: fault signal (0x605E applies)
  DisableVoltage,  ///< 2: as if the disable voltage command was received
  QuickStop,       ///< 3: as if the quick stop command was received
};
constexpr bool IsValidAbortConnection(int16_t c) noexcept { return c >= 0 && c <= 3; }
constexpr AbortEffect AbortConnectionEffect(int16_t c) noexcept {
  return IsValidAbortConnection(c) ? static_cast<AbortEffect>(c) : AbortEffect::Fault;
}
/** @brief Command injected by 0x6007 (None when the effect is not a command). */
constexpr Command AbortConnectionCommand(int16_t c) noexcept {
  switch (AbortConnectionEffect(c)) {
    case AbortEffect::DisableVoltage: return Command::DisableVoltage;
    case AbortEffect::QuickStop: return Command::QuickStop;
    default: return Command::None;
  }
}

}  // namespace option

// ---------------------------------------------------------------------------
// Modes of operation (0x6060 / 0x6061 / 0x6502)
// ---------------------------------------------------------------------------

enum class Mode : int8_t {
  None = 0,
  ProfilePosition = 1,  ///< pp
  Velocity = 2,         ///< vl (not supported)
  ProfileVelocity = 3,  ///< pv
  ProfileTorque = 4,    ///< tq
  Homing = 6,           ///< hm
  Interpolated = 7,     ///< ip (not supported; csp covers it)
  CyclicPosition = 8,   ///< csp
  CyclicVelocity = 9,   ///< csv
  CyclicTorque = 10,    ///< cst
};

/** @brief 0x6502 bit for @p m (bit n-1 for mode n), 0 for none / manufacturer modes. */
constexpr uint32_t SupportedModeBit(int8_t m) noexcept {
  return (m >= 1 && m <= 10) ? (static_cast<uint32_t>(1U) << static_cast<uint8_t>(m - 1)) : 0U;
}

/** @brief 0x6502 value: pp, pv, tq, hm, csp, csv, cst. */
inline constexpr uint32_t kSupportedDriveModes =
    SupportedModeBit(1) | SupportedModeBit(3) | SupportedModeBit(4) | SupportedModeBit(6) |
    SupportedModeBit(8) | SupportedModeBit(9) | SupportedModeBit(10);
static_assert(kSupportedDriveModes == 0x000003ADUL, "design §5.2: 0x6502 = 0x000003AD");

constexpr bool IsSupportedMode(int8_t m) noexcept { return (SupportedModeBit(m) & kSupportedDriveModes) != 0U; }
constexpr bool IsCyclicMode(int8_t m) noexcept { return m >= 8 && m <= 10; }

/**
 * @brief May 0x6060 change now?
 * @details Only while the drive function is off (SwitchedOn or below, or
 *          Fault), or in OperationEnabled with the velocity below threshold.
 */
constexpr bool ModeChangeAllowed(State s, bool velocity_zero) noexcept {
  switch (s) {
    case State::NotReadyToSwitchOn:
    case State::SwitchOnDisabled:
    case State::ReadyToSwitchOn:
    case State::SwitchedOn:
    case State::Fault:
      return true;
    case State::OperationEnabled:
      return velocity_zero;
    default:
      return false;
  }
}

// ---------------------------------------------------------------------------
// Rule table
// ---------------------------------------------------------------------------

/** @brief What the axis does when a transition fires. */
enum class Action : uint8_t {
  None = 0,
  ApplyConfig,           ///< 3: validate OD application parameters, push them to the TMC9660
  EnableDrive,           ///< 4, 16: set commutation mode, assert DRV_EN
  StopDisableOperation,  ///< 5: stop per 0x605C, then drive off
  StopShutdown,          ///< 8: stop per 0x605B, then drive off
  DisableDrive,          ///< 9, 12, 14: drive function off now (coast)
  StopQuickStop,         ///< 11: stop per 0x605A
  StopFaultReaction,     ///< 13: react per 0x605E
  ResetFault,            ///< 15: clear the fault record
};

/** @brief Why a commanded transition was refused (the state does not change). */
enum class Refusal : uint8_t {
  None = 0,
  Authority,      ///< the controlword source does not hold authority (§7.4)
  Liveness,       ///< no master liveness source configured (§5.5) → warning + EMCY 0x8100
  ConfigInvalid,  ///< application parameters invalid (§6.2) → EMCY 0x6320
  FaultActive,    ///< fault reset while a cause has not cleared (§6.4)
};

/** @brief Condition that fires a transition without a command. */
enum class Auto : uint8_t {
  None = 0,          ///< command driven
  Always,            ///< 0
  InitDone,          ///< 1: self-test passed
  FaultActive,       ///< 13
  ReactionDone,      ///< 14
  QuickStopFinished, ///< 12: option 0 at once, options 1..4 when the stop is done
};

/** @brief Gate bits of a rule. */
namespace gate {
inline constexpr uint8_t kAuthority = 0x01U;
inline constexpr uint8_t kLiveness = 0x02U;
inline constexpr uint8_t kConfig = 0x04U;
inline constexpr uint8_t kNoFault = 0x08U;
inline constexpr uint8_t kQsStays = 0x10U;  ///< only when 0x605A is 5..8 (not a refusal: no match)
}  // namespace gate

/** @brief One row of the table. First matching row wins. */
struct Rule {
  uint8_t transition;  ///< CiA 402 transition number
  uint16_t from;       ///< StateBit mask of source states
  uint8_t commands;    ///< CommandBit mask (0 for Auto rules)
  Auto automatic;
  State to;
  Action action;
  uint8_t gates;
};

inline constexpr uint16_t kAnyButFault =
    StateBit(State::NotReadyToSwitchOn) | StateBit(State::SwitchOnDisabled) | StateBit(State::ReadyToSwitchOn) |
    StateBit(State::SwitchedOn) | StateBit(State::OperationEnabled) | StateBit(State::QuickStopActive);

/**
 * @brief The one CiA 402 transition table.
 *
 * | # | From | Trigger | To | Action | Gates |
 * |---|------|---------|----|--------|-------|
 * | 0 | Start | power on | NotReadyToSwitchOn | — | — |
 * | 13 | any but FRA / Fault | fault active | FaultReactionActive | 0x605E | — |
 * | 14 | FaultReactionActive | reaction done | Fault | drive off | — |
 * | 15 | Fault | fault reset (bit 7 edge) | SwitchOnDisabled | clear faults | authority, causes cleared |
 * | 1 | NotReadyToSwitchOn | self-test ok | SwitchOnDisabled | — | — |
 * | 2 | SwitchOnDisabled | shutdown | ReadyToSwitchOn | — | authority |
 * | 3 | ReadyToSwitchOn | switch on / enable op | SwitchedOn | apply config | authority, config |
 * | 7 | ReadyToSwitchOn | disable voltage / quick stop | SwitchOnDisabled | — | — |
 * | 4 | SwitchedOn | enable operation | OperationEnabled | enable drive | authority, liveness |
 * | 6 | SwitchedOn | shutdown | ReadyToSwitchOn | — | — |
 * | 10 | SwitchedOn | disable voltage / quick stop | SwitchOnDisabled | — | — |
 * | 5 | OperationEnabled | disable operation | SwitchedOn | 0x605C | — |
 * | 8 | OperationEnabled | shutdown | ReadyToSwitchOn | 0x605B | — |
 * | 9 | OperationEnabled | disable voltage | SwitchOnDisabled | drive off | — |
 * | 11 | OperationEnabled | quick stop | QuickStopActive | 0x605A | — |
 * | 12 | QuickStopActive | disable voltage | SwitchOnDisabled | drive off | — |
 * | 12 | QuickStopActive | stop finished (0x605A 0..4) | SwitchOnDisabled | drive off | — |
 * | 16 | QuickStopActive | enable operation | OperationEnabled | enable drive | 0x605A 5..8, authority, liveness |
 */
inline constexpr Rule kRules[] = {
    {0, StateBit(State::Start), 0, Auto::Always, State::NotReadyToSwitchOn, Action::None, 0},
    {13, kAnyButFault, 0, Auto::FaultActive, State::FaultReactionActive, Action::StopFaultReaction, 0},
    {14, StateBit(State::FaultReactionActive), 0, Auto::ReactionDone, State::Fault, Action::DisableDrive, 0},
    {15, StateBit(State::Fault), CommandBit(Command::FaultReset), Auto::None, State::SwitchOnDisabled,
     Action::ResetFault, gate::kAuthority | gate::kNoFault},
    {1, StateBit(State::NotReadyToSwitchOn), 0, Auto::InitDone, State::SwitchOnDisabled, Action::None, 0},
    {2, StateBit(State::SwitchOnDisabled), CommandBit(Command::Shutdown), Auto::None, State::ReadyToSwitchOn,
     Action::None, gate::kAuthority},
    {3, StateBit(State::ReadyToSwitchOn), static_cast<uint8_t>(CommandBit(Command::SwitchOn) | CommandBit(Command::EnableOperation)),
     Auto::None, State::SwitchedOn, Action::ApplyConfig, gate::kAuthority | gate::kConfig},
    {7, StateBit(State::ReadyToSwitchOn), static_cast<uint8_t>(CommandBit(Command::DisableVoltage) | CommandBit(Command::QuickStop)),
     Auto::None, State::SwitchOnDisabled, Action::None, 0},
    {4, StateBit(State::SwitchedOn), CommandBit(Command::EnableOperation), Auto::None, State::OperationEnabled,
     Action::EnableDrive, gate::kAuthority | gate::kLiveness},
    {6, StateBit(State::SwitchedOn), CommandBit(Command::Shutdown), Auto::None, State::ReadyToSwitchOn, Action::None, 0},
    {10, StateBit(State::SwitchedOn), static_cast<uint8_t>(CommandBit(Command::DisableVoltage) | CommandBit(Command::QuickStop)),
     Auto::None, State::SwitchOnDisabled, Action::None, 0},
    {5, StateBit(State::OperationEnabled), CommandBit(Command::SwitchOn), Auto::None, State::SwitchedOn,
     Action::StopDisableOperation, 0},
    {8, StateBit(State::OperationEnabled), CommandBit(Command::Shutdown), Auto::None, State::ReadyToSwitchOn,
     Action::StopShutdown, 0},
    {9, StateBit(State::OperationEnabled), CommandBit(Command::DisableVoltage), Auto::None, State::SwitchOnDisabled,
     Action::DisableDrive, 0},
    {11, StateBit(State::OperationEnabled), CommandBit(Command::QuickStop), Auto::None, State::QuickStopActive,
     Action::StopQuickStop, 0},
    {12, StateBit(State::QuickStopActive), CommandBit(Command::DisableVoltage), Auto::None, State::SwitchOnDisabled,
     Action::DisableDrive, 0},
    {12, StateBit(State::QuickStopActive), 0, Auto::QuickStopFinished, State::SwitchOnDisabled, Action::DisableDrive, 0},
    {16, StateBit(State::QuickStopActive), CommandBit(Command::EnableOperation), Auto::None, State::OperationEnabled,
     Action::EnableDrive, gate::kQsStays | gate::kAuthority | gate::kLiveness},
};
inline constexpr uint8_t kRuleCount = static_cast<uint8_t>(sizeof(kRules) / sizeof(kRules[0]));
inline constexpr uint8_t kTransitionCount = 17U;  ///< transitions 0..16
inline constexpr uint8_t kNoTransition = 0xFFU;

/** @brief Gate inputs gathered by the caller each cycle. */
struct Inputs {
  bool init_done{false};            ///< self-test / boot bring-up finished (1)
  bool fault_active{false};         ///< any fault-class cause active (13; blocks 15)
  bool authority_ok{false};         ///< controlword source holds authority (2, 3, 4, 15, 16)
  bool liveness_ok{false};          ///< §5.5 master liveness source configured (4, 16)
  bool config_valid{false};         ///< application parameters valid (3)
  bool quick_stop_done{false};      ///< the quick-stop ramp has finished (12)
  bool fault_reaction_done{false};  ///< the 0x605E reaction has finished (14)
  bool velocity_zero{false};        ///< |actual velocity| below threshold (mode change)
  int16_t quick_stop_option{option::kQuickStopDefault};  ///< 0x605A
};

/** @brief Outcome of one step. */
struct Decision {
  State next{State::Start};
  uint8_t transition{kNoTransition};  ///< fired transition, or kNoTransition
  Action action{Action::None};
  Refusal refusal{Refusal::None};     ///< set when a commanded transition was refused
  uint16_t emcy{0};                   ///< EMCY code to raise for the refusal (0 = none)
  bool warning{false};                ///< set statusword bit 7 (warning)
};

/** EMCY codes raised on refusals. */
inline constexpr uint16_t kEmcyLivenessMissing = 0x8100U;  ///< §5.5
inline constexpr uint16_t kEmcyParameterError = 0x6320U;   ///< §6.2

namespace detail {
constexpr bool AutoHolds(Auto a, const Inputs& in) noexcept {
  switch (a) {
    case Auto::Always: return true;
    case Auto::InitDone: return in.init_done;
    case Auto::FaultActive: return in.fault_active;
    case Auto::ReactionDone: return in.fault_reaction_done;
    case Auto::QuickStopFinished:
      return option::IsValidQuickStop(in.quick_stop_option) && !option::QuickStopStays(in.quick_stop_option) &&
             (in.quick_stop_option == 0 || in.quick_stop_done);
    case Auto::None: return false;
  }
  return false;
}
}  // namespace detail

/**
 * @brief One step of the state machine.
 * @param s Current state.
 * @param cmd Decoded controlword (DecodeControlword) or an injected command.
 * @param in Gate inputs.
 * @return Next state, transition, action, refusal.
 */
constexpr Decision Decide(State s, Command cmd, const Inputs& in) noexcept {
  Decision d{};
  d.next = s;
  for (uint8_t i = 0; i < kRuleCount; ++i) {
    const Rule& r = kRules[i];
    if ((r.from & StateBit(s)) == 0U) continue;
    if (r.automatic != Auto::None) {
      if (!detail::AutoHolds(r.automatic, in)) continue;
    } else if ((r.commands & CommandBit(cmd)) == 0U) {
      continue;
    }
    if ((r.gates & gate::kQsStays) != 0U && !option::QuickStopStays(in.quick_stop_option)) continue;
    if ((r.gates & gate::kAuthority) != 0U && !in.authority_ok) {
      d.refusal = Refusal::Authority;
      return d;
    }
    if ((r.gates & gate::kNoFault) != 0U && in.fault_active) {
      d.refusal = Refusal::FaultActive;
      return d;
    }
    if ((r.gates & gate::kConfig) != 0U && !in.config_valid) {
      d.refusal = Refusal::ConfigInvalid;
      d.emcy = kEmcyParameterError;
      return d;
    }
    if ((r.gates & gate::kLiveness) != 0U && !in.liveness_ok) {
      d.refusal = Refusal::Liveness;
      d.emcy = kEmcyLivenessMissing;
      d.warning = true;
      return d;
    }
    d.next = r.to;
    d.transition = r.transition;
    d.action = r.action;
    return d;
  }
  return d;
}

// ---------------------------------------------------------------------------
// Statusword (0x6041)
// ---------------------------------------------------------------------------

namespace sw {
inline constexpr uint16_t kReadyToSwitchOn = 0x0001U;
inline constexpr uint16_t kSwitchedOn = 0x0002U;
inline constexpr uint16_t kOperationEnabled = 0x0004U;
inline constexpr uint16_t kFault = 0x0008U;
inline constexpr uint16_t kVoltageEnabled = 0x0010U;
inline constexpr uint16_t kQuickStop = 0x0020U;  ///< active low: 0 = quick stop active
inline constexpr uint16_t kSwitchOnDisabled = 0x0040U;
inline constexpr uint16_t kWarning = 0x0080U;
inline constexpr uint16_t kManufacturer8 = 0x0100U;
inline constexpr uint16_t kRemote = 0x0200U;
inline constexpr uint16_t kTargetReached = 0x0400U;
inline constexpr uint16_t kInternalLimit = 0x0800U;
inline constexpr uint16_t kOpSpecific12 = 0x1000U;  ///< pp: set-point ack; hm: attained; csx: target ignored
inline constexpr uint16_t kOpSpecific13 = 0x2000U;  ///< pp/csp: following error; hm: homing error

/** State decode masks (CiA 402-2 statusword table). */
inline constexpr uint16_t kMaskWide = 0x006FU;    ///< states that define bit 5
inline constexpr uint16_t kMaskNarrow = 0x004FU;  ///< states with bit 5 "x"
}  // namespace sw

/** @brief Flags the caller supplies for the non-state statusword bits. */
struct StatusFlags {
  bool voltage_enabled{false};  ///< bit 4: DC link present
  bool warning{false};          ///< bit 7
  bool remote{false};           ///< bit 9: controlword source holds authority
  bool target_reached{false};   ///< bit 10
  bool internal_limit{false};   ///< bit 11
  bool op_specific_12{false};   ///< bit 12 (mode specific)
  bool op_specific_13{false};   ///< bit 13 (mode specific)
};

/** @brief State bits 0–3, 5, 6 for @p s. Bit 5 is set only where the table requires it. */
constexpr uint16_t StateBits(State s) noexcept {
  switch (s) {
    case State::Start:
    case State::NotReadyToSwitchOn: return 0x0000U;
    case State::SwitchOnDisabled: return sw::kSwitchOnDisabled;
    case State::ReadyToSwitchOn: return sw::kQuickStop | sw::kReadyToSwitchOn;
    case State::SwitchedOn: return sw::kQuickStop | sw::kSwitchedOn | sw::kReadyToSwitchOn;
    case State::OperationEnabled:
      return sw::kQuickStop | sw::kOperationEnabled | sw::kSwitchedOn | sw::kReadyToSwitchOn;
    case State::QuickStopActive: return sw::kOperationEnabled | sw::kSwitchedOn | sw::kReadyToSwitchOn;
    case State::FaultReactionActive:
      return sw::kFault | sw::kOperationEnabled | sw::kSwitchedOn | sw::kReadyToSwitchOn;
    case State::Fault: return sw::kFault;
  }
  return 0U;
}

constexpr uint16_t EncodeStatusword(State s, const StatusFlags& f) noexcept {
  uint16_t w = StateBits(s);
  if (f.voltage_enabled) w = static_cast<uint16_t>(w | sw::kVoltageEnabled);
  if (f.warning) w = static_cast<uint16_t>(w | sw::kWarning);
  if (f.remote) w = static_cast<uint16_t>(w | sw::kRemote);
  if (f.target_reached) w = static_cast<uint16_t>(w | sw::kTargetReached);
  if (f.internal_limit) w = static_cast<uint16_t>(w | sw::kInternalLimit);
  if (f.op_specific_12) w = static_cast<uint16_t>(w | sw::kOpSpecific12);
  if (f.op_specific_13) w = static_cast<uint16_t>(w | sw::kOpSpecific13);
  return w;
}

/**
 * @brief Statusword → state (what a master does).
 * @return State::Start when the pattern matches no state.
 */
constexpr State DecodeStatusword(uint16_t w) noexcept {
  if ((w & sw::kMaskNarrow) == 0x0000U) return State::NotReadyToSwitchOn;
  if ((w & sw::kMaskNarrow) == 0x0040U) return State::SwitchOnDisabled;
  if ((w & sw::kMaskWide) == 0x0021U) return State::ReadyToSwitchOn;
  if ((w & sw::kMaskWide) == 0x0023U) return State::SwitchedOn;
  if ((w & sw::kMaskWide) == 0x0027U) return State::OperationEnabled;
  if ((w & sw::kMaskWide) == 0x0007U) return State::QuickStopActive;
  if ((w & sw::kMaskNarrow) == 0x000FU) return State::FaultReactionActive;
  if ((w & sw::kMaskNarrow) == 0x0008U) return State::Fault;
  return State::Start;
}

}  // namespace vortex::contracts::cia402
