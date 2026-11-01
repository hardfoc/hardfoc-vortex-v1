/// Host test — CiA 402 rule table (contracts/cia402_rules.hpp, design §5.6).
///
/// Every state × every command × every gate-input combination × every quick
/// stop option code is checked against an independent reference written
/// from the CiA 402-2 state diagram. Each transition 0–16 must fire from
/// exactly its source states. Controlword and statusword encodings are
/// round-tripped exhaustively.

#include "contracts/cia402_rules.hpp"

#include <set>
#include <utility>

#include "support/check.hpp"

namespace {

using namespace vortex::contracts::cia402;

constexpr State kStates[] = {State::Start,           State::NotReadyToSwitchOn, State::SwitchOnDisabled,
                             State::ReadyToSwitchOn, State::SwitchedOn,         State::OperationEnabled,
                             State::QuickStopActive, State::FaultReactionActive, State::Fault};
constexpr Command kCommands[] = {Command::None,           Command::Shutdown,  Command::SwitchOn,
                                 Command::EnableOperation, Command::DisableVoltage, Command::QuickStop,
                                 Command::FaultReset};

static_assert(sizeof(kStates) / sizeof(kStates[0]) == kStateCount, "all states listed");
static_assert(sizeof(kCommands) / sizeof(kCommands[0]) == kCommandCount, "all commands listed");

Inputs MakeInputs(unsigned bits, int16_t qs_option) {
  Inputs in{};
  in.init_done = (bits & 0x01U) != 0U;
  in.fault_active = (bits & 0x02U) != 0U;
  in.authority_ok = (bits & 0x04U) != 0U;
  in.liveness_ok = (bits & 0x08U) != 0U;
  in.config_valid = (bits & 0x10U) != 0U;
  in.quick_stop_done = (bits & 0x20U) != 0U;
  in.fault_reaction_done = (bits & 0x40U) != 0U;
  in.velocity_zero = (bits & 0x80U) != 0U;
  in.quick_stop_option = qs_option;
  return in;
}

/** Independent reference, written as a plain switch from the CiA 402 diagram. */
Decision Reference(State s, Command c, const Inputs& in) {
  Decision d{};
  d.next = s;
  auto go = [&](uint8_t t, State to, Action a) {
    d.next = to;
    d.transition = t;
    d.action = a;
    return d;
  };
  auto refuse = [&](Refusal r) {
    d.refusal = r;
    if (r == Refusal::Liveness) {
      d.emcy = 0x8100;
      d.warning = true;
    }
    if (r == Refusal::ConfigInvalid) d.emcy = 0x6320;
    return d;
  };
  const bool dv_or_qs = c == Command::DisableVoltage || c == Command::QuickStop;
  if (s == State::Start) return go(0, State::NotReadyToSwitchOn, Action::None);
  if (in.fault_active && s != State::FaultReactionActive && s != State::Fault) {
    return go(13, State::FaultReactionActive, Action::StopFaultReaction);
  }
  switch (s) {
    case State::NotReadyToSwitchOn:
      if (in.init_done) return go(1, State::SwitchOnDisabled, Action::None);
      return d;
    case State::SwitchOnDisabled:
      if (c == Command::Shutdown) {
        if (!in.authority_ok) return refuse(Refusal::Authority);
        return go(2, State::ReadyToSwitchOn, Action::None);
      }
      return d;
    case State::ReadyToSwitchOn:
      if (c == Command::SwitchOn || c == Command::EnableOperation) {
        if (!in.authority_ok) return refuse(Refusal::Authority);
        if (!in.config_valid) return refuse(Refusal::ConfigInvalid);
        return go(3, State::SwitchedOn, Action::ApplyConfig);
      }
      if (dv_or_qs) return go(7, State::SwitchOnDisabled, Action::None);
      return d;
    case State::SwitchedOn:
      if (c == Command::EnableOperation) {
        if (!in.authority_ok) return refuse(Refusal::Authority);
        if (!in.liveness_ok) return refuse(Refusal::Liveness);
        return go(4, State::OperationEnabled, Action::EnableDrive);
      }
      if (c == Command::Shutdown) return go(6, State::ReadyToSwitchOn, Action::None);
      if (dv_or_qs) return go(10, State::SwitchOnDisabled, Action::None);
      return d;
    case State::OperationEnabled:
      if (c == Command::SwitchOn) return go(5, State::SwitchedOn, Action::StopDisableOperation);
      if (c == Command::Shutdown) return go(8, State::ReadyToSwitchOn, Action::StopShutdown);
      if (c == Command::DisableVoltage) return go(9, State::SwitchOnDisabled, Action::DisableDrive);
      if (c == Command::QuickStop) return go(11, State::QuickStopActive, Action::StopQuickStop);
      return d;
    case State::QuickStopActive: {
      const int16_t q = in.quick_stop_option;
      if (c == Command::DisableVoltage) return go(12, State::SwitchOnDisabled, Action::DisableDrive);
      if (q >= 0 && q <= 4 && (q == 0 || in.quick_stop_done)) {
        return go(12, State::SwitchOnDisabled, Action::DisableDrive);
      }
      if (c == Command::EnableOperation && q >= 5 && q <= 8) {
        if (!in.authority_ok) return refuse(Refusal::Authority);
        if (!in.liveness_ok) return refuse(Refusal::Liveness);
        return go(16, State::OperationEnabled, Action::EnableDrive);
      }
      return d;
    }
    case State::FaultReactionActive:
      if (in.fault_reaction_done) return go(14, State::Fault, Action::DisableDrive);
      return d;
    case State::Fault:
      if (c == Command::FaultReset) {
        if (!in.authority_ok) return refuse(Refusal::Authority);
        if (in.fault_active) return refuse(Refusal::FaultActive);
        return go(15, State::SwitchOnDisabled, Action::ResetFault);
      }
      return d;
    case State::Start:
      break;
  }
  return d;
}

bool Same(const Decision& a, const Decision& b) {
  return a.next == b.next && a.transition == b.transition && a.action == b.action && a.refusal == b.refusal &&
         a.emcy == b.emcy && a.warning == b.warning;
}

TEST_CASE(decide_matches_reference_exhaustively) {
  unsigned long mismatches = 0;
  for (State s : kStates) {
    for (Command c : kCommands) {
      for (unsigned bits = 0; bits < 256U; ++bits) {
        for (int16_t q = -1; q <= 9; ++q) {
          const Inputs in = MakeInputs(bits, q);
          if (!Same(Decide(s, c, in), Reference(s, c, in))) {
            if (mismatches < 10U) {
              std::fprintf(stderr, "mismatch: %s cmd=%u bits=0x%02x q=%d\n", StateName(s), static_cast<unsigned>(c),
                           bits, q);
            }
            ++mismatches;
          }
        }
      }
    }
  }
  CHECK_EQ(mismatches, 0UL);
}

TEST_CASE(every_transition_fires_from_exactly_its_source_states) {
  const std::set<std::pair<int, State>> expected = {
      {0, State::Start},
      {1, State::NotReadyToSwitchOn},
      {2, State::SwitchOnDisabled},
      {3, State::ReadyToSwitchOn},
      {4, State::SwitchedOn},
      {5, State::OperationEnabled},
      {6, State::SwitchedOn},
      {7, State::ReadyToSwitchOn},
      {8, State::OperationEnabled},
      {9, State::OperationEnabled},
      {10, State::SwitchedOn},
      {11, State::OperationEnabled},
      {12, State::QuickStopActive},
      {13, State::NotReadyToSwitchOn},
      {13, State::SwitchOnDisabled},
      {13, State::ReadyToSwitchOn},
      {13, State::SwitchedOn},
      {13, State::OperationEnabled},
      {13, State::QuickStopActive},
      {14, State::FaultReactionActive},
      {15, State::Fault},
      {16, State::QuickStopActive},
  };
  std::set<std::pair<int, State>> seen;
  for (State s : kStates) {
    for (Command c : kCommands) {
      for (unsigned bits = 0; bits < 256U; ++bits) {
        for (int16_t q = 0; q <= 8; ++q) {
          const Decision d = Decide(s, c, MakeInputs(bits, q));
          if (d.transition != kNoTransition) seen.insert({d.transition, s});
          if (d.transition == kNoTransition) CHECK(d.next == s);
        }
      }
    }
  }
  CHECK(seen == expected);
  for (int t = 0; t < kTransitionCount; ++t) {
    bool found = false;
    for (const auto& p : seen) found = found || p.first == t;
    if (!found) std::fprintf(stderr, "transition %d never fired\n", t);
    CHECK(found);
  }
}

/** One explicit row per transition with all gates open (documentation of the happy path). */
TEST_CASE(transitions_happy_path) {
  Inputs ok = MakeInputs(0x01U | 0x04U | 0x08U | 0x10U, 2);
  struct Row {
    State from;
    Command cmd;
    Inputs in;
    uint8_t t;
    State to;
    Action a;
  };
  Inputs fault = ok;
  fault.fault_active = true;
  Inputs reacted = fault;
  reacted.fault_reaction_done = true;
  Inputs qs_done = ok;
  qs_done.quick_stop_done = true;
  Inputs qs_stay = ok;
  qs_stay.quick_stop_option = 6;
  const Row rows[] = {
      {State::Start, Command::None, ok, 0, State::NotReadyToSwitchOn, Action::None},
      {State::NotReadyToSwitchOn, Command::None, ok, 1, State::SwitchOnDisabled, Action::None},
      {State::SwitchOnDisabled, Command::Shutdown, ok, 2, State::ReadyToSwitchOn, Action::None},
      {State::ReadyToSwitchOn, Command::SwitchOn, ok, 3, State::SwitchedOn, Action::ApplyConfig},
      {State::ReadyToSwitchOn, Command::EnableOperation, ok, 3, State::SwitchedOn, Action::ApplyConfig},
      {State::SwitchedOn, Command::EnableOperation, ok, 4, State::OperationEnabled, Action::EnableDrive},
      {State::OperationEnabled, Command::SwitchOn, ok, 5, State::SwitchedOn, Action::StopDisableOperation},
      {State::SwitchedOn, Command::Shutdown, ok, 6, State::ReadyToSwitchOn, Action::None},
      {State::ReadyToSwitchOn, Command::DisableVoltage, ok, 7, State::SwitchOnDisabled, Action::None},
      {State::ReadyToSwitchOn, Command::QuickStop, ok, 7, State::SwitchOnDisabled, Action::None},
      {State::OperationEnabled, Command::Shutdown, ok, 8, State::ReadyToSwitchOn, Action::StopShutdown},
      {State::OperationEnabled, Command::DisableVoltage, ok, 9, State::SwitchOnDisabled, Action::DisableDrive},
      {State::SwitchedOn, Command::DisableVoltage, ok, 10, State::SwitchOnDisabled, Action::None},
      {State::SwitchedOn, Command::QuickStop, ok, 10, State::SwitchOnDisabled, Action::None},
      {State::OperationEnabled, Command::QuickStop, ok, 11, State::QuickStopActive, Action::StopQuickStop},
      {State::QuickStopActive, Command::QuickStop, qs_done, 12, State::SwitchOnDisabled, Action::DisableDrive},
      {State::QuickStopActive, Command::DisableVoltage, qs_stay, 12, State::SwitchOnDisabled, Action::DisableDrive},
      {State::OperationEnabled, Command::EnableOperation, fault, 13, State::FaultReactionActive,
       Action::StopFaultReaction},
      {State::FaultReactionActive, Command::None, reacted, 14, State::Fault, Action::DisableDrive},
      {State::Fault, Command::FaultReset, ok, 15, State::SwitchOnDisabled, Action::ResetFault},
      {State::QuickStopActive, Command::EnableOperation, qs_stay, 16, State::OperationEnabled, Action::EnableDrive},
  };
  for (const Row& r : rows) {
    const Decision d = Decide(r.from, r.cmd, r.in);
    CHECK_EQ(d.transition, r.t);
    CHECK(d.next == r.to);
    CHECK(d.action == r.a);
    CHECK(d.refusal == Refusal::None);
  }
}

TEST_CASE(gates_refuse_and_report) {
  Inputs in = MakeInputs(0x01U | 0x08U | 0x10U, 2);  // no authority
  CHECK(Decide(State::SwitchOnDisabled, Command::Shutdown, in).refusal == Refusal::Authority);
  CHECK(Decide(State::SwitchedOn, Command::EnableOperation, in).refusal == Refusal::Authority);
  CHECK(Decide(State::Fault, Command::FaultReset, in).refusal == Refusal::Authority);
  // towards safety is never gated
  CHECK_EQ(Decide(State::OperationEnabled, Command::QuickStop, in).transition, 11);
  CHECK_EQ(Decide(State::OperationEnabled, Command::DisableVoltage, in).transition, 9);
  CHECK_EQ(Decide(State::OperationEnabled, Command::Shutdown, in).transition, 8);
  CHECK_EQ(Decide(State::OperationEnabled, Command::SwitchOn, in).transition, 5);

  in.authority_ok = true;
  in.liveness_ok = false;
  const Decision live = Decide(State::SwitchedOn, Command::EnableOperation, in);
  CHECK(live.refusal == Refusal::Liveness);
  CHECK_EQ(live.emcy, 0x8100U);
  CHECK(live.warning);
  CHECK(live.next == State::SwitchedOn);

  in.config_valid = false;
  const Decision cfg = Decide(State::ReadyToSwitchOn, Command::SwitchOn, in);
  CHECK(cfg.refusal == Refusal::ConfigInvalid);
  CHECK_EQ(cfg.emcy, 0x6320U);
  CHECK(cfg.next == State::ReadyToSwitchOn);
}

TEST_CASE(fault_reset_needs_cleared_causes_and_edge) {
  Inputs in = MakeInputs(0x01U | 0x04U | 0x08U | 0x10U | 0x02U, 2);  // fault still active
  CHECK(Decide(State::Fault, Command::FaultReset, in).refusal == Refusal::FaultActive);
  // a held bit 7 is not a command
  CHECK(DecodeControlword(0x0080U, 0x0080U) == Command::None);
  CHECK(DecodeControlword(0x0080U, 0x0000U) == Command::FaultReset);
  CHECK(DecodeControlword(0x008FU, 0x0006U) == Command::FaultReset);
  CHECK(DecodeControlword(0x008FU, 0x008FU) == Command::None);
}

TEST_CASE(quick_stop_option_codes) {
  Inputs in = MakeInputs(0x01U | 0x04U | 0x08U | 0x10U, 0);
  for (int16_t q = 0; q <= 8; ++q) {
    in.quick_stop_option = q;
    in.quick_stop_done = false;
    const Decision busy = Decide(State::QuickStopActive, Command::QuickStop, in);
    if (q == 0) {
      CHECK_EQ(busy.transition, 12);  // disable drive function at once
    } else {
      CHECK(busy.transition == kNoTransition);  // still ramping
    }
    in.quick_stop_done = true;
    const Decision done = Decide(State::QuickStopActive, Command::QuickStop, in);
    if (q <= 4) {
      CHECK_EQ(done.transition, 12);
    } else {
      CHECK(done.transition == kNoTransition);  // 5..8 stay in QuickStopActive
    }
    const Decision enable = Decide(State::QuickStopActive, Command::EnableOperation, in);
    CHECK_EQ(enable.transition, q >= 5 ? 16 : 12);
  }
}

TEST_CASE(controlword_decode_exhaustive) {
  for (uint32_t w = 0; w <= 0xFFFFU; ++w) {
    const uint16_t cwv = static_cast<uint16_t>(w);
    const unsigned low = cwv & 0x0FU;
    Command want;
    if ((cwv & 0x80U) != 0U) {
      want = Command::None;  // previous had bit 7 high
    } else if ((low & 0x2U) == 0U) {
      want = Command::DisableVoltage;
    } else if ((low & 0x4U) == 0U) {
      want = Command::QuickStop;
    } else if ((low & 0x1U) == 0U) {
      want = Command::Shutdown;
    } else if ((low & 0x8U) == 0U) {
      want = Command::SwitchOn;
    } else {
      want = Command::EnableOperation;
    }
    CHECK(DecodeControlword(cwv, 0x0080U) == want);
    // bits 4–6 and 8–15 never change the command
    CHECK(DecodeControlword(cwv, 0x0080U) == DecodeControlword(static_cast<uint16_t>(cwv & 0x008FU), 0x0080U));
  }
}

TEST_CASE(controlword_round_trip) {
  for (Command c : kCommands) {
    if (c == Command::None) continue;
    CHECK(DecodeControlword(EncodeControlword(c), 0x0000U) == c);
    // with mode bits and halt set the command survives
    CHECK(DecodeControlword(static_cast<uint16_t>(EncodeControlword(c) | 0x0170U), 0x0000U) == c);
  }
  CHECK(DecodeControlword(EncodeControlword(Command::None), EncodeControlword(Command::None)) == Command::None);
  CHECK_EQ(EncodeControlword(Command::Shutdown), 0x0006U);
  CHECK_EQ(EncodeControlword(Command::SwitchOn), 0x0007U);
  CHECK_EQ(EncodeControlword(Command::EnableOperation), 0x000FU);
  CHECK_EQ(EncodeControlword(Command::DisableVoltage), 0x0000U);
  CHECK_EQ(EncodeControlword(Command::QuickStop), 0x0002U);
  CHECK_EQ(EncodeControlword(Command::FaultReset), 0x0080U);
  CHECK(IsHalt(0x010FU));
  CHECK(!IsHalt(0x000FU));
}

TEST_CASE(statusword_round_trip) {
  for (State s : kStates) {
    for (unsigned bits = 0; bits < 128U; ++bits) {
      StatusFlags f{};
      f.voltage_enabled = (bits & 0x01U) != 0U;
      f.warning = (bits & 0x02U) != 0U;
      f.remote = (bits & 0x04U) != 0U;
      f.target_reached = (bits & 0x08U) != 0U;
      f.internal_limit = (bits & 0x10U) != 0U;
      f.op_specific_12 = (bits & 0x20U) != 0U;
      f.op_specific_13 = (bits & 0x40U) != 0U;
      const uint16_t w = EncodeStatusword(s, f);
      const State want = s == State::Start ? State::NotReadyToSwitchOn : s;
      CHECK(DecodeStatusword(w) == want);
      CHECK_EQ(w & 0x006FU, StateBits(s));
      CHECK_EQ((w & sw::kVoltageEnabled) != 0U, f.voltage_enabled);
      CHECK_EQ((w & sw::kWarning) != 0U, f.warning);
      CHECK_EQ((w & sw::kRemote) != 0U, f.remote);
      CHECK_EQ((w & sw::kTargetReached) != 0U, f.target_reached);
      CHECK_EQ((w & sw::kInternalLimit) != 0U, f.internal_limit);
      CHECK_EQ((w & sw::kOpSpecific12) != 0U, f.op_specific_12);
      CHECK_EQ((w & sw::kOpSpecific13) != 0U, f.op_specific_13);
    }
  }
  // known values (CiA 402-2 statusword table)
  StatusFlags v{};
  v.voltage_enabled = true;
  v.remote = true;
  CHECK_EQ(EncodeStatusword(State::SwitchOnDisabled, v), 0x0250U);
  CHECK_EQ(EncodeStatusword(State::ReadyToSwitchOn, v), 0x0231U);
  CHECK_EQ(EncodeStatusword(State::SwitchedOn, v), 0x0233U);
  CHECK_EQ(EncodeStatusword(State::OperationEnabled, v), 0x0237U);
  CHECK_EQ(EncodeStatusword(State::QuickStopActive, v), 0x0217U);
  CHECK_EQ(EncodeStatusword(State::FaultReactionActive, v), 0x021FU);
  CHECK_EQ(EncodeStatusword(State::Fault, v), 0x0218U);
  // decode ignores the x bits a master may see from other drives
  CHECK(DecodeStatusword(0x0060U) == State::SwitchOnDisabled);
  CHECK(DecodeStatusword(0x0028U) == State::Fault);
  CHECK(DecodeStatusword(0x002FU) == State::FaultReactionActive);
  CHECK(DecodeStatusword(0x0001U) == State::Start);  // no state has this pattern
}

TEST_CASE(option_codes) {
  using namespace option;
  CHECK_EQ(kQuickStopDefault, 2);
  CHECK_EQ(kShutdownDefault, 0);
  CHECK_EQ(kDisableOperationDefault, 1);
  CHECK_EQ(kHaltDefault, 1);
  CHECK_EQ(kFaultReactionDefault, 0);
  CHECK_EQ(kAbortConnectionDefault, 1);
  for (int v = -40000; v <= 40000; v += 1) {
    if (v < -32768 || v > 32767) continue;
    const int16_t c = static_cast<int16_t>(v);
    CHECK_EQ(IsValidQuickStop(c), v >= 0 && v <= 8);
    CHECK_EQ(IsValidShutdown(c), v == 0 || v == 1);
    CHECK_EQ(IsValidDisableOperation(c), v == 0 || v == 1);
    CHECK_EQ(IsValidHalt(c), v >= 1 && v <= 4);
    CHECK_EQ(IsValidFaultReaction(c), v >= 0 && v <= 4);
    CHECK_EQ(IsValidAbortConnection(c), v >= 0 && v <= 3);
  }
  const StopRamp qs[] = {StopRamp::Coast,         StopRamp::SlowDown,     StopRamp::QuickStopRamp,
                         StopRamp::CurrentLimit,  StopRamp::VoltageLimit, StopRamp::SlowDown,
                         StopRamp::QuickStopRamp, StopRamp::CurrentLimit, StopRamp::VoltageLimit};
  for (int16_t c = 0; c <= 8; ++c) {
    CHECK(QuickStopRampOf(c) == qs[c]);
    CHECK_EQ(QuickStopStays(c), c >= 5);
  }
  CHECK(ShutdownRampOf(0) == StopRamp::Coast);
  CHECK(ShutdownRampOf(1) == StopRamp::SlowDown);
  CHECK(DisableOperationRampOf(0) == StopRamp::Coast);
  CHECK(DisableOperationRampOf(1) == StopRamp::SlowDown);
  CHECK(HaltRampOf(1) == StopRamp::SlowDown);
  CHECK(HaltRampOf(2) == StopRamp::QuickStopRamp);
  CHECK(HaltRampOf(3) == StopRamp::CurrentLimit);
  CHECK(HaltRampOf(4) == StopRamp::VoltageLimit);
  for (int16_t c = 0; c <= 4; ++c) {
    CHECK(FaultReactionRampOf(c, true) == static_cast<StopRamp>(c));
    CHECK(FaultReactionRampOf(c, false) == StopRamp::Coast);  // bridge faulted: coast only
  }
  CHECK(AbortConnectionEffect(0) == AbortEffect::None);
  CHECK(AbortConnectionEffect(1) == AbortEffect::Fault);
  CHECK(AbortConnectionEffect(2) == AbortEffect::DisableVoltage);
  CHECK(AbortConnectionEffect(3) == AbortEffect::QuickStop);
  CHECK(AbortConnectionEffect(7) == AbortEffect::Fault);  // invalid: fail safe
  CHECK(AbortConnectionCommand(2) == Command::DisableVoltage);
  CHECK(AbortConnectionCommand(3) == Command::QuickStop);
  CHECK(AbortConnectionCommand(1) == Command::None);
  // 0x6007 = 3 from OperationEnabled is a quick stop (transition 11)
  Inputs in = MakeInputs(0x1DU, 2);  // all gates open, no fault
  CHECK_EQ(Decide(State::OperationEnabled, AbortConnectionCommand(3), in).transition, 11);
  CHECK_EQ(Decide(State::OperationEnabled, AbortConnectionCommand(2), in).transition, 9);
}

TEST_CASE(modes_of_operation) {
  CHECK_EQ(kSupportedDriveModes, 0x000003ADUL);
  for (int m = -128; m <= 127; ++m) {
    const int8_t v = static_cast<int8_t>(m);
    const bool want = m == 1 || m == 3 || m == 4 || m == 6 || m == 8 || m == 9 || m == 10;
    CHECK_EQ(IsSupportedMode(v), want);
    CHECK_EQ(IsCyclicMode(v), m >= 8 && m <= 10);
  }
  for (State s : kStates) {
    const bool off = s == State::NotReadyToSwitchOn || s == State::SwitchOnDisabled ||
                     s == State::ReadyToSwitchOn || s == State::SwitchedOn || s == State::Fault;
    CHECK_EQ(ModeChangeAllowed(s, false), off);
    CHECK_EQ(ModeChangeAllowed(s, true), off || s == State::OperationEnabled);
  }
  CHECK(DriveFunctionEnabled(State::OperationEnabled));
  CHECK(!DriveFunctionEnabled(State::SwitchedOn));
}

// The table is constexpr: a walk through the enable sequence compiles to constants.
constexpr State Walk() {
  Inputs in{};
  in.init_done = in.authority_ok = in.liveness_ok = in.config_valid = true;
  State s = State::Start;
  s = Decide(s, Command::None, in).next;
  s = Decide(s, Command::None, in).next;
  s = Decide(s, Command::Shutdown, in).next;
  s = Decide(s, Command::EnableOperation, in).next;  // 3
  s = Decide(s, Command::EnableOperation, in).next;  // 4
  return s;
}
static_assert(Walk() == State::OperationEnabled, "constexpr enable walk");

}  // namespace
