/// Host test — fault policy table (contracts/vortex_fault_policy.hpp, design §6.4).
///
/// Pins every row (a changed code, ER bit or reaction is a changed safety
/// decision and must fail here until the design table changes with it),
/// checks the rules every row keeps, the conditional reactions and the
/// escalation / fault-reset rules of the FaultBook.

#include "contracts/vortex_fault_policy.hpp"

#include <cstring>

#include "support/check.hpp"

namespace {

using namespace vortex::contracts::fault;

struct Expect {
  Cause cause;
  uint16_t emcy;
  uint8_t er;
  Reaction reaction;
  uint8_t conditions;
};

constexpr uint8_t G = er::kGeneric, I = er::kCurrent, V = er::kVoltage, T = er::kTemperature,
                  C = er::kCommunication, P = er::kProfile, M = er::kManufacturer;

/** The design §6.4 table, transcribed independently of the header. */
const Expect kApproved[] = {
    {Cause::GateShortCircuit, 0x2320, G | I, Reaction::DisableNow, cond::kNone},
    {Cause::GateOvercurrent, 0x2310, G | I, Reaction::DisableNow, cond::kNone},
    {Cause::I2tLimit1, 0x2310, G | I, Reaction::Fault, cond::kNone},
    {Cause::I2tLimit2, 0x2310, G | I, Reaction::Fault, cond::kNone},
    {Cause::SupplyOvervoltage, 0x3210, G | V, Reaction::DisableNow, cond::kNone},
    {Cause::SupplyUndervoltage, 0x3220, G | V, Reaction::Fault, cond::kNone},
    {Cause::PowerGoodLost, 0x3220, G | V, Reaction::Fault, cond::kNone},
    {Cause::SupplyVoltageWarning, 0x3200, V, Reaction::Warn, cond::kNone},
    {Cause::ChipOvertemperature, 0x4210, G | T, Reaction::Fault, cond::kNone},
    {Cause::ChipTemperatureWarning, 0x4210, T, Reaction::Warn, cond::kNone},
    {Cause::NtcOvertemperature, 0x4310, G | T, Reaction::Fault, cond::kNone},
    {Cause::HallInvalid, 0x7305, G | P, Reaction::Fault, cond::kNone},
    {Cause::HostEncoderFault, 0x7300, G | P, Reaction::Fault, cond::kWarnUnlessPositionSource},
    {Cause::ImuLost, 0xFF10, M, Reaction::Warn, cond::kNone},
    {Cause::TmcCommsFault, 0x7510, G | M, Reaction::DisableNow, cond::kNone},
    {Cause::TmcHeartbeatStopped, 0x6010, G | M, Reaction::Fault, cond::kNone},
    {Cause::FollowingError, 0x8611, G | P, Reaction::Fault, cond::kNone},
    {Cause::SoftwareLimit, 0x8612, P, Reaction::QuickStop, cond::kNone},
    {Cause::CanRxOverflow, 0x8110, C, Reaction::Warn, cond::kNone},
    {Cause::CanErrorPassive, 0x8120, C, Reaction::Warn, cond::kNone},
    {Cause::HeartbeatConsumerLoss, 0x8130, C, Reaction::AbortConnection, cond::kNone},
    {Cause::BusOffRecovered, 0x8140, C, Reaction::Warn, cond::kAbortIfOperationEnabled},
    {Cause::PdoLengthError, 0x8210, C, Reaction::Warn, cond::kNone},
    {Cause::PdoLengthExceeded, 0x8220, C, Reaction::Warn, cond::kNone},
    {Cause::SyncLengthError, 0x8240, C, Reaction::AbortConnection, cond::kAbortOnlyCyclic},
    {Cause::SyncTimeout, 0x8100, C, Reaction::AbortConnection, cond::kAbortOnlyCyclic},
    {Cause::RpdoTimeout, 0x8250, C, Reaction::AbortConnection, cond::kNone},
    {Cause::ParameterInvalid, 0x6320, G | M, Reaction::RefuseSwitchOn, cond::kNone},
    {Cause::StoreCrcMismatch, 0x6320, G | M, Reaction::RefuseSwitchOn, cond::kNone},
    {Cause::RtStall, 0x6100, G | M, Reaction::Reset, cond::kNone},
    {Cause::SvcStall, 0x6100, G | M, Reaction::Reset, cond::kNone},
};
static_assert(sizeof(kApproved) / sizeof(kApproved[0]) == kCauseCount, "every cause is pinned");

TEST_CASE(every_row_matches_the_design_table) {
  for (const Expect& e : kApproved) {
    const Policy& p = PolicyOf(e.cause);
    CHECK(p.cause == e.cause);
    CHECK_EQ(p.emcy, e.emcy);
    CHECK_EQ(p.er, e.er);
    CHECK(p.reaction == e.reaction);
    CHECK_EQ(p.conditions, e.conditions);
  }
}

/** ER bit implied by the EMCY code class (CiA 301 error code table, §6.4 layout). */
uint8_t ClassBit(uint16_t code) {
  switch (code >> 12) {
    case 0x2: return er::kCurrent;
    case 0x3: return er::kVoltage;
    case 0x4: return er::kTemperature;
    case 0x6: return er::kManufacturer;  // device software: reported as manufacturer specific
    case 0x7:
      return (code & 0x0F00U) == 0x0300U ? er::kProfile : er::kManufacturer;  // 73xx sensor, 75xx internal comms
    case 0x8: return (code & 0x0F00U) == 0x0600U ? er::kProfile : er::kCommunication;  // 86xx positioning
    case 0xF: return er::kManufacturer;
    default: return 0U;
  }
}

TEST_CASE(rules_every_row_keeps) {
  for (uint8_t i = 0; i < kCauseCount; ++i) {
    const Policy& p = kPolicy[i];
    CHECK_EQ(static_cast<uint8_t>(p.cause), i);  // index = cause id
    CHECK(p.name != nullptr && std::strlen(p.name) > 0U);
    for (uint8_t j = 0; j < i; ++j) CHECK(std::strcmp(p.name, kPolicy[j].name) != 0);
    CHECK(p.emcy != 0U);                                    // every cause reports one EMCY code
    CHECK((p.er & 0x40U) == 0U);                            // bit 6 is reserved
    CHECK((p.er & ClassBit(p.emcy)) != 0U);                 // ER carries the code's class bit
    if (p.reaction == Reaction::Warn || p.reaction == Reaction::AbortConnection ||
        p.reaction == Reaction::QuickStop) {
      CHECK((p.er & er::kGeneric) == 0U);  // generic error only for faults
    } else {
      CHECK((p.er & er::kGeneric) != 0U);
    }
  }
}

TEST_CASE(conditional_reactions) {
  Context ctx{};
  CHECK(EffectiveReaction(Cause::HostEncoderFault, ctx) == Reaction::Warn);
  ctx.sensor_is_position_source = true;
  CHECK(EffectiveReaction(Cause::HostEncoderFault, ctx) == Reaction::Fault);

  Context oe{};
  CHECK(EffectiveReaction(Cause::BusOffRecovered, oe) == Reaction::Warn);
  oe.operation_enabled = true;
  CHECK(EffectiveReaction(Cause::BusOffRecovered, oe) == Reaction::AbortConnection);

  Context cyc{};
  CHECK(EffectiveReaction(Cause::SyncTimeout, cyc) == Reaction::Warn);
  CHECK(EffectiveReaction(Cause::SyncLengthError, cyc) == Reaction::Warn);
  cyc.cyclic_mode = true;
  CHECK(EffectiveReaction(Cause::SyncTimeout, cyc) == Reaction::AbortConnection);
  CHECK(EffectiveReaction(Cause::SyncLengthError, cyc) == Reaction::AbortConnection);

  // unconditional rows ignore the context
  for (uint8_t i = 0; i < kCauseCount; ++i) {
    if (kPolicy[i].conditions != cond::kNone) continue;
    Context all{true, true, true};
    CHECK(EffectiveReaction(static_cast<Cause>(i), all) == kPolicy[i].reaction);
    CHECK(EffectiveReaction(static_cast<Cause>(i), Context{}) == kPolicy[i].reaction);
  }
  CHECK(!BridgeHealthy(Reaction::DisableNow));
  CHECK(BridgeHealthy(Reaction::Fault));
  CHECK(FaultsAxis(Reaction::AbortConnection, true));
  CHECK(!FaultsAxis(Reaction::AbortConnection, false));
}

TEST_CASE(emcy_manufacturer_bytes) {
  uint8_t b[5]{};
  PackEmcyManufacturerBytes(Cause::GateOvercurrent, 0xA1B2C3D4U, b);
  CHECK_EQ(b[0], 1U);
  CHECK_EQ(b[1], 0xD4U);
  CHECK_EQ(b[2], 0xC3U);
  CHECK_EQ(b[3], 0xB2U);
  CHECK_EQ(b[4], 0xA1U);
}

TEST_CASE(every_row_through_the_fault_book) {
  for (uint8_t i = 0; i < kCauseCount; ++i) {
    const Cause c = static_cast<Cause>(i);
    Context ctx{false, false, true};
    FaultBook fb{};
    const RaiseResult r = fb.Raise(c, 1000U, ctx);
    CHECK(r.first);
    CHECK(!r.escalated);
    CHECK(r.reaction == EffectiveReaction(c, ctx));
    CHECK(fb.IsActive(c));
    CHECK_EQ(fb.ErrorRegister(), kPolicy[i].er);
    const bool latched = Latches(r.reaction);
    CHECK_EQ((fb.ActiveMask() & CauseBit(c)) != 0U, latched);
    CHECK_EQ((fb.WarningMask() & CauseBit(c)) != 0U, !latched);
    CHECK_EQ(fb.FaultActive(ctx, true), FaultsAxis(r.reaction, true));
    // reset while present keeps it; once gone it clears
    CHECK(!fb.FaultReset(CauseBit(c), 0U) || !latched);
    CHECK(fb.FaultReset(0U, 0U));
    CHECK(!fb.IsActive(c));
    CHECK_EQ(fb.ErrorRegister(), 0U);
  }
}

TEST_CASE(escalation_needs_the_cleared_predicate) {
  FaultBook fb{};
  const Context ctx{};
  CHECK(!fb.Raise(Cause::I2tLimit1, 0U, ctx).escalated);
  CHECK(!fb.Raise(Cause::I2tLimit1, 1000U, ctx).escalated);
  const RaiseResult third = fb.Raise(Cause::I2tLimit1, 2000U, ctx);
  CHECK(third.escalated);
  CHECK(!third.first);
  CHECK((fb.EscalatedMask() & CauseBit(Cause::I2tLimit1)) != 0U);
  CHECK(!fb.FaultResetAllowed(0U, 0U));  // condition gone, predicate not yet
  CHECK(!fb.FaultReset(0U, 0U));
  CHECK(fb.IsActive(Cause::I2tLimit1));
  CHECK(fb.FaultResetAllowed(0U, CauseBit(Cause::I2tLimit1)));
  CHECK(fb.FaultReset(0U, CauseBit(Cause::I2tLimit1)));
  CHECK_EQ(fb.EscalatedMask(), 0U);
}

TEST_CASE(escalation_window) {
  FaultBook fb{};
  const Context ctx{};
  fb.Raise(Cause::FollowingError, 0U, ctx);
  fb.Raise(Cause::FollowingError, kEscalateWindowMs, ctx);
  CHECK(fb.Raise(Cause::FollowingError, kEscalateWindowMs + 1U, ctx).escalated == false);  // first fell out
  FaultBook fb2{};
  fb2.Raise(Cause::FollowingError, 0U, ctx);
  fb2.Raise(Cause::FollowingError, 1U, ctx);
  CHECK(fb2.Raise(Cause::FollowingError, kEscalateWindowMs, ctx).escalated);  // inclusive window
  // warnings and resets never escalate
  FaultBook fb3{};
  for (uint32_t t = 0; t < 10U; ++t) {
    CHECK(!fb3.Raise(Cause::ImuLost, t, ctx).escalated);
    CHECK(!fb3.Raise(Cause::RtStall, t, ctx).escalated);
  }
  CHECK_EQ(fb3.EscalatedMask(), 0U);
}

TEST_CASE(non_fault_causes_release_when_gone) {
  FaultBook fb{};
  const Context ctx{};
  fb.Raise(Cause::SoftwareLimit, 0U, ctx);
  fb.Raise(Cause::ImuLost, 0U, ctx);
  fb.Raise(Cause::FollowingError, 0U, ctx);
  CHECK(fb.FaultActive(ctx, true));
  fb.ConditionGone(Cause::SoftwareLimit, ctx, true);
  fb.ConditionGone(Cause::ImuLost, ctx, true);
  fb.ConditionGone(Cause::FollowingError, ctx, true);
  CHECK(!fb.IsActive(Cause::SoftwareLimit));
  CHECK(!fb.IsActive(Cause::ImuLost));
  CHECK(fb.IsActive(Cause::FollowingError));  // faults wait for fault reset
  // 0x6007 = fault: heartbeat loss latches until fault reset; otherwise it releases
  fb.Raise(Cause::HeartbeatConsumerLoss, 0U, ctx);
  fb.ConditionGone(Cause::HeartbeatConsumerLoss, ctx, true);
  CHECK(fb.IsActive(Cause::HeartbeatConsumerLoss));
  fb.ConditionGone(Cause::HeartbeatConsumerLoss, ctx, false);
  CHECK(!fb.IsActive(Cause::HeartbeatConsumerLoss));
}

TEST_CASE(error_register_is_the_or_of_active_rows) {
  FaultBook fb{};
  const Context ctx{};
  fb.Raise(Cause::SupplyVoltageWarning, 0U, ctx);
  fb.Raise(Cause::CanRxOverflow, 0U, ctx);
  CHECK_EQ(fb.ErrorRegister(), er::kVoltage | er::kCommunication);
  CHECK(!fb.FaultActive(ctx, true));
  fb.Raise(Cause::GateShortCircuit, 0U, ctx);
  CHECK_EQ(fb.ErrorRegister(), er::kVoltage | er::kCommunication | er::kGeneric | er::kCurrent);
  CHECK(fb.FaultActive(ctx, false));
}

}  // namespace
