/// Host test — router policy and authority rules (contracts/router_policy.hpp,
/// design §4.3, §7.4) plus the CanFrame structural rules (contracts/can_frame.hpp).

#include "contracts/router_policy.hpp"

#include <initializer_list>

#include "support/check.hpp"

namespace {

using namespace vortex::contracts;
using namespace vortex::contracts::router;

constexpr Port kPorts[] = {Port::Can, Port::WifiTunnel, Port::EspNowTunnel, Port::HostSim};
constexpr PortCaps kClassic{false, 8, 1000000, 0, 1};
constexpr PortCaps kFd{true, 64, 1000000, 5000000, 28};
constexpr PortCaps kTunnel{true, 64, 0, 0, 0};

CanFrame Frame(uint32_t id, uint8_t len, uint8_t b0 = 0) {
  CanFrame f{};
  f.id = id;
  f.len = len;
  f.data[0] = b0;
  return f;
}

TEST_CASE(can_frame_structure) {
  CHECK(CheckFrame(Frame(0x7FF, 8)) == FrameError::None);
  CHECK(CheckFrame(Frame(0x800, 0)) == FrameError::IdRange);
  CanFrame ext = Frame(0x1FFFFFFF, 0);
  ext.flags = frame_flag::kExt;
  CHECK(CheckFrame(ext) == FrameError::None);
  ext.id = 0x20000000;
  CHECK(CheckFrame(ext) == FrameError::IdRange);
  CHECK(CheckFrame(Frame(0x100, 9)) == FrameError::ClassicTooLong);
  CanFrame fd = Frame(0x100, 64);
  fd.flags = frame_flag::kFdf | frame_flag::kBrs;
  CHECK(CheckFrame(fd) == FrameError::None);
  fd.len = 9;
  CHECK(CheckFrame(fd) == FrameError::FdLengthNotExact);
  fd.len = 65;
  CHECK(CheckFrame(fd) == FrameError::FdTooLong);
  fd.len = 8;
  fd.flags = frame_flag::kFdf | frame_flag::kRtr;
  CHECK(CheckFrame(fd) == FrameError::RtrOnFd);
  CanFrame brs = Frame(0x100, 8);
  brs.flags = frame_flag::kBrs;
  CHECK(CheckFrame(brs) == FrameError::FdBitsOnClassic);
  CanFrame bad = Frame(0x100, 0);
  bad.flags = 0x80;
  CHECK(CheckFrame(bad) == FrameError::UnknownFlag);
}

TEST_CASE(dlc_length_round_trip) {
  for (uint8_t dlc = 0; dlc < 16U; ++dlc) {
    CHECK_EQ(LenToDlc(DlcToLen(dlc, true), true), dlc);
    CHECK_EQ(DlcToLen(dlc, false), dlc > 8U ? 8U : dlc);
  }
  for (unsigned len = 0; len <= 64U; ++len) {
    const uint8_t l = static_cast<uint8_t>(len);
    const uint8_t dlc = LenToDlc(l, true);
    CHECK(DlcToLen(dlc, true) >= l);
    CHECK_EQ(LenToDlc(l, false) == 0xFFU, len > 8U);
  }
  CHECK_EQ(LenToDlc(65, true), 0xFFU);
  CHECK(IsExactFdLength(48));
  CHECK(!IsExactFdLength(47));
}

TEST_CASE(fits_port_enforces_caps) {
  CanFrame fd = Frame(0x181, 24);
  fd.flags = frame_flag::kFdf;
  CHECK(!FitsPort(fd, kClassic));
  CHECK(FitsPort(fd, kFd));
  fd.flags |= frame_flag::kBrs;
  CHECK(!FitsPort(fd, kTunnel));  // no data phase on a tunnel
  PortCaps small = kFd;
  small.max_len = 16;
  fd.flags = frame_flag::kFdf;
  CHECK(!FitsPort(fd, small));
  CHECK(FitsPort(Frame(0x181, 8), kClassic));
  CHECK_EQ(static_cast<int>(StateFromCounters(0, 0)), static_cast<int>(BusState::ErrorActive));
  CHECK(StateFromCounters(96, 0) == BusState::ErrorWarning);
  CHECK(StateFromCounters(0, 128) == BusState::ErrorPassive);
  CHECK(StateFromCounters(256, 0) == BusState::BusOff);
}

TEST_CASE(predefined_targets) {
  const uint8_t node = 5;
  for (Port p : kPorts) {
    CHECK(PredefinedTarget(p, Frame(0x000, 2), node) == RxTarget::Nmt);
    CHECK(PredefinedTarget(p, Frame(0x080, 0), node) == RxTarget::Sync);
    CHECK(PredefinedTarget(p, Frame(0x081, 8), node) == RxTarget::EmcyConsumer);
    CHECK(PredefinedTarget(p, Frame(0x100, 6), node) == RxTarget::Time);
    CHECK(PredefinedTarget(p, Frame(0x205, 3), node) == RxTarget::Rpdo);
    CHECK(PredefinedTarget(p, Frame(0x305, 6), node) == RxTarget::Rpdo);
    CHECK(PredefinedTarget(p, Frame(0x405, 6), node) == RxTarget::Rpdo);
    CHECK(PredefinedTarget(p, Frame(0x505, 4), node) == RxTarget::Rpdo);
    CHECK(PredefinedTarget(p, Frame(0x206, 3), node) == RxTarget::Unhandled);  // another node's RPDO
    CHECK(PredefinedTarget(p, Frame(0x605, 8), node) == RxTarget::Sdo1Server);
    CHECK(PredefinedTarget(p, Frame(0x701, 1), node) == RxTarget::HbConsumer);
    CHECK(PredefinedTarget(p, Frame(0x705, 1), node) == RxTarget::Unhandled);  // own heartbeat
    CHECK(PredefinedTarget(p, Frame(0x7E5, 8), node) == RxTarget::LssSlave);
    CHECK(PredefinedTarget(p, Frame(0x185, 8), node) == RxTarget::Unhandled);
  }
  CHECK(PredefinedTarget(Port::WifiTunnel, Frame(0x6C5, 8), node) == RxTarget::Sdo2Server);
  CHECK(PredefinedTarget(Port::EspNowTunnel, Frame(0x6C5, 8), node) == RxTarget::Sdo2Server);
  CHECK(PredefinedTarget(Port::Can, Frame(0x6C5, 8), node) == RxTarget::Unhandled);
  CHECK(PredefinedTarget(Port::HostSim, Frame(0x6C5, 8), node) == RxTarget::Unhandled);
}

/** The SDO server 2 IDs collide with physical IDs for node-IDs >= 0x41 (see header). */
TEST_CASE(sdo2_ids_do_not_shadow_physical_heartbeats) {
  const uint8_t node = 0x41;
  const CanFrame hb_of_node1 = Frame(0x6C0U + node, 1);  // == 0x701
  CHECK_EQ(hb_of_node1.id, 0x701U);
  CHECK(PredefinedTarget(Port::Can, hb_of_node1, node) == RxTarget::HbConsumer);
  CHECK(DecideRx(Port::Can, RxTarget::HbConsumer, hb_of_node1, kClassic, AuthorityPolicy{}) == Verdict::Allow);
  CHECK(PredefinedTarget(Port::WifiTunnel, hb_of_node1, node) == RxTarget::Sdo2Server);
}

TEST_CASE(rx_policy_table) {
  const AuthorityPolicy def{};
  AuthorityPolicy wifi_enable = def;
  wifi_enable.enable_ports = 0x03U;
  constexpr RxTarget targets[] = {RxTarget::Nmt,          RxTarget::Sync,       RxTarget::Time,
                                  RxTarget::EmcyConsumer, RxTarget::HbConsumer, RxTarget::Rpdo,
                                  RxTarget::Sdo1Server,   RxTarget::Sdo2Server, RxTarget::LssSlave};
  const CanFrame f = Frame(0x000, 2, nmt::kStart);
  for (Port p : kPorts) {
    for (RxTarget t : targets) {
      const Verdict v = DecideRx(p, t, f, kTunnel, def);
      Verdict want;
      if (!IsTunnel(p)) {
        want = t == RxTarget::Sdo2Server ? Verdict::DenyPolicy : Verdict::Allow;
      } else {
        want = t == RxTarget::Sdo2Server ? Verdict::Allow : Verdict::DenyPolicy;  // NMT start, RPDO need policy
      }
      CHECK(v == want);
    }
    CHECK(DecideRx(p, RxTarget::Unhandled, f, kTunnel, def) == Verdict::DenyUnhandled);
  }
  // policy grants Wi-Fi RPDO and full NMT, not ESP-NOW
  CHECK(DecideRx(Port::WifiTunnel, RxTarget::Rpdo, Frame(0x205, 3), kTunnel, wifi_enable) == Verdict::Allow);
  CHECK(DecideRx(Port::WifiTunnel, RxTarget::Nmt, f, kTunnel, wifi_enable) == Verdict::Allow);
  CHECK(DecideRx(Port::EspNowTunnel, RxTarget::Rpdo, Frame(0x205, 3), kTunnel, wifi_enable) == Verdict::DenyPolicy);
  CHECK(DecideRx(Port::EspNowTunnel, RxTarget::Nmt, f, kTunnel, wifi_enable) == Verdict::DenyPolicy);
  // length enforced against the arrival port
  CHECK(DecideRx(Port::Can, RxTarget::Rpdo, Frame(0x205, 9), kClassic, def) == Verdict::DenyLength);
}

TEST_CASE(tunnel_nmt_commands) {
  const AuthorityPolicy def{};
  AuthorityPolicy granted = def;
  granted.enable_ports = 0x07U;
  const uint8_t cmds[] = {nmt::kStart, nmt::kStop, nmt::kEnterPreOperational, nmt::kResetNode,
                          nmt::kResetCommunication, 0x55};
  for (Port p : {Port::WifiTunnel, Port::EspNowTunnel}) {
    for (uint8_t c : cmds) {
      const bool safe = c == nmt::kEnterPreOperational || c == nmt::kResetNode;
      const bool known = safe || c == nmt::kStart || c == nmt::kStop || c == nmt::kResetCommunication;
      CHECK_EQ(TunnelNmtAllowed(p, c, def), safe);
      CHECK_EQ(TunnelNmtAllowed(p, c, granted), known);
      CHECK((DecideRx(p, RxTarget::Nmt, Frame(0, 2, c), kTunnel, def) == Verdict::Allow) == safe);
    }
    CHECK(DecideRx(p, RxTarget::Nmt, Frame(0, 0), kTunnel, granted) == Verdict::DenyPolicy);  // empty NMT
  }
}

TEST_CASE(tx_policy_table) {
  constexpr TxOrigin origins[] = {TxOrigin::NmtHeartbeat, TxOrigin::Emcy,         TxOrigin::Tpdo,
                                  TxOrigin::Sdo1Response, TxOrigin::Sdo2Response, TxOrigin::SdoClient,
                                  TxOrigin::LssSlave,     TxOrigin::SyncProducer};
  static_assert(sizeof(origins) / sizeof(origins[0]) == kTxOriginCount, "all origins");
  const CanFrame f = Frame(0x185, 8);
  for (Port p : kPorts) {
    for (TxOrigin o : origins) {
      for (bool sub : {false, true}) {
        const Verdict v = DecideTx(p, o, f, kTunnel, sub, nullptr, 0);
        Verdict want;
        if (o == TxOrigin::SdoClient) {
          want = Verdict::DenyPolicy;
        } else if (!IsTunnel(p)) {
          want = o == TxOrigin::Sdo2Response ? Verdict::DenyPolicy : Verdict::Allow;
        } else if (o == TxOrigin::Sdo2Response) {
          want = Verdict::Allow;
        } else if (o == TxOrigin::NmtHeartbeat || o == TxOrigin::Emcy || o == TxOrigin::Tpdo) {
          want = sub ? Verdict::Allow : Verdict::DenyNotSubscribed;
        } else {
          want = Verdict::DenyPolicy;
        }
        CHECK(v == want);
      }
    }
  }
  CanFrame fd = Frame(0x185, 24);
  fd.flags = frame_flag::kFdf;
  CHECK(DecideTx(Port::Can, TxOrigin::Tpdo, fd, kClassic, false, nullptr, 0) == Verdict::DenyLength);
}

TEST_CASE(mirror_rate_limit) {
  RateLimiter lim{};
  const CanFrame hb = Frame(0x705, 1);
  CHECK(DecideTx(Port::WifiTunnel, TxOrigin::NmtHeartbeat, hb, kTunnel, true, &lim, 0) == Verdict::Allow);
  CHECK(DecideTx(Port::WifiTunnel, TxOrigin::NmtHeartbeat, hb, kTunnel, true, &lim, 9999) ==
        Verdict::DenyRateLimited);
  CHECK(DecideTx(Port::WifiTunnel, TxOrigin::NmtHeartbeat, hb, kTunnel, true, &lim, 10000) == Verdict::Allow);
  // 1 kHz source for one second → at most 100 mirrored frames
  RateLimiter l2{};
  int sent = 0;
  for (uint32_t t = 0; t < 1000000U; t += 1000U) sent += l2.Allow(t) ? 1 : 0;
  CHECK_EQ(sent, 100);
  // wrap-around of the µs clock
  RateLimiter l3{};
  CHECK(l3.Allow(0xFFFFF000U));
  CHECK(!l3.Allow(0x00000100U));  // 0x1100 µs later
  CHECK(l3.Allow(0x00002000U));   // 0x3000 µs later
}

TEST_CASE(authority_rules) {
  const AuthorityPolicy def{};
  AuthorityPolicy service = def;
  service.wireless_service = true;
  for (uint32_t idx = 0; idx <= 0xFFFFU; ++idx) {
    const uint16_t i = static_cast<uint16_t>(idx);
    const bool svc = i == 0x1010U || i == 0x4100U || i == 0x2601U;
    CHECK_EQ(SdoWriteAllowed(Port::Can, i, def), true);
    CHECK_EQ(SdoWriteAllowed(Port::WifiTunnel, i, def), !svc);
    CHECK_EQ(SdoWriteAllowed(Port::EspNowTunnel, i, service), true);
  }
  CHECK_EQ(PortBit(Port::Can), 0x01U);
  CHECK_EQ(PortBit(Port::WifiTunnel), 0x02U);
  CHECK_EQ(PortBit(Port::EspNowTunnel), 0x04U);
  CHECK_EQ(PortBit(Port::HostSim), 0x01U);
  CHECK_EQ(OwnerCode(Port::Can), 1U);
  CHECK_EQ(OwnerCode(Port::WifiTunnel), 2U);
  CHECK_EQ(OwnerCode(Port::EspNowTunnel), 3U);

  // default: only CAN may take authority
  CHECK_EQ(ArbitrateAuthority(0, Port::WifiTunnel, true, def), 0U);
  CHECK_EQ(ArbitrateAuthority(0, Port::Can, true, def), 1U);
  AuthorityPolicy wifi = def;
  wifi.enable_ports = 0x03U;
  CHECK_EQ(ArbitrateAuthority(0, Port::WifiTunnel, true, wifi), 2U);
  CHECK_EQ(ArbitrateAuthority(0, Port::WifiTunnel, false, wifi), 0U);  // only with the drive off
  CHECK_EQ(ArbitrateAuthority(1, Port::WifiTunnel, true, wifi), 1U);   // held by CAN: refused
  CHECK_EQ(ArbitrateAuthority(2, Port::Can, false, wifi), 1U);         // CAN always revokes
  CHECK_EQ(ArbitrateAuthority(2, Port::EspNowTunnel, true, wifi), 2U);
  CHECK(!MayRequestAuthority(Port::EspNowTunnel, wifi));
  CHECK(MayRequestAuthority(Port::Can, def));
}

}  // namespace
