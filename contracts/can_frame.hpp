#pragma once
/**
 * @file can_frame.hpp
 * @brief Transport-neutral CAN frame, port capabilities and the ICanPort
 *        interface (design §4.3, §8).
 *
 * @details Every transport (TWAI today, FDCAN later, the Wi-Fi / ESP-NOW
 *          tunnels and the host sim ports) moves `CanFrame` values. The frame
 *          is FD-sized from day one: `len` is a byte count (0..8 classic,
 *          0..64 FD), never a DLC code, so nothing above the port binding
 *          assumes 8 data bytes.
 *
 *          Contracts rule (§4.1): no .cpp, no RTOS, no HAL. This header only
 *          declares the interface; bindings live in the HAL / hf-core.
 */
#include <cstddef>
#include <cstdint>

namespace vortex::contracts {

/** @brief Largest classic CAN payload in bytes. */
inline constexpr uint8_t kClassicMaxLen = 8U;
/** @brief Largest CAN FD payload in bytes. */
inline constexpr uint8_t kFdMaxLen = 64U;

/** @brief Largest 11-bit (base format) identifier. */
inline constexpr uint32_t kMaxStdId = 0x7FFU;
/** @brief Largest 29-bit (extended format) identifier. */
inline constexpr uint32_t kMaxExtId = 0x1FFFFFFFU;

/** @brief Bits of `CanFrame::flags`. */
namespace frame_flag {
inline constexpr uint8_t kExt = 0x01U;  ///< 29-bit identifier
inline constexpr uint8_t kRtr = 0x02U;  ///< remote request (classic only)
inline constexpr uint8_t kFdf = 0x04U;  ///< FD frame format
inline constexpr uint8_t kBrs = 0x08U;  ///< bit-rate switch (FD only)
inline constexpr uint8_t kEsi = 0x10U;  ///< error state indicator (FD only)
inline constexpr uint8_t kAll = kExt | kRtr | kFdf | kBrs | kEsi;
}  // namespace frame_flag

/**
 * @brief One CAN or CAN FD frame.
 * @details `ts_us` is the RX timestamp captured by the port (ISR side). It is
 *          ignored on transmit.
 */
struct CanFrame {
  uint32_t id{0};       ///< 11- or 29-bit identifier (see frame_flag::kExt)
  uint8_t len{0};       ///< payload length in bytes: 0..8 classic, 0..64 FD
  uint8_t flags{0};     ///< frame_flag bits
  uint8_t data[kFdMaxLen]{};
  uint32_t ts_us{0};    ///< RX timestamp (µs, port clock)

  constexpr bool IsExtended() const noexcept { return (flags & frame_flag::kExt) != 0U; }
  constexpr bool IsRemote() const noexcept { return (flags & frame_flag::kRtr) != 0U; }
  constexpr bool IsFd() const noexcept { return (flags & frame_flag::kFdf) != 0U; }
};

/** @brief Data length code → byte length (ISO 11898-1; classic codes 9..15 mean 8). */
constexpr uint8_t DlcToLen(uint8_t dlc, bool fd) noexcept {
  constexpr uint8_t kFdTable[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  if (dlc > 15U) return 0U;
  if (!fd) return dlc > 8U ? 8U : dlc;
  return kFdTable[dlc];
}

/**
 * @brief Byte length → smallest DLC whose length is at least @p len.
 * @return 0xFF when @p len cannot be carried (above 8 classic or 64 FD).
 */
constexpr uint8_t LenToDlc(uint8_t len, bool fd) noexcept {
  if (len <= 8U) return len;
  if (!fd || len > kFdMaxLen) return 0xFFU;
  for (uint8_t dlc = 9U; dlc < 16U; ++dlc) {
    if (DlcToLen(dlc, true) >= len) return dlc;
  }
  return 0xFFU;
}

/** @brief Is @p len one of the exact FD payload sizes (0..8, 12, 16, 20, 24, 32, 48, 64)? */
constexpr bool IsExactFdLength(uint8_t len) noexcept {
  const uint8_t dlc = LenToDlc(len, true);
  return dlc != 0xFFU && DlcToLen(dlc, true) == len;
}

/** @brief Why a frame is not well formed. */
enum class FrameError : uint8_t {
  None = 0,
  UnknownFlag,      ///< a flag bit outside frame_flag::kAll
  IdRange,          ///< identifier does not fit 11 / 29 bits
  ClassicTooLong,   ///< classic frame with len > 8
  FdTooLong,        ///< FD frame with len > 64
  FdLengthNotExact, ///< FD length that has no DLC code (e.g. 9)
  RtrOnFd,          ///< FD frames have no remote form
  FdBitsOnClassic,  ///< BRS / ESI without FDF
  RtrWithData,      ///< remote frame with a payload length above 8
};

/** @brief Structural check every port applies before Send and after RX. */
constexpr FrameError CheckFrame(const CanFrame& f) noexcept {
  if ((f.flags & static_cast<uint8_t>(~frame_flag::kAll)) != 0U) return FrameError::UnknownFlag;
  if (f.id > (f.IsExtended() ? kMaxExtId : kMaxStdId)) return FrameError::IdRange;
  if (f.IsFd()) {
    if (f.IsRemote()) return FrameError::RtrOnFd;
    if (f.len > kFdMaxLen) return FrameError::FdTooLong;
    if (!IsExactFdLength(f.len)) return FrameError::FdLengthNotExact;
    return FrameError::None;
  }
  if ((f.flags & (frame_flag::kBrs | frame_flag::kEsi)) != 0U) return FrameError::FdBitsOnClassic;
  if (f.len > kClassicMaxLen) return f.IsRemote() ? FrameError::RtrWithData : FrameError::ClassicTooLong;
  return FrameError::None;
}

/** @brief What a port binding can carry. */
struct PortCaps {
  bool fd{false};            ///< FD frames supported
  uint8_t max_len{kClassicMaxLen};  ///< largest payload the port accepts (8 or 64)
  uint32_t nominal_bps{0};   ///< arbitration bit rate (0 = not a physical bus)
  uint32_t data_bps{0};      ///< FD data-phase bit rate (0 = no BRS)
  uint8_t hw_filters{0};     ///< hardware acceptance filters available
};

/** @brief May @p f go out of (or come in through) a port with @p caps? */
constexpr bool FitsPort(const CanFrame& f, const PortCaps& caps) noexcept {
  if (CheckFrame(f) != FrameError::None) return false;
  if (f.IsFd() && !caps.fd) return false;
  if ((f.flags & frame_flag::kBrs) != 0U && caps.data_bps == 0U) return false;
  return f.len <= caps.max_len;
}

/** @brief Result of a non-blocking `ICanPort::Send`. */
enum class TxResult : uint8_t {
  Ok = 0,      ///< queued for transmission
  QueueFull,   ///< no room; the caller retries from its TX-done path
  BusOff,      ///< controller is bus-off; `Recover()` first
  NotAllowed,  ///< the frame does not fit this port (length, FD, policy)
};

/** @brief Controller error state (ISO 11898-1 fault confinement). */
enum class BusState : uint8_t {
  Stopped = 0,   ///< not started / in reset
  ErrorActive,   ///< TEC and REC < 96
  ErrorWarning,  ///< TEC or REC >= 96
  ErrorPassive,  ///< TEC or REC >= 128
  BusOff,        ///< TEC > 255
};

/** @brief Error counters and statistics of one port. */
struct PortStatus {
  BusState state{BusState::Stopped};
  uint16_t tec{0};            ///< transmit error counter
  uint16_t rec{0};            ///< receive error counter
  uint32_t rx_overruns{0};    ///< frames lost because the RX ring was full
  uint32_t tx_queue_full{0};  ///< Send() calls answered QueueFull
  uint32_t bus_off_count{0};  ///< bus-off events since boot
};

/** @brief Error state implied by the counters (warning at 96, passive at 128, bus-off above 255). */
constexpr BusState StateFromCounters(uint16_t tec, uint16_t rec) noexcept {
  if (tec > 255U) return BusState::BusOff;
  if (tec >= 128U || rec >= 128U) return BusState::ErrorPassive;
  if (tec >= 96U || rec >= 96U) return BusState::ErrorWarning;
  return BusState::ErrorActive;
}

/**
 * @brief RX sink: called by the port for every received frame.
 * @details May run in ISR context. It must copy the frame and return; no
 *          blocking, no logging, no allocation.
 */
using RxSink = void (*)(void* ctx, const CanFrame& frame);

/**
 * @brief One CAN transport binding (TWAI, FDCAN, tunnel, SocketCAN, pipe).
 * @details Implemented per binding. `Send` never blocks: a full queue
 *          returns `QueueFull`. Interface only; no implementation here.
 */
class ICanPort {
 public:
  virtual ~ICanPort() = default;

  /** @brief Static capabilities of this binding. */
  virtual PortCaps Caps() const = 0;

  /** @brief Queue @p frame for transmission without blocking. */
  virtual TxResult Send(const CanFrame& frame) = 0;

  /** @brief Install the RX sink (ISR-safe). Passing nullptr detaches it. */
  virtual void SetRxSink(RxSink sink, void* ctx) = 0;

  /** @brief Error counters, state and overrun statistics. */
  virtual PortStatus Status() const = 0;

  /** @brief Request bus-off recovery. Returns false when not possible now. */
  virtual bool Recover() = 0;

 protected:
  ICanPort() = default;
  ICanPort(const ICanPort&) = default;
  ICanPort& operator=(const ICanPort&) = default;
};

}  // namespace vortex::contracts
