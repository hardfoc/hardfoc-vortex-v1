#pragma once
/**
 * @file od_store_codec.hpp
 * @brief Parameter store record format for 0x1010 / 0x1011 (design §6.7).
 *
 * @details One blob per storage group (comm 0x1010:02, app :03, cal :04).
 *          Records are keyed by OD address, not by memory layout, so an OD
 *          change never bricks a stored configuration:
 *
 *          @verbatim
 *          header  { char magic[4] = "VXOD"; u16 format; u32 od_hash; u32 count; u32 crc32 }   18 bytes
 *          record  { u16 index; u8 sub; u8 type; u8 len; u8 value[len] }                        5 + len bytes
 *          @endverbatim
 *
 *          All integers little-endian. `crc32` (IEEE 802.3, reflected,
 *          0xEDB88320) covers the 14 header bytes before it and every record
 *          byte after it.
 *
 *          On restore every record is mapped through the migration list,
 *          then applied only if (index, sub, type) exists in the current OD
 *          with the same persist group, its length matches, and the value
 *          is in range. Anything else is dropped and counted (0x2800);
 *          missing records keep their defaults.
 *
 *          Pure, no allocation (host test: tests/host/test_od_store_codec.cpp).
 */
#include <cstddef>
#include <cstdint>

namespace vortex::contracts::odstore {

inline constexpr uint8_t kMagic[4] = {'V', 'X', 'O', 'D'};
inline constexpr uint16_t kFormat = 1U;
inline constexpr size_t kHeaderSize = 18U;
inline constexpr size_t kRecordOverhead = 5U;
inline constexpr size_t kCrcOffset = 14U;

/** @brief Persist groups (YAML `persist:`). Value = 0x1010 sub-index − 1. */
enum class Group : uint8_t { None = 0, Comm = 1, App = 2, Cal = 3 };

/** @brief CiA 301 data type codes used in records. */
namespace dtype {
inline constexpr uint8_t kBoolean = 0x01U;
inline constexpr uint8_t kInt8 = 0x02U;
inline constexpr uint8_t kInt16 = 0x03U;
inline constexpr uint8_t kInt32 = 0x04U;
inline constexpr uint8_t kUnsigned8 = 0x05U;
inline constexpr uint8_t kUnsigned16 = 0x06U;
inline constexpr uint8_t kUnsigned32 = 0x07U;
inline constexpr uint8_t kReal32 = 0x08U;
inline constexpr uint8_t kVisibleString = 0x09U;
inline constexpr uint8_t kOctetString = 0x0AU;
inline constexpr uint8_t kDomain = 0x0FU;
inline constexpr uint8_t kReal64 = 0x11U;
inline constexpr uint8_t kInt64 = 0x15U;
inline constexpr uint8_t kUnsigned64 = 0x1BU;

constexpr bool IsSigned(uint8_t t) noexcept { return t == kInt8 || t == kInt16 || t == kInt32 || t == kInt64; }
constexpr bool IsUnsigned(uint8_t t) noexcept {
  return t == kBoolean || t == kUnsigned8 || t == kUnsigned16 || t == kUnsigned32 || t == kUnsigned64;
}
constexpr bool IsVariableLength(uint8_t t) noexcept { return t == kVisibleString || t == kOctetString || t == kDomain; }
}  // namespace dtype

/** @brief CRC-32 (IEEE 802.3). Chain calls by passing the previous result as @p crc. */
constexpr uint32_t Crc32(const uint8_t* data, size_t n, uint32_t crc = 0U) noexcept {
  crc = ~crc;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

namespace detail {
constexpr void Put16(uint8_t* p, uint16_t v) noexcept {
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>(v >> 8);
}
constexpr void Put32(uint8_t* p, uint32_t v) noexcept {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFU);
}
constexpr uint16_t Get16(const uint8_t* p) noexcept { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
constexpr uint32_t Get32(const uint8_t* p) noexcept {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace detail

/**
 * @brief Serialises one group into a caller buffer.
 * @details `Add` records, then `Finish` writes the header and CRC and returns
 *          the blob size (0 if anything overflowed).
 */
class StoreWriter {
 public:
  constexpr StoreWriter(uint8_t* buf, size_t cap, uint32_t od_hash) noexcept
      : buf_(buf), cap_(cap), od_hash_(od_hash), pos_(kHeaderSize), ok_(buf != nullptr && cap >= kHeaderSize) {}

  constexpr bool Add(uint16_t index, uint8_t sub, uint8_t type, const uint8_t* value, uint8_t len) noexcept {
    if (!ok_ || (len > 0U && value == nullptr) || pos_ + kRecordOverhead + len > cap_) {
      ok_ = false;
      return false;
    }
    detail::Put16(&buf_[pos_], index);
    buf_[pos_ + 2U] = sub;
    buf_[pos_ + 3U] = type;
    buf_[pos_ + 4U] = len;
    for (uint8_t i = 0; i < len; ++i) buf_[pos_ + kRecordOverhead + i] = value[i];
    pos_ += kRecordOverhead + len;
    ++count_;
    return true;
  }

  constexpr size_t Finish() noexcept {
    if (!ok_) return 0U;
    for (int i = 0; i < 4; ++i) buf_[i] = kMagic[i];
    detail::Put16(&buf_[4], kFormat);
    detail::Put32(&buf_[6], od_hash_);
    detail::Put32(&buf_[10], count_);
    uint32_t crc = Crc32(buf_, kCrcOffset);
    crc = Crc32(&buf_[kHeaderSize], pos_ - kHeaderSize, crc);
    detail::Put32(&buf_[kCrcOffset], crc);
    return pos_;
  }

  constexpr bool ok() const noexcept { return ok_; }
  constexpr uint32_t count() const noexcept { return count_; }

 private:
  uint8_t* buf_;
  size_t cap_;
  uint32_t od_hash_;
  size_t pos_;
  uint32_t count_{0};
  bool ok_;
};

/** @brief Blob-level status. */
enum class BlobStatus : uint8_t {
  Ok = 0,
  Empty,      ///< nothing stored (0 bytes): defaults apply
  Truncated,  ///< shorter than the header or a record runs past the end
  BadMagic,
  BadFormat,
  BadCrc,
  BadCount,   ///< header count differs from the records present
};

/** @brief Header fields of a blob. */
struct Header {
  uint16_t format{0};
  uint32_t od_hash{0};
  uint32_t count{0};
  uint32_t crc{0};
};

/** @brief One record view (points into the blob). */
struct Record {
  uint16_t index{0};
  uint8_t sub{0};
  uint8_t type{0};
  uint8_t len{0};
  const uint8_t* value{nullptr};
};

/**
 * @brief Validate a blob and visit its records.
 * @tparam Visit callable `void(const Record&)`.
 */
template <typename Visit>
constexpr BlobStatus Parse(const uint8_t* buf, size_t n, Header* hdr, Visit&& visit) noexcept {
  if (n == 0U) return BlobStatus::Empty;
  if (buf == nullptr || n < kHeaderSize) return BlobStatus::Truncated;
  for (int i = 0; i < 4; ++i) {
    if (buf[i] != kMagic[i]) return BlobStatus::BadMagic;
  }
  Header h{};
  h.format = detail::Get16(&buf[4]);
  h.od_hash = detail::Get32(&buf[6]);
  h.count = detail::Get32(&buf[10]);
  h.crc = detail::Get32(&buf[kCrcOffset]);
  if (hdr != nullptr) *hdr = h;
  if (h.format != kFormat) return BlobStatus::BadFormat;
  uint32_t crc = Crc32(buf, kCrcOffset);
  crc = Crc32(&buf[kHeaderSize], n - kHeaderSize, crc);
  if (crc != h.crc) return BlobStatus::BadCrc;
  /* first pass: structure only, so a bad blob applies nothing */
  uint32_t seen = 0;
  for (size_t pos = kHeaderSize; pos < n; ++seen) {
    if (pos + kRecordOverhead > n) return BlobStatus::Truncated;
    const size_t len = buf[pos + 4U];
    if (pos + kRecordOverhead + len > n) return BlobStatus::Truncated;
    pos += kRecordOverhead + len;
  }
  if (seen != h.count) return BlobStatus::BadCount;
  for (size_t pos = kHeaderSize; pos < n;) {
    Record r{};
    r.index = detail::Get16(&buf[pos]);
    r.sub = buf[pos + 2U];
    r.type = buf[pos + 3U];
    r.len = buf[pos + 4U];
    r.value = &buf[pos + kRecordOverhead];
    visit(static_cast<const Record&>(r));
    pos += kRecordOverhead + r.len;
  }
  return BlobStatus::Ok;
}

/** @brief One persistable entry of the current OD (generated from the YAML). */
struct ObjectSpec {
  uint16_t index;
  uint8_t sub;
  uint8_t type;      ///< dtype code
  Group group;
  uint8_t len;       ///< byte length (maximum for variable-length types)
  bool has_limits;   ///< low / high apply (integer types only)
  int64_t low;
  int64_t high;
};

/** @brief Renamed address: records stored at (from) restore into (to). */
struct Migration {
  uint16_t from_index;
  uint8_t from_sub;
  uint16_t to_index;
  uint8_t to_sub;
};

/** @brief Why a record was dropped. */
enum class Drop : uint8_t { Unknown = 0, WrongGroup, TypeMismatch, Length, Range, kCount };

/** @brief Restore counters (0x2800 reports applied / dropped). */
struct RestoreStats {
  BlobStatus status{BlobStatus::Empty};
  bool od_hash_changed{false};
  uint32_t total{0};
  uint32_t applied{0};
  uint32_t migrated{0};
  uint32_t dropped{0};
  uint32_t dropped_by[static_cast<uint8_t>(Drop::kCount)]{};
};

/** @brief Little-endian integer value of a record (sign-extended for signed types). */
constexpr int64_t RecordInteger(const Record& r) noexcept {
  uint64_t u = 0;
  for (uint8_t i = 0; i < r.len && i < 8U; ++i) u |= static_cast<uint64_t>(r.value[i]) << (8U * i);
  if (dtype::IsSigned(r.type) && r.len > 0U && r.len < 8U && (r.value[r.len - 1U] & 0x80U) != 0U) {
    u |= ~0ULL << (8U * r.len);
  }
  return static_cast<int64_t>(u);
}

/**
 * @brief Restore one group.
 * @param apply callable `void(const ObjectSpec&, const Record&)`: write the value into the OD.
 */
template <typename Apply>
constexpr RestoreStats Restore(const uint8_t* buf, size_t n, uint32_t current_od_hash, Group group,
                               const ObjectSpec* spec, size_t spec_count, const Migration* migrations,
                               size_t migration_count, Apply&& apply) noexcept {
  RestoreStats st{};
  Header h{};
  auto drop = [&st](Drop why) {
    ++st.dropped;
    ++st.dropped_by[static_cast<uint8_t>(why)];
  };
  st.status = Parse(buf, n, &h, [&](const Record& in) {
    ++st.total;
    Record r = in;
    for (size_t m = 0; m < migration_count; ++m) {
      if (migrations[m].from_index == r.index && migrations[m].from_sub == r.sub) {
        r.index = migrations[m].to_index;
        r.sub = migrations[m].to_sub;
        ++st.migrated;
        break;
      }
    }
    const ObjectSpec* s = nullptr;
    for (size_t i = 0; i < spec_count; ++i) {
      if (spec[i].index == r.index && spec[i].sub == r.sub) {
        s = &spec[i];
        break;
      }
    }
    if (s == nullptr) return drop(Drop::Unknown);
    if (s->group != group) return drop(Drop::WrongGroup);
    if (s->type != r.type) return drop(Drop::TypeMismatch);
    if (dtype::IsVariableLength(r.type) ? r.len > s->len : r.len != s->len) return drop(Drop::Length);
    if (s->has_limits && (dtype::IsSigned(r.type) || dtype::IsUnsigned(r.type))) {
      const int64_t v = RecordInteger(r);
      const bool in_range = dtype::IsUnsigned(r.type)
                                ? static_cast<uint64_t>(v) >= static_cast<uint64_t>(s->low) &&
                                      static_cast<uint64_t>(v) <= static_cast<uint64_t>(s->high)
                                : v >= s->low && v <= s->high;
      if (!in_range) return drop(Drop::Range);
    }
    ++st.applied;
    apply(*s, static_cast<const Record&>(r));
  });
  if (st.status == BlobStatus::Ok) st.od_hash_changed = h.od_hash != current_od_hash;
  return st;
}

}  // namespace vortex::contracts::odstore
