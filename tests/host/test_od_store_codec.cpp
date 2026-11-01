/// Host test — parameter store TLV codec (contracts/od_store_codec.hpp, design §6.7).
///
/// Round trip, header / CRC / truncation rejection, and migration fixtures:
/// an OD v1 blob restored into OD v2 where one object was removed, one
/// renamed (migration), one changed type, one shrank its range and one moved
/// to another persist group. Unknown and invalid records are dropped and
/// counted; missing ones keep defaults.

#include "contracts/od_store_codec.hpp"

#include <cstring>
#include <map>
#include <utility>
#include <vector>

#include "support/check.hpp"

namespace {

using namespace vortex::contracts::odstore;

TEST_CASE(crc32_reference_vector) {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_EQ(Crc32(s, sizeof(s)), 0xCBF43926U);  // the standard check value
  CHECK_EQ(Crc32(s, 4, Crc32(nullptr, 0)), Crc32(s, 4));
  CHECK_EQ(Crc32(s + 4, 5, Crc32(s, 4)), 0xCBF43926U);  // chaining
}

std::vector<uint8_t> Le(uint64_t v, uint8_t n) {
  std::vector<uint8_t> out;
  for (uint8_t i = 0; i < n; ++i) out.push_back(static_cast<uint8_t>((v >> (8U * i)) & 0xFFU));
  return out;
}

struct Rec {
  uint16_t index;
  uint8_t sub;
  uint8_t type;
  std::vector<uint8_t> value;
};

std::vector<uint8_t> Build(uint32_t od_hash, const std::vector<Rec>& recs) {
  std::vector<uint8_t> buf(4096);
  StoreWriter w(buf.data(), buf.size(), od_hash);
  for (const Rec& r : recs) {
    CHECK(w.Add(r.index, r.sub, r.type, r.value.data(), static_cast<uint8_t>(r.value.size())));
  }
  const size_t n = w.Finish();
  buf.resize(n);
  return buf;
}

TEST_CASE(header_layout) {
  const std::vector<uint8_t> b = Build(0x11223344U, {{0x1017, 0, dtype::kUnsigned16, Le(100, 2)}});
  CHECK_EQ(b.size(), kHeaderSize + kRecordOverhead + 2U);
  CHECK(std::memcmp(b.data(), "VXOD", 4) == 0);
  CHECK_EQ(b[4], 1U);  // format, little-endian
  CHECK_EQ(b[5], 0U);
  CHECK_EQ(b[6], 0x44U);  // od hash
  CHECK_EQ(b[9], 0x11U);
  CHECK_EQ(b[10], 1U);  // count
  CHECK_EQ(b[18], 0x17U);  // index lo
  CHECK_EQ(b[19], 0x10U);
  CHECK_EQ(b[20], 0U);   // sub
  CHECK_EQ(b[21], dtype::kUnsigned16);
  CHECK_EQ(b[22], 2U);   // len
  CHECK_EQ(b[23], 100U);
}

TEST_CASE(round_trip_every_type) {
  const std::vector<Rec> recs = {
      {0x2000, 1, dtype::kBoolean, {1}},
      {0x2000, 2, dtype::kInt8, Le(static_cast<uint8_t>(-5), 1)},
      {0x2000, 3, dtype::kInt16, Le(static_cast<uint16_t>(-300), 2)},
      {0x2000, 4, dtype::kInt32, Le(static_cast<uint32_t>(-70000), 4)},
      {0x2000, 5, dtype::kUnsigned8, {200}},
      {0x2000, 6, dtype::kUnsigned16, Le(60000, 2)},
      {0x2000, 7, dtype::kUnsigned32, Le(0xDEADBEEFU, 4)},
      {0x2000, 8, dtype::kReal32, Le(0x3F800000U, 4)},
      {0x2000, 9, dtype::kVisibleString, {'v', 'x'}},
      {0x2000, 10, dtype::kOctetString, {1, 2, 3}},
      {0x2000, 11, dtype::kInt64, Le(static_cast<uint64_t>(-1), 8)},
      {0x2000, 12, dtype::kUnsigned64, Le(0x0102030405060708ULL, 8)},
      {0x2000, 13, dtype::kVisibleString, {}},
  };
  const std::vector<uint8_t> b = Build(7, recs);
  size_t i = 0;
  Header h{};
  const BlobStatus st = Parse(b.data(), b.size(), &h, [&](const Record& r) {
    CHECK(i < recs.size());
    if (i >= recs.size()) return;
    CHECK_EQ(r.index, recs[i].index);
    CHECK_EQ(r.sub, recs[i].sub);
    CHECK_EQ(r.type, recs[i].type);
    CHECK_EQ(r.len, recs[i].value.size());
    CHECK(r.len == 0U || std::memcmp(r.value, recs[i].value.data(), r.len) == 0);
    ++i;
  });
  CHECK(st == BlobStatus::Ok);
  CHECK_EQ(i, recs.size());
  CHECK_EQ(h.count, recs.size());
  CHECK_EQ(h.od_hash, 7U);
}

TEST_CASE(record_integer_sign_extension) {
  const uint8_t m5[] = {0xFB};
  CHECK_EQ(RecordInteger(Record{0, 0, dtype::kInt8, 1, m5}), -5);
  CHECK_EQ(RecordInteger(Record{0, 0, dtype::kUnsigned8, 1, m5}), 0xFB);
  const uint8_t m300[] = {0xD4, 0xFE};
  CHECK_EQ(RecordInteger(Record{0, 0, dtype::kInt16, 2, m300}), -300);
}

TEST_CASE(rejects_damaged_blobs) {
  const std::vector<uint8_t> good = Build(1, {{0x1017, 0, dtype::kUnsigned16, Le(100, 2)},
                                              {0x1005, 0, dtype::kUnsigned32, Le(0x80, 4)}});
  auto status = [](const std::vector<uint8_t>& b) {
    int visited = 0;
    const BlobStatus s = Parse(b.data(), b.size(), nullptr, [&](const Record&) { ++visited; });
    if (s != BlobStatus::Ok) CHECK_EQ(visited, 0);  // nothing applied from a bad blob
    return s;
  };
  CHECK(status(good) == BlobStatus::Ok);
  CHECK(status({}) == BlobStatus::Empty);
  std::vector<uint8_t> b = good;
  b.resize(10);
  CHECK(status(b) == BlobStatus::Truncated);
  b = good;
  b[0] = 'X';
  CHECK(status(b) == BlobStatus::BadMagic);
  b = good;
  b[4] = 2;
  CHECK(status(b) == BlobStatus::BadFormat);
  for (size_t i = kHeaderSize; i < good.size(); ++i) {  // any flipped payload bit
    b = good;
    b[i] ^= 0x10U;
    CHECK(status(b) == BlobStatus::BadCrc);
  }
  b = good;
  b[6] ^= 1U;  // header is covered too
  CHECK(status(b) == BlobStatus::BadCrc);

  // structurally broken but with a valid CRC: a record running past the end
  std::vector<uint8_t> trunc = good;
  trunc.pop_back();
  uint32_t crc = Crc32(trunc.data(), kCrcOffset);
  crc = Crc32(&trunc[kHeaderSize], trunc.size() - kHeaderSize, crc);
  for (unsigned i = 0; i < 4U; ++i) trunc[kCrcOffset + i] = static_cast<uint8_t>((crc >> (8U * i)) & 0xFFU);
  CHECK(status(trunc) == BlobStatus::Truncated);

  // wrong count with a valid CRC
  std::vector<uint8_t> cnt = good;
  cnt[10] = 3;
  crc = Crc32(cnt.data(), kCrcOffset);
  crc = Crc32(&cnt[kHeaderSize], cnt.size() - kHeaderSize, crc);
  for (unsigned i = 0; i < 4U; ++i) cnt[kCrcOffset + i] = static_cast<uint8_t>((crc >> (8U * i)) & 0xFFU);
  CHECK(status(cnt) == BlobStatus::BadCount);
}

TEST_CASE(writer_overflow) {
  uint8_t small[kHeaderSize + 6];
  StoreWriter w(small, sizeof(small), 0);
  const uint8_t v[4] = {1, 2, 3, 4};
  CHECK(!w.Add(0x1000, 0, dtype::kUnsigned32, v, 4));  // needs 9 bytes
  CHECK_EQ(w.Finish(), 0U);
  StoreWriter tiny(small, 4, 0);
  CHECK(!tiny.ok());
  CHECK_EQ(tiny.Finish(), 0U);
}

/** OD v1 → v2 migration fixture. */
TEST_CASE(migration_v1_to_v2) {
  constexpr uint32_t kHashV1 = 0x0000A001U;
  constexpr uint32_t kHashV2 = 0x0000A002U;
  // stored by firmware with OD v1, application group
  const std::vector<uint8_t> blob = Build(kHashV1, {
      {0x3001, 2, dtype::kUnsigned8, {7}},                  // pole pairs: unchanged → applied
      {0x3001, 3, dtype::kUnsigned16, Le(2500, 2)},         // rated current: unchanged → applied
      {0x3050, 1, dtype::kUnsigned32, Le(1, 4)},            // removed in v2 → dropped (unknown)
      {0x3005, 9, dtype::kInt16, Le(50, 2)},                // renamed 0x3005:09 → 0x3005:01 (migration)
      {0x3006, 1, dtype::kUnsigned16, Le(10, 2)},           // type changed to U32 in v2 → dropped
      {0x3007, 1, dtype::kUnsigned16, Le(60000, 2)},        // v2 range is 0..48000 → dropped
      {0x1017, 0, dtype::kUnsigned16, Le(100, 2)},          // comm object in an app blob → dropped
      {0x2500, 2, dtype::kUnsigned32, Le(0x00FF00, 3)},     // length mismatch → dropped
      {0x2410, 9, dtype::kVisibleString, {'a', 'b', 'c'}},  // string within max length → applied
  });
  // the current (v2) OD, as generated from the YAML
  const ObjectSpec spec[] = {
      {0x3001, 2, dtype::kUnsigned8, Group::App, 1, true, 1, 64},
      {0x3001, 3, dtype::kUnsigned16, Group::App, 2, true, 0, 10000},
      {0x3001, 4, dtype::kUnsigned16, Group::App, 2, false, 0, 0},  // new in v2: keeps default
      {0x3005, 1, dtype::kInt16, Group::App, 2, true, -1000, 1000},
      {0x3006, 1, dtype::kUnsigned32, Group::App, 4, false, 0, 0},
      {0x3007, 1, dtype::kUnsigned16, Group::App, 2, true, 0, 48000},
      {0x1017, 0, dtype::kUnsigned16, Group::Comm, 2, false, 0, 0},
      {0x2500, 2, dtype::kUnsigned32, Group::App, 4, false, 0, 0},
      {0x2410, 9, dtype::kVisibleString, Group::App, 16, false, 0, 0},
  };
  const Migration mig[] = {{0x3005, 9, 0x3005, 1}};
  std::map<std::pair<uint16_t, uint8_t>, std::vector<uint8_t>> applied;
  const RestoreStats st =
      Restore(blob.data(), blob.size(), kHashV2, Group::App, spec, sizeof(spec) / sizeof(spec[0]), mig, 1,
              [&](const ObjectSpec& s, const Record& r) {
                applied[{s.index, s.sub}] = std::vector<uint8_t>(r.value, r.value + r.len);
              });
  CHECK(st.status == BlobStatus::Ok);
  CHECK(st.od_hash_changed);
  CHECK_EQ(st.total, 9U);
  CHECK_EQ(st.applied, 4U);
  CHECK_EQ(st.migrated, 1U);
  CHECK_EQ(st.dropped, 5U);
  CHECK_EQ(st.dropped_by[static_cast<uint8_t>(Drop::Unknown)], 1U);
  CHECK_EQ(st.dropped_by[static_cast<uint8_t>(Drop::TypeMismatch)], 1U);
  CHECK_EQ(st.dropped_by[static_cast<uint8_t>(Drop::Range)], 1U);
  CHECK_EQ(st.dropped_by[static_cast<uint8_t>(Drop::WrongGroup)], 1U);
  CHECK_EQ(st.dropped_by[static_cast<uint8_t>(Drop::Length)], 1U);
  CHECK_EQ(st.total, st.applied + st.dropped);
  CHECK((applied.count({0x3001, 2}) == 1U && applied[{0x3001, 2}][0] == 7U));
  CHECK((applied.count({0x3001, 3}) == 1U));
  CHECK((applied.count({0x3005, 1}) == 1U && applied[{0x3005, 1}][0] == 50U));
  CHECK((applied.count({0x2410, 9}) == 1U));
  CHECK((applied.count({0x3001, 4}) == 0U));  // missing: default kept
  CHECK((applied.count({0x3050, 1}) == 0U));
}

TEST_CASE(restore_same_od_applies_everything) {
  const ObjectSpec spec[] = {
      {0x1017, 0, dtype::kUnsigned16, Group::Comm, 2, false, 0, 0},
      {0x1005, 0, dtype::kUnsigned32, Group::Comm, 4, false, 0, 0},
      {0x1006, 0, dtype::kInt32, Group::Comm, 4, true, -10, 10},
  };
  const std::vector<uint8_t> blob = Build(42, {{0x1017, 0, dtype::kUnsigned16, Le(250, 2)},
                                               {0x1005, 0, dtype::kUnsigned32, Le(0x80, 4)},
                                               {0x1006, 0, dtype::kInt32, Le(static_cast<uint32_t>(-10), 4)}});
  int n = 0;
  const RestoreStats st = Restore(blob.data(), blob.size(), 42, Group::Comm, spec, 3, nullptr, 0,
                                  [&](const ObjectSpec&, const Record&) { ++n; });
  CHECK(st.status == BlobStatus::Ok);
  CHECK(!st.od_hash_changed);
  CHECK_EQ(st.applied, 3U);
  CHECK_EQ(st.dropped, 0U);
  CHECK_EQ(n, 3);
  // a bad blob applies nothing and reports the reason
  std::vector<uint8_t> bad = blob;
  bad.back() ^= 1U;
  n = 0;
  const RestoreStats sb = Restore(bad.data(), bad.size(), 42, Group::Comm, spec, 3, nullptr, 0,
                                  [&](const ObjectSpec&, const Record&) { ++n; });
  CHECK(sb.status == BlobStatus::BadCrc);
  CHECK_EQ(n, 0);
  CHECK_EQ(sb.total, 0U);
}

}  // namespace
