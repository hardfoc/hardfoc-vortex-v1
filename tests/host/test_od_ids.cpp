/// Host test — generated OD tables (contracts/vortex_od_ids.hpp) against the
/// contracts that consume them: entries sorted and unique, the persist spec
/// agrees with the entry table, and a store of every persisted default
/// restores with nothing dropped.

#include "contracts/vortex_od_ids.hpp"

#include <cstring>
#include <vector>

#include "support/check.hpp"

namespace {

using namespace vortex;
namespace odstore = vortex::contracts::odstore;

const od::EntryInfo* Find(uint16_t index, uint8_t sub) {
  for (const auto& e : od::kEntries) {
    if (e.address.index == index && e.address.sub == sub) return &e;
  }
  return nullptr;
}

TEST_CASE(entries_sorted_and_unique) {
  for (size_t i = 1; i < od::kEntryCount; ++i) {
    const auto& a = od::kEntries[i - 1].address;
    const auto& b = od::kEntries[i].address;
    CHECK(a.index < b.index || (a.index == b.index && a.sub < b.sub));
  }
  CHECK(od::kEntryCount > 100U);
}

TEST_CASE(meta_objects_match_constants) {
  const od::EntryInfo* hash = Find(0x5FFE, 0);
  const od::EntryInfo* ver = Find(0x5FFF, 0);
  const od::EntryInfo* rev = Find(0x1018, 3);
  CHECK(hash != nullptr && ver != nullptr && rev != nullptr);
  if (hash == nullptr || ver == nullptr || rev == nullptr) return;
  CHECK_EQ(static_cast<uint32_t>(hash->default_int), od::kOdHash);
  CHECK_EQ(static_cast<uint16_t>(ver->default_int), od::kOdVersion);
  CHECK_EQ(static_cast<uint32_t>(rev->default_int), od::kRevisionNumber);
  CHECK_EQ(od::kRevisionNumber, (static_cast<uint32_t>(od::kRevisionMajor) << 16) | od::kRevisionMinor);
  CHECK_EQ(od::addr::kIdentityRevisionNumber.index, 0x1018U);
  CHECK_EQ(od::addr::kIdentityRevisionNumber.sub, 3U);
  CHECK_EQ(od::idx::kSupportedDriveModes, 0x6502U);
  const od::EntryInfo* modes = Find(0x6502, 0);
  CHECK(modes != nullptr && modes->default_int == 0x3AD);
}

TEST_CASE(persist_spec_matches_entries) {
  for (const auto& s : od::kPersistSpec) {
    const od::EntryInfo* e = Find(s.index, s.sub);
    CHECK(e != nullptr);
    if (e == nullptr) continue;
    CHECK_EQ(e->type, s.type);
    CHECK(e->persist == s.group);
    CHECK(s.group != odstore::Group::None);
    CHECK_EQ(e->length, s.len);
    CHECK(e->access == od::Access::Rw || e->access == od::Access::Wo);
    CHECK(e->has_storage);
  }
  // every writable, stored entry in a persist group is in the spec
  for (const auto& e : od::kEntries) {
    if (e.persist == odstore::Group::None || !e.has_storage) continue;
    if (e.access != od::Access::Rw && e.access != od::Access::Wo) continue;
    bool found = false;
    for (const auto& s : od::kPersistSpec) found = found || (s.index == e.address.index && s.sub == e.address.sub);
    CHECK(found);
  }
}

std::vector<uint8_t> DefaultBytes(const od::EntryInfo& e) {
  std::vector<uint8_t> v;
  if (e.type == odstore::dtype::kVisibleString) {
    const size_t n = e.default_str != nullptr ? std::strlen(e.default_str) : 0U;
    v.assign(e.default_str, e.default_str + n);
    return v;
  }
  if (e.type == odstore::dtype::kReal32) {
    const float f = static_cast<float>(e.default_real);
    uint8_t b[4];
    std::memcpy(b, &f, 4);
    v.assign(b, b + 4);
    return v;
  }
  const uint64_t u = static_cast<uint64_t>(e.default_int);
  for (uint8_t i = 0; i < e.length; ++i) v.push_back(static_cast<uint8_t>((u >> (8U * i)) & 0xFFU));
  return v;
}

TEST_CASE(store_of_defaults_restores_cleanly) {
  for (odstore::Group g : {odstore::Group::Comm, odstore::Group::App, odstore::Group::Cal}) {
    std::vector<uint8_t> buf(8192);
    odstore::StoreWriter w(buf.data(), buf.size(), od::kOdHash);
    uint32_t added = 0;
    for (const auto& s : od::kPersistSpec) {
      if (s.group != g) continue;
      const od::EntryInfo* e = Find(s.index, s.sub);
      if (e == nullptr) continue;
      const std::vector<uint8_t> v = DefaultBytes(*e);
      CHECK(w.Add(s.index, s.sub, s.type, v.data(), static_cast<uint8_t>(v.size())));
      ++added;
    }
    const size_t n = w.Finish();
    CHECK(n > 0U);
    uint32_t applied = 0;
    const odstore::RestoreStats st =
        odstore::Restore(buf.data(), n, od::kOdHash, g, od::kPersistSpec, od::kPersistSpecCount, od::kMigrations,
                         od::kMigrationCount, [&](const odstore::ObjectSpec&, const odstore::Record&) { ++applied; });
    CHECK(st.status == odstore::BlobStatus::Ok);
    CHECK(!st.od_hash_changed);
    CHECK_EQ(st.dropped, 0U);
    CHECK_EQ(applied, added);
  }
}

TEST_CASE(node_relative_defaults) {
  bool emcy = false;
  bool sdo2 = false;
  for (const auto& r : od::kNodeIdRelative) {
    const od::EntryInfo* e = Find(r.address.index, r.address.sub);
    CHECK(e != nullptr && e->node_relative && static_cast<uint32_t>(e->default_int) == r.base);
    emcy = emcy || (r.address == od::addr::kCobIdEmcy && r.base == 0x80U);
    sdo2 = sdo2 || (r.address.index == 0x1201U && r.address.sub == 1U && r.base == 0x6C0U);
  }
  CHECK(emcy);
  CHECK(sdo2);
}

}  // namespace
