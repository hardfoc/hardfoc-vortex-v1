#pragma once
/**
 * @file SimNvs.hpp
 * @brief Host stand-in for the HAL's NvsManager (sim HAL skeleton, §9.4).
 *
 * @details Key → bytes, kept in memory and, when a file is given, written
 *          through to it so a test can "power cycle" the node by restarting
 *          the binary. Step 6 uses it for the LSS record only; step 7 adds
 *          the 0x1010 / 0x1011 parameter groups on top of the same store.
 *          File format: one `key=hexbytes` line per record.
 */
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "CoNode.hpp"

namespace vortex::sim {

class SimNvs {
 public:
  explicit SimNvs(std::string path = {}) : path_(std::move(path)) { Load(); }

  bool Get(const std::string& key, std::vector<uint8_t>& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    const auto it = data_.find(key);
    if (it == data_.end()) {
      return false;
    }
    out = it->second;
    return true;
  }

  bool Set(const std::string& key, const std::vector<uint8_t>& value) {
    std::lock_guard<std::mutex> lk(mu_);
    data_[key] = value;
    return Save();
  }

 private:
  void Load() {
    if (path_.empty()) {
      return;
    }
    std::ifstream in(path_);
    std::string line;
    while (std::getline(in, line)) {
      const auto eq = line.find('=');
      if (eq == std::string::npos) {
        continue;
      }
      std::vector<uint8_t> bytes;
      for (std::size_t i = eq + 1; i + 1 < line.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(std::stoul(line.substr(i, 2), nullptr, 16)));
      }
      data_[line.substr(0, eq)] = bytes;
    }
  }

  bool Save() const {
    if (path_.empty()) {
      return true;
    }
    std::ofstream out(path_, std::ios::trunc);
    for (const auto& kv : data_) {
      out << kv.first << '=';
      for (uint8_t b : kv.second) {
        char hex[3];
        std::snprintf(hex, sizeof(hex), "%02x", b);
        out << hex;
      }
      out << '\n';
    }
    return static_cast<bool>(out);
  }

  std::string path_;
  mutable std::mutex mu_;
  std::map<std::string, std::vector<uint8_t>> data_;
};

/** @brief LSS record (`lss` key: node-ID, bit rate kbit/s LE) over SimNvs. */
class SimLssStore final : public node::ILssStore {
 public:
  explicit SimLssStore(SimNvs& nvs) : nvs_(nvs) {}
  bool Load(uint8_t& node_id, uint16_t& bitrate_kbps) override {
    std::vector<uint8_t> v;
    if (!nvs_.Get("lss", v) || v.size() != 3U) {
      return false;
    }
    node_id = v[0];
    bitrate_kbps = static_cast<uint16_t>(v[1] | (v[2] << 8U));
    return true;
  }
  bool Store(uint8_t node_id, uint16_t bitrate_kbps) override {
    return nvs_.Set("lss", {node_id, static_cast<uint8_t>(bitrate_kbps), static_cast<uint8_t>(bitrate_kbps >> 8U)});
  }

 private:
  SimNvs& nvs_;
};

}  // namespace vortex::sim
