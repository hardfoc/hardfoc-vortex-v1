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
 *
 *          The file path comes from the command line, so it is validated
 *          before use (ValidateNvsPath): no `..` component, the canonical
 *          path must lie under an allowed base directory (the working
 *          directory or $TMPDIR, /tmp when TMPDIR is unset), the file name
 *          must match [A-Za-z0-9._-]+, and an existing file must be a
 *          regular file. The path SimNvs opens is rebuilt character by
 *          character from fixed character sets (CopyThroughCharset), so no
 *          byte of the command-line string reaches the open call directly.
 */
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "CoNode.hpp"

namespace vortex::sim {

inline constexpr const char kNvsNameChars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-";
inline constexpr const char kNvsPathChars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-/+";

/**
 * @brief Copy @p in using only characters taken from @p allowed.
 * @return the copy, or nullopt if @p in has a character outside @p allowed.
 *         Every output character comes from the constant set, not from @p in.
 */
inline std::optional<std::string> CopyThroughCharset(const std::string& in, const char* allowed) {
  std::string out;
  out.reserve(in.size());
  for (const char c : in) {
    bool found = false;
    for (const char* a = allowed; *a != '\0'; ++a) {
      if (*a == c) {
        out.push_back(*a);
        found = true;
        break;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }
  return out;
}

/** @brief realpath() of an existing directory, or nullopt. */
inline std::optional<std::string> CanonicalDir(const std::string& dir) {
  char buf[PATH_MAX];
  if (dir.empty() || realpath(dir.c_str(), buf) == nullptr) {
    return std::nullopt;
  }
  struct stat st {};
  if (stat(buf, &st) != 0 || !S_ISDIR(st.st_mode)) {
    return std::nullopt;
  }
  return std::string(buf);
}

/** @brief Directories an NVS file may live under: the working directory and $TMPDIR (or /tmp). */
inline std::vector<std::string> AllowedNvsBases() {
  std::vector<std::string> bases;
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    if (auto c = CanonicalDir(cwd)) bases.push_back(*c);
  }
  const char* tmp = std::getenv("TMPDIR");
  if (auto c = CanonicalDir((tmp != nullptr && *tmp != '\0') ? tmp : "/tmp")) bases.push_back(*c);
  return bases;
}

/**
 * @brief Validate a user-supplied NVS file path.
 * @return the canonical path (canonical parent directory + file name), or
 *         nullopt when the path has a `..` component, names something other
 *         than a regular file, or lies outside every allowed base directory.
 */
inline std::optional<std::string> ValidateNvsPath(const std::string& requested,
                                                  const std::vector<std::string>& bases = AllowedNvsBases()) {
  if (requested.empty() || requested.find('\0') != std::string::npos) {
    return std::nullopt;
  }
  // Reject any ".." component before resolving anything.
  std::size_t start = 0;
  while (start <= requested.size()) {
    const std::size_t end = requested.find('/', start);
    const std::string part = requested.substr(start, end == std::string::npos ? std::string::npos : end - start);
    if (part == "..") {
      return std::nullopt;
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  const std::size_t slash = requested.rfind('/');
  const std::string name = slash == std::string::npos ? requested : requested.substr(slash + 1);
  const std::string dir = slash == std::string::npos ? "." : (slash == 0 ? "/" : requested.substr(0, slash));
  const auto safe_name = CopyThroughCharset(name, kNvsNameChars);
  if (!safe_name || safe_name->empty() || *safe_name == "." || *safe_name == "..") {
    return std::nullopt;
  }
  const auto parent = CanonicalDir(dir);
  if (!parent) {
    return std::nullopt;
  }
  bool inside = false;
  for (const auto& b : bases) {
    if (*parent == b || (parent->size() > b.size() && parent->compare(0, b.size(), b) == 0 &&
                         ((*parent)[b.size()] == '/' || b == "/"))) {
      inside = true;
      break;
    }
  }
  if (!inside) {
    return std::nullopt;
  }
  const auto full_opt = CopyThroughCharset((*parent == "/" ? "" : *parent) + "/" + *safe_name, kNvsPathChars);
  if (!full_opt) {
    return std::nullopt;  // the canonical directory has characters outside the path set
  }
  const std::string& full = *full_opt;
  struct stat st {};
  if (lstat(full.c_str(), &st) == 0 && !S_ISREG(st.st_mode)) {
    return std::nullopt;  // directories, symlinks, devices, sockets
  }
  return full;
}

class SimNvs {
 public:
  /** @param path a path returned by ValidateNvsPath(), or empty for memory only */
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
