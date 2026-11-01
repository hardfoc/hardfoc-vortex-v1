#pragma once
/**
 * @file check.hpp
 * @brief Minimal host test runner (the CHECK pattern of ~/w/pw/tests/host).
 *
 * @details No third-party framework: the contracts are constexpr and
 *          dependency-free, and so are their tests. `TEST_CASE` registers a
 *          function; `main` (check_main.cpp) runs every case and prints the
 *          number of cases, checks and failures. A failed CHECK prints file,
 *          line and expression and lets the case continue.
 */
#include <cstdint>
#include <cstdio>

namespace vxtest {

struct Case {
  const char* name;
  void (*fn)();
  Case* next;
};

struct Registry {
  Case* head{nullptr};
  Case* tail{nullptr};
  unsigned cases{0};
  unsigned long checks{0};
  unsigned long failures{0};
  const char* current{""};

  static Registry& Get() {
    static Registry r;
    return r;
  }
};

struct Register {
  Case c;
  Register(const char* name, void (*fn)()) : c{name, fn, nullptr} {
    Registry& r = Registry::Get();
    if (r.tail == nullptr) {
      r.head = &c;
    } else {
      r.tail->next = &c;
    }
    r.tail = &c;
  }
};

inline bool Check(bool ok, const char* expr, const char* file, int line) {
  Registry& r = Registry::Get();
  ++r.checks;
  if (!ok) {
    ++r.failures;
    std::fprintf(stderr, "%s:%d: [%s] CHECK failed: %s\n", file, line, r.current, expr);
  }
  return ok;
}

template <typename A, typename B>
bool CheckEq(const A& a, const B& b, const char* ea, const char* eb, const char* file, int line) {
  Registry& r = Registry::Get();
  ++r.checks;
  if (!(a == b)) {
    ++r.failures;
    std::fprintf(stderr, "%s:%d: [%s] CHECK_EQ failed: %s == %s (0x%llx vs 0x%llx)\n", file, line, r.current, ea, eb,
                 static_cast<unsigned long long>(a), static_cast<unsigned long long>(b));
    return false;
  }
  return true;
}

}  // namespace vxtest

#define VX_CAT2(a, b) a##b
#define VX_CAT(a, b) VX_CAT2(a, b)

#define TEST_CASE(name)                                                   \
  static void name();                                                     \
  static ::vxtest::Register VX_CAT(vx_reg_, name)(#name, &name);          \
  static void name()

#define CHECK(c) ::vxtest::Check(static_cast<bool>(c), #c, __FILE__, __LINE__)
#define CHECK_EQ(a, b) ::vxtest::CheckEq((a), (b), #a, #b, __FILE__, __LINE__)
