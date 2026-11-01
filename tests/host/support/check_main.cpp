#include "check.hpp"

int main() {
  vxtest::Registry& r = vxtest::Registry::Get();
  for (vxtest::Case* c = r.head; c != nullptr; c = c->next) {
    r.current = c->name;
    const unsigned long before = r.failures;
    c->fn();
    ++r.cases;
    std::printf("[%s] %s\n", r.failures == before ? " ok " : "FAIL", c->name);
  }
  std::printf("%u cases, %lu checks, %lu failures\n", r.cases, r.checks, r.failures);
  return r.failures == 0 ? 0 : 1;
}
