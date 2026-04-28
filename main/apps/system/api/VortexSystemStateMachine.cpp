#include "VortexSystemStateMachine.h"

namespace vortex_app {

VortexSystemStateMachine& VortexSystemStateMachine::GetInstance() noexcept {
  static VortexSystemStateMachine inst;
  return inst;
}

bool VortexSystemStateMachine::TryTransition(VortexSystemState to, VortexChangeReason reason) noexcept {
  state_.store(to, std::memory_order_release);
  last_reason_.store(reason, std::memory_order_release);
  return true;
}

const char* VortexSystemStateMachine::StateName() const noexcept {
  switch (State()) {
    case VortexSystemState::Boot:
      return "Boot";
    case VortexSystemState::HalInit:
      return "HalInit";
    case VortexSystemState::ThreadsUp:
      return "ThreadsUp";
    case VortexSystemState::Ready:
      return "Ready";
    case VortexSystemState::Fault:
      return "Fault";
  }
  return "?";
}

}  // namespace vortex_app
