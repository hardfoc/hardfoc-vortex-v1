#include "VortexSystemThread.h"

#include "CANOpenBLDCThread.h"
#include "VortexCanHostThread.h"
#if CONFIG_VORTEX_APP_CAN_BOOT_CANOPEN_SLAVE
#include "CanopenSlaveRuntime.hpp"
#endif
#include "Vortex.h"
#include "managers/CommChannelsManager.h"

#include "handlers/logger/Logger.h"

#include "base/BaseCan.h"

#include <sdkconfig.h>

namespace vortex_app {

namespace {

static const char* TAG = "VortexSys";

constexpr uint8_t kDefaultMotorNodeId = 1U;

bool SetupVortexHal(VortexSystemStateMachine& sm) noexcept {
  (void)sm.TryTransition(VortexSystemState::HalInit, VortexChangeReason::Init);
  auto& vortex = Vortex::GetInstance();
  if (!vortex.EnsureInitialized()) {
    Logger::GetInstance().Error(TAG, "Vortex HAL EnsureInitialized failed");
    return false;
  }
  Logger::GetInstance().Info(TAG, "Vortex HAL ready");
  return true;
}

bool TeardownVortexHal(VortexSystemStateMachine&) noexcept {
  (void)Vortex::GetInstance().Shutdown();
  return true;
}

bool SetupWs2812(VortexSystemStateMachine& sm) noexcept {
  (void)sm;
  auto& orch = VortexSystemThread::GetInstance();
  auto& led = orch.Ws2812Test();
  if (!led.EnsureInitialized()) {
    Logger::GetInstance().Warn(TAG, "WS2812 thread EnsureInitialized failed");
    return false;
  }
  if (!led.Start()) {
    Logger::GetInstance().Warn(TAG, "WS2812 thread Start failed");
    return false;
  }
  Logger::GetInstance().Info(TAG, "WS2812 thread running");
  return true;
}

bool TeardownWs2812(VortexSystemStateMachine&) noexcept {
  (void)VortexSystemThread::GetInstance().Ws2812Test().Stop();
  return true;
}

bool SetupCanWorker(VortexSystemStateMachine& sm) noexcept {
  (void)sm.TryTransition(VortexSystemState::ThreadsUp, VortexChangeReason::Init);
#if CONFIG_VORTEX_APP_CAN_BOOT_NONE
  Logger::GetInstance().Info(TAG, "CAN worker disabled (menuconfig: none)");
  return true;
#else
  auto& orch = VortexSystemThread::GetInstance();
  BaseCan* pcan = CommChannelsManager::GetInstance().GetCanOpenBus();
  if (!pcan) {
    Logger::GetInstance().Warn(TAG, "CAN bus unavailable (skip CAN worker)");
    return false;
  }
#if CONFIG_VORTEX_APP_CAN_BOOT_VORTEX_HOST
  orch.MutableVortexCanHost() = std::make_unique<VortexCanHostThread>(kDefaultMotorNodeId, *pcan);
  if (!orch.MutableVortexCanHost()->EnsureInitialized()) {
    Logger::GetInstance().Warn(TAG, "VortexCanHost EnsureInitialized failed");
    orch.MutableVortexCanHost().reset();
    return false;
  }
  if (!orch.MutableVortexCanHost()->Start()) {
    Logger::GetInstance().Warn(TAG, "VortexCanHost Start failed");
    orch.MutableVortexCanHost().reset();
    return false;
  }
  Logger::GetInstance().Info(TAG, "VortexCanHost thread running (node %u)",
                             static_cast<unsigned>(kDefaultMotorNodeId));
  return true;
#elif CONFIG_VORTEX_APP_CAN_BOOT_CANOPEN_SLAVE
  if (!StartCanopenSlaveRuntime(*pcan)) {
    Logger::GetInstance().Warn(TAG, "CANopenNode slave runtime failed to start");
    return false;
  }
  Logger::GetInstance().Info(TAG, "CANopenNode slave running (node %u)",
                             static_cast<unsigned>(CONFIG_VORTEX_CANOPEN_NODE_ID));
  return true;
#else
  orch.MutableCanOpenBldc() = std::make_unique<CANOpenBLDCThread>(kDefaultMotorNodeId, *pcan);
  if (!orch.MutableCanOpenBldc()->EnsureInitialized()) {
    Logger::GetInstance().Warn(TAG, "CANOpen BLDC EnsureInitialized failed");
    orch.MutableCanOpenBldc().reset();
    return false;
  }
  if (!orch.MutableCanOpenBldc()->Start()) {
    Logger::GetInstance().Warn(TAG, "CANOpen BLDC Start failed");
    orch.MutableCanOpenBldc().reset();
    return false;
  }
  Logger::GetInstance().Info(TAG, "CANOpen BLDC thread running (node %u)", static_cast<unsigned>(kDefaultMotorNodeId));
  return true;
#endif
#endif
}

bool TeardownCanWorker(VortexSystemStateMachine&) noexcept {
  auto& orch = VortexSystemThread::GetInstance();
#if CONFIG_VORTEX_APP_CAN_BOOT_VORTEX_HOST
  if (orch.MutableVortexCanHost()) {
    (void)orch.MutableVortexCanHost()->Stop();
    orch.MutableVortexCanHost().reset();
  }
#elif CONFIG_VORTEX_APP_CAN_BOOT_CANOPEN_SLAVE
  StopCanopenSlaveRuntime();
#elif CONFIG_VORTEX_APP_CAN_BOOT_NONE
  (void)orch;
#else
  if (orch.MutableCanOpenBldc()) {
    (void)orch.MutableCanOpenBldc()->Stop();
    orch.MutableCanOpenBldc().reset();
  }
#endif
  return true;
}

constexpr VortexSystemThread::InitStep kInitSequence[] = {
    {"vortex_hal", SetupVortexHal, TeardownVortexHal, false},
    {"ws2812", SetupWs2812, TeardownWs2812, true},
    {"can_worker", SetupCanWorker, TeardownCanWorker, true},
};

constexpr std::size_t kInitSequenceCount = sizeof(kInitSequence) / sizeof(kInitSequence[0]);

}  // namespace

VortexSystemThread& VortexSystemThread::GetInstance() noexcept {
  static VortexSystemThread inst;
  return inst;
}

VortexSystemThread::VortexSystemThread() noexcept : BaseThread("VortexSys") {}

bool VortexSystemThread::Initialize() noexcept {
  return CreateBaseThread(thread_stack_, sizeof(thread_stack_), kThreadPriority, kThreadPriority, 0,
                          OS_AUTO_START);
}

std::size_t VortexSystemThread::InitSequenceLength() noexcept { return kInitSequenceCount; }

const VortexSystemThread::InitStep* VortexSystemThread::InitSequenceAt(std::size_t i) noexcept {
  return (i < kInitSequenceCount) ? &kInitSequence[i] : nullptr;
}

bool VortexSystemThread::Setup() noexcept {
  auto& sm = VortexSystemStateMachine::GetInstance();
  sm.TryTransition(VortexSystemState::Boot, VortexChangeReason::Init);
  boot_succeeded_ = false;
  last_completed_step_ = -1;

  Logger::GetInstance().Info(TAG, "Boot: %u init steps", static_cast<unsigned>(kInitSequenceCount));

  uint32_t degraded_mask = 0;
  for (std::size_t i = 0; i < kInitSequenceCount; ++i) {
    const auto& step = kInitSequence[i];
    Logger::GetInstance().Info(TAG, "  [%u/%u] %s", static_cast<unsigned>(i + 1),
                               static_cast<unsigned>(kInitSequenceCount), step.name);
    const bool ok = step.setup ? step.setup(sm) : true;
    if (!ok) {
      Logger::GetInstance().Warn(TAG, "  step '%s' failed (recoverable=%s)", step.name,
                                 step.recoverable ? "yes" : "no");
      if (!step.recoverable) {
        sm.TryTransition(VortexSystemState::Fault, VortexChangeReason::Error);
        TeardownThrough(last_completed_step_);
        return false;
      }
      degraded_mask |= (1U << i);
    }
    last_completed_step_ = static_cast<int>(i);
  }

  (void)degraded_mask;
  sm.TryTransition(VortexSystemState::Ready, VortexChangeReason::Init);
  boot_succeeded_ = true;
  Logger::GetInstance().Info(TAG, "Boot complete — state=%s", sm.StateName());
  return true;
}

uint32_t VortexSystemThread::Step() noexcept {
  ++tick_;
  if ((tick_ % 600U) == 0U) {
#if CONFIG_VORTEX_APP_CAN_BOOT_VORTEX_HOST
    auto* vh = vortex_can_host_.get();
    if (vh && vh->IsThreadRunning()) {
      Logger::GetInstance().Info(TAG, "VortexCanHost: rx_frames_total=%lu",
                                 static_cast<unsigned long>(vh->rxFramesTotal()));
    }
#endif
#if CONFIG_VORTEX_APP_CAN_BOOT_BLDC
    auto* co = canopen_bldc_.get();
    if (co && co->IsThreadRunning()) {
      const auto st = co->GetMotorStatus();
      Logger::GetInstance().Info(
          TAG, "CANOpen: present=%s enabled=%s pos=%d vel=%d", st.nodePresent ? "yes" : "no",
          st.isEnabled ? "yes" : "no", static_cast<int>(st.actualPosition), static_cast<int>(st.actualVelocity));
    }
#endif
  }
  return kDispatchPeriodMs;
}

bool VortexSystemThread::Cleanup() noexcept {
  Logger::GetInstance().Warn(TAG, "VortexSystemThread Cleanup()");
  VortexSystemStateMachine::GetInstance().TryTransition(VortexSystemState::Fault, VortexChangeReason::Cleanup);
  TeardownThrough(last_completed_step_);
  return true;
}

void VortexSystemThread::TeardownThrough(int last_completed_index) noexcept {
  if (last_completed_index < 0) {
    return;
  }
  auto& sm = VortexSystemStateMachine::GetInstance();
  for (int i = last_completed_index; i >= 0; --i) {
    const auto& step = kInitSequence[static_cast<std::size_t>(i)];
    if (!step.teardown) {
      continue;
    }
    Logger::GetInstance().Info(TAG, "  teardown [%d] %s", i + 1, step.name);
    (void)step.teardown(sm);
  }
}

bool VortexSystemThread::ResetVariables() noexcept {
  last_completed_step_ = -1;
  tick_ = 0;
  boot_succeeded_ = false;
  return true;
}

}  // namespace vortex_app
