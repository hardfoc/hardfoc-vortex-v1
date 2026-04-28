#include "CanOpenHostServiceThread.h"
#include "handlers/logger/Logger.h"

static const char* TAG = "CanOpenHost";

CanOpenHostServiceThread::CanOpenHostServiceThread(BaseCan& can)
    : BaseThread("CanOpenHost"), transport_(can) {}

bool CanOpenHostServiceThread::Initialize() noexcept {
  return true;
}

bool CanOpenHostServiceThread::ResetVariables() noexcept { return true; }

bool CanOpenHostServiceThread::Setup() noexcept {
  if (!transport_.can().EnsureInitialized()) {
    Logger::GetInstance().Error(TAG, "CAN not initialized");
    return false;
  }
  Logger::GetInstance().Info(TAG, "Host service: CAN ready");
  return true;
}

uint32_t CanOpenHostServiceThread::Step() noexcept {
  CanOpen::CanFrame f{};
  unsigned n = 0;
  while (n < max_per_step_ && transport_.receive(f, 0)) {
    if (on_frame_) {
      on_frame_(f);
    }
    ++n;
  }
  return 1;
}

bool CanOpenHostServiceThread::Cleanup() noexcept {
  return true;
}
