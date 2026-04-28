#pragma once

#include "base/BaseCan.h"

namespace vortex_app {

/** Start FreeRTOS task running CANopenNode slave on @p can (single RX owner). */
bool StartCanopenSlaveRuntime(BaseCan& can) noexcept;

/** Stop task and tear down stack. */
void StopCanopenSlaveRuntime() noexcept;

bool IsCanopenSlaveRuntimeRunning() noexcept;

}  // namespace vortex_app
