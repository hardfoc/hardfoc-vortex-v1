#pragma once

class Vortex;

namespace vortex_app {
class VortexSystemThread;
}

/**
 * @brief Product demo loop + health monitoring (runs on the IDF `main` task after boot).
 * @details HAL and worker threads are already started by `VortexSystemThread` before this runs.
 */
void RunVortexSupervisorLoop(Vortex& vortex, vortex_app::VortexSystemThread& orch);
