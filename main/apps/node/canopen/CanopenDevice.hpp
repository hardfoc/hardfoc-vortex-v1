#pragma once
/**
 * @file CanopenDevice.hpp
 * @brief Firmware entry points of the CANopen device: CoNode over the TWAI
 *        port, served by the `co_svc` task. Replaces the interim
 *        CanopenSlaveRuntime.
 */
#include <cstdint>

class EspCan;

namespace vortex_app {

/**
 * @brief Start the CANopen device on @p can (the HAL's TWAI controller).
 * @param bitrate_bps the bit rate @p can runs at (CommChannelsManager::GetCanBitRate())
 * @return false if the stack could not start; the bus is then left alone
 */
bool StartCanopenDevice(EspCan& can, uint32_t bitrate_bps) noexcept;

/** @brief Stop the task, detach the port and free the stack. */
void StopCanopenDevice() noexcept;

bool IsCanopenDeviceRunning() noexcept;

}  // namespace vortex_app
