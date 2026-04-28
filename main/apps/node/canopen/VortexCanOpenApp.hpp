/**
 * @file VortexCanOpenApp.hpp
 * @brief Barrel include for the Vortex **CANopen client** app layer (NMT + expedited SDO path).
 * @details Prefer including this from new `apps/node` code instead of scattering low-level
 *          headers. Threads may still include specifics they need (e.g. `CANOpenBLDCThread.h`).
 */
#pragma once

#include "CanOpenExtras.h"
#include "CanOpenMotorUtils.h"
#include "CanOpenUtils.h"
#include "vortex/VortexProtocol.hpp"
