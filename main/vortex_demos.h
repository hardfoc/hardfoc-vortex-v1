/**
 * @file vortex_demos.h
 * @brief Comprehensive demonstration functions for HardFOC Vortex V1 API
 * 
 * @details This file contains declaration of demonstration functions that showcase
 *          all aspects of the Vortex API including GPIO, ADC, motor controllers,
 *          IMU, encoders, LEDs, and temperature sensors.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 */

#ifndef VORTEX_DEMOS_H_
#define VORTEX_DEMOS_H_

#include "API/Vortex.h"

#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// DEMONSTRATION FUNCTION DECLARATIONS
//==============================================================================

/**
 * @brief Run comprehensive Vortex API demonstration
 * @param vortex Reference to the Vortex API instance
 */
void RunComprehensiveVortexDemo(Vortex& vortex);

/**
 * @brief Demonstrate communication channel functionality
 * @param vortex Reference to the Vortex API instance
 */
void DemoCommChannels(Vortex& vortex);

/**
 * @brief Demonstrate GPIO management functionality
 * @param vortex Reference to the Vortex API instance
 */
void DemoGpioManagement(Vortex& vortex);

/**
 * @brief Demonstrate ADC functionality with batch operations
 * @param vortex Reference to the Vortex API instance
 */
void DemoAdcOperations(Vortex& vortex);

/**
 * @brief Demonstrate motor controller functionality
 * @param vortex Reference to the Vortex API instance
 */
void DemoMotorControl(Vortex& vortex);

/**
 * @brief Demonstrate IMU sensor functionality
 * @param vortex Reference to the Vortex API instance
 */
void DemoImuSensors(Vortex& vortex);

/**
 * @brief Demonstrate encoder functionality
 * @param vortex Reference to the Vortex API instance
 */
void DemoEncoderOperations(Vortex& vortex);

/**
 * @brief Demonstrate LED management with animations
 * @param vortex Reference to the Vortex API instance
 */
void DemoLedAnimations(Vortex& vortex);

/**
 * @brief Demonstrate temperature monitoring
 * @param vortex Reference to the Vortex API instance
 */
void DemoTemperatureMonitoring(Vortex& vortex);

/**
 * @brief Demonstrate system health monitoring
 * @param vortex Reference to the Vortex API instance
 */
void DemoSystemHealth(Vortex& vortex);

/**
 * @brief Demonstrate performance optimization techniques
 * @param vortex Reference to the Vortex API instance
 */
void DemoPerformanceOptimization(Vortex& vortex);

/**
 * @brief Run stress test for all components
 * @param vortex Reference to the Vortex API instance
 * @param duration_seconds Duration to run stress test
 */
void RunStressTest(Vortex& vortex, uint32_t duration_seconds);

#ifdef __cplusplus
}
#endif

#endif // VORTEX_DEMOS_H_