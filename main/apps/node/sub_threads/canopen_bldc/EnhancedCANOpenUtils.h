#pragma once

#include "CanOpenMotorUtils.h"
#include "CanOpenUtils.h"
#include "vortex/VortexProtocol.hpp"

namespace CanOpen {

/**
 * @brief Extended CANOpen utilities for enhanced BLDC motor control
 *
 * This class extends the basic CANOpen utilities with additional
 * functionality for comprehensive motor control and diagnostics.
 */
class EnhancedCANOpenUtils {
public:
  /**
   * @brief Build an emergency (EMCY) frame
   * @param nodeId Node ID of the device
   * @param errorCode Emergency error code
   * @param errorRegister Error register content
   * @param manufacturerData Additional manufacturer-specific data
   * @return Populated CAN frame
   */
  static CanFrame BuildEmergency(uint8_t nodeId, uint16_t errorCode,
                                 uint8_t errorRegister,
                                 uint32_t manufacturerData = 0) noexcept;

  /**
   * @brief Build a PDO (Process Data Object) frame
   * @param nodeId Node ID of the device
   * @param pdoNumber PDO number (1-4)
   * @param isReceive true for RPDO, false for TPDO
   * @param data Data bytes (up to 8 bytes)
   * @param dataLength Length of data
   * @return Populated CAN frame
   */
  static CanFrame BuildPDO(uint8_t nodeId, uint8_t pdoNumber, bool isReceive,
                           const uint8_t *data, uint8_t dataLength) noexcept;

  /**
   * @brief Build a SYNC frame
   * @return Populated CAN frame
   */
  static CanFrame BuildSync() noexcept;

  /**
   * @brief Build a TIME frame
   * @param timeOfDay Time of day in milliseconds
   * @param days Days since January 1, 1984
   * @return Populated CAN frame
   */
  static CanFrame BuildTime(uint32_t timeOfDay, uint16_t days) noexcept;

  /**
   * @brief Build manufacturer-specific frame
   * @param nodeId Node ID of the device
   * @param functionCode Manufacturer function code
   * @param data Data bytes
   * @param dataLength Length of data
   * @return Populated CAN frame
   */
  static CanFrame BuildManufacturerSpecific(uint8_t nodeId,
                                            uint8_t functionCode,
                                            const uint8_t *data,
                                            uint8_t dataLength) noexcept;

  /**
   * @brief Extended motor control commands
   */

  /**
   * @brief Build a motor enable sequence (multiple frames)
   * @param nodeId Node ID of the device
   * @param frames Array to store the sequence frames
   * @param maxFrames Maximum number of frames in array
   * @return Number of frames in the sequence
   */
  static uint8_t BuildMotorEnableSequence(uint8_t nodeId, CanFrame *frames,
                                          uint8_t maxFrames) noexcept;

  /**
   * @brief Build a motor disable sequence (multiple frames)
   * @param nodeId Node ID of the device
   * @param frames Array to store the sequence frames
   * @param maxFrames Maximum number of frames in array
   * @return Number of frames in the sequence
   */
  static uint8_t BuildMotorDisableSequence(uint8_t nodeId, CanFrame *frames,
                                           uint8_t maxFrames) noexcept;

  /**
   * @brief Build interpolated position mode frame (index 0x60C0)
   * @param nodeId Node ID of the device
   * @param subIndex Sub-index (0x00 for interpolation period)
   * @param value Value to set
   * @return Populated CAN frame
   */
  static CanFrame BuildInterpolationPeriod(uint8_t nodeId, uint8_t subIndex,
                                           uint8_t value) noexcept;

  /**
   * @brief Build motor configuration frames for typical BLDC setup
   * @param nodeId Node ID of the device
   * @param frames Array to store configuration frames
   * @param maxFrames Maximum number of frames in array
   * @param polePairs Number of motor pole pairs
   * @param encoderResolution Encoder resolution in counts per revolution
   * @return Number of frames in the configuration sequence
   */
  static uint8_t BuildBLDCConfigSequence(uint8_t nodeId, CanFrame *frames,
                                         uint8_t maxFrames, uint8_t polePairs,
                                         uint32_t encoderResolution) noexcept;

  /**
   * @brief Utility functions for frame analysis
   */

  /**
   * @brief Check if frame is a CANOpen emergency frame
   * @param frame Frame to check
   * @return true if emergency frame
   */
  static bool IsEmergencyFrame(const CanFrame &frame) noexcept;

  /**
   * @brief Check if frame is a CANOpen heartbeat frame
   * @param frame Frame to check
   * @return true if heartbeat frame
   */
  static bool IsHeartbeatFrame(const CanFrame &frame) noexcept;

  /**
   * @brief Check if frame is an SDO response
   * @param frame Frame to check
   * @return true if SDO response
   */
  static bool IsSDOResponse(const CanFrame &frame) noexcept;

  /**
   * @brief Extract node ID from CANOpen frame
   * @param frame Frame to analyze
   * @return Node ID (0 if not a node-specific frame)
   */
  static uint8_t ExtractNodeId(const CanFrame &frame) noexcept;

  /**
   * @brief Extract object dictionary index from SDO frame
   * @param frame SDO frame to analyze
   * @param index Reference to store the index
   * @param subIndex Reference to store the sub-index
   * @return true if extraction successful
   */
  static bool ExtractSDOIndex(const CanFrame &frame, uint16_t &index,
                              uint8_t &subIndex) noexcept;

  /**
   * @brief Decode DS402 status word
   * @param statusWord Raw status word value
   * @return Decoded drive state
   */
  enum class DriveState : uint8_t {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 1,
    READY_TO_SWITCH_ON = 2,
    SWITCHED_ON = 3,
    OPERATION_ENABLED = 4,
    QUICK_STOP_ACTIVE = 5,
    FAULT_REACTION_ACTIVE = 6,
    FAULT = 7
  };

  static DriveState DecodeStatusWord(uint16_t statusWord) noexcept;

  /**
   * @brief Check if target is reached from status word
   * @param statusWord Raw status word value
   * @return true if target reached
   */
  static bool IsTargetReached(uint16_t statusWord) noexcept;

  /**
   * @brief Check if homing is completed from status word
   * @param statusWord Raw status word value
   * @return true if homing completed
   */
  static bool IsHomingCompleted(uint16_t statusWord) noexcept;
};

/**
 * @brief CANopen CiA 301 / 402 indices — aliases to **`hf::vortex::VortexProtocol`**.
 * @details Canonical definitions live in `vortex/VortexProtocol.hpp` (hf-vortex-driver).
 *          Keep using `CanOpen::ObjectDictionary::*` in app code for backward compatibility.
 */
namespace ObjectDictionary {
constexpr uint16_t DEVICE_TYPE = hf::vortex::kIdxDeviceType;
constexpr uint16_t ERROR_REGISTER = hf::vortex::kIdxErrorRegister;
constexpr uint16_t MANUFACTURER_STATUS_REGISTER = hf::vortex::kIdxManufacturerStatusRegister;
constexpr uint16_t PREDEFINED_ERROR_FIELD = hf::vortex::kIdxPredefError;
constexpr uint16_t MANUFACTURER_DEVICE_NAME = hf::vortex::kIdxManufacturerDeviceName;
constexpr uint16_t MANUFACTURER_HARDWARE_VERSION = hf::vortex::kIdxManufacturerHardwareVersion;
constexpr uint16_t MANUFACTURER_SOFTWARE_VERSION = hf::vortex::kIdxManufacturerSoftwareVersion;
constexpr uint16_t GUARD_TIME = hf::vortex::kIdxGuardTime;
constexpr uint16_t LIFE_TIME_FACTOR = hf::vortex::kIdxLifeTimeFactor;
constexpr uint16_t STORE_PARAMETERS = hf::vortex::kIdxStoreParameters;
constexpr uint16_t RESTORE_DEFAULT_PARAMETERS = hf::vortex::kIdxRestoreDefaultParameters;

constexpr uint16_t CONTROL_WORD = hf::vortex::kIdxControlword;
constexpr uint16_t STATUS_WORD = hf::vortex::kIdxStatusword;
constexpr uint16_t MODES_OF_OPERATION = hf::vortex::kIdxModesOfOperation;
constexpr uint16_t MODES_OF_OPERATION_DISPLAY = hf::vortex::kIdxModesDisplay;
constexpr uint16_t POSITION_DEMAND_VALUE = hf::vortex::kIdxPosDemand;
constexpr uint16_t POSITION_ACTUAL_VALUE = hf::vortex::kIdxPosActual;
constexpr uint16_t VELOCITY_DEMAND_VALUE = hf::vortex::kIdxVelDemand;
constexpr uint16_t VELOCITY_ACTUAL_VALUE = hf::vortex::kIdxVelActual;
constexpr uint16_t TARGET_TORQUE = hf::vortex::kIdxTargetTorque;
constexpr uint16_t TORQUE_ACTUAL_VALUE = hf::vortex::kIdxTorqueActual;
constexpr uint16_t TARGET_POSITION = hf::vortex::kIdxTargetPosition;
constexpr uint16_t POSITION_RANGE_LIMIT = hf::vortex::kIdxPositionRangeLimit;
constexpr uint16_t HOME_OFFSET = hf::vortex::kIdxHomeOffset;
constexpr uint16_t SOFTWARE_POSITION_LIMIT = hf::vortex::kIdxSoftwarePositionLimit;
constexpr uint16_t MAX_PROFILE_VELOCITY = hf::vortex::kIdxMaxProfileVelocity;
constexpr uint16_t PROFILE_VELOCITY = hf::vortex::kIdxProfileVelocity;
constexpr uint16_t PROFILE_ACCELERATION = hf::vortex::kIdxProfileAcceleration;
constexpr uint16_t PROFILE_DECELERATION = hf::vortex::kIdxProfileDeceleration;
constexpr uint16_t QUICK_STOP_DECELERATION = hf::vortex::kIdxQuickStopDeceleration;
constexpr uint16_t MOTION_PROFILE_TYPE = hf::vortex::kIdxMotionProfileType;
constexpr uint16_t POSITION_WINDOW = hf::vortex::kIdxPositionWindow;
constexpr uint16_t POSITION_WINDOW_TIME = hf::vortex::kIdxPositionWindowTime;
constexpr uint16_t VELOCITY_WINDOW = hf::vortex::kIdxVelocityWindow;
constexpr uint16_t VELOCITY_WINDOW_TIME = hf::vortex::kIdxVelocityWindowTime;
constexpr uint16_t TARGET_VELOCITY = hf::vortex::kIdxTargetVelocity;
constexpr uint16_t INTERPOLATION_TIME_PERIOD = hf::vortex::kIdxInterpolationTimePeriod;

constexpr uint16_t HOMING_METHOD = hf::vortex::kIdxHomingMethod;
constexpr uint16_t HOMING_SPEEDS = hf::vortex::kIdxHomingSpeeds;
constexpr uint16_t HOMING_ACCELERATION = hf::vortex::kIdxHomingAcceleration;

constexpr uint16_t MOTOR_TYPE = hf::vortex::kIdxMotorType;
constexpr uint16_t MOTOR_CATALOGUE_NUMBER = hf::vortex::kIdxMotorCatalogueNumber;
constexpr uint16_t MOTOR_MANUFACTURER = hf::vortex::kIdxMotorManufacturer;
constexpr uint16_t MOTOR_RATED_CURRENT = hf::vortex::kIdxMotorRatedCurrent;
constexpr uint16_t MOTOR_RATED_TORQUE = hf::vortex::kIdxMotorRatedTorque;
constexpr uint16_t MOTOR_RATED_VELOCITY = hf::vortex::kIdxMotorRatedVelocity;
constexpr uint16_t MOTOR_RATED_VOLTAGE = hf::vortex::kIdxMotorRatedVoltage;

constexpr uint16_t ENCODER_INCREMENTS = hf::vortex::kIdxEncoderIncrements;
constexpr uint16_t ENCODER_RESOLUTION = hf::vortex::kIdxEncoderResolution;
} // namespace ObjectDictionary

} // namespace CanOpen
