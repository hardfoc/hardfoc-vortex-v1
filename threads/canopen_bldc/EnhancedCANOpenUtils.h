#pragma once

#include "CanOpenMotorUtils.h"
#include "CanOpenUtils.h"

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
 * @brief CANOpen object dictionary indices for motor control
 */
namespace ObjectDictionary {
// Device control
constexpr uint16_t DEVICE_TYPE = 0x1000;
constexpr uint16_t ERROR_REGISTER = 0x1001;
constexpr uint16_t MANUFACTURER_STATUS_REGISTER = 0x1002;
constexpr uint16_t PREDEFINED_ERROR_FIELD = 0x1003;
constexpr uint16_t MANUFACTURER_DEVICE_NAME = 0x1008;
constexpr uint16_t MANUFACTURER_HARDWARE_VERSION = 0x1009;
constexpr uint16_t MANUFACTURER_SOFTWARE_VERSION = 0x100A;
constexpr uint16_t GUARD_TIME = 0x100C;
constexpr uint16_t LIFE_TIME_FACTOR = 0x100D;
constexpr uint16_t STORE_PARAMETERS = 0x1010;
constexpr uint16_t RESTORE_DEFAULT_PARAMETERS = 0x1011;

// DS402 Motor control objects
constexpr uint16_t CONTROL_WORD = 0x6040;
constexpr uint16_t STATUS_WORD = 0x6041;
constexpr uint16_t MODES_OF_OPERATION = 0x6060;
constexpr uint16_t MODES_OF_OPERATION_DISPLAY = 0x6061;
constexpr uint16_t POSITION_DEMAND_VALUE = 0x6062;
constexpr uint16_t POSITION_ACTUAL_VALUE = 0x6064;
constexpr uint16_t VELOCITY_DEMAND_VALUE = 0x606B;
constexpr uint16_t VELOCITY_ACTUAL_VALUE = 0x606C;
constexpr uint16_t TARGET_TORQUE = 0x6071;
constexpr uint16_t TORQUE_ACTUAL_VALUE = 0x6077;
constexpr uint16_t TARGET_POSITION = 0x607A;
constexpr uint16_t POSITION_RANGE_LIMIT = 0x607B;
constexpr uint16_t HOME_OFFSET = 0x607C;
constexpr uint16_t SOFTWARE_POSITION_LIMIT = 0x607D;
constexpr uint16_t MAX_PROFILE_VELOCITY = 0x607F;
constexpr uint16_t PROFILE_VELOCITY = 0x6081;
constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
constexpr uint16_t PROFILE_DECELERATION = 0x6084;
constexpr uint16_t QUICK_STOP_DECELERATION = 0x6085;
constexpr uint16_t MOTION_PROFILE_TYPE = 0x6086;
constexpr uint16_t POSITION_WINDOW = 0x6067;
constexpr uint16_t POSITION_WINDOW_TIME = 0x6068;
constexpr uint16_t VELOCITY_WINDOW = 0x606D;
constexpr uint16_t VELOCITY_WINDOW_TIME = 0x606E;
constexpr uint16_t TARGET_VELOCITY = 0x60FF;

// Homing mode objects
constexpr uint16_t HOMING_METHOD = 0x6098;
constexpr uint16_t HOMING_SPEEDS = 0x6099;
constexpr uint16_t HOMING_ACCELERATION = 0x609A;

// Motor specific objects
constexpr uint16_t MOTOR_TYPE = 0x6402;
constexpr uint16_t MOTOR_CATALOGUE_NUMBER = 0x6403;
constexpr uint16_t MOTOR_MANUFACTURER = 0x6404;
constexpr uint16_t MOTOR_RATED_CURRENT = 0x6410;
constexpr uint16_t MOTOR_RATED_TORQUE = 0x6411;
constexpr uint16_t MOTOR_RATED_VELOCITY = 0x6412;
constexpr uint16_t MOTOR_RATED_VOLTAGE = 0x6413;

// Encoder objects
constexpr uint16_t ENCODER_INCREMENTS = 0x608F;
constexpr uint16_t ENCODER_RESOLUTION = 0x6090;
} // namespace ObjectDictionary

} // namespace CanOpen
