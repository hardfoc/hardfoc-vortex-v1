#include "EnhancedCANOpenUtils.h"
#include <cstring>

namespace CanOpen {

CanFrame
EnhancedCANOpenUtils::BuildEmergency(uint8_t nodeId, uint16_t errorCode,
                                     uint8_t errorRegister,
                                     uint32_t manufacturerData) noexcept {
  CanFrame frame{};
  frame.id = 0x080 + nodeId; // Emergency object
  frame.dlc = 8;
  frame.extended = false;
  frame.rtr = false;

  frame.data[0] = static_cast<uint8_t>(errorCode & 0xFF);
  frame.data[1] = static_cast<uint8_t>((errorCode >> 8) & 0xFF);
  frame.data[2] = errorRegister;
  frame.data[3] = static_cast<uint8_t>(manufacturerData & 0xFF);
  frame.data[4] = static_cast<uint8_t>((manufacturerData >> 8) & 0xFF);
  frame.data[5] = static_cast<uint8_t>((manufacturerData >> 16) & 0xFF);
  frame.data[6] = static_cast<uint8_t>((manufacturerData >> 24) & 0xFF);
  frame.data[7] = 0x00;

  return frame;
}

CanFrame EnhancedCANOpenUtils::BuildPDO(uint8_t nodeId, uint8_t pdoNumber,
                                        bool isReceive, const uint8_t *data,
                                        uint8_t dataLength) noexcept {
  CanFrame frame{};

  // Calculate COB-ID based on PDO type and number
  if (isReceive) {
    // RPDO (Receive PDO)
    switch (pdoNumber) {
    case 1:
      frame.id = 0x200 + nodeId;
      break;
    case 2:
      frame.id = 0x300 + nodeId;
      break;
    case 3:
      frame.id = 0x400 + nodeId;
      break;
    case 4:
      frame.id = 0x500 + nodeId;
      break;
    default:
      frame.id = 0x200 + nodeId;
      break;
    }
  } else {
    // TPDO (Transmit PDO)
    switch (pdoNumber) {
    case 1:
      frame.id = 0x180 + nodeId;
      break;
    case 2:
      frame.id = 0x280 + nodeId;
      break;
    case 3:
      frame.id = 0x380 + nodeId;
      break;
    case 4:
      frame.id = 0x480 + nodeId;
      break;
    default:
      frame.id = 0x180 + nodeId;
      break;
    }
  }

  frame.dlc = (dataLength > 8) ? 8 : dataLength;
  frame.extended = false;
  frame.rtr = false;

  if (data != nullptr && dataLength > 0) {
    std::memcpy(frame.data, data, frame.dlc);
  }

  return frame;
}

CanFrame EnhancedCANOpenUtils::BuildSync() noexcept {
  CanFrame frame{};
  frame.id = 0x080; // SYNC COB-ID
  frame.dlc = 0;
  frame.extended = false;
  frame.rtr = false;
  return frame;
}

CanFrame EnhancedCANOpenUtils::BuildTime(uint32_t timeOfDay,
                                         uint16_t days) noexcept {
  CanFrame frame{};
  frame.id = 0x100; // TIME COB-ID
  frame.dlc = 6;
  frame.extended = false;
  frame.rtr = false;

  frame.data[0] = static_cast<uint8_t>(timeOfDay & 0xFF);
  frame.data[1] = static_cast<uint8_t>((timeOfDay >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((timeOfDay >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((timeOfDay >> 24) & 0xFF);
  frame.data[4] = static_cast<uint8_t>(days & 0xFF);
  frame.data[5] = static_cast<uint8_t>((days >> 8) & 0xFF);

  return frame;
}

CanFrame EnhancedCANOpenUtils::BuildManufacturerSpecific(
    uint8_t nodeId, uint8_t functionCode, const uint8_t *data,
    uint8_t dataLength) noexcept {
  CanFrame frame{};
  frame.id = 0x780 + nodeId; // Manufacturer-specific range
  frame.dlc = (dataLength > 7) ? 8 : dataLength + 1;
  frame.extended = false;
  frame.rtr = false;

  frame.data[0] = functionCode;
  if (data != nullptr && dataLength > 0) {
    std::memcpy(&frame.data[1], data, (dataLength > 7) ? 7 : dataLength);
  }

  return frame;
}

uint8_t
EnhancedCANOpenUtils::BuildMotorEnableSequence(uint8_t nodeId, CanFrame *frames,
                                               uint8_t maxFrames) noexcept {
  if (frames == nullptr || maxFrames < 3) {
    return 0;
  }

  uint8_t frameCount = 0;

  // 1. Shutdown
  frames[frameCount++] = BuildControlWord(
      nodeId, static_cast<uint16_t>(ControlWordCommand::Shutdown));

  // 2. Switch On
  frames[frameCount++] = BuildControlWord(
      nodeId, static_cast<uint16_t>(ControlWordCommand::SwitchOn));

  // 3. Enable Operation
  frames[frameCount++] = BuildControlWord(
      nodeId, static_cast<uint16_t>(ControlWordCommand::EnableOperation));

  return frameCount;
}

uint8_t EnhancedCANOpenUtils::BuildMotorDisableSequence(
    uint8_t nodeId, CanFrame *frames, uint8_t maxFrames) noexcept {
  if (frames == nullptr || maxFrames < 1) {
    return 0;
  }

  uint8_t frameCount = 0;

  // Disable voltage
  frames[frameCount++] = BuildControlWord(
      nodeId, static_cast<uint16_t>(ControlWordCommand::DisableVoltage));

  return frameCount;
}

CanFrame
EnhancedCANOpenUtils::BuildInterpolationPeriod(uint8_t nodeId, uint8_t subIndex,
                                               uint8_t value) noexcept {
  return BuildSdoDownload(nodeId, 0x60C0, subIndex, value, 1);
}

uint8_t EnhancedCANOpenUtils::BuildBLDCConfigSequence(
    uint8_t nodeId, CanFrame *frames, uint8_t maxFrames, uint8_t polePairs,
    uint32_t encoderResolution) noexcept {
  if (frames == nullptr || maxFrames < 4) {
    return 0;
  }

  uint8_t frameCount = 0;

  // 1. Set motor type (assuming BLDC motor type value)
  frames[frameCount++] = BuildSdoDownload(nodeId, ObjectDictionary::MOTOR_TYPE,
                                          0x00, 3, 2); // BLDC motor

  // 2. Set encoder resolution
  frames[frameCount++] = BuildSdoDownload(
      nodeId, ObjectDictionary::ENCODER_INCREMENTS, 0x00, encoderResolution, 4);

  // 3. Set profile acceleration (default value)
  frames[frameCount++] = BuildSdoDownload(
      nodeId, ObjectDictionary::PROFILE_ACCELERATION, 0x00, 10000, 4);

  // 4. Set profile deceleration (default value)
  frames[frameCount++] = BuildSdoDownload(
      nodeId, ObjectDictionary::PROFILE_DECELERATION, 0x00, 10000, 4);

  return frameCount;
}

bool EnhancedCANOpenUtils::IsEmergencyFrame(const CanFrame &frame) noexcept {
  return (frame.id >= 0x081 && frame.id <= 0x0FF);
}

bool EnhancedCANOpenUtils::IsHeartbeatFrame(const CanFrame &frame) noexcept {
  return (frame.id >= 0x701 && frame.id <= 0x77F);
}

bool EnhancedCANOpenUtils::IsSDOResponse(const CanFrame &frame) noexcept {
  return (frame.id >= 0x581 && frame.id <= 0x5FF);
}

uint8_t EnhancedCANOpenUtils::ExtractNodeId(const CanFrame &frame) noexcept {
  // Check different COB-ID ranges
  if (frame.id >= 0x581 && frame.id <= 0x5FF) {
    return static_cast<uint8_t>(frame.id - 0x580); // SDO response
  } else if (frame.id >= 0x701 && frame.id <= 0x77F) {
    return static_cast<uint8_t>(frame.id - 0x700); // Heartbeat
  } else if (frame.id >= 0x081 && frame.id <= 0x0FF) {
    return static_cast<uint8_t>(frame.id - 0x080); // Emergency
  } else if (frame.id >= 0x180 && frame.id <= 0x1FF) {
    return static_cast<uint8_t>(frame.id - 0x180); // TPDO1
  } else if (frame.id >= 0x280 && frame.id <= 0x2FF) {
    return static_cast<uint8_t>(frame.id - 0x280); // TPDO2
  } else if (frame.id >= 0x380 && frame.id <= 0x3FF) {
    return static_cast<uint8_t>(frame.id - 0x380); // TPDO3
  } else if (frame.id >= 0x480 && frame.id <= 0x4FF) {
    return static_cast<uint8_t>(frame.id - 0x480); // TPDO4
  }

  return 0; // Not a node-specific frame
}

bool EnhancedCANOpenUtils::ExtractSDOIndex(const CanFrame &frame,
                                           uint16_t &index,
                                           uint8_t &subIndex) noexcept {
  if (!IsSDOResponse(frame) || frame.dlc < 4) {
    return false;
  }

  index = static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
  subIndex = frame.data[3];
  return true;
}

EnhancedCANOpenUtils::DriveState
EnhancedCANOpenUtils::DecodeStatusWord(uint16_t statusWord) noexcept {
  // DS402 state machine decoding
  uint16_t stateMask = statusWord & 0x006F; // Bits 0,1,2,3,5,6

  switch (stateMask) {
  case 0x0000: // xxxx xxxx x0xx 0000
    return DriveState::NOT_READY_TO_SWITCH_ON;
  case 0x0040: // xxxx xxxx x1xx 0000
    return DriveState::SWITCH_ON_DISABLED;
  case 0x0021: // xxxx xxxx x01x 0001
    return DriveState::READY_TO_SWITCH_ON;
  case 0x0023: // xxxx xxxx x01x 0011
    return DriveState::SWITCHED_ON;
  case 0x0027: // xxxx xxxx x01x 0111
    return DriveState::OPERATION_ENABLED;
  case 0x0007: // xxxx xxxx x00x 0111
    return DriveState::QUICK_STOP_ACTIVE;
  case 0x000F: // xxxx xxxx x00x 1111
    return DriveState::FAULT_REACTION_ACTIVE;
  case 0x0008: // xxxx xxxx x00x 1000
    return DriveState::FAULT;
  default:
    return DriveState::NOT_READY_TO_SWITCH_ON;
  }
}

bool EnhancedCANOpenUtils::IsTargetReached(uint16_t statusWord) noexcept {
  return (statusWord & 0x0400) != 0; // Bit 10
}

bool EnhancedCANOpenUtils::IsHomingCompleted(uint16_t statusWord) noexcept {
  return (statusWord & 0x1000) != 0; // Bit 12
}

} // namespace CanOpen
