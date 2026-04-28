#pragma once

#include "BaseThread.h"
#include "CanOpenBaseCanLink.hpp"
#include "VortexCanOpenApp.hpp"
#include "base/BaseCan.h"

#include <memory>
#include <cstdint>

/**
 * @brief CiA402-style motor **client** over CAN (NMT + expedited SDO + basic RX decode).
 * @details Uses **`hf-utils-canopen`** / **`CanOpenExtras`** for frames and parsers, and
 *          **`hf::vortex::VortexProtocol`** for OD indices. This is **not** a full CANopen
 *          slave stack (no CANopenNode); see `apps/node/canopen/README.md`.
 */
class CANOpenBLDCThread : public BaseThread {
public:
  using DriveState = CanOpen::DriveState;
  using OperationMode = CanOpen::OperationMode;

  struct MotorConfig {
    uint8_t nodeId;
    uint32_t maxVelocity;
    uint32_t maxAcceleration;
    uint32_t profileVelocity;
    uint32_t profileAcceleration;
    uint32_t profileDeceleration;
    int32_t softwareLimitPos;
    int32_t softwareLimitNeg;
    uint16_t quickStopDeceleration;
    uint16_t shutdownOption;
    uint16_t disableOption;
    uint16_t faultReaction;
  };

  struct MotorStatus {
    uint16_t statusWord;
    int32_t actualPosition;
    int32_t actualVelocity;
    int16_t actualTorque;
    uint32_t lastUpdateTime;
    bool isEnabled;
    bool isInFault;
    bool isHomed;
    bool nodePresent;
  };

  CANOpenBLDCThread(uint8_t nodeId, BaseCan& canBus);
  virtual ~CANOpenBLDCThread();

  bool EnableMotor();
  bool DisableMotor();
  bool ResetFault();
  bool PerformHoming();

  bool SetPositionMode(int32_t position, uint32_t velocity = 0, uint32_t acceleration = 0);
  bool SetVelocityMode(int32_t velocity, uint32_t acceleration = 0);
  bool SetTorqueMode(int16_t torque);

  bool ConfigureMotor(const MotorConfig& config);
  bool SetProfileParameters(uint32_t velocity, uint32_t acceleration, uint32_t deceleration);

  MotorStatus GetMotorStatus() const;
  bool IsMotorReady() const;
  bool IsMotorEnabled() const;
  bool IsInFault() const;
  DriveState GetDriveState() const;

protected:
  virtual bool Initialize() noexcept override;
  virtual bool Setup() noexcept override;
  virtual uint32_t Step() noexcept override;
  virtual bool Cleanup() noexcept override;
  virtual bool ResetVariables() noexcept override;

private:
  using CanFrame = CanOpen::CanFrame;

  bool setupCANOpenCommunication() noexcept;

  void processIncomingMessages();
  void updateMotorStatus();
  void handleStateMachine();
  void sendPeriodicMessages();
  void handleMotionCommands();
  void processCANOpenMessage(const CanFrame& frame);
  void sendStatusRequest();

  void processEmergencyMessage(const CanFrame& frame);
  void processPDO1Transmit(const CanFrame& frame);
  void processPDO2Transmit(const CanFrame& frame);
  void handleSdoResponseFrame(const CanFrame& frame);
  void handleNmtHeartbeatFrame(const CanFrame& frame);

  bool sendControlWord(uint16_t controlWord);

  BaseCan& m_canBus;
  std::unique_ptr<CanOpenBaseCanLink> m_canLink;
  MotorConfig m_config{};
  MotorStatus m_motorStatus{};
  uint32_t m_lastStatusRequest{0};
  uint32_t m_lastCommunication{0};

  DriveState m_currentState{DriveState::NotReadyToSwitchOn};
  OperationMode m_operationMode{OperationMode::ProfilePosition};

  bool m_initialized{false};
  bool m_communicationReady{false};

  static constexpr uint32_t STATUS_REQUEST_INTERVAL_MS = 100;
  static constexpr uint32_t COMMUNICATION_TIMEOUT_MS = 5000;

  static constexpr uint16_t CONTROL_SWITCH_ON = 0x0001;
  static constexpr uint16_t CONTROL_ENABLE_VOLTAGE = 0x0002;
  static constexpr uint16_t CONTROL_QUICK_STOP = 0x0004;
  static constexpr uint16_t CONTROL_ENABLE_OPERATION = 0x0008;
  static constexpr uint16_t CONTROL_FAULT_RESET = 0x0080;
};
