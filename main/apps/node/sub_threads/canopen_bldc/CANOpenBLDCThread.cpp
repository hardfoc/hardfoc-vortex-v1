#include "CANOpenBLDCThread.h"
#include "handlers/logger/Logger.h"

static const char *TAG = "CANOpenBLDC";

CANOpenBLDCThread::CANOpenBLDCThread(uint8_t nodeId, BaseCan& canBus)
    : BaseThread("CANOpenBLDC"), m_canBus(canBus), m_config{}, m_motorStatus{},
      m_lastStatusRequest(0), m_lastCommunication(0),
      m_currentState(DriveState::NotReadyToSwitchOn),
      m_operationMode(OperationMode::ProfilePosition), m_initialized(false),
      m_communicationReady(false) {
  // Initialize configuration with default values
  m_config.nodeId = nodeId;
  m_config.maxVelocity = 1000;
  m_config.maxAcceleration = 1000;
  m_config.profileVelocity = 500;
  m_config.profileAcceleration = 500;
  m_config.profileDeceleration = 500;
  m_config.softwareLimitPos = 100000;
  m_config.softwareLimitNeg = -100000;
  m_config.quickStopDeceleration = 1000;
  m_config.shutdownOption = 0;
  m_config.disableOption = 1;
  m_config.faultReaction = 2;

  Logger::GetInstance().Info(TAG, "CANOpenBLDCThread created for node %d", nodeId);
}

CANOpenBLDCThread::~CANOpenBLDCThread() {
  Logger::GetInstance().Info(TAG, "CANOpenBLDCThread destroyed");
}

bool CANOpenBLDCThread::Initialize() {
  Logger::GetInstance().Info(TAG, "Initializing CANOpenBLDCThread...");

  Logger::GetInstance().Info(TAG, "CANOpenBLDCThread initialized successfully");
  m_initialized = true;
  return true;
}

bool CANOpenBLDCThread::ResetVariables() {
  // Reset state variables
  m_currentState = DriveState::NotReadyToSwitchOn;
  m_operationMode = OperationMode::ProfilePosition;
  m_communicationReady = false;

  // Reset timing
  m_lastStatusRequest = 0;
  m_lastCommunication = 0;

  // Reset motor status
  m_motorStatus = {};
  m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();

  Logger::GetInstance().Info(TAG, "CANOpenBLDCThread variables reset");
  return true;
}

bool CANOpenBLDCThread::Setup() {
  Logger::GetInstance().Info(TAG, "Setting up CANOpen communication...");

  m_canLink = std::make_unique<CanOpenBaseCanLink>(m_canBus);
  if (!m_canLink->Open()) {
    Logger::GetInstance().Error(TAG, "Failed to initialize CAN (EnsureInitialized)");
    m_canLink.reset();
    return false;
  }

  // Setup CANOpen communication
  if (!setupCANOpenCommunication()) {
    Logger::GetInstance().Error(TAG, "Failed to setup CANOpen communication");
    return false;
  }

  Logger::GetInstance().Info(TAG, "CANOpen communication setup complete");
  return true;
}

bool CANOpenBLDCThread::setupCANOpenCommunication() {
  const CanFrame nmt_frame =
      CanOpen::BuildNmt(m_config.nodeId, CanOpen::NmtCommand::StartNode);

  if (!m_canLink->Write(nmt_frame)) {
    Logger::GetInstance().Error(TAG, "Failed to send NMT start command");
    return false;
  }

  Logger::GetInstance().Info(TAG, "Sent NMT start command to node %d", m_config.nodeId);
  m_communicationReady = true;
  return true;
}

uint32_t CANOpenBLDCThread::Step() {
  // Process incoming CAN messages
  processIncomingMessages();

  // Handle motor state machine
  handleStateMachine();

  // Update motor status
  updateMotorStatus();

  // Send periodic messages
  sendPeriodicMessages();

  // Handle motion commands
  handleMotionCommands();

  return 10; // 10ms step interval
}

void CANOpenBLDCThread::processIncomingMessages() {
  CanFrame frame;
  while (m_canLink && m_canLink->Read(frame, 0)) {
    processCANOpenMessage(frame);
  }
}

void CANOpenBLDCThread::updateMotorStatus() {
  uint32_t currentTime = os_get_elapsed_time_msec();

  // Check communication timeout
  if (m_communicationReady &&
      (currentTime - m_lastCommunication) > COMMUNICATION_TIMEOUT_MS) {
    Logger::GetInstance().Warn(TAG, "Communication timeout detected");
    m_communicationReady = false;
  }

  // Update derived status flags
  m_motorStatus.isEnabled = (m_currentState == DriveState::OperationEnabled);
  m_motorStatus.isInFault = (m_currentState == DriveState::Fault ||
                             m_currentState == DriveState::FaultReactionActive);
}

void CANOpenBLDCThread::handleStateMachine() {
  // Basic state machine handling
  switch (m_currentState) {
  case DriveState::NotReadyToSwitchOn:
    // Wait for motor to become ready
    break;
  case DriveState::SwitchOnDisabled:
    // Motor is disabled, can enable voltage
    break;
  case DriveState::ReadyToSwitchOn:
    // Ready to switch on
    break;
  case DriveState::SwitchedOn:
    // Switched on, can enable operation
    break;
  case DriveState::OperationEnabled:
    // Operation enabled, normal operation
    break;
  case DriveState::Fault:
    // Fault state, need fault reset
    Logger::GetInstance().Warn(TAG, "Motor in fault state");
    break;
  default:
    break;
  }
}

void CANOpenBLDCThread::handleMotionCommands() {
  // Motion command handling would go here
  // This is a placeholder for actual motion control logic
}

void CANOpenBLDCThread::sendPeriodicMessages() {
  uint32_t currentTime = os_get_elapsed_time_msec();

  // Send status request
  if ((currentTime - m_lastStatusRequest) >= STATUS_REQUEST_INTERVAL_MS) {
    sendStatusRequest();
    m_lastStatusRequest = currentTime;
  }
}

void CANOpenBLDCThread::processCANOpenMessage(const CanFrame &frame) {
  uint16_t cob_id = frame.id & 0x7FF;
  uint8_t function_code = (cob_id >> 7) & 0x0F;
  uint8_t node_id = cob_id & 0x7F;

  // Only process messages from our node
  if (node_id != m_config.nodeId) {
    return;
  }

  // Update communication timestamp
  m_lastCommunication = os_get_elapsed_time_msec();

  // Process based on function code
  switch (function_code) {
  case 0x01: // Emergency
    processEmergencyMessage(frame);
    break;
  case 0x03: // PDO1 Transmit
    processPDO1Transmit(frame);
    break;
  case 0x05: // PDO2 Transmit
    processPDO2Transmit(frame);
    break;
  case 0x0B: // SDO Response
    handleSdoResponseFrame(frame);
    break;
  case 0x0E: // Heartbeat
    handleNmtHeartbeatFrame(frame);
    break;
  default:
    Logger::GetInstance().Debug(TAG, "Unknown function code: 0x%02X", function_code);
    break;
  }
}

void CANOpenBLDCThread::sendStatusRequest() {
  const CanFrame sdo_frame = CanOpen::BuildSdoUpload(
      m_config.nodeId, hf::vortex::kIdxStatusword, hf::vortex::kSubDefault);

  if (m_canLink) {
    (void)m_canLink->Write(sdo_frame);
  }
}

void CANOpenBLDCThread::processEmergencyMessage(const CanFrame &frame) {
  const CanOpen::EmergencyMessage em = CanOpen::ParseEmergency(frame);
  if (!em.valid) {
    return;
  }
  Logger::GetInstance().Warn(TAG, "EMCY node=%u code=0x%04X reg=0x%02X", static_cast<unsigned>(em.nodeId),
                             static_cast<unsigned>(em.error_code), static_cast<unsigned>(em.error_register));
}

void CANOpenBLDCThread::processPDO1Transmit(const CanFrame &frame) {
  if (frame.dlc >= 8) {
    // Typically contains status word and position
    m_motorStatus.statusWord = frame.data[0] | (frame.data[1] << 8);
    m_motorStatus.actualPosition = frame.data[2] | (frame.data[3] << 8) |
                                   (frame.data[4] << 16) |
                                   (frame.data[5] << 24);

    // Update drive state
    m_currentState = CanOpen::DecodeStatusWord(m_motorStatus.statusWord);
    m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
    m_motorStatus.nodePresent = true; // Node is responding
  }
}

void CANOpenBLDCThread::processPDO2Transmit(const CanFrame &frame) {
  if (frame.dlc >= 8) {
    // Typically contains velocity and torque
    m_motorStatus.actualVelocity = frame.data[0] | (frame.data[1] << 8) |
                                   (frame.data[2] << 16) |
                                   (frame.data[3] << 24);
    m_motorStatus.actualTorque = frame.data[4] | (frame.data[5] << 8);
    m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
    m_motorStatus.nodePresent = true; // Node is responding
  }
}

void CANOpenBLDCThread::handleSdoResponseFrame(const CanFrame &frame) {
  const CanOpen::ExpeditedSdoResponse r = CanOpen::ParseSdoResponse(frame);
  if (!r.valid) {
    Logger::GetInstance().Debug(TAG, "SDO RX: non-expedited or parse fail");
    return;
  }
  Logger::GetInstance().Debug(TAG, "SDO RX kind=%u idx=0x%04X sub=%u", static_cast<unsigned>(r.kind),
                              static_cast<unsigned>(r.index), static_cast<unsigned>(r.subIndex));
  if (r.kind == CanOpen::SdoResponseKind::UploadResponse && r.index == hf::vortex::kIdxStatusword &&
      r.subIndex == hf::vortex::kSubDefault && r.data_len >= 2) {
    const uint16_t sw = static_cast<uint16_t>(r.data & 0xFFFFU);
    m_motorStatus.statusWord = sw;
    m_currentState = CanOpen::DecodeStatusWord(sw);
    m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
    m_motorStatus.nodePresent = true;
  }
}

void CANOpenBLDCThread::handleNmtHeartbeatFrame(const CanFrame &frame) {
  const CanOpen::HeartbeatMessage hb = CanOpen::ParseHeartbeat(frame);
  if (!hb.valid) {
    return;
  }
  Logger::GetInstance().Debug(TAG, "HB node=%u state=%u", static_cast<unsigned>(hb.nodeId),
                              static_cast<unsigned>(static_cast<uint8_t>(hb.state)));
}

bool CANOpenBLDCThread::sendControlWord(uint16_t controlWord) {
  const CanFrame sdo_frame = CanOpen::BuildSdoDownload(
      m_config.nodeId, hf::vortex::kIdxControlword, hf::vortex::kSubDefault,
      static_cast<uint32_t>(controlWord), 2);

  if (m_canLink) {
    bool result = m_canLink->Write(sdo_frame);
    if (result) {
      Logger::GetInstance().Debug(TAG, "Sent control word: 0x%04X", controlWord);
    }
    return result;
  }

  return false;
}

bool CANOpenBLDCThread::Cleanup() {
  Logger::GetInstance().Info(TAG, "Cleaning up CANOpenBLDCThread...");

  // Stop motor if running
  if (m_canLink && m_communicationReady) {
    DisableMotor();
  }

  if (m_canLink) {
    m_canLink->Close();
    m_canLink.reset();
  }

  Logger::GetInstance().Info(TAG, "CANOpenBLDCThread cleanup complete");
  return true;
}

// Public interface implementations
bool CANOpenBLDCThread::EnableMotor() {
  if (!m_communicationReady) {
    Logger::GetInstance().Error(TAG, "Communication not ready");
    return false;
  }

  // State machine to enable motor
  switch (m_currentState) {
  case DriveState::SwitchOnDisabled:
    return sendControlWord(CONTROL_ENABLE_VOLTAGE);
  case DriveState::ReadyToSwitchOn:
    return sendControlWord(CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE);
  case DriveState::SwitchedOn:
    return sendControlWord(CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE |
                           CONTROL_ENABLE_OPERATION);
  default:
    Logger::GetInstance().Warn(TAG, "Cannot enable motor in state: %s",
                               CanOpen::DriveStateName(m_currentState));
    return false;
  }
}

bool CANOpenBLDCThread::DisableMotor() {
  if (!m_communicationReady) {
    return false;
  }

  return sendControlWord(0x0000); // Clear all control bits
}

bool CANOpenBLDCThread::ResetFault() {
  if (!m_communicationReady) {
    return false;
  }

  return sendControlWord(CONTROL_FAULT_RESET);
}

bool CANOpenBLDCThread::SetPositionMode(int32_t position, uint32_t velocity,
                                        uint32_t acceleration) {
  // Implementation for position mode would go here
  Logger::GetInstance().Info(TAG, "SetPositionMode: pos=%ld, vel=%lu, acc=%lu", position,
               velocity, acceleration);
  return true;
}

bool CANOpenBLDCThread::SetVelocityMode(int32_t velocity,
                                        uint32_t acceleration) {
  // Implementation for velocity mode would go here
  Logger::GetInstance().Info(TAG, "SetVelocityMode: vel=%ld, acc=%lu", velocity,
               acceleration);
  return true;
}

bool CANOpenBLDCThread::SetTorqueMode(int16_t torque) {
  // Implementation for torque mode would go here
  Logger::GetInstance().Info(TAG, "SetTorqueMode: torque=%d", torque);
  return true;
}

bool CANOpenBLDCThread::ConfigureMotor(const MotorConfig& config) {
  m_config = config;
  return true;
}

bool CANOpenBLDCThread::PerformHoming() {
  Logger::GetInstance().Warn(TAG, "PerformHoming: not implemented (wire 0x6098/0x6099/0x609A + mode 6)");
  return false;
}

bool CANOpenBLDCThread::SetProfileParameters(uint32_t velocity, uint32_t acceleration,
                                             uint32_t deceleration) {
  if (!m_communicationReady || !m_canLink) {
    return false;
  }
  const bool a =
      m_canLink->Write(CanOpen::BuildSdoDownload(m_config.nodeId, hf::vortex::kIdxProfileVelocity,
                                                 hf::vortex::kSubDefault, velocity, 4));
  const bool b =
      m_canLink->Write(CanOpen::BuildSdoDownload(m_config.nodeId, hf::vortex::kIdxProfileAcceleration,
                                                 hf::vortex::kSubDefault, acceleration, 4));
  const bool c =
      m_canLink->Write(CanOpen::BuildSdoDownload(m_config.nodeId, hf::vortex::kIdxProfileDeceleration,
                                                 hf::vortex::kSubDefault, deceleration, 4));
  return a && b && c;
}

CANOpenBLDCThread::MotorStatus CANOpenBLDCThread::GetMotorStatus() const {
  return m_motorStatus;
}

bool CANOpenBLDCThread::IsMotorReady() const {
  return m_communicationReady &&
         (m_currentState == DriveState::OperationEnabled);
}

bool CANOpenBLDCThread::IsMotorEnabled() const {
  return m_motorStatus.isEnabled;
}

bool CANOpenBLDCThread::IsInFault() const { return m_motorStatus.isInFault; }

CANOpenBLDCThread::DriveState CANOpenBLDCThread::GetDriveState() const {
  return m_currentState;
}
