#include "CANOpenBLDCThread.h"
#include "ConsolePort.h"

static const char* TAG = "CANOpenBLDC";

CANOpenBLDCThread::CANOpenBLDCThread(uint8_t nodeId, uint32_t canBaudRate)
    : BaseThread("CANOpenBLDC")
    , m_canInterface(nullptr)
    , m_config{}
    , m_motorStatus{}
    , m_canBaudRate(canBaudRate)
    , m_lastHeartbeat(0)
    , m_lastStatusRequest(0)
    , m_lastCommunication(0)
    , m_currentState(DriveState::NotReadyToSwitchOn)
    , m_operationMode(OperationMode::PositionMode)
    , m_initialized(false)
    , m_communicationReady(false)
{
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
    
    console_info(TAG, "CANOpenBLDCThread created for node %d", nodeId);
}

CANOpenBLDCThread::~CANOpenBLDCThread()
{
    console_info(TAG, "CANOpenBLDCThread destroyed");
}

bool CANOpenBLDCThread::Initialize()
{
    console_info(TAG, "Initializing CANOpenBLDCThread...");
    
    // Initialize GPIO configuration
    init_mcu_pinconfig();
    
    console_info(TAG, "CANOpenBLDCThread initialized successfully");
    m_initialized = true;
    return true;
}

bool CANOpenBLDCThread::ResetVariables()
{
    // Reset state variables
    m_currentState = DriveState::NotReadyToSwitchOn;
    m_operationMode = OperationMode::PositionMode;
    m_communicationReady = false;
    
    // Reset timing
    m_lastHeartbeat = 0;
    m_lastStatusRequest = 0;
    m_lastCommunication = 0;
    
    // Reset motor status
    m_motorStatus = {};
    m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
    
    console_info(TAG, "CANOpenBLDCThread variables reset");
    return true;
}

bool CANOpenBLDCThread::Setup()
{
    console_info(TAG, "Setting up CANOpen communication...");
    
    // Create FlexCan interface with configured pins
    m_canInterface = std::make_unique<FlexCan>(0, m_canBaudRate, TWAI_TX_PIN, TWAI_RX_PIN);
    
    if (!m_canInterface) {
        console_error(TAG, "Failed to create FlexCan interface");
        return false;
    }
    
    // Initialize CAN interface
    if (!m_canInterface->Open()) {
        console_error(TAG, "Failed to initialize CAN interface");
        return false;
    }
    
    // Setup CANOpen communication
    if (!setupCANOpenCommunication()) {
        console_error(TAG, "Failed to setup CANOpen communication");
        return false;
    }
    
    console_info(TAG, "CANOpen communication setup complete");
    return true;
}

bool CANOpenBLDCThread::setupCANOpenCommunication()
{
    // Send NMT start command to begin operation
    CanFrame nmt_frame;
    nmt_frame.id = 0x000;  // NMT command
    nmt_frame.dlc = 2;
    nmt_frame.extended = false;
    nmt_frame.rtr = false;
    nmt_frame.data[0] = 0x01;  // Start Remote Node
    nmt_frame.data[1] = m_config.nodeId;  // Node ID
    
    if (!m_canInterface->Write(nmt_frame)) {
        console_error(TAG, "Failed to send NMT start command");
        return false;
    }
    
    console_info(TAG, "Sent NMT start command to node %d", m_config.nodeId);
    m_communicationReady = true;
    return true;
}

uint32_t CANOpenBLDCThread::Step()
{
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
    
    return 10;  // 10ms step interval
}

void CANOpenBLDCThread::processIncomingMessages()
{
    CanFrame frame;
    while (m_canInterface && m_canInterface->Read(frame, 0)) {
        processCANOpenMessage(frame);
    }
}

void CANOpenBLDCThread::updateMotorStatus()
{
    uint32_t currentTime = os_get_elapsed_time_msec();
    
    // Check communication timeout
    if (m_communicationReady && 
        (currentTime - m_lastCommunication) > COMMUNICATION_TIMEOUT_MS) {
        console_warn(TAG, "Communication timeout detected");
        m_communicationReady = false;
    }
    
    // Update derived status flags
    m_motorStatus.isEnabled = (m_currentState == DriveState::OperationEnabled);
    m_motorStatus.isInFault = (m_currentState == DriveState::Fault || 
                              m_currentState == DriveState::FaultReactionActive);
}

void CANOpenBLDCThread::handleStateMachine()
{
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
            console_warn(TAG, "Motor in fault state");
            break;
        default:
            break;
    }
}

void CANOpenBLDCThread::handleMotionCommands()
{
    // Motion command handling would go here
    // This is a placeholder for actual motion control logic
}

void CANOpenBLDCThread::sendPeriodicMessages()
{
    uint32_t currentTime = os_get_elapsed_time_msec();
    
    // Send heartbeat
    if ((currentTime - m_lastHeartbeat) >= HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
        m_lastHeartbeat = currentTime;
    }
    
    // Send status request
    if ((currentTime - m_lastStatusRequest) >= STATUS_REQUEST_INTERVAL_MS) {
        sendStatusRequest();
        m_lastStatusRequest = currentTime;
    }
}

void CANOpenBLDCThread::processCANOpenMessage(const CanFrame& frame)
{
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
            processSDOResponse(frame);
            break;
        case 0x0E: // Heartbeat
            processHeartbeat(frame);
            break;
        default:
            console_debug(TAG, "Unknown function code: 0x%02X", function_code);
            break;
    }
}

void CANOpenBLDCThread::sendHeartbeat()
{
    CanFrame heartbeat_frame;
    heartbeat_frame.id = 0x700 + m_config.nodeId;  // Heartbeat COB-ID
    heartbeat_frame.dlc = 1;
    heartbeat_frame.extended = false;
    heartbeat_frame.rtr = false;
    heartbeat_frame.data[0] = static_cast<uint8_t>(m_currentState);  // Node state
    
    if (m_canInterface) {
        m_canInterface->Write(heartbeat_frame);
    }
}

void CANOpenBLDCThread::sendStatusRequest()
{
    CanFrame sdo_frame;
    sdo_frame.id = 0x600 + m_config.nodeId;  // SDO Request COB-ID
    sdo_frame.dlc = 8;
    sdo_frame.extended = false;
    sdo_frame.rtr = false;
    sdo_frame.data[0] = 0x40;  // SDO upload request
    sdo_frame.data[1] = OD_STATUS_WORD & 0xFF;
    sdo_frame.data[2] = (OD_STATUS_WORD >> 8) & 0xFF;
    sdo_frame.data[3] = 0x00;  // Sub-index
    sdo_frame.data[4] = 0x00;
    sdo_frame.data[5] = 0x00;
    sdo_frame.data[6] = 0x00;
    sdo_frame.data[7] = 0x00;
    
    if (m_canInterface) {
        m_canInterface->Write(sdo_frame);
    }
}

void CANOpenBLDCThread::processEmergencyMessage(const CanFrame& frame)
{
    if (frame.dlc >= 8) {
        uint16_t errorCode = frame.data[0] | (frame.data[1] << 8);
        uint8_t errorRegister = frame.data[2];
        
        console_warn(TAG, "Emergency message: Error code 0x%04X, Register 0x%02X", 
                 errorCode, errorRegister);
    }
}

void CANOpenBLDCThread::processPDO1Transmit(const CanFrame& frame)
{
    if (frame.dlc >= 8) {
        // Typically contains status word and position
        m_motorStatus.statusWord = frame.data[0] | (frame.data[1] << 8);
        m_motorStatus.actualPosition = frame.data[2] | (frame.data[3] << 8) |
                                       (frame.data[4] << 16) | (frame.data[5] << 24);
        
        // Update drive state
        m_currentState = decodeStatusWord(m_motorStatus.statusWord);
        m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
        m_motorStatus.nodePresent = true;  // Node is responding
    }
}

void CANOpenBLDCThread::processPDO2Transmit(const CanFrame& frame)
{
    if (frame.dlc >= 8) {
        // Typically contains velocity and torque
        m_motorStatus.actualVelocity = frame.data[0] | (frame.data[1] << 8) |
                                       (frame.data[2] << 16) | (frame.data[3] << 24);
        m_motorStatus.actualTorque = frame.data[4] | (frame.data[5] << 8);
        m_motorStatus.lastUpdateTime = os_get_elapsed_time_msec();
        m_motorStatus.nodePresent = true;  // Node is responding
    }
}

void CANOpenBLDCThread::processSDOResponse(const CanFrame& frame)
{
    if (frame.dlc >= 8) {
        uint8_t command = frame.data[0];
        uint16_t index = frame.data[1] | (frame.data[2] << 8);
        uint8_t subIndex = frame.data[3];
        
        console_debug(TAG, "SDO response: cmd=0x%02X, index=0x%04X, sub=0x%02X",
                 command, index, subIndex);
    }
}

void CANOpenBLDCThread::processHeartbeat(const CanFrame& frame)
{
    if (frame.dlc >= 1) {
        uint8_t state = frame.data[0];
        console_debug(TAG, "Heartbeat received: state=%d", state);
    }
}

CANOpenBLDCThread::DriveState CANOpenBLDCThread::decodeStatusWord(uint16_t statusWord)
{
    uint16_t state = statusWord & STATUS_STATE_MASK;
    
    switch (state) {
        case 0x0000: return DriveState::NotReadyToSwitchOn;
        case 0x0040: return DriveState::SwitchOnDisabled;
        case 0x0021: return DriveState::ReadyToSwitchOn;
        case 0x0023: return DriveState::SwitchedOn;
        case 0x0027: return DriveState::OperationEnabled;
        case 0x0007: return DriveState::QuickStopActive;
        case 0x000F: return DriveState::FaultReactionActive;
        case 0x0008: return DriveState::Fault;
        default: return DriveState::NotReadyToSwitchOn;
    }
}

bool CANOpenBLDCThread::sendControlWord(uint16_t controlWord)
{
    CanFrame sdo_frame;
    sdo_frame.id = 0x600 + m_config.nodeId;  // SDO Request COB-ID
    sdo_frame.dlc = 8;
    sdo_frame.extended = false;
    sdo_frame.rtr = false;
    sdo_frame.data[0] = 0x2B;  // SDO download, 2 bytes
    sdo_frame.data[1] = OD_CONTROL_WORD & 0xFF;
    sdo_frame.data[2] = (OD_CONTROL_WORD >> 8) & 0xFF;
    sdo_frame.data[3] = 0x00;  // Sub-index
    sdo_frame.data[4] = controlWord & 0xFF;
    sdo_frame.data[5] = (controlWord >> 8) & 0xFF;
    sdo_frame.data[6] = 0x00;
    sdo_frame.data[7] = 0x00;
    
    if (m_canInterface) {
        bool result = m_canInterface->Write(sdo_frame);
        if (result) {
            console_debug(TAG, "Sent control word: 0x%04X", controlWord);
        }
        return result;
    }
    
    return false;
}

bool CANOpenBLDCThread::Cleanup()
{
    console_info(TAG, "Cleaning up CANOpenBLDCThread...");
    
    // Stop motor if running
    if (m_canInterface && m_communicationReady) {
        DisableMotor();
    }
    
    // Close CAN interface
    if (m_canInterface) {
        m_canInterface->Close();
        m_canInterface.reset();
    }
    
    console_info(TAG, "CANOpenBLDCThread cleanup complete");
    return true;
}

// Public interface implementations
bool CANOpenBLDCThread::EnableMotor()
{
    if (!m_communicationReady) {
        console_error(TAG, "Communication not ready");
        return false;
    }
    
    // State machine to enable motor
    switch (m_currentState) {
        case DriveState::SwitchOnDisabled:
            return sendControlWord(CONTROL_ENABLE_VOLTAGE);
        case DriveState::ReadyToSwitchOn:
            return sendControlWord(CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE);
        case DriveState::SwitchedOn:
            return sendControlWord(CONTROL_SWITCH_ON | CONTROL_ENABLE_VOLTAGE | CONTROL_ENABLE_OPERATION);
        default:
            console_warn(TAG, "Cannot enable motor in current state: %d", static_cast<int>(m_currentState));
            return false;
    }
}

bool CANOpenBLDCThread::DisableMotor()
{
    if (!m_communicationReady) {
        return false;
    }
    
    return sendControlWord(0x0000);  // Clear all control bits
}

bool CANOpenBLDCThread::ResetFault()
{
    if (!m_communicationReady) {
        return false;
    }
    
    return sendControlWord(CONTROL_FAULT_RESET);
}

bool CANOpenBLDCThread::SetPositionMode(int32_t position, uint32_t velocity, uint32_t acceleration)
{
    // Implementation for position mode would go here
    console_info(TAG, "SetPositionMode: pos=%ld, vel=%lu, acc=%lu", position, velocity, acceleration);
    return true;
}

bool CANOpenBLDCThread::SetVelocityMode(int32_t velocity, uint32_t acceleration)
{
    // Implementation for velocity mode would go here
    console_info(TAG, "SetVelocityMode: vel=%ld, acc=%lu", velocity, acceleration);
    return true;
}

bool CANOpenBLDCThread::SetTorqueMode(int16_t torque)
{
    // Implementation for torque mode would go here
    console_info(TAG, "SetTorqueMode: torque=%d", torque);
    return true;
}

CANOpenBLDCThread::MotorStatus CANOpenBLDCThread::GetMotorStatus() const
{
    return m_motorStatus;
}

bool CANOpenBLDCThread::IsMotorReady() const
{
    return m_communicationReady && (m_currentState == DriveState::OperationEnabled);
}

bool CANOpenBLDCThread::IsMotorEnabled() const
{
    return m_motorStatus.isEnabled;
}

bool CANOpenBLDCThread::IsInFault() const
{
    return m_motorStatus.isInFault;
}

CANOpenBLDCThread::DriveState CANOpenBLDCThread::GetDriveState() const
{
    return m_currentState;
}
