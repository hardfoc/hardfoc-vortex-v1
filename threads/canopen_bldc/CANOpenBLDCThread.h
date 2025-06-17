#pragma once

#include "BaseThread.h"
#include "FlexCan.h"
#include "CanOpenMotorUtils.h"
#include "CanOpenUtils.h"
#include "hf_gpio_config.hpp"

/**
 * @brief Thread for controlling a BLDC motor via CANOpen protocol
 * 
 * This thread manages CANOpen communication with a BLDC motor controller,
 * handling initialization, state management, and motion control commands.
 */
class CANOpenBLDCThread : public BaseThread {
public:
    /**
     * @brief Motor configuration structure
     */
    struct MotorConfig {
        uint8_t nodeId;                 ///< CANOpen node ID of the motor controller
        uint32_t maxVelocity;           ///< Maximum velocity in counts/second
        uint32_t maxAcceleration;       ///< Maximum acceleration in counts/second²
        uint32_t profileVelocity;       ///< Profile velocity for position mode
        uint32_t profileAcceleration;   ///< Profile acceleration for position mode
        uint32_t profileDeceleration;   ///< Profile deceleration for position mode
        int32_t softwareLimitPos;       ///< Positive software limit
        int32_t softwareLimitNeg;       ///< Negative software limit
        uint16_t quickStopDeceleration; ///< Quick stop deceleration
        uint16_t shutdownOption;        ///< Shutdown option code
        uint16_t disableOption;         ///< Disable option code
        uint16_t faultReaction;         ///< Fault reaction option code
    };

    /**
     * @brief Motor status structure
     */
    struct MotorStatus {
        uint16_t statusWord;            ///< CANOpen status word
        int32_t actualPosition;         ///< Current position in counts
        int32_t actualVelocity;         ///< Current velocity in counts/second
        int16_t actualTorque;           ///< Current torque in mNm
        uint32_t lastUpdateTime;        ///< Last status update timestamp
        bool isEnabled;                 ///< Motor enabled state
        bool isInFault;                 ///< Fault state
        bool isHomed;                   ///< Homing complete state
        bool nodePresent;               ///< Node detected on network
    };

    /**
     * @brief CANOpen drive states
     */
    enum class DriveState : uint8_t {
        NotReadyToSwitchOn = 0,
        SwitchOnDisabled = 1,
        ReadyToSwitchOn = 2,
        SwitchedOn = 3,
        OperationEnabled = 4,
        QuickStopActive = 5,
        FaultReactionActive = 6,
        Fault = 7
    };

    /**
     * @brief Operation modes
     */
    enum class OperationMode : int8_t {
        PositionMode = 1,
        VelocityMode = 3,
        TorqueMode = 4,
        HomingMode = 6,
        InterpolatedPositionMode = 7,
        CyclicSynchronousPosition = 8,
        CyclicSynchronousVelocity = 9,
        CyclicSynchronousTorque = 10
    };

    /**
     * @brief Constructor
     * @param nodeId CANOpen node ID of the motor controller
     * @param canBaudRate CAN bus baud rate
     */
    CANOpenBLDCThread(uint8_t nodeId = 1, uint32_t canBaudRate = 500000);

    /**
     * @brief Destructor
     */
    virtual ~CANOpenBLDCThread();

    // Public interface methods
    bool EnableMotor();
    bool DisableMotor();
    bool ResetFault();
    bool PerformHoming();
    
    // Motion control methods
    bool SetPositionMode(int32_t position, uint32_t velocity = 0, uint32_t acceleration = 0);
    bool SetVelocityMode(int32_t velocity, uint32_t acceleration = 0);
    bool SetTorqueMode(int16_t torque);
    
    // Configuration methods
    bool ConfigureMotor(const MotorConfig& config);
    bool SetProfileParameters(uint32_t velocity, uint32_t acceleration, uint32_t deceleration);
    
    // Status and monitoring
    MotorStatus GetMotorStatus() const;
    bool IsMotorReady() const;
    bool IsMotorEnabled() const;
    bool IsInFault() const;
    DriveState GetDriveState() const;

protected:
    /**
     * @brief Initialize the thread and CAN interface
     * @return true if successful
     */
    virtual bool Initialize() noexcept override;

    /**
     * @brief Setup CAN interface and motor communication
     * @return true if successful
     */
    virtual bool Setup() noexcept override;

    /**
     * @brief Main thread step - handles CAN communication and motor control
     * @return delay in milliseconds before next step
     */
    virtual uint32_t Step() noexcept override;

    /**
     * @brief Cleanup when thread stops
     * @return true if cleanup successful
     */
    virtual bool Cleanup() noexcept override;

    /**
     * @brief Reset variables to initial state
     * @return true if reset successful
     */
    virtual bool ResetVariables() noexcept override;

private:
    // Type aliases for convenience
    using CanFrame = FlexCan::Frame;
    
    // CAN communication methods
    bool sendCanFrame(const CanFrame& frame, uint32_t timeoutMs = 100);
    bool receiveCanFrame(CanFrame& frame, uint32_t timeoutMs = 0);
    void processCanMessages();
    void sendPeriodicRequests();
    
    // Setup helper
    bool setupCANOpenCommunication() noexcept;
    
    // Step helper methods
    void processIncomingMessages();
    void updateMotorStatus();
    void handleStateMachine();
    void sendPeriodicMessages();
    void handleMotionCommands();
    void processCANOpenMessage(const CanFrame& frame);
    void sendHeartbeat();
    void sendStatusRequest();
    
    // Message processing methods
    void processEmergencyMessage(const CanFrame& frame);
    void processPDO1Transmit(const CanFrame& frame);
    void processPDO2Transmit(const CanFrame& frame);
    void processSDOResponse(const CanFrame& frame);
    void processHeartbeat(const CanFrame& frame);
    
    // Command sending methods
    void sendPositionCommand(int32_t position);
    void sendVelocityCommand(int32_t velocity);
    void sendTorqueCommand(int16_t torque);
    
    // Status decoding
    DriveState decodeStatusWord(uint16_t statusWord);
    
    // Control word sending
    bool sendControlWord(uint16_t controlWord);
    
    // Motor state management
    bool executeStateTransition(DriveState targetState);
    bool waitForStateTransition(DriveState targetState, uint32_t timeoutMs = 5000);
    
    // SDO communication helpers
    bool sendSDODownload(uint16_t index, uint8_t subIndex, uint32_t data, uint8_t size);
    bool sendSDOUpload(uint16_t index, uint8_t subIndex);
    bool processSDOResponse(uint16_t index, uint8_t subIndex, uint32_t* data);
    
    // Command building helpers
    CanFrame buildSDOFrame(uint8_t command, uint16_t index, uint8_t subIndex, uint32_t data = 0);
    CanFrame buildPDOFrame(uint16_t cobId, const uint8_t* data, uint8_t length);
    
    // Member variables
    std::unique_ptr<FlexCan> m_canInterface;     ///< CAN interface
    MotorConfig m_config;                        ///< Motor configuration
    MotorStatus m_motorStatus;                   ///< Current motor status
    
    uint32_t m_canBaudRate;                      ///< CAN bus baud rate
    uint32_t m_lastHeartbeat;                    ///< Last heartbeat timestamp
    uint32_t m_lastStatusRequest;                ///< Last status request timestamp
    uint32_t m_lastCommunication;                ///< Last communication timestamp
    
    DriveState m_currentState;                   ///< Current drive state
    OperationMode m_operationMode;               ///< Current operation mode
    
    bool m_initialized;                          ///< Initialization state
    bool m_communicationReady;                   ///< Communication ready flag
    
    // Communication timing
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;     ///< Heartbeat interval
    static constexpr uint32_t STATUS_REQUEST_INTERVAL_MS = 100; ///< Status request interval
    static constexpr uint32_t COMMUNICATION_TIMEOUT_MS = 5000;  ///< Communication timeout
    
    // CANOpen object dictionary indices
    static constexpr uint16_t OD_DEVICE_TYPE = 0x1000;
    static constexpr uint16_t OD_ERROR_REGISTER = 0x1001;
    static constexpr uint16_t OD_MANUFACTURER_STATUS = 0x1002;
    static constexpr uint16_t OD_IDENTITY_VENDOR_ID = 0x1018;
    static constexpr uint16_t OD_CONTROL_WORD = 0x6040;
    static constexpr uint16_t OD_STATUS_WORD = 0x6041;
    static constexpr uint16_t OD_OPERATION_MODE = 0x6060;
    static constexpr uint16_t OD_OPERATION_MODE_DISPLAY = 0x6061;
    static constexpr uint16_t OD_POSITION_ACTUAL = 0x6064;
    static constexpr uint16_t OD_VELOCITY_ACTUAL = 0x606C;
    static constexpr uint16_t OD_TORQUE_ACTUAL = 0x6077;
    static constexpr uint16_t OD_TARGET_POSITION = 0x607A;
    static constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;
    static constexpr uint16_t OD_TARGET_TORQUE = 0x6071;
    
    // Control word bits
    static constexpr uint16_t CONTROL_SWITCH_ON = 0x0001;
    static constexpr uint16_t CONTROL_ENABLE_VOLTAGE = 0x0002;
    static constexpr uint16_t CONTROL_QUICK_STOP = 0x0004;
    static constexpr uint16_t CONTROL_ENABLE_OPERATION = 0x0008;
    static constexpr uint16_t CONTROL_FAULT_RESET = 0x0080;
    
    // Status word bits and masks
    static constexpr uint16_t STATUS_READY_TO_SWITCH_ON = 0x0001;
    static constexpr uint16_t STATUS_SWITCHED_ON = 0x0002;
    static constexpr uint16_t STATUS_OPERATION_ENABLED = 0x0004;
    static constexpr uint16_t STATUS_FAULT = 0x0008;
    static constexpr uint16_t STATUS_VOLTAGE_ENABLED = 0x0010;
    static constexpr uint16_t STATUS_QUICK_STOP = 0x0020;
    static constexpr uint16_t STATUS_SWITCH_ON_DISABLED = 0x0040;
    static constexpr uint16_t STATUS_WARNING = 0x0080;
    static constexpr uint16_t STATUS_REMOTE = 0x0200;
    static constexpr uint16_t STATUS_TARGET_REACHED = 0x0400;
    static constexpr uint16_t STATUS_INTERNAL_LIMIT = 0x0800;
    
    static constexpr uint16_t STATUS_STATE_MASK = 0x006F;
};
