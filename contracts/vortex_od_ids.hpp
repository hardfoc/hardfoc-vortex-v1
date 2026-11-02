#pragma once
/**
 * @file vortex_od_ids.hpp
 * @brief Object dictionary identifiers and tables.
 *
 * Source: od/vortex_od.yaml, revision 1.0, OD hash 0xFF85661C.
 * Do not edit: change the YAML and run `python3 tools/odgen`.
 */
#include <cstddef>
#include <cstdint>

#include "contracts/od_store_codec.hpp"

namespace vortex::od {

namespace odstore = ::vortex::contracts::odstore;

inline constexpr uint32_t kOdHash = 0xFF85661CU;  ///< also 0x5FFE
inline constexpr uint8_t kRevisionMajor = 1U;
inline constexpr uint8_t kRevisionMinor = 0U;
inline constexpr uint32_t kRevisionNumber = 0x00010000U;  ///< 0x1018:03
inline constexpr uint16_t kOdVersion = 0x0100U;  ///< 0x5FFF

/** @brief One OD address. */
struct Address {
  uint16_t index;
  uint8_t sub;
};
constexpr bool operator==(Address a, Address b) noexcept { return a.index == b.index && a.sub == b.sub; }

/** @brief Object indices. */
namespace idx {
inline constexpr uint16_t kDeviceType = 0x1000U;  ///< Device type
inline constexpr uint16_t kErrorRegister = 0x1001U;  ///< Error register
inline constexpr uint16_t kPreDefinedErrorField = 0x1003U;  ///< Pre-defined error field
inline constexpr uint16_t kCobIdSync = 0x1005U;  ///< COB-ID SYNC message
inline constexpr uint16_t kCommunicationCyclePeriod = 0x1006U;  ///< Communication cycle period
inline constexpr uint16_t kSynchronousWindowLength = 0x1007U;  ///< Synchronous window length
inline constexpr uint16_t kManufacturerDeviceName = 0x1008U;  ///< Manufacturer device name
inline constexpr uint16_t kManufacturerHardwareVersion = 0x1009U;  ///< Manufacturer hardware version
inline constexpr uint16_t kManufacturerSoftwareVersion = 0x100AU;  ///< Manufacturer software version
inline constexpr uint16_t kStoreParameters = 0x1010U;  ///< Store parameters
inline constexpr uint16_t kRestoreDefaultParameters = 0x1011U;  ///< Restore default parameters
inline constexpr uint16_t kCobIdTime = 0x1012U;  ///< COB-ID time stamp object
inline constexpr uint16_t kCobIdEmcy = 0x1014U;  ///< COB-ID EMCY
inline constexpr uint16_t kInhibitTimeEmcy = 0x1015U;  ///< Inhibit time EMCY
inline constexpr uint16_t kConsumerHeartbeatTime = 0x1016U;  ///< Consumer heartbeat time
inline constexpr uint16_t kProducerHeartbeatTime = 0x1017U;  ///< Producer heartbeat time
inline constexpr uint16_t kIdentity = 0x1018U;  ///< Identity
inline constexpr uint16_t kSynchronousCounterOverflowValue = 0x1019U;  ///< Synchronous counter overflow value
inline constexpr uint16_t kErrorBehavior = 0x1029U;  ///< Error behavior
inline constexpr uint16_t kSdoServer1 = 0x1200U;  ///< SDO server parameter
inline constexpr uint16_t kSdoServer2 = 0x1201U;  ///< SDO server parameter 2
inline constexpr uint16_t kSdoClient1 = 0x1280U;  ///< SDO client parameter
inline constexpr uint16_t kRpdo1Communication = 0x1400U;  ///< RPDO communication parameter
inline constexpr uint16_t kRpdo2Communication = 0x1401U;  ///< RPDO communication parameter
inline constexpr uint16_t kRpdo3Communication = 0x1402U;  ///< RPDO communication parameter
inline constexpr uint16_t kRpdo4Communication = 0x1403U;  ///< RPDO communication parameter
inline constexpr uint16_t kRpdo1Mapping = 0x1600U;  ///< RPDO mapping parameter
inline constexpr uint16_t kRpdo2Mapping = 0x1601U;  ///< RPDO mapping parameter
inline constexpr uint16_t kRpdo3Mapping = 0x1602U;  ///< RPDO mapping parameter
inline constexpr uint16_t kRpdo4Mapping = 0x1603U;  ///< RPDO mapping parameter
inline constexpr uint16_t kTpdo1Communication = 0x1800U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo2Communication = 0x1801U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo3Communication = 0x1802U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo4Communication = 0x1803U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo5Communication = 0x1804U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo6Communication = 0x1805U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo7Communication = 0x1806U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo8Communication = 0x1807U;  ///< TPDO communication parameter
inline constexpr uint16_t kTpdo1Mapping = 0x1A00U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo2Mapping = 0x1A01U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo3Mapping = 0x1A02U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo4Mapping = 0x1A03U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo5Mapping = 0x1A04U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo6Mapping = 0x1A05U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo7Mapping = 0x1A06U;  ///< TPDO mapping parameter
inline constexpr uint16_t kTpdo8Mapping = 0x1A07U;  ///< TPDO mapping parameter
inline constexpr uint16_t kNmtStartup = 0x1F80U;  ///< NMT startup
inline constexpr uint16_t kFirmwareBuild = 0x2000U;  ///< Firmware build
inline constexpr uint16_t kBoardInfo = 0x2001U;  ///< Board info
inline constexpr uint16_t kComponentVersions = 0x2002U;  ///< Component versions
inline constexpr uint16_t kEncoderStatus = 0x2101U;  ///< Encoder status
inline constexpr uint16_t kImuPackedQuaternion = 0x2210U;  ///< IMU packed quaternion
inline constexpr uint16_t kImuPackedAccel = 0x2211U;  ///< IMU packed acceleration
inline constexpr uint16_t kImuPackedGyro = 0x2212U;  ///< IMU packed gyro
inline constexpr uint16_t kTemperatures = 0x2300U;  ///< Temperatures
inline constexpr uint16_t kPower = 0x2301U;  ///< Power
inline constexpr uint16_t kControlAuthority = 0x2700U;  ///< Control authority
inline constexpr uint16_t kAuthorityPolicy = 0x2701U;  ///< Authority policy
inline constexpr uint16_t kParameterStoreMeta = 0x2800U;  ///< Parameter store meta
inline constexpr uint16_t kFaultJournalHeader = 0x4000U;  ///< Fault journal header
inline constexpr uint16_t kFaultJournal = 0x4001U;  ///< Fault journal
inline constexpr uint16_t kFaultJournalClear = 0x4002U;  ///< Fault journal clear
inline constexpr uint16_t kActiveCauses = 0x4003U;  ///< Active causes
inline constexpr uint16_t kThreadStats = 0x4200U;  ///< Thread stats
inline constexpr uint16_t kTimingHistogram = 0x4201U;  ///< Timing histogram
inline constexpr uint16_t kCanStats = 0x4202U;  ///< CAN stats
inline constexpr uint16_t kOdLayoutHash = 0x5FFEU;  ///< OD layout hash
inline constexpr uint16_t kManufacturerOdVersion = 0x5FFFU;  ///< Manufacturer OD version
inline constexpr uint16_t kAbortConnectionOptionCode = 0x6007U;  ///< Abort connection option code
inline constexpr uint16_t kErrorCode = 0x603FU;  ///< Error code
inline constexpr uint16_t kControlword = 0x6040U;  ///< Controlword
inline constexpr uint16_t kStatusword = 0x6041U;  ///< Statusword
inline constexpr uint16_t kQuickStopOptionCode = 0x605AU;  ///< Quick stop option code
inline constexpr uint16_t kShutdownOptionCode = 0x605BU;  ///< Shutdown option code
inline constexpr uint16_t kDisableOperationOptionCode = 0x605CU;  ///< Disable operation option code
inline constexpr uint16_t kHaltOptionCode = 0x605DU;  ///< Halt option code
inline constexpr uint16_t kFaultReactionOptionCode = 0x605EU;  ///< Fault reaction option code
inline constexpr uint16_t kModesOfOperation = 0x6060U;  ///< Modes of operation
inline constexpr uint16_t kModesOfOperationDisplay = 0x6061U;  ///< Modes of operation display
inline constexpr uint16_t kPositionDemandValue = 0x6062U;  ///< Position demand value
inline constexpr uint16_t kPositionActualValue = 0x6064U;  ///< Position actual value
inline constexpr uint16_t kFollowingErrorWindow = 0x6065U;  ///< Following error window
inline constexpr uint16_t kFollowingErrorTimeOut = 0x6066U;  ///< Following error time out
inline constexpr uint16_t kPositionWindow = 0x6067U;  ///< Position window
inline constexpr uint16_t kPositionWindowTime = 0x6068U;  ///< Position window time
inline constexpr uint16_t kVelocityDemandValue = 0x606BU;  ///< Velocity demand value
inline constexpr uint16_t kVelocityActualValue = 0x606CU;  ///< Velocity actual value
inline constexpr uint16_t kVelocityWindow = 0x606DU;  ///< Velocity window
inline constexpr uint16_t kVelocityWindowTime = 0x606EU;  ///< Velocity window time
inline constexpr uint16_t kTargetTorque = 0x6071U;  ///< Target torque
inline constexpr uint16_t kMaxTorque = 0x6072U;  ///< Max torque
inline constexpr uint16_t kMaxCurrent = 0x6073U;  ///< Max current
inline constexpr uint16_t kMotorRatedCurrent = 0x6075U;  ///< Motor rated current
inline constexpr uint16_t kMotorRatedTorque = 0x6076U;  ///< Motor rated torque
inline constexpr uint16_t kTorqueActualValue = 0x6077U;  ///< Torque actual value
inline constexpr uint16_t kCurrentActualValue = 0x6078U;  ///< Current actual value
inline constexpr uint16_t kDcLinkCircuitVoltage = 0x6079U;  ///< DC link circuit voltage
inline constexpr uint16_t kTargetPosition = 0x607AU;  ///< Target position
inline constexpr uint16_t kHomeOffset = 0x607CU;  ///< Home offset
inline constexpr uint16_t kSoftwarePositionLimit = 0x607DU;  ///< Software position limit
inline constexpr uint16_t kPolarity = 0x607EU;  ///< Polarity
inline constexpr uint16_t kMaxProfileVelocity = 0x607FU;  ///< Max profile velocity
inline constexpr uint16_t kMaxMotorSpeed = 0x6080U;  ///< Max motor speed
inline constexpr uint16_t kProfileVelocity = 0x6081U;  ///< Profile velocity
inline constexpr uint16_t kProfileAcceleration = 0x6083U;  ///< Profile acceleration
inline constexpr uint16_t kProfileDeceleration = 0x6084U;  ///< Profile deceleration
inline constexpr uint16_t kQuickStopDeceleration = 0x6085U;  ///< Quick stop deceleration
inline constexpr uint16_t kMotionProfileType = 0x6086U;  ///< Motion profile type
inline constexpr uint16_t kTorqueSlope = 0x6087U;  ///< Torque slope
inline constexpr uint16_t kPositionEncoderResolution = 0x608FU;  ///< Position encoder resolution
inline constexpr uint16_t kGearRatio = 0x6091U;  ///< Gear ratio
inline constexpr uint16_t kFeedConstant = 0x6092U;  ///< Feed constant
inline constexpr uint16_t kHomingMethod = 0x6098U;  ///< Homing method
inline constexpr uint16_t kHomingSpeeds = 0x6099U;  ///< Homing speeds
inline constexpr uint16_t kHomingAcceleration = 0x609AU;  ///< Homing acceleration
inline constexpr uint16_t kPositionOffset = 0x60B0U;  ///< Position offset
inline constexpr uint16_t kVelocityOffset = 0x60B1U;  ///< Velocity offset
inline constexpr uint16_t kTorqueOffset = 0x60B2U;  ///< Torque offset
inline constexpr uint16_t kInterpolationTimePeriod = 0x60C2U;  ///< Interpolation time period
inline constexpr uint16_t kPositiveTorqueLimitValue = 0x60E0U;  ///< Positive torque limit value
inline constexpr uint16_t kNegativeTorqueLimitValue = 0x60E1U;  ///< Negative torque limit value
inline constexpr uint16_t kAdditionalPositionActualValue = 0x60E4U;  ///< Additional position actual value
inline constexpr uint16_t kFollowingErrorActualValue = 0x60F4U;  ///< Following error actual value
inline constexpr uint16_t kDigitalInputs = 0x60FDU;  ///< Digital inputs
inline constexpr uint16_t kDigitalOutputs = 0x60FEU;  ///< Digital outputs
inline constexpr uint16_t kTargetVelocity = 0x60FFU;  ///< Target velocity
inline constexpr uint16_t kMotorType = 0x6402U;  ///< Motor type
inline constexpr uint16_t kMotorCatalogueNumber = 0x6403U;  ///< Motor catalogue number
inline constexpr uint16_t kMotorManufacturer = 0x6404U;  ///< Motor manufacturer
inline constexpr uint16_t kSupportedDriveModes = 0x6502U;  ///< Supported drive modes
}  // namespace idx

/** @brief Entry addresses (object, or object + sub). */
namespace addr {
inline constexpr Address kDeviceType{0x1000U, 0x00U};
inline constexpr Address kErrorRegister{0x1001U, 0x00U};
inline constexpr Address kPreDefinedErrorFieldNumberOfErrors{0x1003U, 0x00U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField1{0x1003U, 0x01U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField2{0x1003U, 0x02U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField3{0x1003U, 0x03U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField4{0x1003U, 0x04U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField5{0x1003U, 0x05U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField6{0x1003U, 0x06U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField7{0x1003U, 0x07U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField8{0x1003U, 0x08U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField9{0x1003U, 0x09U};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField10{0x1003U, 0x0AU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField11{0x1003U, 0x0BU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField12{0x1003U, 0x0CU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField13{0x1003U, 0x0DU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField14{0x1003U, 0x0EU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField15{0x1003U, 0x0FU};
inline constexpr Address kPreDefinedErrorFieldStandardErrorField16{0x1003U, 0x10U};
inline constexpr Address kCobIdSync{0x1005U, 0x00U};
inline constexpr Address kCommunicationCyclePeriod{0x1006U, 0x00U};
inline constexpr Address kSynchronousWindowLength{0x1007U, 0x00U};
inline constexpr Address kManufacturerDeviceName{0x1008U, 0x00U};
inline constexpr Address kManufacturerHardwareVersion{0x1009U, 0x00U};
inline constexpr Address kManufacturerSoftwareVersion{0x100AU, 0x00U};
inline constexpr Address kStoreParametersHighestSubIndex{0x1010U, 0x00U};
inline constexpr Address kStoreParametersSaveAll{0x1010U, 0x01U};
inline constexpr Address kStoreParametersSaveCommunication{0x1010U, 0x02U};
inline constexpr Address kStoreParametersSaveApplication{0x1010U, 0x03U};
inline constexpr Address kStoreParametersSaveCalibration{0x1010U, 0x04U};
inline constexpr Address kRestoreDefaultParametersHighestSubIndex{0x1011U, 0x00U};
inline constexpr Address kRestoreDefaultParametersRestoreAll{0x1011U, 0x01U};
inline constexpr Address kRestoreDefaultParametersRestoreCommunication{0x1011U, 0x02U};
inline constexpr Address kRestoreDefaultParametersRestoreApplication{0x1011U, 0x03U};
inline constexpr Address kRestoreDefaultParametersRestoreCalibration{0x1011U, 0x04U};
inline constexpr Address kCobIdTime{0x1012U, 0x00U};
inline constexpr Address kCobIdEmcy{0x1014U, 0x00U};
inline constexpr Address kInhibitTimeEmcy{0x1015U, 0x00U};
inline constexpr Address kConsumerHeartbeatTimeHighestSubIndex{0x1016U, 0x00U};
inline constexpr Address kConsumerHeartbeatTimeConsumerHeartbeatTime1{0x1016U, 0x01U};
inline constexpr Address kConsumerHeartbeatTimeConsumerHeartbeatTime2{0x1016U, 0x02U};
inline constexpr Address kConsumerHeartbeatTimeConsumerHeartbeatTime3{0x1016U, 0x03U};
inline constexpr Address kConsumerHeartbeatTimeConsumerHeartbeatTime4{0x1016U, 0x04U};
inline constexpr Address kProducerHeartbeatTime{0x1017U, 0x00U};
inline constexpr Address kIdentityHighestSubIndex{0x1018U, 0x00U};
inline constexpr Address kIdentityVendorId{0x1018U, 0x01U};
inline constexpr Address kIdentityProductCode{0x1018U, 0x02U};
inline constexpr Address kIdentityRevisionNumber{0x1018U, 0x03U};
inline constexpr Address kIdentitySerialNumber{0x1018U, 0x04U};
inline constexpr Address kSynchronousCounterOverflowValue{0x1019U, 0x00U};
inline constexpr Address kErrorBehaviorHighestSubIndex{0x1029U, 0x00U};
inline constexpr Address kErrorBehaviorCommunicationError{0x1029U, 0x01U};
inline constexpr Address kErrorBehaviorProfileError{0x1029U, 0x02U};
inline constexpr Address kSdoServer1HighestSubIndex{0x1200U, 0x00U};
inline constexpr Address kSdoServer1CobIdClientToServer{0x1200U, 0x01U};
inline constexpr Address kSdoServer1CobIdServerToClient{0x1200U, 0x02U};
inline constexpr Address kSdoServer2HighestSubIndex{0x1201U, 0x00U};
inline constexpr Address kSdoServer2CobIdClientToServer{0x1201U, 0x01U};
inline constexpr Address kSdoServer2CobIdServerToClient{0x1201U, 0x02U};
inline constexpr Address kSdoClient1HighestSubIndex{0x1280U, 0x00U};
inline constexpr Address kSdoClient1CobIdClientToServer{0x1280U, 0x01U};
inline constexpr Address kSdoClient1CobIdServerToClient{0x1280U, 0x02U};
inline constexpr Address kSdoClient1NodeIdOfTheSdoServer{0x1280U, 0x03U};
inline constexpr Address kRpdo1CommunicationHighestSubIndex{0x1400U, 0x00U};
inline constexpr Address kRpdo1CommunicationCobIdUsedByRpdo{0x1400U, 0x01U};
inline constexpr Address kRpdo1CommunicationTransmissionType{0x1400U, 0x02U};
inline constexpr Address kRpdo1CommunicationEventTimer{0x1400U, 0x05U};
inline constexpr Address kRpdo2CommunicationHighestSubIndex{0x1401U, 0x00U};
inline constexpr Address kRpdo2CommunicationCobIdUsedByRpdo{0x1401U, 0x01U};
inline constexpr Address kRpdo2CommunicationTransmissionType{0x1401U, 0x02U};
inline constexpr Address kRpdo2CommunicationEventTimer{0x1401U, 0x05U};
inline constexpr Address kRpdo3CommunicationHighestSubIndex{0x1402U, 0x00U};
inline constexpr Address kRpdo3CommunicationCobIdUsedByRpdo{0x1402U, 0x01U};
inline constexpr Address kRpdo3CommunicationTransmissionType{0x1402U, 0x02U};
inline constexpr Address kRpdo3CommunicationEventTimer{0x1402U, 0x05U};
inline constexpr Address kRpdo4CommunicationHighestSubIndex{0x1403U, 0x00U};
inline constexpr Address kRpdo4CommunicationCobIdUsedByRpdo{0x1403U, 0x01U};
inline constexpr Address kRpdo4CommunicationTransmissionType{0x1403U, 0x02U};
inline constexpr Address kRpdo4CommunicationEventTimer{0x1403U, 0x05U};
inline constexpr Address kRpdo1MappingNumberOfMappedObjects{0x1600U, 0x00U};
inline constexpr Address kRpdo1MappingApplicationObject1{0x1600U, 0x01U};
inline constexpr Address kRpdo1MappingApplicationObject2{0x1600U, 0x02U};
inline constexpr Address kRpdo1MappingApplicationObject3{0x1600U, 0x03U};
inline constexpr Address kRpdo1MappingApplicationObject4{0x1600U, 0x04U};
inline constexpr Address kRpdo1MappingApplicationObject5{0x1600U, 0x05U};
inline constexpr Address kRpdo1MappingApplicationObject6{0x1600U, 0x06U};
inline constexpr Address kRpdo1MappingApplicationObject7{0x1600U, 0x07U};
inline constexpr Address kRpdo1MappingApplicationObject8{0x1600U, 0x08U};
inline constexpr Address kRpdo2MappingNumberOfMappedObjects{0x1601U, 0x00U};
inline constexpr Address kRpdo2MappingApplicationObject1{0x1601U, 0x01U};
inline constexpr Address kRpdo2MappingApplicationObject2{0x1601U, 0x02U};
inline constexpr Address kRpdo2MappingApplicationObject3{0x1601U, 0x03U};
inline constexpr Address kRpdo2MappingApplicationObject4{0x1601U, 0x04U};
inline constexpr Address kRpdo2MappingApplicationObject5{0x1601U, 0x05U};
inline constexpr Address kRpdo2MappingApplicationObject6{0x1601U, 0x06U};
inline constexpr Address kRpdo2MappingApplicationObject7{0x1601U, 0x07U};
inline constexpr Address kRpdo2MappingApplicationObject8{0x1601U, 0x08U};
inline constexpr Address kRpdo3MappingNumberOfMappedObjects{0x1602U, 0x00U};
inline constexpr Address kRpdo3MappingApplicationObject1{0x1602U, 0x01U};
inline constexpr Address kRpdo3MappingApplicationObject2{0x1602U, 0x02U};
inline constexpr Address kRpdo3MappingApplicationObject3{0x1602U, 0x03U};
inline constexpr Address kRpdo3MappingApplicationObject4{0x1602U, 0x04U};
inline constexpr Address kRpdo3MappingApplicationObject5{0x1602U, 0x05U};
inline constexpr Address kRpdo3MappingApplicationObject6{0x1602U, 0x06U};
inline constexpr Address kRpdo3MappingApplicationObject7{0x1602U, 0x07U};
inline constexpr Address kRpdo3MappingApplicationObject8{0x1602U, 0x08U};
inline constexpr Address kRpdo4MappingNumberOfMappedObjects{0x1603U, 0x00U};
inline constexpr Address kRpdo4MappingApplicationObject1{0x1603U, 0x01U};
inline constexpr Address kRpdo4MappingApplicationObject2{0x1603U, 0x02U};
inline constexpr Address kRpdo4MappingApplicationObject3{0x1603U, 0x03U};
inline constexpr Address kRpdo4MappingApplicationObject4{0x1603U, 0x04U};
inline constexpr Address kRpdo4MappingApplicationObject5{0x1603U, 0x05U};
inline constexpr Address kRpdo4MappingApplicationObject6{0x1603U, 0x06U};
inline constexpr Address kRpdo4MappingApplicationObject7{0x1603U, 0x07U};
inline constexpr Address kRpdo4MappingApplicationObject8{0x1603U, 0x08U};
inline constexpr Address kTpdo1CommunicationHighestSubIndex{0x1800U, 0x00U};
inline constexpr Address kTpdo1CommunicationCobIdUsedByTpdo{0x1800U, 0x01U};
inline constexpr Address kTpdo1CommunicationTransmissionType{0x1800U, 0x02U};
inline constexpr Address kTpdo1CommunicationInhibitTime{0x1800U, 0x03U};
inline constexpr Address kTpdo1CommunicationEventTimer{0x1800U, 0x05U};
inline constexpr Address kTpdo1CommunicationSyncStartValue{0x1800U, 0x06U};
inline constexpr Address kTpdo2CommunicationHighestSubIndex{0x1801U, 0x00U};
inline constexpr Address kTpdo2CommunicationCobIdUsedByTpdo{0x1801U, 0x01U};
inline constexpr Address kTpdo2CommunicationTransmissionType{0x1801U, 0x02U};
inline constexpr Address kTpdo2CommunicationInhibitTime{0x1801U, 0x03U};
inline constexpr Address kTpdo2CommunicationEventTimer{0x1801U, 0x05U};
inline constexpr Address kTpdo2CommunicationSyncStartValue{0x1801U, 0x06U};
inline constexpr Address kTpdo3CommunicationHighestSubIndex{0x1802U, 0x00U};
inline constexpr Address kTpdo3CommunicationCobIdUsedByTpdo{0x1802U, 0x01U};
inline constexpr Address kTpdo3CommunicationTransmissionType{0x1802U, 0x02U};
inline constexpr Address kTpdo3CommunicationInhibitTime{0x1802U, 0x03U};
inline constexpr Address kTpdo3CommunicationEventTimer{0x1802U, 0x05U};
inline constexpr Address kTpdo3CommunicationSyncStartValue{0x1802U, 0x06U};
inline constexpr Address kTpdo4CommunicationHighestSubIndex{0x1803U, 0x00U};
inline constexpr Address kTpdo4CommunicationCobIdUsedByTpdo{0x1803U, 0x01U};
inline constexpr Address kTpdo4CommunicationTransmissionType{0x1803U, 0x02U};
inline constexpr Address kTpdo4CommunicationInhibitTime{0x1803U, 0x03U};
inline constexpr Address kTpdo4CommunicationEventTimer{0x1803U, 0x05U};
inline constexpr Address kTpdo4CommunicationSyncStartValue{0x1803U, 0x06U};
inline constexpr Address kTpdo5CommunicationHighestSubIndex{0x1804U, 0x00U};
inline constexpr Address kTpdo5CommunicationCobIdUsedByTpdo{0x1804U, 0x01U};
inline constexpr Address kTpdo5CommunicationTransmissionType{0x1804U, 0x02U};
inline constexpr Address kTpdo5CommunicationInhibitTime{0x1804U, 0x03U};
inline constexpr Address kTpdo5CommunicationEventTimer{0x1804U, 0x05U};
inline constexpr Address kTpdo5CommunicationSyncStartValue{0x1804U, 0x06U};
inline constexpr Address kTpdo6CommunicationHighestSubIndex{0x1805U, 0x00U};
inline constexpr Address kTpdo6CommunicationCobIdUsedByTpdo{0x1805U, 0x01U};
inline constexpr Address kTpdo6CommunicationTransmissionType{0x1805U, 0x02U};
inline constexpr Address kTpdo6CommunicationInhibitTime{0x1805U, 0x03U};
inline constexpr Address kTpdo6CommunicationEventTimer{0x1805U, 0x05U};
inline constexpr Address kTpdo6CommunicationSyncStartValue{0x1805U, 0x06U};
inline constexpr Address kTpdo7CommunicationHighestSubIndex{0x1806U, 0x00U};
inline constexpr Address kTpdo7CommunicationCobIdUsedByTpdo{0x1806U, 0x01U};
inline constexpr Address kTpdo7CommunicationTransmissionType{0x1806U, 0x02U};
inline constexpr Address kTpdo7CommunicationInhibitTime{0x1806U, 0x03U};
inline constexpr Address kTpdo7CommunicationEventTimer{0x1806U, 0x05U};
inline constexpr Address kTpdo7CommunicationSyncStartValue{0x1806U, 0x06U};
inline constexpr Address kTpdo8CommunicationHighestSubIndex{0x1807U, 0x00U};
inline constexpr Address kTpdo8CommunicationCobIdUsedByTpdo{0x1807U, 0x01U};
inline constexpr Address kTpdo8CommunicationTransmissionType{0x1807U, 0x02U};
inline constexpr Address kTpdo8CommunicationInhibitTime{0x1807U, 0x03U};
inline constexpr Address kTpdo8CommunicationEventTimer{0x1807U, 0x05U};
inline constexpr Address kTpdo8CommunicationSyncStartValue{0x1807U, 0x06U};
inline constexpr Address kTpdo1MappingNumberOfMappedObjects{0x1A00U, 0x00U};
inline constexpr Address kTpdo1MappingApplicationObject1{0x1A00U, 0x01U};
inline constexpr Address kTpdo1MappingApplicationObject2{0x1A00U, 0x02U};
inline constexpr Address kTpdo1MappingApplicationObject3{0x1A00U, 0x03U};
inline constexpr Address kTpdo1MappingApplicationObject4{0x1A00U, 0x04U};
inline constexpr Address kTpdo1MappingApplicationObject5{0x1A00U, 0x05U};
inline constexpr Address kTpdo1MappingApplicationObject6{0x1A00U, 0x06U};
inline constexpr Address kTpdo1MappingApplicationObject7{0x1A00U, 0x07U};
inline constexpr Address kTpdo1MappingApplicationObject8{0x1A00U, 0x08U};
inline constexpr Address kTpdo2MappingNumberOfMappedObjects{0x1A01U, 0x00U};
inline constexpr Address kTpdo2MappingApplicationObject1{0x1A01U, 0x01U};
inline constexpr Address kTpdo2MappingApplicationObject2{0x1A01U, 0x02U};
inline constexpr Address kTpdo2MappingApplicationObject3{0x1A01U, 0x03U};
inline constexpr Address kTpdo2MappingApplicationObject4{0x1A01U, 0x04U};
inline constexpr Address kTpdo2MappingApplicationObject5{0x1A01U, 0x05U};
inline constexpr Address kTpdo2MappingApplicationObject6{0x1A01U, 0x06U};
inline constexpr Address kTpdo2MappingApplicationObject7{0x1A01U, 0x07U};
inline constexpr Address kTpdo2MappingApplicationObject8{0x1A01U, 0x08U};
inline constexpr Address kTpdo3MappingNumberOfMappedObjects{0x1A02U, 0x00U};
inline constexpr Address kTpdo3MappingApplicationObject1{0x1A02U, 0x01U};
inline constexpr Address kTpdo3MappingApplicationObject2{0x1A02U, 0x02U};
inline constexpr Address kTpdo3MappingApplicationObject3{0x1A02U, 0x03U};
inline constexpr Address kTpdo3MappingApplicationObject4{0x1A02U, 0x04U};
inline constexpr Address kTpdo3MappingApplicationObject5{0x1A02U, 0x05U};
inline constexpr Address kTpdo3MappingApplicationObject6{0x1A02U, 0x06U};
inline constexpr Address kTpdo3MappingApplicationObject7{0x1A02U, 0x07U};
inline constexpr Address kTpdo3MappingApplicationObject8{0x1A02U, 0x08U};
inline constexpr Address kTpdo4MappingNumberOfMappedObjects{0x1A03U, 0x00U};
inline constexpr Address kTpdo4MappingApplicationObject1{0x1A03U, 0x01U};
inline constexpr Address kTpdo4MappingApplicationObject2{0x1A03U, 0x02U};
inline constexpr Address kTpdo4MappingApplicationObject3{0x1A03U, 0x03U};
inline constexpr Address kTpdo4MappingApplicationObject4{0x1A03U, 0x04U};
inline constexpr Address kTpdo4MappingApplicationObject5{0x1A03U, 0x05U};
inline constexpr Address kTpdo4MappingApplicationObject6{0x1A03U, 0x06U};
inline constexpr Address kTpdo4MappingApplicationObject7{0x1A03U, 0x07U};
inline constexpr Address kTpdo4MappingApplicationObject8{0x1A03U, 0x08U};
inline constexpr Address kTpdo5MappingNumberOfMappedObjects{0x1A04U, 0x00U};
inline constexpr Address kTpdo5MappingApplicationObject1{0x1A04U, 0x01U};
inline constexpr Address kTpdo5MappingApplicationObject2{0x1A04U, 0x02U};
inline constexpr Address kTpdo5MappingApplicationObject3{0x1A04U, 0x03U};
inline constexpr Address kTpdo5MappingApplicationObject4{0x1A04U, 0x04U};
inline constexpr Address kTpdo5MappingApplicationObject5{0x1A04U, 0x05U};
inline constexpr Address kTpdo5MappingApplicationObject6{0x1A04U, 0x06U};
inline constexpr Address kTpdo5MappingApplicationObject7{0x1A04U, 0x07U};
inline constexpr Address kTpdo5MappingApplicationObject8{0x1A04U, 0x08U};
inline constexpr Address kTpdo6MappingNumberOfMappedObjects{0x1A05U, 0x00U};
inline constexpr Address kTpdo6MappingApplicationObject1{0x1A05U, 0x01U};
inline constexpr Address kTpdo6MappingApplicationObject2{0x1A05U, 0x02U};
inline constexpr Address kTpdo6MappingApplicationObject3{0x1A05U, 0x03U};
inline constexpr Address kTpdo6MappingApplicationObject4{0x1A05U, 0x04U};
inline constexpr Address kTpdo6MappingApplicationObject5{0x1A05U, 0x05U};
inline constexpr Address kTpdo6MappingApplicationObject6{0x1A05U, 0x06U};
inline constexpr Address kTpdo6MappingApplicationObject7{0x1A05U, 0x07U};
inline constexpr Address kTpdo6MappingApplicationObject8{0x1A05U, 0x08U};
inline constexpr Address kTpdo7MappingNumberOfMappedObjects{0x1A06U, 0x00U};
inline constexpr Address kTpdo7MappingApplicationObject1{0x1A06U, 0x01U};
inline constexpr Address kTpdo7MappingApplicationObject2{0x1A06U, 0x02U};
inline constexpr Address kTpdo7MappingApplicationObject3{0x1A06U, 0x03U};
inline constexpr Address kTpdo7MappingApplicationObject4{0x1A06U, 0x04U};
inline constexpr Address kTpdo7MappingApplicationObject5{0x1A06U, 0x05U};
inline constexpr Address kTpdo7MappingApplicationObject6{0x1A06U, 0x06U};
inline constexpr Address kTpdo7MappingApplicationObject7{0x1A06U, 0x07U};
inline constexpr Address kTpdo7MappingApplicationObject8{0x1A06U, 0x08U};
inline constexpr Address kTpdo8MappingNumberOfMappedObjects{0x1A07U, 0x00U};
inline constexpr Address kTpdo8MappingApplicationObject1{0x1A07U, 0x01U};
inline constexpr Address kTpdo8MappingApplicationObject2{0x1A07U, 0x02U};
inline constexpr Address kTpdo8MappingApplicationObject3{0x1A07U, 0x03U};
inline constexpr Address kTpdo8MappingApplicationObject4{0x1A07U, 0x04U};
inline constexpr Address kTpdo8MappingApplicationObject5{0x1A07U, 0x05U};
inline constexpr Address kTpdo8MappingApplicationObject6{0x1A07U, 0x06U};
inline constexpr Address kTpdo8MappingApplicationObject7{0x1A07U, 0x07U};
inline constexpr Address kTpdo8MappingApplicationObject8{0x1A07U, 0x08U};
inline constexpr Address kNmtStartup{0x1F80U, 0x00U};
inline constexpr Address kFirmwareBuildHighestSubIndex{0x2000U, 0x00U};
inline constexpr Address kFirmwareBuildAppVersion{0x2000U, 0x01U};
inline constexpr Address kFirmwareBuildHalSha{0x2000U, 0x02U};
inline constexpr Address kFirmwareBuildCoreSha{0x2000U, 0x03U};
inline constexpr Address kFirmwareBuildTmc9660DriverSha{0x2000U, 0x04U};
inline constexpr Address kFirmwareBuildAs5047uDriverSha{0x2000U, 0x05U};
inline constexpr Address kFirmwareBuildBno08xDriverSha{0x2000U, 0x06U};
inline constexpr Address kFirmwareBuildCanopennodeSha{0x2000U, 0x07U};
inline constexpr Address kFirmwareBuildBuildTime{0x2000U, 0x08U};
inline constexpr Address kFirmwareBuildIdfVersion{0x2000U, 0x09U};
inline constexpr Address kBoardInfoHighestSubIndex{0x2001U, 0x00U};
inline constexpr Address kBoardInfoHwRevision{0x2001U, 0x01U};
inline constexpr Address kBoardInfoMac{0x2001U, 0x02U};
inline constexpr Address kBoardInfoResetReason{0x2001U, 0x03U};
inline constexpr Address kBoardInfoUptime{0x2001U, 0x04U};
inline constexpr Address kBoardInfoFreeHeap{0x2001U, 0x05U};
inline constexpr Address kComponentVersionsHighestSubIndex{0x2002U, 0x00U};
inline constexpr Address kComponentVersionsTmc9660FirmwareId{0x2002U, 0x01U};
inline constexpr Address kComponentVersionsTmc9660BootloaderId{0x2002U, 0x02U};
inline constexpr Address kComponentVersionsBno08xSh2Version{0x2002U, 0x03U};
inline constexpr Address kComponentVersionsAs5047uId{0x2002U, 0x04U};
inline constexpr Address kEncoderStatusHighestSubIndex{0x2101U, 0x00U};
inline constexpr Address kEncoderStatusAngleRaw{0x2101U, 0x01U};
inline constexpr Address kEncoderStatusAngleCompensated{0x2101U, 0x02U};
inline constexpr Address kEncoderStatusVelocityRaw{0x2101U, 0x03U};
inline constexpr Address kEncoderStatusMultiTurnPosition{0x2101U, 0x04U};
inline constexpr Address kEncoderStatusAgc{0x2101U, 0x05U};
inline constexpr Address kEncoderStatusMagnitude{0x2101U, 0x06U};
inline constexpr Address kEncoderStatusErrorFlags{0x2101U, 0x07U};
inline constexpr Address kEncoderStatusCommErrors{0x2101U, 0x08U};
inline constexpr Address kImuPackedQuaternionHighestSubIndex{0x2210U, 0x00U};
inline constexpr Address kImuPackedQuaternionI{0x2210U, 0x01U};
inline constexpr Address kImuPackedQuaternionJ{0x2210U, 0x02U};
inline constexpr Address kImuPackedQuaternionK{0x2210U, 0x03U};
inline constexpr Address kImuPackedQuaternionReal{0x2210U, 0x04U};
inline constexpr Address kImuPackedAccelHighestSubIndex{0x2211U, 0x00U};
inline constexpr Address kImuPackedAccelX{0x2211U, 0x01U};
inline constexpr Address kImuPackedAccelY{0x2211U, 0x02U};
inline constexpr Address kImuPackedAccelZ{0x2211U, 0x03U};
inline constexpr Address kImuPackedAccelSampleCounter{0x2211U, 0x04U};
inline constexpr Address kImuPackedGyroHighestSubIndex{0x2212U, 0x00U};
inline constexpr Address kImuPackedGyroX{0x2212U, 0x01U};
inline constexpr Address kImuPackedGyroY{0x2212U, 0x02U};
inline constexpr Address kImuPackedGyroZ{0x2212U, 0x03U};
inline constexpr Address kImuPackedGyroCalibrationStatus{0x2212U, 0x04U};
inline constexpr Address kImuPackedGyroAccuracy{0x2212U, 0x05U};
inline constexpr Address kTemperaturesHighestSubIndex{0x2300U, 0x00U};
inline constexpr Address kTemperaturesNtc{0x2300U, 0x01U};
inline constexpr Address kTemperaturesTmcChip{0x2300U, 0x02U};
inline constexpr Address kTemperaturesEsp32Die{0x2300U, 0x03U};
inline constexpr Address kTemperaturesMotor{0x2300U, 0x04U};
inline constexpr Address kPowerHighestSubIndex{0x2301U, 0x00U};
inline constexpr Address kPowerSupplyVoltage{0x2301U, 0x01U};
inline constexpr Address kPowerDriverVoltage{0x2301U, 0x02U};
inline constexpr Address kPowerPowerGood{0x2301U, 0x03U};
inline constexpr Address kControlAuthorityHighestSubIndex{0x2700U, 0x00U};
inline constexpr Address kControlAuthorityOwner{0x2700U, 0x01U};
inline constexpr Address kControlAuthorityRequest{0x2700U, 0x02U};
inline constexpr Address kAuthorityPolicyHighestSubIndex{0x2701U, 0x00U};
inline constexpr Address kAuthorityPolicyEnableTransports{0x2701U, 0x01U};
inline constexpr Address kAuthorityPolicyWirelessDeadman{0x2701U, 0x02U};
inline constexpr Address kAuthorityPolicyRequireMasterLiveness{0x2701U, 0x03U};
inline constexpr Address kAuthorityPolicyWirelessService{0x2701U, 0x04U};
inline constexpr Address kParameterStoreMetaHighestSubIndex{0x2800U, 0x00U};
inline constexpr Address kParameterStoreMetaStoreCount{0x2800U, 0x01U};
inline constexpr Address kParameterStoreMetaLastStoreResult{0x2800U, 0x02U};
inline constexpr Address kParameterStoreMetaStoredOdHash{0x2800U, 0x03U};
inline constexpr Address kParameterStoreMetaRecordsApplied{0x2800U, 0x04U};
inline constexpr Address kParameterStoreMetaRecordsDropped{0x2800U, 0x05U};
inline constexpr Address kFaultJournalHeaderHighestSubIndex{0x4000U, 0x00U};
inline constexpr Address kFaultJournalHeaderCount{0x4000U, 0x01U};
inline constexpr Address kFaultJournalHeaderHead{0x4000U, 0x02U};
inline constexpr Address kFaultJournalHeaderCapacity{0x4000U, 0x03U};
inline constexpr Address kFaultJournalHeaderOverflowCount{0x4000U, 0x04U};
inline constexpr Address kFaultJournal{0x4001U, 0x00U};
inline constexpr Address kFaultJournalClear{0x4002U, 0x00U};
inline constexpr Address kActiveCausesHighestSubIndex{0x4003U, 0x00U};
inline constexpr Address kActiveCausesActive{0x4003U, 0x01U};
inline constexpr Address kActiveCausesEscalated{0x4003U, 0x02U};
inline constexpr Address kThreadStatsHighestSubIndex{0x4200U, 0x00U};
inline constexpr Address kThreadStatsDriveRtLast{0x4200U, 0x01U};
inline constexpr Address kThreadStatsDriveRtMax{0x4200U, 0x02U};
inline constexpr Address kThreadStatsDriveRtOverruns{0x4200U, 0x03U};
inline constexpr Address kThreadStatsCoSvcLast{0x4200U, 0x04U};
inline constexpr Address kThreadStatsCoSvcMax{0x4200U, 0x05U};
inline constexpr Address kThreadStatsCoSvcOverruns{0x4200U, 0x06U};
inline constexpr Address kThreadStatsI2cIoLast{0x4200U, 0x07U};
inline constexpr Address kThreadStatsI2cIoMax{0x4200U, 0x08U};
inline constexpr Address kThreadStatsI2cIoOverruns{0x4200U, 0x09U};
inline constexpr Address kThreadStatsLivenessStaleMask{0x4200U, 0x0AU};
inline constexpr Address kTimingHistogramHighestSubIndex{0x4201U, 0x00U};
inline constexpr Address kTimingHistogramJitterLe10{0x4201U, 0x01U};
inline constexpr Address kTimingHistogramJitterLe20{0x4201U, 0x02U};
inline constexpr Address kTimingHistogramJitterLe50{0x4201U, 0x03U};
inline constexpr Address kTimingHistogramJitterLe100{0x4201U, 0x04U};
inline constexpr Address kTimingHistogramJitterLe200{0x4201U, 0x05U};
inline constexpr Address kTimingHistogramJitterLe500{0x4201U, 0x06U};
inline constexpr Address kTimingHistogramJitterLe1000{0x4201U, 0x07U};
inline constexpr Address kTimingHistogramJitterGt1000{0x4201U, 0x08U};
inline constexpr Address kTimingHistogramWcetLe100{0x4201U, 0x09U};
inline constexpr Address kTimingHistogramWcetLe150{0x4201U, 0x0AU};
inline constexpr Address kTimingHistogramWcetLe200{0x4201U, 0x0BU};
inline constexpr Address kTimingHistogramWcetLe250{0x4201U, 0x0CU};
inline constexpr Address kTimingHistogramWcetLe300{0x4201U, 0x0DU};
inline constexpr Address kTimingHistogramWcetLe500{0x4201U, 0x0EU};
inline constexpr Address kTimingHistogramWcetLe1000{0x4201U, 0x0FU};
inline constexpr Address kTimingHistogramWcetGt1000{0x4201U, 0x10U};
inline constexpr Address kCanStatsHighestSubIndex{0x4202U, 0x00U};
inline constexpr Address kCanStatsTec{0x4202U, 0x01U};
inline constexpr Address kCanStatsRec{0x4202U, 0x02U};
inline constexpr Address kCanStatsState{0x4202U, 0x03U};
inline constexpr Address kCanStatsBusOffCount{0x4202U, 0x04U};
inline constexpr Address kCanStatsRxOverruns{0x4202U, 0x05U};
inline constexpr Address kCanStatsTxQueueFull{0x4202U, 0x06U};
inline constexpr Address kCanStatsDropsCan{0x4202U, 0x07U};
inline constexpr Address kCanStatsDropsWifi{0x4202U, 0x08U};
inline constexpr Address kCanStatsDropsEspnow{0x4202U, 0x09U};
inline constexpr Address kCanStatsDropsHost{0x4202U, 0x0AU};
inline constexpr Address kOdLayoutHash{0x5FFEU, 0x00U};
inline constexpr Address kManufacturerOdVersion{0x5FFFU, 0x00U};
inline constexpr Address kAbortConnectionOptionCode{0x6007U, 0x00U};
inline constexpr Address kErrorCode{0x603FU, 0x00U};
inline constexpr Address kControlword{0x6040U, 0x00U};
inline constexpr Address kStatusword{0x6041U, 0x00U};
inline constexpr Address kQuickStopOptionCode{0x605AU, 0x00U};
inline constexpr Address kShutdownOptionCode{0x605BU, 0x00U};
inline constexpr Address kDisableOperationOptionCode{0x605CU, 0x00U};
inline constexpr Address kHaltOptionCode{0x605DU, 0x00U};
inline constexpr Address kFaultReactionOptionCode{0x605EU, 0x00U};
inline constexpr Address kModesOfOperation{0x6060U, 0x00U};
inline constexpr Address kModesOfOperationDisplay{0x6061U, 0x00U};
inline constexpr Address kPositionDemandValue{0x6062U, 0x00U};
inline constexpr Address kPositionActualValue{0x6064U, 0x00U};
inline constexpr Address kFollowingErrorWindow{0x6065U, 0x00U};
inline constexpr Address kFollowingErrorTimeOut{0x6066U, 0x00U};
inline constexpr Address kPositionWindow{0x6067U, 0x00U};
inline constexpr Address kPositionWindowTime{0x6068U, 0x00U};
inline constexpr Address kVelocityDemandValue{0x606BU, 0x00U};
inline constexpr Address kVelocityActualValue{0x606CU, 0x00U};
inline constexpr Address kVelocityWindow{0x606DU, 0x00U};
inline constexpr Address kVelocityWindowTime{0x606EU, 0x00U};
inline constexpr Address kTargetTorque{0x6071U, 0x00U};
inline constexpr Address kMaxTorque{0x6072U, 0x00U};
inline constexpr Address kMaxCurrent{0x6073U, 0x00U};
inline constexpr Address kMotorRatedCurrent{0x6075U, 0x00U};
inline constexpr Address kMotorRatedTorque{0x6076U, 0x00U};
inline constexpr Address kTorqueActualValue{0x6077U, 0x00U};
inline constexpr Address kCurrentActualValue{0x6078U, 0x00U};
inline constexpr Address kDcLinkCircuitVoltage{0x6079U, 0x00U};
inline constexpr Address kTargetPosition{0x607AU, 0x00U};
inline constexpr Address kHomeOffset{0x607CU, 0x00U};
inline constexpr Address kSoftwarePositionLimitHighestSubIndex{0x607DU, 0x00U};
inline constexpr Address kSoftwarePositionLimitMin{0x607DU, 0x01U};
inline constexpr Address kSoftwarePositionLimitMax{0x607DU, 0x02U};
inline constexpr Address kPolarity{0x607EU, 0x00U};
inline constexpr Address kMaxProfileVelocity{0x607FU, 0x00U};
inline constexpr Address kMaxMotorSpeed{0x6080U, 0x00U};
inline constexpr Address kProfileVelocity{0x6081U, 0x00U};
inline constexpr Address kProfileAcceleration{0x6083U, 0x00U};
inline constexpr Address kProfileDeceleration{0x6084U, 0x00U};
inline constexpr Address kQuickStopDeceleration{0x6085U, 0x00U};
inline constexpr Address kMotionProfileType{0x6086U, 0x00U};
inline constexpr Address kTorqueSlope{0x6087U, 0x00U};
inline constexpr Address kPositionEncoderResolutionHighestSubIndex{0x608FU, 0x00U};
inline constexpr Address kPositionEncoderResolutionEncoderIncrements{0x608FU, 0x01U};
inline constexpr Address kPositionEncoderResolutionMotorRevolutions{0x608FU, 0x02U};
inline constexpr Address kGearRatioHighestSubIndex{0x6091U, 0x00U};
inline constexpr Address kGearRatioMotorRevolutions{0x6091U, 0x01U};
inline constexpr Address kGearRatioShaftRevolutions{0x6091U, 0x02U};
inline constexpr Address kFeedConstantHighestSubIndex{0x6092U, 0x00U};
inline constexpr Address kFeedConstantFeed{0x6092U, 0x01U};
inline constexpr Address kFeedConstantShaftRevolutions{0x6092U, 0x02U};
inline constexpr Address kHomingMethod{0x6098U, 0x00U};
inline constexpr Address kHomingSpeedsHighestSubIndex{0x6099U, 0x00U};
inline constexpr Address kHomingSpeedsSwitchSearch{0x6099U, 0x01U};
inline constexpr Address kHomingSpeedsZeroSearch{0x6099U, 0x02U};
inline constexpr Address kHomingAcceleration{0x609AU, 0x00U};
inline constexpr Address kPositionOffset{0x60B0U, 0x00U};
inline constexpr Address kVelocityOffset{0x60B1U, 0x00U};
inline constexpr Address kTorqueOffset{0x60B2U, 0x00U};
inline constexpr Address kInterpolationTimePeriodHighestSubIndex{0x60C2U, 0x00U};
inline constexpr Address kInterpolationTimePeriodValue{0x60C2U, 0x01U};
inline constexpr Address kInterpolationTimePeriodIndex{0x60C2U, 0x02U};
inline constexpr Address kPositiveTorqueLimitValue{0x60E0U, 0x00U};
inline constexpr Address kNegativeTorqueLimitValue{0x60E1U, 0x00U};
inline constexpr Address kAdditionalPositionActualValueHighestSubIndex{0x60E4U, 0x00U};
inline constexpr Address kAdditionalPositionActualValueHostEncoder{0x60E4U, 0x01U};
inline constexpr Address kFollowingErrorActualValue{0x60F4U, 0x00U};
inline constexpr Address kDigitalInputs{0x60FDU, 0x00U};
inline constexpr Address kDigitalOutputsHighestSubIndex{0x60FEU, 0x00U};
inline constexpr Address kDigitalOutputsPhysicalOutputs{0x60FEU, 0x01U};
inline constexpr Address kDigitalOutputsBitMask{0x60FEU, 0x02U};
inline constexpr Address kTargetVelocity{0x60FFU, 0x00U};
inline constexpr Address kMotorType{0x6402U, 0x00U};
inline constexpr Address kMotorCatalogueNumber{0x6403U, 0x00U};
inline constexpr Address kMotorManufacturer{0x6404U, 0x00U};
inline constexpr Address kSupportedDriveModes{0x6502U, 0x00U};
}  // namespace addr

enum class Access : uint8_t { Ro = 0, Rw, Wo, Const };
namespace pdo {
inline constexpr uint8_t kNone = 0U;
inline constexpr uint8_t kTpdo = 1U;
inline constexpr uint8_t kRpdo = 2U;
}  // namespace pdo

/** @brief Description of one entry. Integers hold the default in `default_int` (the base value for
 *         $NODEID-relative defaults), reals in `default_real`, strings in `default_str`. */
struct EntryInfo {
  Address address;
  uint8_t type;  ///< odstore::dtype code
  Access access;
  uint8_t pdo;   ///< pdo bits
  odstore::Group persist;
  uint8_t length;  ///< bytes (string capacity for VS / OS)
  bool has_storage;
  bool node_relative;
  int64_t default_int;
  double default_real;
  const char* default_str;
  const char* name;
};

inline constexpr EntryInfo kEntries[] = {
    {{0x1000U, 0x00U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x20192, 0.0, nullptr, "Device type"},
    {{0x1001U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Error register"},
    {{0x1003U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, false, false, 0, 0.0, nullptr, "Number of errors"},
    {{0x1003U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x03U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x06U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x07U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x08U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x09U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0AU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0BU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0CU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0DU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0EU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x0FU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1003U, 0x10U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, false, false, 0, 0.0, nullptr, "Standard error field"},
    {{0x1005U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x80, 0.0, nullptr, "COB-ID SYNC message"},
    {{0x1006U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Communication cycle period"},
    {{0x1007U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Synchronous window length"},
    {{0x1008U, 0x00U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 17U, true, false, 0, 0.0, "HardFOC Vortex V1", "Manufacturer device name"},
    {{0x1009U, 0x00U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 16U, true, false, 0, 0.0, "V1", "Manufacturer hardware version"},
    {{0x100AU, 0x00U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 48U, true, false, 0, 0.0, "", "Manufacturer software version"},
    {{0x1010U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1010U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Save all parameters"},
    {{0x1010U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Save communication parameters"},
    {{0x1010U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Save application parameters"},
    {{0x1010U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Save calibration parameters"},
    {{0x1011U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1011U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Restore all default parameters"},
    {{0x1011U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Restore communication default parameters"},
    {{0x1011U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Restore application default parameters"},
    {{0x1011U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x1, 0.0, nullptr, "Restore calibration default parameters"},
    {{0x1012U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x100, 0.0, nullptr, "COB-ID time stamp object"},
    {{0x1014U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x80, 0.0, nullptr, "COB-ID EMCY"},
    {{0x1015U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0xA, 0.0, nullptr, "Inhibit time EMCY"},
    {{0x1016U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1016U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Consumer heartbeat time"},
    {{0x1016U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Consumer heartbeat time"},
    {{0x1016U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Consumer heartbeat time"},
    {{0x1016U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Consumer heartbeat time"},
    {{0x1017U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x64, 0.0, nullptr, "Producer heartbeat time"},
    {{0x1018U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1018U, 0x01U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Vendor-ID"},
    {{0x1018U, 0x02U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x560001, 0.0, nullptr, "Product code"},
    {{0x1018U, 0x03U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x10000, 0.0, nullptr, "Revision number"},
    {{0x1018U, 0x04U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Serial number"},
    {{0x1019U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "Synchronous counter overflow value"},
    {{0x1029U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1029U, 0x01U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "Communication error"},
    {{0x1029U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Profile error"},
    {{0x1200U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1200U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, true, 0x600, 0.0, nullptr, "COB-ID client to server (rx)"},
    {{0x1200U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, true, 0x580, 0.0, nullptr, "COB-ID server to client (tx)"},
    {{0x1201U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1201U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x6C0, 0.0, nullptr, "COB-ID client to server (rx)"},
    {{0x1201U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x5C0, 0.0, nullptr, "COB-ID server to client (tx)"},
    {{0x1280U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x3, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1280U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kTpdo | pdo::kRpdo), odstore::Group::None, 4U, true, false, 0x80000000, 0.0, nullptr, "COB-ID client to server (tx)"},
    {{0x1280U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kTpdo | pdo::kRpdo), odstore::Group::None, 4U, true, false, 0x80000000, 0.0, nullptr, "COB-ID server to client (rx)"},
    {{0x1280U, 0x03U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x1, 0.0, nullptr, "Node-ID of the SDO server"},
    {{0x1400U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1400U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x200, 0.0, nullptr, "COB-ID used by RPDO"},
    {{0x1400U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFF, 0.0, nullptr, "Transmission type"},
    {{0x1400U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1401U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1401U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x300, 0.0, nullptr, "COB-ID used by RPDO"},
    {{0x1401U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Transmission type"},
    {{0x1401U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1402U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1402U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x400, 0.0, nullptr, "COB-ID used by RPDO"},
    {{0x1402U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Transmission type"},
    {{0x1402U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1403U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1403U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x500, 0.0, nullptr, "COB-ID used by RPDO"},
    {{0x1403U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Transmission type"},
    {{0x1403U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1600U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1600U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60400010, 0.0, nullptr, "Application object 1"},
    {{0x1600U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60600008, 0.0, nullptr, "Application object 2"},
    {{0x1600U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1600U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1600U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1600U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1600U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1600U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1601U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1601U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60400010, 0.0, nullptr, "Application object 1"},
    {{0x1601U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x607A0020, 0.0, nullptr, "Application object 2"},
    {{0x1601U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1601U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1601U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1601U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1601U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1601U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1602U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1602U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60400010, 0.0, nullptr, "Application object 1"},
    {{0x1602U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60FF0020, 0.0, nullptr, "Application object 2"},
    {{0x1602U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1602U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1602U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1602U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1602U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1602U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1603U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1603U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60400010, 0.0, nullptr, "Application object 1"},
    {{0x1603U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60710010, 0.0, nullptr, "Application object 2"},
    {{0x1603U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1603U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1603U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1603U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1603U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1603U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1800U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1800U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x40000180, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1800U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1800U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x64, 0.0, nullptr, "Inhibit time"},
    {{0x1800U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x64, 0.0, nullptr, "Event timer"},
    {{0x1800U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1801U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1801U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x40000280, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1801U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Transmission type"},
    {{0x1801U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1801U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1801U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1802U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1802U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x40000380, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1802U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x1, 0.0, nullptr, "Transmission type"},
    {{0x1802U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1802U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Event timer"},
    {{0x1802U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1803U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1803U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0x40000480, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1803U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1803U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1803U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x64, 0.0, nullptr, "Event timer"},
    {{0x1803U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1804U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1804U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0xC00001C0, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1804U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1804U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1804U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0xA, 0.0, nullptr, "Event timer"},
    {{0x1804U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1805U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1805U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0xC00002C0, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1805U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1805U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1805U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0xA, 0.0, nullptr, "Event timer"},
    {{0x1805U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1806U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1806U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0xC00003C0, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1806U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1806U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1806U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0xA, 0.0, nullptr, "Event timer"},
    {{0x1806U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1807U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x6, 0.0, nullptr, "Highest sub-index supported"},
    {{0x1807U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, true, 0xC00004C0, 0.0, nullptr, "COB-ID used by TPDO"},
    {{0x1807U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0xFE, 0.0, nullptr, "Transmission type"},
    {{0x1807U, 0x03U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0x0, 0.0, nullptr, "Inhibit time"},
    {{0x1807U, 0x05U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 2U, true, false, 0xA, 0.0, nullptr, "Event timer"},
    {{0x1807U, 0x06U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x0, 0.0, nullptr, "SYNC start value"},
    {{0x1A00U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A00U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60410010, 0.0, nullptr, "Application object 1"},
    {{0x1A00U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60610008, 0.0, nullptr, "Application object 2"},
    {{0x1A00U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1A00U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1A00U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A00U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A00U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A00U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A01U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A01U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60410010, 0.0, nullptr, "Application object 1"},
    {{0x1A01U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60640020, 0.0, nullptr, "Application object 2"},
    {{0x1A01U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1A01U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1A01U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A01U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A01U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A01U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A02U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x2, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A02U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60410010, 0.0, nullptr, "Application object 1"},
    {{0x1A02U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x606C0020, 0.0, nullptr, "Application object 2"},
    {{0x1A02U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 3"},
    {{0x1A02U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1A02U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A02U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A02U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A02U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A03U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x4, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A03U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x60770010, 0.0, nullptr, "Application object 1"},
    {{0x1A03U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x603F0010, 0.0, nullptr, "Application object 2"},
    {{0x1A03U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x23000110, 0.0, nullptr, "Application object 3"},
    {{0x1A03U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x23010110, 0.0, nullptr, "Application object 4"},
    {{0x1A03U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A03U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A03U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A03U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A04U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x4, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A04U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22100110, 0.0, nullptr, "Application object 1"},
    {{0x1A04U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22100210, 0.0, nullptr, "Application object 2"},
    {{0x1A04U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22100310, 0.0, nullptr, "Application object 3"},
    {{0x1A04U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22100410, 0.0, nullptr, "Application object 4"},
    {{0x1A04U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A04U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A04U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A04U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A05U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x4, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A05U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22110110, 0.0, nullptr, "Application object 1"},
    {{0x1A05U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22110210, 0.0, nullptr, "Application object 2"},
    {{0x1A05U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22110310, 0.0, nullptr, "Application object 3"},
    {{0x1A05U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22110410, 0.0, nullptr, "Application object 4"},
    {{0x1A05U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A05U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A05U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A05U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A06U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x5, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A06U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22120110, 0.0, nullptr, "Application object 1"},
    {{0x1A06U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22120210, 0.0, nullptr, "Application object 2"},
    {{0x1A06U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22120310, 0.0, nullptr, "Application object 3"},
    {{0x1A06U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22120408, 0.0, nullptr, "Application object 4"},
    {{0x1A06U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x22120508, 0.0, nullptr, "Application object 5"},
    {{0x1A06U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A06U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A06U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1A07U, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 1U, true, false, 0x3, 0.0, nullptr, "Number of mapped application objects in PDO"},
    {{0x1A07U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x21010110, 0.0, nullptr, "Application object 1"},
    {{0x1A07U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x21010310, 0.0, nullptr, "Application object 2"},
    {{0x1A07U, 0x03U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x21010710, 0.0, nullptr, "Application object 3"},
    {{0x1A07U, 0x04U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 4"},
    {{0x1A07U, 0x05U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 5"},
    {{0x1A07U, 0x06U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 6"},
    {{0x1A07U, 0x07U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 7"},
    {{0x1A07U, 0x08U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "Application object 8"},
    {{0x1F80U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::Comm, 4U, true, false, 0x0, 0.0, nullptr, "NMT startup"},
    {{0x2000U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x9, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2000U, 0x01U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 32U, true, false, 0, 0.0, "", "Application version"},
    {{0x2000U, 0x02U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "HAL git SHA"},
    {{0x2000U, 0x03U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "Core git SHA"},
    {{0x2000U, 0x04U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "TMC9660 driver git SHA"},
    {{0x2000U, 0x05U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "AS5047U driver git SHA"},
    {{0x2000U, 0x06U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "BNO08x driver git SHA"},
    {{0x2000U, 0x07U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 40U, true, false, 0, 0.0, "", "CANopenNode git SHA"},
    {{0x2000U, 0x08U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Build time"},
    {{0x2000U, 0x09U}, 0x09U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 32U, true, false, 0, 0.0, "", "IDF version"},
    {{0x2001U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2001U, 0x01U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Hardware revision"},
    {{0x2001U, 0x02U}, 0x0AU, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 6U, true, false, 0, 0.0, nullptr, "MAC address"},
    {{0x2001U, 0x03U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Reset reason"},
    {{0x2001U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Uptime"},
    {{0x2001U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Free heap"},
    {{0x2002U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2002U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "TMC9660 firmware id"},
    {{0x2002U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "TMC9660 bootloader id"},
    {{0x2002U, 0x03U}, 0x09U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 24U, true, false, 0, 0.0, "", "BNO08x SH-2 version"},
    {{0x2002U, 0x04U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "AS5047U id"},
    {{0x2101U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x8, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2101U, 0x01U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Angle raw"},
    {{0x2101U, 0x02U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Angle compensated"},
    {{0x2101U, 0x03U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Velocity raw"},
    {{0x2101U, 0x04U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Multi-turn position"},
    {{0x2101U, 0x05U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "AGC"},
    {{0x2101U, 0x06U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Magnitude"},
    {{0x2101U, 0x07U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Error flags"},
    {{0x2101U, 0x08U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Communication errors"},
    {{0x2210U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2210U, 0x01U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Quaternion i"},
    {{0x2210U, 0x02U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Quaternion j"},
    {{0x2210U, 0x03U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Quaternion k"},
    {{0x2210U, 0x04U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Quaternion real"},
    {{0x2211U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2211U, 0x01U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Acceleration x"},
    {{0x2211U, 0x02U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Acceleration y"},
    {{0x2211U, 0x03U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Acceleration z"},
    {{0x2211U, 0x04U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Sample counter"},
    {{0x2212U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2212U, 0x01U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Angular rate x"},
    {{0x2212U, 0x02U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Angular rate y"},
    {{0x2212U, 0x03U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Angular rate z"},
    {{0x2212U, 0x04U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Calibration status"},
    {{0x2212U, 0x05U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Accuracy"},
    {{0x2300U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2300U, 0x01U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Power stage NTC"},
    {{0x2300U, 0x02U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "TMC9660 chip"},
    {{0x2300U, 0x03U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "ESP32 die"},
    {{0x2300U, 0x04U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Motor"},
    {{0x2301U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x3, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2301U, 0x01U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Supply voltage"},
    {{0x2301U, 0x02U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Driver voltage"},
    {{0x2301U, 0x03U}, 0x01U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Power good"},
    {{0x2700U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2700U, 0x01U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Current owner"},
    {{0x2700U, 0x02U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Request"},
    {{0x2701U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2701U, 0x01U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x1, 0.0, nullptr, "Allowed enable transports"},
    {{0x2701U, 0x02U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0xC8, 0.0, nullptr, "Wireless deadman"},
    {{0x2701U, 0x03U}, 0x01U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x1, 0.0, nullptr, "Require master liveness"},
    {{0x2701U, 0x04U}, 0x01U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x0, 0.0, nullptr, "Wireless service"},
    {{0x2800U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x5, 0.0, nullptr, "Highest sub-index supported"},
    {{0x2800U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Store count"},
    {{0x2800U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Last store result"},
    {{0x2800U, 0x03U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "OD hash at last store"},
    {{0x2800U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Records applied at restore"},
    {{0x2800U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Records dropped at restore"},
    {{0x4000U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x4, 0.0, nullptr, "Highest sub-index supported"},
    {{0x4000U, 0x01U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Entry count"},
    {{0x4000U, 0x02U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Head"},
    {{0x4000U, 0x03U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x40, 0.0, nullptr, "Capacity"},
    {{0x4000U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Overflow count"},
    {{0x4001U, 0x00U}, 0x0FU, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 0U, false, false, 0, 0.0, nullptr, "Fault journal"},
    {{0x4002U, 0x00U}, 0x07U, Access::Wo, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Fault journal clear"},
    {{0x4003U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x4003U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Active causes"},
    {{0x4003U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Escalated causes"},
    {{0x4200U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0xA, 0.0, nullptr, "Highest sub-index supported"},
    {{0x4200U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "drive_rt last step"},
    {{0x4200U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "drive_rt max step"},
    {{0x4200U, 0x03U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "drive_rt overruns"},
    {{0x4200U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "co_svc last step"},
    {{0x4200U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "co_svc max step"},
    {{0x4200U, 0x06U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "co_svc overruns"},
    {{0x4200U, 0x07U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "i2c_io last step"},
    {{0x4200U, 0x08U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "i2c_io max step"},
    {{0x4200U, 0x09U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "i2c_io overruns"},
    {{0x4200U, 0x0AU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Liveness stale mask"},
    {{0x4201U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x10, 0.0, nullptr, "Highest sub-index supported"},
    {{0x4201U, 0x01U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 10 us"},
    {{0x4201U, 0x02U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 20 us"},
    {{0x4201U, 0x03U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 50 us"},
    {{0x4201U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 100 us"},
    {{0x4201U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 200 us"},
    {{0x4201U, 0x06U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 500 us"},
    {{0x4201U, 0x07U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter up to 1000 us"},
    {{0x4201U, 0x08U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Jitter above 1000 us"},
    {{0x4201U, 0x09U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 100 us"},
    {{0x4201U, 0x0AU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 150 us"},
    {{0x4201U, 0x0BU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 200 us"},
    {{0x4201U, 0x0CU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 250 us"},
    {{0x4201U, 0x0DU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 300 us"},
    {{0x4201U, 0x0EU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 500 us"},
    {{0x4201U, 0x0FU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step up to 1000 us"},
    {{0x4201U, 0x10U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Step above 1000 us"},
    {{0x4202U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0xA, 0.0, nullptr, "Highest sub-index supported"},
    {{0x4202U, 0x01U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "TEC"},
    {{0x4202U, 0x02U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "REC"},
    {{0x4202U, 0x03U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x0, 0.0, nullptr, "Bus state"},
    {{0x4202U, 0x04U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Bus-off count"},
    {{0x4202U, 0x05U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "RX overruns"},
    {{0x4202U, 0x06U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "TX queue full"},
    {{0x4202U, 0x07U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Router drops CAN"},
    {{0x4202U, 0x08U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Router drops Wi-Fi"},
    {{0x4202U, 0x09U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Router drops ESP-NOW"},
    {{0x4202U, 0x0AU}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Router drops host"},
    {{0x5FFEU, 0x00U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0xFF85661C, 0.0, nullptr, "OD layout hash"},
    {{0x5FFFU, 0x00U}, 0x06U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 2U, true, false, 0x100, 0.0, nullptr, "Manufacturer OD version"},
    {{0x6007U, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 1, 0.0, nullptr, "Abort connection option code"},
    {{0x603FU, 0x00U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Error code"},
    {{0x6040U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Controlword"},
    {{0x6041U, 0x00U}, 0x06U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0x0, 0.0, nullptr, "Statusword"},
    {{0x605AU, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 2, 0.0, nullptr, "Quick stop option code"},
    {{0x605BU, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0, 0.0, nullptr, "Shutdown option code"},
    {{0x605CU, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 1, 0.0, nullptr, "Disable operation option code"},
    {{0x605DU, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 1, 0.0, nullptr, "Halt option code"},
    {{0x605EU, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0, 0.0, nullptr, "Fault reaction option code"},
    {{0x6060U, 0x00U}, 0x02U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 1U, true, false, 0, 0.0, nullptr, "Modes of operation"},
    {{0x6061U, 0x00U}, 0x02U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 1U, true, false, 0, 0.0, nullptr, "Modes of operation display"},
    {{0x6062U, 0x00U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Position demand value"},
    {{0x6064U, 0x00U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Position actual value"},
    {{0x6065U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0xFFFFFFFF, 0.0, nullptr, "Following error window"},
    {{0x6066U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x0, 0.0, nullptr, "Following error time out"},
    {{0x6067U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0xFFFFFFFF, 0.0, nullptr, "Position window"},
    {{0x6068U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x0, 0.0, nullptr, "Position window time"},
    {{0x606BU, 0x00U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Velocity demand value"},
    {{0x606CU, 0x00U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Velocity actual value"},
    {{0x606DU, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x0, 0.0, nullptr, "Velocity window"},
    {{0x606EU, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x0, 0.0, nullptr, "Velocity window time"},
    {{0x6071U, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Target torque"},
    {{0x6072U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x3E8, 0.0, nullptr, "Max torque"},
    {{0x6073U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x3E8, 0.0, nullptr, "Max current"},
    {{0x6075U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Motor rated current"},
    {{0x6076U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Motor rated torque"},
    {{0x6077U, 0x00U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Torque actual value"},
    {{0x6078U, 0x00U}, 0x03U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Current actual value"},
    {{0x6079U, 0x00U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "DC link circuit voltage"},
    {{0x607AU, 0x00U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Target position"},
    {{0x607CU, 0x00U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0, 0.0, nullptr, "Home offset"},
    {{0x607DU, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x607DU, 0x01U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0, 0.0, nullptr, "Min position limit"},
    {{0x607DU, 0x02U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0, 0.0, nullptr, "Max position limit"},
    {{0x607EU, 0x00U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x0, 0.0, nullptr, "Polarity"},
    {{0x607FU, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Max profile velocity"},
    {{0x6080U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Max motor speed"},
    {{0x6081U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Profile velocity"},
    {{0x6083U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Profile acceleration"},
    {{0x6084U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Profile deceleration"},
    {{0x6085U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Quick stop deceleration"},
    {{0x6086U, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0, 0.0, nullptr, "Motion profile type"},
    {{0x6087U, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Torque slope"},
    {{0x608FU, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x608FU, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Encoder increments"},
    {{0x608FU, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Motor revolutions"},
    {{0x6091U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x6091U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Motor revolutions"},
    {{0x6091U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Shaft revolutions"},
    {{0x6092U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x6092U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Feed"},
    {{0x6092U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x1, 0.0, nullptr, "Shaft revolutions"},
    {{0x6098U, 0x00U}, 0x02U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0, 0.0, nullptr, "Homing method"},
    {{0x6099U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x6099U, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Speed during search for switch"},
    {{0x6099U, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Speed during search for zero"},
    {{0x609AU, 0x00U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 4U, true, false, 0x0, 0.0, nullptr, "Homing acceleration"},
    {{0x60B0U, 0x00U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Position offset"},
    {{0x60B1U, 0x00U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Velocity offset"},
    {{0x60B2U, 0x00U}, 0x03U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 2U, true, false, 0, 0.0, nullptr, "Torque offset"},
    {{0x60C2U, 0x00U}, 0x05U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x60C2U, 0x01U}, 0x05U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x1, 0.0, nullptr, "Interpolation time period value"},
    {{0x60C2U, 0x02U}, 0x02U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, -3, 0.0, nullptr, "Interpolation time index"},
    {{0x60E0U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x3E8, 0.0, nullptr, "Positive torque limit value"},
    {{0x60E1U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x3E8, 0.0, nullptr, "Negative torque limit value"},
    {{0x60E4U, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x1, 0.0, nullptr, "Highest sub-index supported"},
    {{0x60E4U, 0x01U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Host encoder position"},
    {{0x60F4U, 0x00U}, 0x04U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Following error actual value"},
    {{0x60FDU, 0x00U}, 0x07U, Access::Ro, static_cast<uint8_t>(pdo::kTpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Digital inputs"},
    {{0x60FEU, 0x00U}, 0x05U, Access::Ro, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 1U, true, false, 0x2, 0.0, nullptr, "Highest sub-index supported"},
    {{0x60FEU, 0x01U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Physical outputs"},
    {{0x60FEU, 0x02U}, 0x07U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0x0, 0.0, nullptr, "Bit mask"},
    {{0x60FFU, 0x00U}, 0x04U, Access::Rw, static_cast<uint8_t>(pdo::kRpdo), odstore::Group::None, 4U, true, false, 0, 0.0, nullptr, "Target velocity"},
    {{0x6402U, 0x00U}, 0x06U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 2U, true, false, 0x0, 0.0, nullptr, "Motor type"},
    {{0x6403U, 0x00U}, 0x09U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 32U, true, false, 0, 0.0, "", "Motor catalogue number"},
    {{0x6404U, 0x00U}, 0x09U, Access::Rw, static_cast<uint8_t>(pdo::kNone), odstore::Group::App, 32U, true, false, 0, 0.0, "", "Motor manufacturer"},
    {{0x6502U, 0x00U}, 0x07U, Access::Const, static_cast<uint8_t>(pdo::kNone), odstore::Group::None, 4U, true, false, 0x3AD, 0.0, nullptr, "Supported drive modes"},
};
inline constexpr size_t kEntryCount = sizeof(kEntries) / sizeof(kEntries[0]);

/** @brief Persistable entries for the TLV store (od_store_codec.hpp). */
inline constexpr odstore::ObjectSpec kPersistSpec[] = {
    {0x1005U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1006U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1007U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1012U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1014U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1015U, 0x00U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1016U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1016U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1016U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1016U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1017U, 0x00U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1019U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1029U, 0x01U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1029U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1201U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1201U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1400U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1400U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1400U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1401U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1401U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1401U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1402U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1402U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1402U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1403U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1403U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1403U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1600U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1600U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1600U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1601U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1601U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1602U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1602U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1603U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1603U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1800U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1800U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1800U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1800U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1800U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1801U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1801U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1801U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1801U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1801U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1802U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1802U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1802U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1802U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1802U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1803U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1803U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1803U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1803U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1803U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1804U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1804U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1804U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1804U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1804U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1805U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1805U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1805U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1805U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1805U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1806U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1806U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1806U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1806U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1806U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1807U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1807U, 0x02U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1807U, 0x03U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1807U, 0x05U, 0x06U, odstore::Group::Comm, 2U, false, 0, 0},
    {0x1807U, 0x06U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A00U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A00U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A00U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A01U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A01U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A02U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A02U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A03U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A03U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A04U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A04U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A05U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A05U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A06U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A06U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x00U, 0x05U, odstore::Group::Comm, 1U, false, 0, 0},
    {0x1A07U, 0x01U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x02U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x03U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x04U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x05U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x06U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x07U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1A07U, 0x08U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x1F80U, 0x00U, 0x07U, odstore::Group::Comm, 4U, false, 0, 0},
    {0x2701U, 0x01U, 0x05U, odstore::Group::App, 1U, true, 0, 7},
    {0x2701U, 0x02U, 0x06U, odstore::Group::App, 2U, true, 20, 65535},
    {0x2701U, 0x03U, 0x01U, odstore::Group::App, 1U, false, 0, 0},
    {0x2701U, 0x04U, 0x01U, odstore::Group::App, 1U, false, 0, 0},
    {0x6007U, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 3},
    {0x605AU, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 8},
    {0x605BU, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 1},
    {0x605CU, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 1},
    {0x605DU, 0x00U, 0x03U, odstore::Group::App, 2U, true, 1, 4},
    {0x605EU, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 4},
    {0x6065U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6066U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6067U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6068U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x606DU, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x606EU, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6072U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6073U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6075U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6076U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x607CU, 0x00U, 0x04U, odstore::Group::App, 4U, false, 0, 0},
    {0x607DU, 0x01U, 0x04U, odstore::Group::App, 4U, false, 0, 0},
    {0x607DU, 0x02U, 0x04U, odstore::Group::App, 4U, false, 0, 0},
    {0x607EU, 0x00U, 0x05U, odstore::Group::App, 1U, false, 0, 0},
    {0x607FU, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6080U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6081U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6083U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6084U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6085U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6086U, 0x00U, 0x03U, odstore::Group::App, 2U, true, 0, 0},
    {0x6087U, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x608FU, 0x01U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x608FU, 0x02U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6091U, 0x01U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6091U, 0x02U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6092U, 0x01U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6092U, 0x02U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6098U, 0x00U, 0x02U, odstore::Group::App, 1U, false, 0, 0},
    {0x6099U, 0x01U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x6099U, 0x02U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x609AU, 0x00U, 0x07U, odstore::Group::App, 4U, false, 0, 0},
    {0x60E0U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x60E1U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6402U, 0x00U, 0x06U, odstore::Group::App, 2U, false, 0, 0},
    {0x6403U, 0x00U, 0x09U, odstore::Group::App, 32U, false, 0, 0},
    {0x6404U, 0x00U, 0x09U, odstore::Group::App, 32U, false, 0, 0},
};
inline constexpr size_t kPersistSpecCount = sizeof(kPersistSpec) / sizeof(kPersistSpec[0]);

/** @brief Renamed addresses (YAML `migrations:`). */
inline constexpr odstore::Migration kMigrations[1] = {{0U, 0U, 0U, 0U}};
inline constexpr size_t kMigrationCount = 0U;

/** @brief Defaults that are $NODEID + base. CANopenNode adds the node-ID itself only for 0x1014,
 *         0x1200 and PDOs 1-4; CoNode applies this table before CO_CANopenInit. */
struct NodeIdDefault {
  Address address;
  uint32_t base;
};
inline constexpr NodeIdDefault kNodeIdRelative[] = {
    {{0x1014U, 0x00U}, 0x00000080U},
    {{0x1200U, 0x01U}, 0x00000600U},
    {{0x1200U, 0x02U}, 0x00000580U},
    {{0x1201U, 0x01U}, 0x000006C0U},
    {{0x1201U, 0x02U}, 0x000005C0U},
    {{0x1400U, 0x01U}, 0x00000200U},
    {{0x1401U, 0x01U}, 0x00000300U},
    {{0x1402U, 0x01U}, 0x00000400U},
    {{0x1403U, 0x01U}, 0x00000500U},
    {{0x1800U, 0x01U}, 0x40000180U},
    {{0x1801U, 0x01U}, 0x40000280U},
    {{0x1802U, 0x01U}, 0x40000380U},
    {{0x1803U, 0x01U}, 0x40000480U},
    {{0x1804U, 0x01U}, 0xC00001C0U},
    {{0x1805U, 0x01U}, 0xC00002C0U},
    {{0x1806U, 0x01U}, 0xC00003C0U},
    {{0x1807U, 0x01U}, 0xC00004C0U},
};
inline constexpr size_t kNodeIdRelativeCount = sizeof(kNodeIdRelative) / sizeof(kNodeIdRelative[0]);

/** @brief Objects served by an OD extension (YAML `od_extension:`). */
struct Extension {
  uint16_t index;
  const char* handler;
};
inline constexpr Extension kExtensions[] = {
    {0x1003U, "CO_EM"},
    {0x1010U, "CO_storage"},
    {0x1011U, "CO_storage"},
    {0x2001U, "BoardInfo"},
    {0x2101U, "EncoderSnapshot"},
    {0x2210U, "ImuSnapshot"},
    {0x2211U, "ImuSnapshot"},
    {0x2212U, "ImuSnapshot"},
    {0x2300U, "TemperatureSnapshot"},
    {0x2301U, "PowerSnapshot"},
    {0x2700U, "Authority"},
    {0x2800U, "StoreMeta"},
    {0x4000U, "FaultJournal"},
    {0x4001U, "FaultJournal"},
    {0x4002U, "FaultJournal"},
    {0x4003U, "FaultBook"},
    {0x4200U, "ThreadStats"},
    {0x4201U, "ThreadStats"},
    {0x4202U, "CanStats"},
    {0x60E4U, "EncoderSnapshot"},
};
inline constexpr size_t kExtensionCount = sizeof(kExtensions) / sizeof(kExtensions[0]);

}  // namespace vortex::od
