/*******************************************************************************
    Vortex V1 object dictionary for CANopenNode V4

    Source: od/vortex_od.yaml, revision 1.0, OD hash 0xFF85661C.
    Do not edit: change the YAML and run `python3 tools/odgen`.
*******************************************************************************/

#define OD_DEFINITION
#include "301/CO_ODinterface.h"
#include "OD.h"

#if CO_VERSION_MAJOR < 4
#error This Object dictionary is compatible with CANopenNode V4.0 and above!
#endif

/*******************************************************************************
    OD data initialization of all groups
*******************************************************************************/
OD_ATTR_RAM OD_RAM_t OD_RAM = {
    .x1000_deviceType = 0x00020192,
    .x1001_errorRegister = 0x00,
    .x1008_manufacturerDeviceName = "HardFOC Vortex V1",
    .x1009_manufacturerHardwareVersion = "V1",
    .x100A_manufacturerSoftwareVersion = "",
    .x1010_storeParameters_sub0 = 0x04,
    .x1010_storeParameters = {0x00000001, 0x00000001, 0x00000001, 0x00000001},
    .x1011_restoreDefaultParameters_sub0 = 0x04,
    .x1011_restoreDefaultParameters = {0x00000001, 0x00000001, 0x00000001, 0x00000001},
    .x1018_identity = {
        .highestSub_indexSupported = 0x04,
        .vendor_ID = 0x00000000,
        .productCode = 0x00560001,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000000
    },
    .x1200_SDOServerParameter = {
        .highestSub_indexSupported = 0x02,
        .COB_IDClientToServerRx = 0x00000600,
        .COB_IDServerToClientTx = 0x00000580
    },
    .x1280_SDOClientParameter = {
        .highestSub_indexSupported = 0x03,
        .COB_IDClientToServerTx = 0x80000000,
        .COB_IDServerToClientRx = 0x80000000,
        .node_IDOfTheSDOServer = 0x01
    },
    .x2000_firmwareBuild = {
        .highestSub_indexSupported = 0x09,
        .applicationVersion = "",
        .HALGitSHA = "",
        .coreGitSHA = "",
        .TMC9660DriverGitSHA = "",
        .AS5047UDriverGitSHA = "",
        .BNO08xDriverGitSHA = "",
        .CANopenNodeGitSHA = "",
        .buildTime = 0x00000000,
        .IDFVersion = ""
    },
    .x2001_boardInfo = {
        .highestSub_indexSupported = 0x05,
        .hardwareRevision = 0x00,
        .MACAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .resetReason = 0x00000000,
        .uptime = 0x00000000,
        .freeHeap = 0x00000000
    },
    .x2002_componentVersions = {
        .highestSub_indexSupported = 0x04,
        .TMC9660FirmwareId = 0x00000000,
        .TMC9660BootloaderId = 0x00000000,
        .BNO08xSH_2Version = "",
        .AS5047UId = 0x0000
    },
    .x2101_encoderStatus = {
        .highestSub_indexSupported = 0x08,
        .angleRaw = 0x0000,
        .angleCompensated = 0x0000,
        .velocityRaw = 0,
        .multi_turnPosition = 0,
        .AGC = 0x00,
        .magnitude = 0x0000,
        .errorFlags = 0x0000,
        .communicationErrors = 0x00000000
    },
    .x2210_IMUPackedQuaternion = {
        .highestSub_indexSupported = 0x04,
        .quaternionI = 0,
        .quaternionJ = 0,
        .quaternionK = 0,
        .quaternionReal = 0
    },
    .x2211_IMUPackedAcceleration = {
        .highestSub_indexSupported = 0x04,
        .accelerationX = 0,
        .accelerationY = 0,
        .accelerationZ = 0,
        .sampleCounter = 0x0000
    },
    .x2212_IMUPackedGyro = {
        .highestSub_indexSupported = 0x05,
        .angularRateX = 0,
        .angularRateY = 0,
        .angularRateZ = 0,
        .calibrationStatus = 0x00,
        .accuracy = 0x00
    },
    .x2300_temperatures = {
        .highestSub_indexSupported = 0x04,
        .powerStageNTC = 0,
        .TMC9660Chip = 0,
        .ESP32Die = 0,
        .motor = 0
    },
    .x2301_power = {
        .highestSub_indexSupported = 0x03,
        .supplyVoltage = 0x0000,
        .driverVoltage = 0x0000,
        .powerGood = false
    },
    .x2700_controlAuthority = {
        .highestSub_indexSupported = 0x02,
        .currentOwner = 0x00,
        .request = 0x00
    },
    .x2800_parameterStoreMeta = {
        .highestSub_indexSupported = 0x05,
        .storeCount = 0x00000000,
        .lastStoreResult = 0x00000000,
        .ODHashAtLastStore = 0x00000000,
        .recordsAppliedAtRestore = 0x00000000,
        .recordsDroppedAtRestore = 0x00000000
    },
    .x4000_faultJournalHeader = {
        .highestSub_indexSupported = 0x04,
        .entryCount = 0x0000,
        .head = 0x0000,
        .capacity = 0x0040,
        .overflowCount = 0x00000000
    },
    .x4002_faultJournalClear = 0x00000000,
    .x4003_activeCauses = {
        .highestSub_indexSupported = 0x02,
        .activeCauses = 0x00000000,
        .escalatedCauses = 0x00000000
    },
    .x4200_threadStats = {
        .highestSub_indexSupported = 0x0A,
        .driveRtLastStep = 0x00000000,
        .driveRtMaxStep = 0x00000000,
        .driveRtOverruns = 0x00000000,
        .coSvcLastStep = 0x00000000,
        .coSvcMaxStep = 0x00000000,
        .coSvcOverruns = 0x00000000,
        .i2cIoLastStep = 0x00000000,
        .i2cIoMaxStep = 0x00000000,
        .i2cIoOverruns = 0x00000000,
        .livenessStaleMask = 0x00000000
    },
    .x4201_timingHistogram = {
        .highestSub_indexSupported = 0x10,
        .jitterUpTo10Us = 0x00000000,
        .jitterUpTo20Us = 0x00000000,
        .jitterUpTo50Us = 0x00000000,
        .jitterUpTo100Us = 0x00000000,
        .jitterUpTo200Us = 0x00000000,
        .jitterUpTo500Us = 0x00000000,
        .jitterUpTo1000Us = 0x00000000,
        .jitterAbove1000Us = 0x00000000,
        .stepUpTo100Us = 0x00000000,
        .stepUpTo150Us = 0x00000000,
        .stepUpTo200Us = 0x00000000,
        .stepUpTo250Us = 0x00000000,
        .stepUpTo300Us = 0x00000000,
        .stepUpTo500Us = 0x00000000,
        .stepUpTo1000Us = 0x00000000,
        .stepAbove1000Us = 0x00000000
    },
    .x4202_CANStats = {
        .highestSub_indexSupported = 0x0A,
        .TEC = 0x0000,
        .REC = 0x0000,
        .busState = 0x00,
        .bus_offCount = 0x00000000,
        .RXOverruns = 0x00000000,
        .TXQueueFull = 0x00000000,
        .routerDropsCAN = 0x00000000,
        .routerDropsWi_Fi = 0x00000000,
        .routerDropsESP_NOW = 0x00000000,
        .routerDropsHost = 0x00000000
    },
    .x5FFE_ODLayoutHash = 0xFF85661C,
    .x5FFF_manufacturerODVersion = 0x0100,
    .x603F_errorCode = 0x0000,
    .x6040_controlword = 0x0000,
    .x6041_statusword = 0x0000,
    .x6060_modesOfOperation = 0,
    .x6061_modesOfOperationDisplay = 0,
    .x6062_positionDemandValue = 0,
    .x6064_positionActualValue = 0,
    .x606B_velocityDemandValue = 0,
    .x606C_velocityActualValue = 0,
    .x6071_targetTorque = 0,
    .x6077_torqueActualValue = 0,
    .x6078_currentActualValue = 0,
    .x6079_DCLinkCircuitVoltage = 0x00000000,
    .x607A_targetPosition = 0,
    .x60B0_positionOffset = 0,
    .x60B1_velocityOffset = 0,
    .x60B2_torqueOffset = 0,
    .x60C2_interpolationTimePeriod = {
        .highestSub_indexSupported = 0x02,
        .interpolationTimePeriodValue = 0x01,
        .interpolationTimeIndex = -3
    },
    .x60E4_additionalPositionActualValue_sub0 = 0x01,
    .x60E4_additionalPositionActualValue = {0},
    .x60F4_followingErrorActualValue = 0,
    .x60FD_digitalInputs = 0x00000000,
    .x60FE_digitalOutputs_sub0 = 0x02,
    .x60FE_digitalOutputs = {0x00000000, 0x00000000},
    .x60FF_targetVelocity = 0,
    .x6502_supportedDriveModes = 0x000003AD
};

OD_ATTR_PERSIST_COMM OD_PERSIST_COMM_t OD_PERSIST_COMM = {
    .x1005_COB_ID_SYNCMessage = 0x00000080,
    .x1006_communicationCyclePeriod = 0x00000000,
    .x1007_synchronousWindowLength = 0x00000000,
    .x1012_COB_IDTimeStampObject = 0x00000100,
    .x1014_COB_ID_EMCY = 0x00000080,
    .x1015_inhibitTimeEMCY = 0x000A,
    .x1016_consumerHeartbeatTime_sub0 = 0x04,
    .x1016_consumerHeartbeatTime = {0x00000000, 0x00000000, 0x00000000, 0x00000000},
    .x1017_producerHeartbeatTime = 0x0064,
    .x1019_synchronousCounterOverflowValue = 0x00,
    .x1029_errorBehavior_sub0 = 0x02,
    .x1029_errorBehavior = {0x00, 0x01},
    .x1201_SDOServerParameter2 = {
        .highestSub_indexSupported = 0x02,
        .COB_IDClientToServerRx = 0x000006C0,
        .COB_IDServerToClientTx = 0x000005C0
    },
    .x1400_RPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x05,
        .COB_IDUsedByRPDO = 0x00000200,
        .transmissionType = 0xFF,
        .eventTimer = 0x0000
    },
    .x1401_RPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x05,
        .COB_IDUsedByRPDO = 0x00000300,
        .transmissionType = 0x01,
        .eventTimer = 0x0000
    },
    .x1402_RPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x05,
        .COB_IDUsedByRPDO = 0x00000400,
        .transmissionType = 0x01,
        .eventTimer = 0x0000
    },
    .x1403_RPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x05,
        .COB_IDUsedByRPDO = 0x00000500,
        .transmissionType = 0x01,
        .eventTimer = 0x0000
    },
    .x1600_RPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60400010,
        .applicationObject2 = 0x60600008,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1601_RPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60400010,
        .applicationObject2 = 0x607A0020,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1602_RPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60400010,
        .applicationObject2 = 0x60FF0020,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1603_RPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60400010,
        .applicationObject2 = 0x60710010,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1800_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0x40000180,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0064,
        .eventTimer = 0x0064,
        .SYNCStartValue = 0x00
    },
    .x1801_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0x40000280,
        .transmissionType = 0x01,
        .inhibitTime = 0x0000,
        .eventTimer = 0x0000,
        .SYNCStartValue = 0x00
    },
    .x1802_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0x40000380,
        .transmissionType = 0x01,
        .inhibitTime = 0x0000,
        .eventTimer = 0x0000,
        .SYNCStartValue = 0x00
    },
    .x1803_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0x40000480,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0000,
        .eventTimer = 0x0064,
        .SYNCStartValue = 0x00
    },
    .x1804_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0xC00001C0,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0000,
        .eventTimer = 0x000A,
        .SYNCStartValue = 0x00
    },
    .x1805_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0xC00002C0,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0000,
        .eventTimer = 0x000A,
        .SYNCStartValue = 0x00
    },
    .x1806_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0xC00003C0,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0000,
        .eventTimer = 0x000A,
        .SYNCStartValue = 0x00
    },
    .x1807_TPDOCommunicationParameter = {
        .highestSub_indexSupported = 0x06,
        .COB_IDUsedByTPDO = 0xC00004C0,
        .transmissionType = 0xFE,
        .inhibitTime = 0x0000,
        .eventTimer = 0x000A,
        .SYNCStartValue = 0x00
    },
    .x1A00_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60410010,
        .applicationObject2 = 0x60610008,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A01_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60410010,
        .applicationObject2 = 0x60640020,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A02_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x02,
        .applicationObject1 = 0x60410010,
        .applicationObject2 = 0x606C0020,
        .applicationObject3 = 0x00000000,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A03_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x04,
        .applicationObject1 = 0x60770010,
        .applicationObject2 = 0x603F0010,
        .applicationObject3 = 0x23000110,
        .applicationObject4 = 0x23010110,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A04_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x04,
        .applicationObject1 = 0x22100110,
        .applicationObject2 = 0x22100210,
        .applicationObject3 = 0x22100310,
        .applicationObject4 = 0x22100410,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A05_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x04,
        .applicationObject1 = 0x22110110,
        .applicationObject2 = 0x22110210,
        .applicationObject3 = 0x22110310,
        .applicationObject4 = 0x22110410,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A06_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x05,
        .applicationObject1 = 0x22120110,
        .applicationObject2 = 0x22120210,
        .applicationObject3 = 0x22120310,
        .applicationObject4 = 0x22120408,
        .applicationObject5 = 0x22120508,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1A07_TPDOMappingParameter = {
        .numberOfMappedApplicationObjectsInPDO = 0x03,
        .applicationObject1 = 0x21010110,
        .applicationObject2 = 0x21010310,
        .applicationObject3 = 0x21010710,
        .applicationObject4 = 0x00000000,
        .applicationObject5 = 0x00000000,
        .applicationObject6 = 0x00000000,
        .applicationObject7 = 0x00000000,
        .applicationObject8 = 0x00000000
    },
    .x1F80_NMTStartup = 0x00000000
};

OD_ATTR_PERSIST_APP OD_PERSIST_APP_t OD_PERSIST_APP = {
    .x2701_authorityPolicy = {
        .highestSub_indexSupported = 0x04,
        .allowedEnableTransports = 0x01,
        .wirelessDeadman = 0x00C8,
        .requireMasterLiveness = true,
        .wirelessService = false
    },
    .x6007_abortConnectionOptionCode = 1,
    .x605A_quickStopOptionCode = 2,
    .x605B_shutdownOptionCode = 0,
    .x605C_disableOperationOptionCode = 1,
    .x605D_haltOptionCode = 1,
    .x605E_faultReactionOptionCode = 0,
    .x6065_followingErrorWindow = 0xFFFFFFFF,
    .x6066_followingErrorTimeOut = 0x0000,
    .x6067_positionWindow = 0xFFFFFFFF,
    .x6068_positionWindowTime = 0x0000,
    .x606D_velocityWindow = 0x0000,
    .x606E_velocityWindowTime = 0x0000,
    .x6072_maxTorque = 0x03E8,
    .x6073_maxCurrent = 0x03E8,
    .x6075_motorRatedCurrent = 0x00000000,
    .x6076_motorRatedTorque = 0x00000000,
    .x607C_homeOffset = 0,
    .x607D_softwarePositionLimit_sub0 = 0x02,
    .x607D_softwarePositionLimit = {0, 0},
    .x607E_polarity = 0x00,
    .x607F_maxProfileVelocity = 0x00000000,
    .x6080_maxMotorSpeed = 0x00000000,
    .x6081_profileVelocity = 0x00000000,
    .x6083_profileAcceleration = 0x00000000,
    .x6084_profileDeceleration = 0x00000000,
    .x6085_quickStopDeceleration = 0x00000000,
    .x6086_motionProfileType = 0,
    .x6087_torqueSlope = 0x00000000,
    .x608F_positionEncoderResolution_sub0 = 0x02,
    .x608F_positionEncoderResolution = {0x00000001, 0x00000001},
    .x6091_gearRatio_sub0 = 0x02,
    .x6091_gearRatio = {0x00000001, 0x00000001},
    .x6092_feedConstant_sub0 = 0x02,
    .x6092_feedConstant = {0x00000001, 0x00000001},
    .x6098_homingMethod = 0,
    .x6099_homingSpeeds_sub0 = 0x02,
    .x6099_homingSpeeds = {0x00000000, 0x00000000},
    .x609A_homingAcceleration = 0x00000000,
    .x60E0_positiveTorqueLimitValue = 0x03E8,
    .x60E1_negativeTorqueLimitValue = 0x03E8,
    .x6402_motorType = 0x0000,
    .x6403_motorCatalogueNumber = "",
    .x6404_motorManufacturer = ""
};



/*******************************************************************************
    All OD objects (constant definitions)
*******************************************************************************/
typedef struct {
    OD_obj_var_t o_1000_deviceType;
    OD_obj_var_t o_1001_errorRegister;
    OD_obj_array_t o_1003_pre_definedErrorField;
    OD_obj_var_t o_1005_COB_ID_SYNCMessage;
    OD_obj_var_t o_1006_communicationCyclePeriod;
    OD_obj_var_t o_1007_synchronousWindowLength;
    OD_obj_var_t o_1008_manufacturerDeviceName;
    OD_obj_var_t o_1009_manufacturerHardwareVersion;
    OD_obj_var_t o_100A_manufacturerSoftwareVersion;
    OD_obj_array_t o_1010_storeParameters;
    OD_obj_array_t o_1011_restoreDefaultParameters;
    OD_obj_var_t o_1012_COB_IDTimeStampObject;
    OD_obj_var_t o_1014_COB_ID_EMCY;
    OD_obj_var_t o_1015_inhibitTimeEMCY;
    OD_obj_array_t o_1016_consumerHeartbeatTime;
    OD_obj_var_t o_1017_producerHeartbeatTime;
    OD_obj_record_t o_1018_identity[5];
    OD_obj_var_t o_1019_synchronousCounterOverflowValue;
    OD_obj_array_t o_1029_errorBehavior;
    OD_obj_record_t o_1200_SDOServerParameter[3];
    OD_obj_record_t o_1201_SDOServerParameter2[3];
    OD_obj_record_t o_1280_SDOClientParameter[4];
    OD_obj_record_t o_1400_RPDOCommunicationParameter[4];
    OD_obj_record_t o_1401_RPDOCommunicationParameter[4];
    OD_obj_record_t o_1402_RPDOCommunicationParameter[4];
    OD_obj_record_t o_1403_RPDOCommunicationParameter[4];
    OD_obj_record_t o_1600_RPDOMappingParameter[9];
    OD_obj_record_t o_1601_RPDOMappingParameter[9];
    OD_obj_record_t o_1602_RPDOMappingParameter[9];
    OD_obj_record_t o_1603_RPDOMappingParameter[9];
    OD_obj_record_t o_1800_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1801_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1802_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1803_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1804_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1805_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1806_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1807_TPDOCommunicationParameter[6];
    OD_obj_record_t o_1A00_TPDOMappingParameter[9];
    OD_obj_record_t o_1A01_TPDOMappingParameter[9];
    OD_obj_record_t o_1A02_TPDOMappingParameter[9];
    OD_obj_record_t o_1A03_TPDOMappingParameter[9];
    OD_obj_record_t o_1A04_TPDOMappingParameter[9];
    OD_obj_record_t o_1A05_TPDOMappingParameter[9];
    OD_obj_record_t o_1A06_TPDOMappingParameter[9];
    OD_obj_record_t o_1A07_TPDOMappingParameter[9];
    OD_obj_var_t o_1F80_NMTStartup;
    OD_obj_record_t o_2000_firmwareBuild[10];
    OD_obj_record_t o_2001_boardInfo[6];
    OD_obj_record_t o_2002_componentVersions[5];
    OD_obj_record_t o_2101_encoderStatus[9];
    OD_obj_record_t o_2210_IMUPackedQuaternion[5];
    OD_obj_record_t o_2211_IMUPackedAcceleration[5];
    OD_obj_record_t o_2212_IMUPackedGyro[6];
    OD_obj_record_t o_2300_temperatures[5];
    OD_obj_record_t o_2301_power[4];
    OD_obj_record_t o_2700_controlAuthority[3];
    OD_obj_record_t o_2701_authorityPolicy[5];
    OD_obj_record_t o_2800_parameterStoreMeta[6];
    OD_obj_record_t o_4000_faultJournalHeader[5];
    OD_obj_var_t o_4001_faultJournal;
    OD_obj_var_t o_4002_faultJournalClear;
    OD_obj_record_t o_4003_activeCauses[3];
    OD_obj_record_t o_4200_threadStats[11];
    OD_obj_record_t o_4201_timingHistogram[17];
    OD_obj_record_t o_4202_CANStats[11];
    OD_obj_var_t o_5FFE_ODLayoutHash;
    OD_obj_var_t o_5FFF_manufacturerODVersion;
    OD_obj_var_t o_6007_abortConnectionOptionCode;
    OD_obj_var_t o_603F_errorCode;
    OD_obj_var_t o_6040_controlword;
    OD_obj_var_t o_6041_statusword;
    OD_obj_var_t o_605A_quickStopOptionCode;
    OD_obj_var_t o_605B_shutdownOptionCode;
    OD_obj_var_t o_605C_disableOperationOptionCode;
    OD_obj_var_t o_605D_haltOptionCode;
    OD_obj_var_t o_605E_faultReactionOptionCode;
    OD_obj_var_t o_6060_modesOfOperation;
    OD_obj_var_t o_6061_modesOfOperationDisplay;
    OD_obj_var_t o_6062_positionDemandValue;
    OD_obj_var_t o_6064_positionActualValue;
    OD_obj_var_t o_6065_followingErrorWindow;
    OD_obj_var_t o_6066_followingErrorTimeOut;
    OD_obj_var_t o_6067_positionWindow;
    OD_obj_var_t o_6068_positionWindowTime;
    OD_obj_var_t o_606B_velocityDemandValue;
    OD_obj_var_t o_606C_velocityActualValue;
    OD_obj_var_t o_606D_velocityWindow;
    OD_obj_var_t o_606E_velocityWindowTime;
    OD_obj_var_t o_6071_targetTorque;
    OD_obj_var_t o_6072_maxTorque;
    OD_obj_var_t o_6073_maxCurrent;
    OD_obj_var_t o_6075_motorRatedCurrent;
    OD_obj_var_t o_6076_motorRatedTorque;
    OD_obj_var_t o_6077_torqueActualValue;
    OD_obj_var_t o_6078_currentActualValue;
    OD_obj_var_t o_6079_DCLinkCircuitVoltage;
    OD_obj_var_t o_607A_targetPosition;
    OD_obj_var_t o_607C_homeOffset;
    OD_obj_array_t o_607D_softwarePositionLimit;
    OD_obj_var_t o_607E_polarity;
    OD_obj_var_t o_607F_maxProfileVelocity;
    OD_obj_var_t o_6080_maxMotorSpeed;
    OD_obj_var_t o_6081_profileVelocity;
    OD_obj_var_t o_6083_profileAcceleration;
    OD_obj_var_t o_6084_profileDeceleration;
    OD_obj_var_t o_6085_quickStopDeceleration;
    OD_obj_var_t o_6086_motionProfileType;
    OD_obj_var_t o_6087_torqueSlope;
    OD_obj_array_t o_608F_positionEncoderResolution;
    OD_obj_array_t o_6091_gearRatio;
    OD_obj_array_t o_6092_feedConstant;
    OD_obj_var_t o_6098_homingMethod;
    OD_obj_array_t o_6099_homingSpeeds;
    OD_obj_var_t o_609A_homingAcceleration;
    OD_obj_var_t o_60B0_positionOffset;
    OD_obj_var_t o_60B1_velocityOffset;
    OD_obj_var_t o_60B2_torqueOffset;
    OD_obj_record_t o_60C2_interpolationTimePeriod[3];
    OD_obj_var_t o_60E0_positiveTorqueLimitValue;
    OD_obj_var_t o_60E1_negativeTorqueLimitValue;
    OD_obj_array_t o_60E4_additionalPositionActualValue;
    OD_obj_var_t o_60F4_followingErrorActualValue;
    OD_obj_var_t o_60FD_digitalInputs;
    OD_obj_array_t o_60FE_digitalOutputs;
    OD_obj_var_t o_60FF_targetVelocity;
    OD_obj_var_t o_6402_motorType;
    OD_obj_var_t o_6403_motorCatalogueNumber;
    OD_obj_var_t o_6404_motorManufacturer;
    OD_obj_var_t o_6502_supportedDriveModes;
} ODObjs_t;

static CO_PROGMEM ODObjs_t ODObjs = {
    .o_1000_deviceType = {
        .dataOrig = &OD_RAM.x1000_deviceType,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataLength = 4
    },
    .o_1001_errorRegister = {
        .dataOrig = &OD_RAM.x1001_errorRegister,
        .attribute = ODA_SDO_R | ODA_TPDO,
        .dataLength = 1
    },
    .o_1003_pre_definedErrorField = {
        .dataOrig0 = NULL,
        .dataOrig = NULL,
        .attribute0 = ODA_SDO_RW,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_1005_COB_ID_SYNCMessage = {
        .dataOrig = &OD_PERSIST_COMM.x1005_COB_ID_SYNCMessage,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_1006_communicationCyclePeriod = {
        .dataOrig = &OD_PERSIST_COMM.x1006_communicationCyclePeriod,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_1007_synchronousWindowLength = {
        .dataOrig = &OD_PERSIST_COMM.x1007_synchronousWindowLength,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_1008_manufacturerDeviceName = {
        .dataOrig = &OD_RAM.x1008_manufacturerDeviceName[0],
        .attribute = ODA_SDO_R | ODA_STR,
        .dataLength = 17
    },
    .o_1009_manufacturerHardwareVersion = {
        .dataOrig = &OD_RAM.x1009_manufacturerHardwareVersion[0],
        .attribute = ODA_SDO_R | ODA_STR,
        .dataLength = 16
    },
    .o_100A_manufacturerSoftwareVersion = {
        .dataOrig = &OD_RAM.x100A_manufacturerSoftwareVersion[0],
        .attribute = ODA_SDO_R | ODA_STR,
        .dataLength = 48
    },
    .o_1010_storeParameters = {
        .dataOrig0 = &OD_RAM.x1010_storeParameters_sub0,
        .dataOrig = &OD_RAM.x1010_storeParameters[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_1011_restoreDefaultParameters = {
        .dataOrig0 = &OD_RAM.x1011_restoreDefaultParameters_sub0,
        .dataOrig = &OD_RAM.x1011_restoreDefaultParameters[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_1012_COB_IDTimeStampObject = {
        .dataOrig = &OD_PERSIST_COMM.x1012_COB_IDTimeStampObject,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_1014_COB_ID_EMCY = {
        .dataOrig = &OD_PERSIST_COMM.x1014_COB_ID_EMCY,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_1015_inhibitTimeEMCY = {
        .dataOrig = &OD_PERSIST_COMM.x1015_inhibitTimeEMCY,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_1016_consumerHeartbeatTime = {
        .dataOrig0 = &OD_PERSIST_COMM.x1016_consumerHeartbeatTime_sub0,
        .dataOrig = &OD_PERSIST_COMM.x1016_consumerHeartbeatTime[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_1017_producerHeartbeatTime = {
        .dataOrig = &OD_PERSIST_COMM.x1017_producerHeartbeatTime,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_1018_identity = {
        {
            .dataOrig = &OD_RAM.x1018_identity.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x1018_identity.vendor_ID,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1018_identity.productCode,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1018_identity.revisionNumber,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1018_identity.serialNumber,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1019_synchronousCounterOverflowValue = {
        .dataOrig = &OD_PERSIST_COMM.x1019_synchronousCounterOverflowValue,
        .attribute = ODA_SDO_RW,
        .dataLength = 1
    },
    .o_1029_errorBehavior = {
        .dataOrig0 = &OD_PERSIST_COMM.x1029_errorBehavior_sub0,
        .dataOrig = &OD_PERSIST_COMM.x1029_errorBehavior[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW,
        .dataElementLength = 1,
        .dataElementSizeof = sizeof(uint8_t)
    },
    .o_1200_SDOServerParameter = {
        {
            .dataOrig = &OD_RAM.x1200_SDOServerParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x1200_SDOServerParameter.COB_IDClientToServerRx,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1200_SDOServerParameter.COB_IDServerToClientTx,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1201_SDOServerParameter2 = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1201_SDOServerParameter2.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1201_SDOServerParameter2.COB_IDClientToServerRx,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1201_SDOServerParameter2.COB_IDServerToClientTx,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1280_SDOClientParameter = {
        {
            .dataOrig = &OD_RAM.x1280_SDOClientParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x1280_SDOClientParameter.COB_IDClientToServerTx,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_TRPDO | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1280_SDOClientParameter.COB_IDServerToClientRx,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_TRPDO | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x1280_SDOClientParameter.node_IDOfTheSDOServer,
            .subIndex = 3,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1400_RPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.COB_IDUsedByRPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        }
    },
    .o_1401_RPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.COB_IDUsedByRPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        }
    },
    .o_1402_RPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1402_RPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1402_RPDOCommunicationParameter.COB_IDUsedByRPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1402_RPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1402_RPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        }
    },
    .o_1403_RPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1403_RPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1403_RPDOCommunicationParameter.COB_IDUsedByRPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1403_RPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1403_RPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        }
    },
    .o_1600_RPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1600_RPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1601_RPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1601_RPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1602_RPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1602_RPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1603_RPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1603_RPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1800_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1801_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1802_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1803_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1804_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1804_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1805_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1805_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1806_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1806_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1807_TPDOCommunicationParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.COB_IDUsedByTPDO,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.transmissionType,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.inhibitTime,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.eventTimer,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1807_TPDOCommunicationParameter.SYNCStartValue,
            .subIndex = 6,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_1A00_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A00_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A01_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A01_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A02_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A02_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A03_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A03_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A04_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A04_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A05_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A05_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A06_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A06_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1A07_TPDOMappingParameter = {
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.numberOfMappedApplicationObjectsInPDO,
            .subIndex = 0,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject1,
            .subIndex = 1,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject2,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject3,
            .subIndex = 3,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject4,
            .subIndex = 4,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject5,
            .subIndex = 5,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject6,
            .subIndex = 6,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject7,
            .subIndex = 7,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_PERSIST_COMM.x1A07_TPDOMappingParameter.applicationObject8,
            .subIndex = 8,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 4
        }
    },
    .o_1F80_NMTStartup = {
        .dataOrig = &OD_PERSIST_COMM.x1F80_NMTStartup,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_2000_firmwareBuild = {
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.applicationVersion[0],
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 32
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.HALGitSHA[0],
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.coreGitSHA[0],
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.TMC9660DriverGitSHA[0],
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.AS5047UDriverGitSHA[0],
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.BNO08xDriverGitSHA[0],
            .subIndex = 6,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.CANopenNodeGitSHA[0],
            .subIndex = 7,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 40
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.buildTime,
            .subIndex = 8,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2000_firmwareBuild.IDFVersion[0],
            .subIndex = 9,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 32
        }
    },
    .o_2001_boardInfo = {
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.hardwareRevision,
            .subIndex = 1,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.MACAddress[0],
            .subIndex = 2,
            .attribute = ODA_SDO_R,
            .dataLength = 6
        },
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.resetReason,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.uptime,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2001_boardInfo.freeHeap,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_2002_componentVersions = {
        {
            .dataOrig = &OD_RAM.x2002_componentVersions.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2002_componentVersions.TMC9660FirmwareId,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2002_componentVersions.TMC9660BootloaderId,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2002_componentVersions.BNO08xSH_2Version[0],
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_STR,
            .dataLength = 24
        },
        {
            .dataOrig = &OD_RAM.x2002_componentVersions.AS5047UId,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        }
    },
    .o_2101_encoderStatus = {
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.angleRaw,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.angleCompensated,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.velocityRaw,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.multi_turnPosition,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.AGC,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_TPDO,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.magnitude,
            .subIndex = 6,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.errorFlags,
            .subIndex = 7,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2101_encoderStatus.communicationErrors,
            .subIndex = 8,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        }
    },
    .o_2210_IMUPackedQuaternion = {
        {
            .dataOrig = &OD_RAM.x2210_IMUPackedQuaternion.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2210_IMUPackedQuaternion.quaternionI,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2210_IMUPackedQuaternion.quaternionJ,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2210_IMUPackedQuaternion.quaternionK,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2210_IMUPackedQuaternion.quaternionReal,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        }
    },
    .o_2211_IMUPackedAcceleration = {
        {
            .dataOrig = &OD_RAM.x2211_IMUPackedAcceleration.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2211_IMUPackedAcceleration.accelerationX,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2211_IMUPackedAcceleration.accelerationY,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2211_IMUPackedAcceleration.accelerationZ,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2211_IMUPackedAcceleration.sampleCounter,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        }
    },
    .o_2212_IMUPackedGyro = {
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.angularRateX,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.angularRateY,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.angularRateZ,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.calibrationStatus,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_TPDO,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2212_IMUPackedGyro.accuracy,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_TPDO,
            .dataLength = 1
        }
    },
    .o_2300_temperatures = {
        {
            .dataOrig = &OD_RAM.x2300_temperatures.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2300_temperatures.powerStageNTC,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2300_temperatures.TMC9660Chip,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2300_temperatures.ESP32Die,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2300_temperatures.motor,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        }
    },
    .o_2301_power = {
        {
            .dataOrig = &OD_RAM.x2301_power.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2301_power.supplyVoltage,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2301_power.driverVoltage,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x2301_power.powerGood,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_TPDO,
            .dataLength = 1
        }
    },
    .o_2700_controlAuthority = {
        {
            .dataOrig = &OD_RAM.x2700_controlAuthority.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2700_controlAuthority.currentOwner,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2700_controlAuthority.request,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_2701_authorityPolicy = {
        {
            .dataOrig = &OD_PERSIST_APP.x2701_authorityPolicy.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_APP.x2701_authorityPolicy.allowedEnableTransports,
            .subIndex = 1,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_APP.x2701_authorityPolicy.wirelessDeadman,
            .subIndex = 2,
            .attribute = ODA_SDO_RW | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_PERSIST_APP.x2701_authorityPolicy.requireMasterLiveness,
            .subIndex = 3,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_PERSIST_APP.x2701_authorityPolicy.wirelessService,
            .subIndex = 4,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_2800_parameterStoreMeta = {
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.storeCount,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.lastStoreResult,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.ODHashAtLastStore,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.recordsAppliedAtRestore,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x2800_parameterStoreMeta.recordsDroppedAtRestore,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_4000_faultJournalHeader = {
        {
            .dataOrig = &OD_RAM.x4000_faultJournalHeader.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4000_faultJournalHeader.entryCount,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x4000_faultJournalHeader.head,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x4000_faultJournalHeader.capacity,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x4000_faultJournalHeader.overflowCount,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_4001_faultJournal = {
        .dataOrig = NULL,
        .attribute = ODA_SDO_R,
        .dataLength = 0
    },
    .o_4002_faultJournalClear = {
        .dataOrig = &OD_RAM.x4002_faultJournalClear,
        .attribute = ODA_SDO_W | ODA_MB,
        .dataLength = 4
    },
    .o_4003_activeCauses = {
        {
            .dataOrig = &OD_RAM.x4003_activeCauses.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4003_activeCauses.activeCauses,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4003_activeCauses.escalatedCauses,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
            .dataLength = 4
        }
    },
    .o_4200_threadStats = {
        {
            .dataOrig = &OD_RAM.x4200_threadStats.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.driveRtLastStep,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.driveRtMaxStep,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.driveRtOverruns,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.coSvcLastStep,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.coSvcMaxStep,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.coSvcOverruns,
            .subIndex = 6,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.i2cIoLastStep,
            .subIndex = 7,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.i2cIoMaxStep,
            .subIndex = 8,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.i2cIoOverruns,
            .subIndex = 9,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4200_threadStats.livenessStaleMask,
            .subIndex = 10,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_4201_timingHistogram = {
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo10Us,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo20Us,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo50Us,
            .subIndex = 3,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo100Us,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo200Us,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo500Us,
            .subIndex = 6,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterUpTo1000Us,
            .subIndex = 7,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.jitterAbove1000Us,
            .subIndex = 8,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo100Us,
            .subIndex = 9,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo150Us,
            .subIndex = 10,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo200Us,
            .subIndex = 11,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo250Us,
            .subIndex = 12,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo300Us,
            .subIndex = 13,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo500Us,
            .subIndex = 14,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepUpTo1000Us,
            .subIndex = 15,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4201_timingHistogram.stepAbove1000Us,
            .subIndex = 16,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_4202_CANStats = {
        {
            .dataOrig = &OD_RAM.x4202_CANStats.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.TEC,
            .subIndex = 1,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.REC,
            .subIndex = 2,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 2
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.busState,
            .subIndex = 3,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.bus_offCount,
            .subIndex = 4,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.RXOverruns,
            .subIndex = 5,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.TXQueueFull,
            .subIndex = 6,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.routerDropsCAN,
            .subIndex = 7,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.routerDropsWi_Fi,
            .subIndex = 8,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.routerDropsESP_NOW,
            .subIndex = 9,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        },
        {
            .dataOrig = &OD_RAM.x4202_CANStats.routerDropsHost,
            .subIndex = 10,
            .attribute = ODA_SDO_R | ODA_MB,
            .dataLength = 4
        }
    },
    .o_5FFE_ODLayoutHash = {
        .dataOrig = &OD_RAM.x5FFE_ODLayoutHash,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataLength = 4
    },
    .o_5FFF_manufacturerODVersion = {
        .dataOrig = &OD_RAM.x5FFF_manufacturerODVersion,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataLength = 2
    },
    .o_6007_abortConnectionOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x6007_abortConnectionOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_603F_errorCode = {
        .dataOrig = &OD_RAM.x603F_errorCode,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 2
    },
    .o_6040_controlword = {
        .dataOrig = &OD_RAM.x6040_controlword,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 2
    },
    .o_6041_statusword = {
        .dataOrig = &OD_RAM.x6041_statusword,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 2
    },
    .o_605A_quickStopOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x605A_quickStopOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_605B_shutdownOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x605B_shutdownOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_605C_disableOperationOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x605C_disableOperationOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_605D_haltOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x605D_haltOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_605E_faultReactionOptionCode = {
        .dataOrig = &OD_PERSIST_APP.x605E_faultReactionOptionCode,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6060_modesOfOperation = {
        .dataOrig = &OD_RAM.x6060_modesOfOperation,
        .attribute = ODA_SDO_RW | ODA_RPDO,
        .dataLength = 1
    },
    .o_6061_modesOfOperationDisplay = {
        .dataOrig = &OD_RAM.x6061_modesOfOperationDisplay,
        .attribute = ODA_SDO_R | ODA_TPDO,
        .dataLength = 1
    },
    .o_6062_positionDemandValue = {
        .dataOrig = &OD_RAM.x6062_positionDemandValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_6064_positionActualValue = {
        .dataOrig = &OD_RAM.x6064_positionActualValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_6065_followingErrorWindow = {
        .dataOrig = &OD_PERSIST_APP.x6065_followingErrorWindow,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6066_followingErrorTimeOut = {
        .dataOrig = &OD_PERSIST_APP.x6066_followingErrorTimeOut,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6067_positionWindow = {
        .dataOrig = &OD_PERSIST_APP.x6067_positionWindow,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6068_positionWindowTime = {
        .dataOrig = &OD_PERSIST_APP.x6068_positionWindowTime,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_606B_velocityDemandValue = {
        .dataOrig = &OD_RAM.x606B_velocityDemandValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_606C_velocityActualValue = {
        .dataOrig = &OD_RAM.x606C_velocityActualValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_606D_velocityWindow = {
        .dataOrig = &OD_PERSIST_APP.x606D_velocityWindow,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_606E_velocityWindowTime = {
        .dataOrig = &OD_PERSIST_APP.x606E_velocityWindowTime,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6071_targetTorque = {
        .dataOrig = &OD_RAM.x6071_targetTorque,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 2
    },
    .o_6072_maxTorque = {
        .dataOrig = &OD_PERSIST_APP.x6072_maxTorque,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6073_maxCurrent = {
        .dataOrig = &OD_PERSIST_APP.x6073_maxCurrent,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6075_motorRatedCurrent = {
        .dataOrig = &OD_PERSIST_APP.x6075_motorRatedCurrent,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6076_motorRatedTorque = {
        .dataOrig = &OD_PERSIST_APP.x6076_motorRatedTorque,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6077_torqueActualValue = {
        .dataOrig = &OD_RAM.x6077_torqueActualValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 2
    },
    .o_6078_currentActualValue = {
        .dataOrig = &OD_RAM.x6078_currentActualValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 2
    },
    .o_6079_DCLinkCircuitVoltage = {
        .dataOrig = &OD_RAM.x6079_DCLinkCircuitVoltage,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_607A_targetPosition = {
        .dataOrig = &OD_RAM.x607A_targetPosition,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 4
    },
    .o_607C_homeOffset = {
        .dataOrig = &OD_PERSIST_APP.x607C_homeOffset,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_607D_softwarePositionLimit = {
        .dataOrig0 = &OD_PERSIST_APP.x607D_softwarePositionLimit_sub0,
        .dataOrig = &OD_PERSIST_APP.x607D_softwarePositionLimit[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(int32_t)
    },
    .o_607E_polarity = {
        .dataOrig = &OD_PERSIST_APP.x607E_polarity,
        .attribute = ODA_SDO_RW,
        .dataLength = 1
    },
    .o_607F_maxProfileVelocity = {
        .dataOrig = &OD_PERSIST_APP.x607F_maxProfileVelocity,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6080_maxMotorSpeed = {
        .dataOrig = &OD_PERSIST_APP.x6080_maxMotorSpeed,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6081_profileVelocity = {
        .dataOrig = &OD_PERSIST_APP.x6081_profileVelocity,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6083_profileAcceleration = {
        .dataOrig = &OD_PERSIST_APP.x6083_profileAcceleration,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6084_profileDeceleration = {
        .dataOrig = &OD_PERSIST_APP.x6084_profileDeceleration,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6085_quickStopDeceleration = {
        .dataOrig = &OD_PERSIST_APP.x6085_quickStopDeceleration,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_6086_motionProfileType = {
        .dataOrig = &OD_PERSIST_APP.x6086_motionProfileType,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6087_torqueSlope = {
        .dataOrig = &OD_PERSIST_APP.x6087_torqueSlope,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_608F_positionEncoderResolution = {
        .dataOrig0 = &OD_PERSIST_APP.x608F_positionEncoderResolution_sub0,
        .dataOrig = &OD_PERSIST_APP.x608F_positionEncoderResolution[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_6091_gearRatio = {
        .dataOrig0 = &OD_PERSIST_APP.x6091_gearRatio_sub0,
        .dataOrig = &OD_PERSIST_APP.x6091_gearRatio[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_6092_feedConstant = {
        .dataOrig0 = &OD_PERSIST_APP.x6092_feedConstant_sub0,
        .dataOrig = &OD_PERSIST_APP.x6092_feedConstant[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_6098_homingMethod = {
        .dataOrig = &OD_PERSIST_APP.x6098_homingMethod,
        .attribute = ODA_SDO_RW,
        .dataLength = 1
    },
    .o_6099_homingSpeeds = {
        .dataOrig0 = &OD_PERSIST_APP.x6099_homingSpeeds_sub0,
        .dataOrig = &OD_PERSIST_APP.x6099_homingSpeeds[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_609A_homingAcceleration = {
        .dataOrig = &OD_PERSIST_APP.x609A_homingAcceleration,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 4
    },
    .o_60B0_positionOffset = {
        .dataOrig = &OD_RAM.x60B0_positionOffset,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 4
    },
    .o_60B1_velocityOffset = {
        .dataOrig = &OD_RAM.x60B1_velocityOffset,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 4
    },
    .o_60B2_torqueOffset = {
        .dataOrig = &OD_RAM.x60B2_torqueOffset,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 2
    },
    .o_60C2_interpolationTimePeriod = {
        {
            .dataOrig = &OD_RAM.x60C2_interpolationTimePeriod.highestSub_indexSupported,
            .subIndex = 0,
            .attribute = ODA_SDO_R,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x60C2_interpolationTimePeriod.interpolationTimePeriodValue,
            .subIndex = 1,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        },
        {
            .dataOrig = &OD_RAM.x60C2_interpolationTimePeriod.interpolationTimeIndex,
            .subIndex = 2,
            .attribute = ODA_SDO_RW,
            .dataLength = 1
        }
    },
    .o_60E0_positiveTorqueLimitValue = {
        .dataOrig = &OD_PERSIST_APP.x60E0_positiveTorqueLimitValue,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_60E1_negativeTorqueLimitValue = {
        .dataOrig = &OD_PERSIST_APP.x60E1_negativeTorqueLimitValue,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_60E4_additionalPositionActualValue = {
        .dataOrig0 = &OD_RAM.x60E4_additionalPositionActualValue_sub0,
        .dataOrig = &OD_RAM.x60E4_additionalPositionActualValue[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(int32_t)
    },
    .o_60F4_followingErrorActualValue = {
        .dataOrig = &OD_RAM.x60F4_followingErrorActualValue,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_60FD_digitalInputs = {
        .dataOrig = &OD_RAM.x60FD_digitalInputs,
        .attribute = ODA_SDO_R | ODA_TPDO | ODA_MB,
        .dataLength = 4
    },
    .o_60FE_digitalOutputs = {
        .dataOrig0 = &OD_RAM.x60FE_digitalOutputs_sub0,
        .dataOrig = &OD_RAM.x60FE_digitalOutputs[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t)
    },
    .o_60FF_targetVelocity = {
        .dataOrig = &OD_RAM.x60FF_targetVelocity,
        .attribute = ODA_SDO_RW | ODA_RPDO | ODA_MB,
        .dataLength = 4
    },
    .o_6402_motorType = {
        .dataOrig = &OD_PERSIST_APP.x6402_motorType,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataLength = 2
    },
    .o_6403_motorCatalogueNumber = {
        .dataOrig = &OD_PERSIST_APP.x6403_motorCatalogueNumber[0],
        .attribute = ODA_SDO_RW | ODA_STR,
        .dataLength = 32
    },
    .o_6404_motorManufacturer = {
        .dataOrig = &OD_PERSIST_APP.x6404_motorManufacturer[0],
        .attribute = ODA_SDO_RW | ODA_STR,
        .dataLength = 32
    },
    .o_6502_supportedDriveModes = {
        .dataOrig = &OD_RAM.x6502_supportedDriveModes,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataLength = 4
    }
};


/*******************************************************************************
    Object dictionary
*******************************************************************************/
static OD_ATTR_OD OD_entry_t ODList[] = {
    {0x1000, 0x01, ODT_VAR, &ODObjs.o_1000_deviceType, NULL},
    {0x1001, 0x01, ODT_VAR, &ODObjs.o_1001_errorRegister, NULL},
    {0x1003, 0x11, ODT_ARR, &ODObjs.o_1003_pre_definedErrorField, NULL},
    {0x1005, 0x01, ODT_VAR, &ODObjs.o_1005_COB_ID_SYNCMessage, NULL},
    {0x1006, 0x01, ODT_VAR, &ODObjs.o_1006_communicationCyclePeriod, NULL},
    {0x1007, 0x01, ODT_VAR, &ODObjs.o_1007_synchronousWindowLength, NULL},
    {0x1008, 0x01, ODT_VAR, &ODObjs.o_1008_manufacturerDeviceName, NULL},
    {0x1009, 0x01, ODT_VAR, &ODObjs.o_1009_manufacturerHardwareVersion, NULL},
    {0x100A, 0x01, ODT_VAR, &ODObjs.o_100A_manufacturerSoftwareVersion, NULL},
    {0x1010, 0x05, ODT_ARR, &ODObjs.o_1010_storeParameters, NULL},
    {0x1011, 0x05, ODT_ARR, &ODObjs.o_1011_restoreDefaultParameters, NULL},
    {0x1012, 0x01, ODT_VAR, &ODObjs.o_1012_COB_IDTimeStampObject, NULL},
    {0x1014, 0x01, ODT_VAR, &ODObjs.o_1014_COB_ID_EMCY, NULL},
    {0x1015, 0x01, ODT_VAR, &ODObjs.o_1015_inhibitTimeEMCY, NULL},
    {0x1016, 0x05, ODT_ARR, &ODObjs.o_1016_consumerHeartbeatTime, NULL},
    {0x1017, 0x01, ODT_VAR, &ODObjs.o_1017_producerHeartbeatTime, NULL},
    {0x1018, 0x05, ODT_REC, &ODObjs.o_1018_identity, NULL},
    {0x1019, 0x01, ODT_VAR, &ODObjs.o_1019_synchronousCounterOverflowValue, NULL},
    {0x1029, 0x03, ODT_ARR, &ODObjs.o_1029_errorBehavior, NULL},
    {0x1200, 0x03, ODT_REC, &ODObjs.o_1200_SDOServerParameter, NULL},
    {0x1201, 0x03, ODT_REC, &ODObjs.o_1201_SDOServerParameter2, NULL},
    {0x1280, 0x04, ODT_REC, &ODObjs.o_1280_SDOClientParameter, NULL},
    {0x1400, 0x04, ODT_REC, &ODObjs.o_1400_RPDOCommunicationParameter, NULL},
    {0x1401, 0x04, ODT_REC, &ODObjs.o_1401_RPDOCommunicationParameter, NULL},
    {0x1402, 0x04, ODT_REC, &ODObjs.o_1402_RPDOCommunicationParameter, NULL},
    {0x1403, 0x04, ODT_REC, &ODObjs.o_1403_RPDOCommunicationParameter, NULL},
    {0x1600, 0x09, ODT_REC, &ODObjs.o_1600_RPDOMappingParameter, NULL},
    {0x1601, 0x09, ODT_REC, &ODObjs.o_1601_RPDOMappingParameter, NULL},
    {0x1602, 0x09, ODT_REC, &ODObjs.o_1602_RPDOMappingParameter, NULL},
    {0x1603, 0x09, ODT_REC, &ODObjs.o_1603_RPDOMappingParameter, NULL},
    {0x1800, 0x06, ODT_REC, &ODObjs.o_1800_TPDOCommunicationParameter, NULL},
    {0x1801, 0x06, ODT_REC, &ODObjs.o_1801_TPDOCommunicationParameter, NULL},
    {0x1802, 0x06, ODT_REC, &ODObjs.o_1802_TPDOCommunicationParameter, NULL},
    {0x1803, 0x06, ODT_REC, &ODObjs.o_1803_TPDOCommunicationParameter, NULL},
    {0x1804, 0x06, ODT_REC, &ODObjs.o_1804_TPDOCommunicationParameter, NULL},
    {0x1805, 0x06, ODT_REC, &ODObjs.o_1805_TPDOCommunicationParameter, NULL},
    {0x1806, 0x06, ODT_REC, &ODObjs.o_1806_TPDOCommunicationParameter, NULL},
    {0x1807, 0x06, ODT_REC, &ODObjs.o_1807_TPDOCommunicationParameter, NULL},
    {0x1A00, 0x09, ODT_REC, &ODObjs.o_1A00_TPDOMappingParameter, NULL},
    {0x1A01, 0x09, ODT_REC, &ODObjs.o_1A01_TPDOMappingParameter, NULL},
    {0x1A02, 0x09, ODT_REC, &ODObjs.o_1A02_TPDOMappingParameter, NULL},
    {0x1A03, 0x09, ODT_REC, &ODObjs.o_1A03_TPDOMappingParameter, NULL},
    {0x1A04, 0x09, ODT_REC, &ODObjs.o_1A04_TPDOMappingParameter, NULL},
    {0x1A05, 0x09, ODT_REC, &ODObjs.o_1A05_TPDOMappingParameter, NULL},
    {0x1A06, 0x09, ODT_REC, &ODObjs.o_1A06_TPDOMappingParameter, NULL},
    {0x1A07, 0x09, ODT_REC, &ODObjs.o_1A07_TPDOMappingParameter, NULL},
    {0x1F80, 0x01, ODT_VAR, &ODObjs.o_1F80_NMTStartup, NULL},
    {0x2000, 0x0A, ODT_REC, &ODObjs.o_2000_firmwareBuild, NULL},
    {0x2001, 0x06, ODT_REC, &ODObjs.o_2001_boardInfo, NULL},
    {0x2002, 0x05, ODT_REC, &ODObjs.o_2002_componentVersions, NULL},
    {0x2101, 0x09, ODT_REC, &ODObjs.o_2101_encoderStatus, NULL},
    {0x2210, 0x05, ODT_REC, &ODObjs.o_2210_IMUPackedQuaternion, NULL},
    {0x2211, 0x05, ODT_REC, &ODObjs.o_2211_IMUPackedAcceleration, NULL},
    {0x2212, 0x06, ODT_REC, &ODObjs.o_2212_IMUPackedGyro, NULL},
    {0x2300, 0x05, ODT_REC, &ODObjs.o_2300_temperatures, NULL},
    {0x2301, 0x04, ODT_REC, &ODObjs.o_2301_power, NULL},
    {0x2700, 0x03, ODT_REC, &ODObjs.o_2700_controlAuthority, NULL},
    {0x2701, 0x05, ODT_REC, &ODObjs.o_2701_authorityPolicy, NULL},
    {0x2800, 0x06, ODT_REC, &ODObjs.o_2800_parameterStoreMeta, NULL},
    {0x4000, 0x05, ODT_REC, &ODObjs.o_4000_faultJournalHeader, NULL},
    {0x4001, 0x01, ODT_VAR, &ODObjs.o_4001_faultJournal, NULL},
    {0x4002, 0x01, ODT_VAR, &ODObjs.o_4002_faultJournalClear, NULL},
    {0x4003, 0x03, ODT_REC, &ODObjs.o_4003_activeCauses, NULL},
    {0x4200, 0x0B, ODT_REC, &ODObjs.o_4200_threadStats, NULL},
    {0x4201, 0x11, ODT_REC, &ODObjs.o_4201_timingHistogram, NULL},
    {0x4202, 0x0B, ODT_REC, &ODObjs.o_4202_CANStats, NULL},
    {0x5FFE, 0x01, ODT_VAR, &ODObjs.o_5FFE_ODLayoutHash, NULL},
    {0x5FFF, 0x01, ODT_VAR, &ODObjs.o_5FFF_manufacturerODVersion, NULL},
    {0x6007, 0x01, ODT_VAR, &ODObjs.o_6007_abortConnectionOptionCode, NULL},
    {0x603F, 0x01, ODT_VAR, &ODObjs.o_603F_errorCode, NULL},
    {0x6040, 0x01, ODT_VAR, &ODObjs.o_6040_controlword, NULL},
    {0x6041, 0x01, ODT_VAR, &ODObjs.o_6041_statusword, NULL},
    {0x605A, 0x01, ODT_VAR, &ODObjs.o_605A_quickStopOptionCode, NULL},
    {0x605B, 0x01, ODT_VAR, &ODObjs.o_605B_shutdownOptionCode, NULL},
    {0x605C, 0x01, ODT_VAR, &ODObjs.o_605C_disableOperationOptionCode, NULL},
    {0x605D, 0x01, ODT_VAR, &ODObjs.o_605D_haltOptionCode, NULL},
    {0x605E, 0x01, ODT_VAR, &ODObjs.o_605E_faultReactionOptionCode, NULL},
    {0x6060, 0x01, ODT_VAR, &ODObjs.o_6060_modesOfOperation, NULL},
    {0x6061, 0x01, ODT_VAR, &ODObjs.o_6061_modesOfOperationDisplay, NULL},
    {0x6062, 0x01, ODT_VAR, &ODObjs.o_6062_positionDemandValue, NULL},
    {0x6064, 0x01, ODT_VAR, &ODObjs.o_6064_positionActualValue, NULL},
    {0x6065, 0x01, ODT_VAR, &ODObjs.o_6065_followingErrorWindow, NULL},
    {0x6066, 0x01, ODT_VAR, &ODObjs.o_6066_followingErrorTimeOut, NULL},
    {0x6067, 0x01, ODT_VAR, &ODObjs.o_6067_positionWindow, NULL},
    {0x6068, 0x01, ODT_VAR, &ODObjs.o_6068_positionWindowTime, NULL},
    {0x606B, 0x01, ODT_VAR, &ODObjs.o_606B_velocityDemandValue, NULL},
    {0x606C, 0x01, ODT_VAR, &ODObjs.o_606C_velocityActualValue, NULL},
    {0x606D, 0x01, ODT_VAR, &ODObjs.o_606D_velocityWindow, NULL},
    {0x606E, 0x01, ODT_VAR, &ODObjs.o_606E_velocityWindowTime, NULL},
    {0x6071, 0x01, ODT_VAR, &ODObjs.o_6071_targetTorque, NULL},
    {0x6072, 0x01, ODT_VAR, &ODObjs.o_6072_maxTorque, NULL},
    {0x6073, 0x01, ODT_VAR, &ODObjs.o_6073_maxCurrent, NULL},
    {0x6075, 0x01, ODT_VAR, &ODObjs.o_6075_motorRatedCurrent, NULL},
    {0x6076, 0x01, ODT_VAR, &ODObjs.o_6076_motorRatedTorque, NULL},
    {0x6077, 0x01, ODT_VAR, &ODObjs.o_6077_torqueActualValue, NULL},
    {0x6078, 0x01, ODT_VAR, &ODObjs.o_6078_currentActualValue, NULL},
    {0x6079, 0x01, ODT_VAR, &ODObjs.o_6079_DCLinkCircuitVoltage, NULL},
    {0x607A, 0x01, ODT_VAR, &ODObjs.o_607A_targetPosition, NULL},
    {0x607C, 0x01, ODT_VAR, &ODObjs.o_607C_homeOffset, NULL},
    {0x607D, 0x03, ODT_ARR, &ODObjs.o_607D_softwarePositionLimit, NULL},
    {0x607E, 0x01, ODT_VAR, &ODObjs.o_607E_polarity, NULL},
    {0x607F, 0x01, ODT_VAR, &ODObjs.o_607F_maxProfileVelocity, NULL},
    {0x6080, 0x01, ODT_VAR, &ODObjs.o_6080_maxMotorSpeed, NULL},
    {0x6081, 0x01, ODT_VAR, &ODObjs.o_6081_profileVelocity, NULL},
    {0x6083, 0x01, ODT_VAR, &ODObjs.o_6083_profileAcceleration, NULL},
    {0x6084, 0x01, ODT_VAR, &ODObjs.o_6084_profileDeceleration, NULL},
    {0x6085, 0x01, ODT_VAR, &ODObjs.o_6085_quickStopDeceleration, NULL},
    {0x6086, 0x01, ODT_VAR, &ODObjs.o_6086_motionProfileType, NULL},
    {0x6087, 0x01, ODT_VAR, &ODObjs.o_6087_torqueSlope, NULL},
    {0x608F, 0x03, ODT_ARR, &ODObjs.o_608F_positionEncoderResolution, NULL},
    {0x6091, 0x03, ODT_ARR, &ODObjs.o_6091_gearRatio, NULL},
    {0x6092, 0x03, ODT_ARR, &ODObjs.o_6092_feedConstant, NULL},
    {0x6098, 0x01, ODT_VAR, &ODObjs.o_6098_homingMethod, NULL},
    {0x6099, 0x03, ODT_ARR, &ODObjs.o_6099_homingSpeeds, NULL},
    {0x609A, 0x01, ODT_VAR, &ODObjs.o_609A_homingAcceleration, NULL},
    {0x60B0, 0x01, ODT_VAR, &ODObjs.o_60B0_positionOffset, NULL},
    {0x60B1, 0x01, ODT_VAR, &ODObjs.o_60B1_velocityOffset, NULL},
    {0x60B2, 0x01, ODT_VAR, &ODObjs.o_60B2_torqueOffset, NULL},
    {0x60C2, 0x03, ODT_REC, &ODObjs.o_60C2_interpolationTimePeriod, NULL},
    {0x60E0, 0x01, ODT_VAR, &ODObjs.o_60E0_positiveTorqueLimitValue, NULL},
    {0x60E1, 0x01, ODT_VAR, &ODObjs.o_60E1_negativeTorqueLimitValue, NULL},
    {0x60E4, 0x02, ODT_ARR, &ODObjs.o_60E4_additionalPositionActualValue, NULL},
    {0x60F4, 0x01, ODT_VAR, &ODObjs.o_60F4_followingErrorActualValue, NULL},
    {0x60FD, 0x01, ODT_VAR, &ODObjs.o_60FD_digitalInputs, NULL},
    {0x60FE, 0x03, ODT_ARR, &ODObjs.o_60FE_digitalOutputs, NULL},
    {0x60FF, 0x01, ODT_VAR, &ODObjs.o_60FF_targetVelocity, NULL},
    {0x6402, 0x01, ODT_VAR, &ODObjs.o_6402_motorType, NULL},
    {0x6403, 0x01, ODT_VAR, &ODObjs.o_6403_motorCatalogueNumber, NULL},
    {0x6404, 0x01, ODT_VAR, &ODObjs.o_6404_motorManufacturer, NULL},
    {0x6502, 0x01, ODT_VAR, &ODObjs.o_6502_supportedDriveModes, NULL},
    {0x0000, 0x00, 0, NULL, NULL}
};

static OD_t _OD = {
    (sizeof(ODList) / sizeof(ODList[0])) - 1,
    &ODList[0]
};

OD_t *OD = &_OD;
