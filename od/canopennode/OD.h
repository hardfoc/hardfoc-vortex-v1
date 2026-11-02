/*******************************************************************************
    Vortex V1 object dictionary for CANopenNode V4

    Source: od/vortex_od.yaml, revision 1.0, OD hash 0xFF85661C.
    Do not edit: change the YAML and run `python3 tools/odgen`.
*******************************************************************************/

#ifndef OD_H
#define OD_H
/*******************************************************************************
    Counters of OD objects
*******************************************************************************/
#define OD_CNT_NMT 1
#define OD_CNT_EM 1
#define OD_CNT_SYNC 1
#define OD_CNT_SYNC_PROD 1
#define OD_CNT_STORAGE 1
#define OD_CNT_TIME 1
#define OD_CNT_EM_PROD 1
#define OD_CNT_HB_CONS 1
#define OD_CNT_HB_PROD 1
#define OD_CNT_SDO_SRV 2
#define OD_CNT_SDO_CLI 1
#define OD_CNT_RPDO 4
#define OD_CNT_TPDO 8


/*******************************************************************************
    Sizes of OD arrays
*******************************************************************************/
#define OD_CNT_ARR_1003 16
#define OD_CNT_ARR_1010 4
#define OD_CNT_ARR_1011 4
#define OD_CNT_ARR_1016 4
#define OD_CNT_ARR_1029 2
#define OD_CNT_ARR_607D 2
#define OD_CNT_ARR_608F 2
#define OD_CNT_ARR_6091 2
#define OD_CNT_ARR_6092 2
#define OD_CNT_ARR_6099 2
#define OD_CNT_ARR_60E4 1
#define OD_CNT_ARR_60FE 2


/*******************************************************************************
    OD data declaration of all groups
*******************************************************************************/
typedef struct {
    uint32_t x1000_deviceType;
    uint8_t x1001_errorRegister;
    char x1008_manufacturerDeviceName[18];
    char x1009_manufacturerHardwareVersion[17];
    char x100A_manufacturerSoftwareVersion[49];
    uint8_t x1010_storeParameters_sub0;
    uint32_t x1010_storeParameters[OD_CNT_ARR_1010];
    uint8_t x1011_restoreDefaultParameters_sub0;
    uint32_t x1011_restoreDefaultParameters[OD_CNT_ARR_1011];
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t vendor_ID;
        uint32_t productCode;
        uint32_t revisionNumber;
        uint32_t serialNumber;
    } x1018_identity;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDClientToServerRx;
        uint32_t COB_IDServerToClientTx;
    } x1200_SDOServerParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDClientToServerTx;
        uint32_t COB_IDServerToClientRx;
        uint8_t node_IDOfTheSDOServer;
    } x1280_SDOClientParameter;
    struct {
        uint8_t highestSub_indexSupported;
        char applicationVersion[33];
        char HALGitSHA[41];
        char coreGitSHA[41];
        char TMC9660DriverGitSHA[41];
        char AS5047UDriverGitSHA[41];
        char BNO08xDriverGitSHA[41];
        char CANopenNodeGitSHA[41];
        uint32_t buildTime;
        char IDFVersion[33];
    } x2000_firmwareBuild;
    struct {
        uint8_t highestSub_indexSupported;
        uint8_t hardwareRevision;
        uint8_t MACAddress[6];
        uint32_t resetReason;
        uint32_t uptime;
        uint32_t freeHeap;
    } x2001_boardInfo;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t TMC9660FirmwareId;
        uint32_t TMC9660BootloaderId;
        char BNO08xSH_2Version[25];
        uint16_t AS5047UId;
    } x2002_componentVersions;
    struct {
        uint8_t highestSub_indexSupported;
        uint16_t angleRaw;
        uint16_t angleCompensated;
        int16_t velocityRaw;
        int32_t multi_turnPosition;
        uint8_t AGC;
        uint16_t magnitude;
        uint16_t errorFlags;
        uint32_t communicationErrors;
    } x2101_encoderStatus;
    struct {
        uint8_t highestSub_indexSupported;
        int16_t quaternionI;
        int16_t quaternionJ;
        int16_t quaternionK;
        int16_t quaternionReal;
    } x2210_IMUPackedQuaternion;
    struct {
        uint8_t highestSub_indexSupported;
        int16_t accelerationX;
        int16_t accelerationY;
        int16_t accelerationZ;
        uint16_t sampleCounter;
    } x2211_IMUPackedAcceleration;
    struct {
        uint8_t highestSub_indexSupported;
        int16_t angularRateX;
        int16_t angularRateY;
        int16_t angularRateZ;
        uint8_t calibrationStatus;
        uint8_t accuracy;
    } x2212_IMUPackedGyro;
    struct {
        uint8_t highestSub_indexSupported;
        int16_t powerStageNTC;
        int16_t TMC9660Chip;
        int16_t ESP32Die;
        int16_t motor;
    } x2300_temperatures;
    struct {
        uint8_t highestSub_indexSupported;
        uint16_t supplyVoltage;
        uint16_t driverVoltage;
        bool_t powerGood;
    } x2301_power;
    struct {
        uint8_t highestSub_indexSupported;
        uint8_t currentOwner;
        uint8_t request;
    } x2700_controlAuthority;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t storeCount;
        uint32_t lastStoreResult;
        uint32_t ODHashAtLastStore;
        uint32_t recordsAppliedAtRestore;
        uint32_t recordsDroppedAtRestore;
    } x2800_parameterStoreMeta;
    struct {
        uint8_t highestSub_indexSupported;
        uint16_t entryCount;
        uint16_t head;
        uint16_t capacity;
        uint32_t overflowCount;
    } x4000_faultJournalHeader;
    uint32_t x4002_faultJournalClear;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t activeCauses;
        uint32_t escalatedCauses;
    } x4003_activeCauses;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t driveRtLastStep;
        uint32_t driveRtMaxStep;
        uint32_t driveRtOverruns;
        uint32_t coSvcLastStep;
        uint32_t coSvcMaxStep;
        uint32_t coSvcOverruns;
        uint32_t i2cIoLastStep;
        uint32_t i2cIoMaxStep;
        uint32_t i2cIoOverruns;
        uint32_t livenessStaleMask;
    } x4200_threadStats;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t jitterUpTo10Us;
        uint32_t jitterUpTo20Us;
        uint32_t jitterUpTo50Us;
        uint32_t jitterUpTo100Us;
        uint32_t jitterUpTo200Us;
        uint32_t jitterUpTo500Us;
        uint32_t jitterUpTo1000Us;
        uint32_t jitterAbove1000Us;
        uint32_t stepUpTo100Us;
        uint32_t stepUpTo150Us;
        uint32_t stepUpTo200Us;
        uint32_t stepUpTo250Us;
        uint32_t stepUpTo300Us;
        uint32_t stepUpTo500Us;
        uint32_t stepUpTo1000Us;
        uint32_t stepAbove1000Us;
    } x4201_timingHistogram;
    struct {
        uint8_t highestSub_indexSupported;
        uint16_t TEC;
        uint16_t REC;
        uint8_t busState;
        uint32_t bus_offCount;
        uint32_t RXOverruns;
        uint32_t TXQueueFull;
        uint32_t routerDropsCAN;
        uint32_t routerDropsWi_Fi;
        uint32_t routerDropsESP_NOW;
        uint32_t routerDropsHost;
    } x4202_CANStats;
    uint32_t x5FFE_ODLayoutHash;
    uint16_t x5FFF_manufacturerODVersion;
    uint16_t x603F_errorCode;
    uint16_t x6040_controlword;
    uint16_t x6041_statusword;
    int8_t x6060_modesOfOperation;
    int8_t x6061_modesOfOperationDisplay;
    int32_t x6062_positionDemandValue;
    int32_t x6064_positionActualValue;
    int32_t x606B_velocityDemandValue;
    int32_t x606C_velocityActualValue;
    int16_t x6071_targetTorque;
    int16_t x6077_torqueActualValue;
    int16_t x6078_currentActualValue;
    uint32_t x6079_DCLinkCircuitVoltage;
    int32_t x607A_targetPosition;
    int32_t x60B0_positionOffset;
    int32_t x60B1_velocityOffset;
    int16_t x60B2_torqueOffset;
    struct {
        uint8_t highestSub_indexSupported;
        uint8_t interpolationTimePeriodValue;
        int8_t interpolationTimeIndex;
    } x60C2_interpolationTimePeriod;
    uint8_t x60E4_additionalPositionActualValue_sub0;
    int32_t x60E4_additionalPositionActualValue[OD_CNT_ARR_60E4];
    int32_t x60F4_followingErrorActualValue;
    uint32_t x60FD_digitalInputs;
    uint8_t x60FE_digitalOutputs_sub0;
    uint32_t x60FE_digitalOutputs[OD_CNT_ARR_60FE];
    int32_t x60FF_targetVelocity;
    uint32_t x6502_supportedDriveModes;
} OD_RAM_t;

typedef struct {
    uint32_t x1005_COB_ID_SYNCMessage;
    uint32_t x1006_communicationCyclePeriod;
    uint32_t x1007_synchronousWindowLength;
    uint32_t x1012_COB_IDTimeStampObject;
    uint32_t x1014_COB_ID_EMCY;
    uint16_t x1015_inhibitTimeEMCY;
    uint8_t x1016_consumerHeartbeatTime_sub0;
    uint32_t x1016_consumerHeartbeatTime[OD_CNT_ARR_1016];
    uint16_t x1017_producerHeartbeatTime;
    uint8_t x1019_synchronousCounterOverflowValue;
    uint8_t x1029_errorBehavior_sub0;
    uint8_t x1029_errorBehavior[OD_CNT_ARR_1029];
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDClientToServerRx;
        uint32_t COB_IDServerToClientTx;
    } x1201_SDOServerParameter2;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1400_RPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1401_RPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1402_RPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1403_RPDOCommunicationParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1600_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1601_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1602_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1603_RPDOMappingParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1800_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1801_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1802_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1803_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1804_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1805_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1806_TPDOCommunicationParameter;
    struct {
        uint8_t highestSub_indexSupported;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t SYNCStartValue;
    } x1807_TPDOCommunicationParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A00_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A01_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A02_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A03_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A04_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A05_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A06_TPDOMappingParameter;
    struct {
        uint8_t numberOfMappedApplicationObjectsInPDO;
        uint32_t applicationObject1;
        uint32_t applicationObject2;
        uint32_t applicationObject3;
        uint32_t applicationObject4;
        uint32_t applicationObject5;
        uint32_t applicationObject6;
        uint32_t applicationObject7;
        uint32_t applicationObject8;
    } x1A07_TPDOMappingParameter;
    uint32_t x1F80_NMTStartup;
} OD_PERSIST_COMM_t;

typedef struct {
    struct {
        uint8_t highestSub_indexSupported;
        uint8_t allowedEnableTransports;
        uint16_t wirelessDeadman;
        bool_t requireMasterLiveness;
        bool_t wirelessService;
    } x2701_authorityPolicy;
    int16_t x6007_abortConnectionOptionCode;
    int16_t x605A_quickStopOptionCode;
    int16_t x605B_shutdownOptionCode;
    int16_t x605C_disableOperationOptionCode;
    int16_t x605D_haltOptionCode;
    int16_t x605E_faultReactionOptionCode;
    uint32_t x6065_followingErrorWindow;
    uint16_t x6066_followingErrorTimeOut;
    uint32_t x6067_positionWindow;
    uint16_t x6068_positionWindowTime;
    uint16_t x606D_velocityWindow;
    uint16_t x606E_velocityWindowTime;
    uint16_t x6072_maxTorque;
    uint16_t x6073_maxCurrent;
    uint32_t x6075_motorRatedCurrent;
    uint32_t x6076_motorRatedTorque;
    int32_t x607C_homeOffset;
    uint8_t x607D_softwarePositionLimit_sub0;
    int32_t x607D_softwarePositionLimit[OD_CNT_ARR_607D];
    uint8_t x607E_polarity;
    uint32_t x607F_maxProfileVelocity;
    uint32_t x6080_maxMotorSpeed;
    uint32_t x6081_profileVelocity;
    uint32_t x6083_profileAcceleration;
    uint32_t x6084_profileDeceleration;
    uint32_t x6085_quickStopDeceleration;
    int16_t x6086_motionProfileType;
    uint32_t x6087_torqueSlope;
    uint8_t x608F_positionEncoderResolution_sub0;
    uint32_t x608F_positionEncoderResolution[OD_CNT_ARR_608F];
    uint8_t x6091_gearRatio_sub0;
    uint32_t x6091_gearRatio[OD_CNT_ARR_6091];
    uint8_t x6092_feedConstant_sub0;
    uint32_t x6092_feedConstant[OD_CNT_ARR_6092];
    int8_t x6098_homingMethod;
    uint8_t x6099_homingSpeeds_sub0;
    uint32_t x6099_homingSpeeds[OD_CNT_ARR_6099];
    uint32_t x609A_homingAcceleration;
    uint16_t x60E0_positiveTorqueLimitValue;
    uint16_t x60E1_negativeTorqueLimitValue;
    uint16_t x6402_motorType;
    char x6403_motorCatalogueNumber[33];
    char x6404_motorManufacturer[33];
} OD_PERSIST_APP_t;

#ifndef OD_ATTR_RAM
#define OD_ATTR_RAM
#endif
extern OD_ATTR_RAM OD_RAM_t OD_RAM;

#ifndef OD_ATTR_PERSIST_COMM
#define OD_ATTR_PERSIST_COMM
#endif
extern OD_ATTR_PERSIST_COMM OD_PERSIST_COMM_t OD_PERSIST_COMM;

#ifndef OD_ATTR_PERSIST_APP
#define OD_ATTR_PERSIST_APP
#endif
extern OD_ATTR_PERSIST_APP OD_PERSIST_APP_t OD_PERSIST_APP;

#ifndef OD_ATTR_OD
#define OD_ATTR_OD
#endif
extern OD_ATTR_OD OD_t *OD;


/*******************************************************************************
    Object dictionary entries - shortcuts
*******************************************************************************/
#define OD_ENTRY_H1000 &OD->list[0]
#define OD_ENTRY_H1001 &OD->list[1]
#define OD_ENTRY_H1003 &OD->list[2]
#define OD_ENTRY_H1005 &OD->list[3]
#define OD_ENTRY_H1006 &OD->list[4]
#define OD_ENTRY_H1007 &OD->list[5]
#define OD_ENTRY_H1008 &OD->list[6]
#define OD_ENTRY_H1009 &OD->list[7]
#define OD_ENTRY_H100A &OD->list[8]
#define OD_ENTRY_H1010 &OD->list[9]
#define OD_ENTRY_H1011 &OD->list[10]
#define OD_ENTRY_H1012 &OD->list[11]
#define OD_ENTRY_H1014 &OD->list[12]
#define OD_ENTRY_H1015 &OD->list[13]
#define OD_ENTRY_H1016 &OD->list[14]
#define OD_ENTRY_H1017 &OD->list[15]
#define OD_ENTRY_H1018 &OD->list[16]
#define OD_ENTRY_H1019 &OD->list[17]
#define OD_ENTRY_H1029 &OD->list[18]
#define OD_ENTRY_H1200 &OD->list[19]
#define OD_ENTRY_H1201 &OD->list[20]
#define OD_ENTRY_H1280 &OD->list[21]
#define OD_ENTRY_H1400 &OD->list[22]
#define OD_ENTRY_H1401 &OD->list[23]
#define OD_ENTRY_H1402 &OD->list[24]
#define OD_ENTRY_H1403 &OD->list[25]
#define OD_ENTRY_H1600 &OD->list[26]
#define OD_ENTRY_H1601 &OD->list[27]
#define OD_ENTRY_H1602 &OD->list[28]
#define OD_ENTRY_H1603 &OD->list[29]
#define OD_ENTRY_H1800 &OD->list[30]
#define OD_ENTRY_H1801 &OD->list[31]
#define OD_ENTRY_H1802 &OD->list[32]
#define OD_ENTRY_H1803 &OD->list[33]
#define OD_ENTRY_H1804 &OD->list[34]
#define OD_ENTRY_H1805 &OD->list[35]
#define OD_ENTRY_H1806 &OD->list[36]
#define OD_ENTRY_H1807 &OD->list[37]
#define OD_ENTRY_H1A00 &OD->list[38]
#define OD_ENTRY_H1A01 &OD->list[39]
#define OD_ENTRY_H1A02 &OD->list[40]
#define OD_ENTRY_H1A03 &OD->list[41]
#define OD_ENTRY_H1A04 &OD->list[42]
#define OD_ENTRY_H1A05 &OD->list[43]
#define OD_ENTRY_H1A06 &OD->list[44]
#define OD_ENTRY_H1A07 &OD->list[45]
#define OD_ENTRY_H1F80 &OD->list[46]
#define OD_ENTRY_H2000 &OD->list[47]
#define OD_ENTRY_H2001 &OD->list[48]
#define OD_ENTRY_H2002 &OD->list[49]
#define OD_ENTRY_H2101 &OD->list[50]
#define OD_ENTRY_H2210 &OD->list[51]
#define OD_ENTRY_H2211 &OD->list[52]
#define OD_ENTRY_H2212 &OD->list[53]
#define OD_ENTRY_H2300 &OD->list[54]
#define OD_ENTRY_H2301 &OD->list[55]
#define OD_ENTRY_H2700 &OD->list[56]
#define OD_ENTRY_H2701 &OD->list[57]
#define OD_ENTRY_H2800 &OD->list[58]
#define OD_ENTRY_H4000 &OD->list[59]
#define OD_ENTRY_H4001 &OD->list[60]
#define OD_ENTRY_H4002 &OD->list[61]
#define OD_ENTRY_H4003 &OD->list[62]
#define OD_ENTRY_H4200 &OD->list[63]
#define OD_ENTRY_H4201 &OD->list[64]
#define OD_ENTRY_H4202 &OD->list[65]
#define OD_ENTRY_H5FFE &OD->list[66]
#define OD_ENTRY_H5FFF &OD->list[67]
#define OD_ENTRY_H6007 &OD->list[68]
#define OD_ENTRY_H603F &OD->list[69]
#define OD_ENTRY_H6040 &OD->list[70]
#define OD_ENTRY_H6041 &OD->list[71]
#define OD_ENTRY_H605A &OD->list[72]
#define OD_ENTRY_H605B &OD->list[73]
#define OD_ENTRY_H605C &OD->list[74]
#define OD_ENTRY_H605D &OD->list[75]
#define OD_ENTRY_H605E &OD->list[76]
#define OD_ENTRY_H6060 &OD->list[77]
#define OD_ENTRY_H6061 &OD->list[78]
#define OD_ENTRY_H6062 &OD->list[79]
#define OD_ENTRY_H6064 &OD->list[80]
#define OD_ENTRY_H6065 &OD->list[81]
#define OD_ENTRY_H6066 &OD->list[82]
#define OD_ENTRY_H6067 &OD->list[83]
#define OD_ENTRY_H6068 &OD->list[84]
#define OD_ENTRY_H606B &OD->list[85]
#define OD_ENTRY_H606C &OD->list[86]
#define OD_ENTRY_H606D &OD->list[87]
#define OD_ENTRY_H606E &OD->list[88]
#define OD_ENTRY_H6071 &OD->list[89]
#define OD_ENTRY_H6072 &OD->list[90]
#define OD_ENTRY_H6073 &OD->list[91]
#define OD_ENTRY_H6075 &OD->list[92]
#define OD_ENTRY_H6076 &OD->list[93]
#define OD_ENTRY_H6077 &OD->list[94]
#define OD_ENTRY_H6078 &OD->list[95]
#define OD_ENTRY_H6079 &OD->list[96]
#define OD_ENTRY_H607A &OD->list[97]
#define OD_ENTRY_H607C &OD->list[98]
#define OD_ENTRY_H607D &OD->list[99]
#define OD_ENTRY_H607E &OD->list[100]
#define OD_ENTRY_H607F &OD->list[101]
#define OD_ENTRY_H6080 &OD->list[102]
#define OD_ENTRY_H6081 &OD->list[103]
#define OD_ENTRY_H6083 &OD->list[104]
#define OD_ENTRY_H6084 &OD->list[105]
#define OD_ENTRY_H6085 &OD->list[106]
#define OD_ENTRY_H6086 &OD->list[107]
#define OD_ENTRY_H6087 &OD->list[108]
#define OD_ENTRY_H608F &OD->list[109]
#define OD_ENTRY_H6091 &OD->list[110]
#define OD_ENTRY_H6092 &OD->list[111]
#define OD_ENTRY_H6098 &OD->list[112]
#define OD_ENTRY_H6099 &OD->list[113]
#define OD_ENTRY_H609A &OD->list[114]
#define OD_ENTRY_H60B0 &OD->list[115]
#define OD_ENTRY_H60B1 &OD->list[116]
#define OD_ENTRY_H60B2 &OD->list[117]
#define OD_ENTRY_H60C2 &OD->list[118]
#define OD_ENTRY_H60E0 &OD->list[119]
#define OD_ENTRY_H60E1 &OD->list[120]
#define OD_ENTRY_H60E4 &OD->list[121]
#define OD_ENTRY_H60F4 &OD->list[122]
#define OD_ENTRY_H60FD &OD->list[123]
#define OD_ENTRY_H60FE &OD->list[124]
#define OD_ENTRY_H60FF &OD->list[125]
#define OD_ENTRY_H6402 &OD->list[126]
#define OD_ENTRY_H6403 &OD->list[127]
#define OD_ENTRY_H6404 &OD->list[128]
#define OD_ENTRY_H6502 &OD->list[129]


/*******************************************************************************
    Object dictionary entries - shortcuts with names
*******************************************************************************/
#define OD_ENTRY_H1000_deviceType &OD->list[0]
#define OD_ENTRY_H1001_errorRegister &OD->list[1]
#define OD_ENTRY_H1003_pre_definedErrorField &OD->list[2]
#define OD_ENTRY_H1005_COB_ID_SYNCMessage &OD->list[3]
#define OD_ENTRY_H1006_communicationCyclePeriod &OD->list[4]
#define OD_ENTRY_H1007_synchronousWindowLength &OD->list[5]
#define OD_ENTRY_H1008_manufacturerDeviceName &OD->list[6]
#define OD_ENTRY_H1009_manufacturerHardwareVersion &OD->list[7]
#define OD_ENTRY_H100A_manufacturerSoftwareVersion &OD->list[8]
#define OD_ENTRY_H1010_storeParameters &OD->list[9]
#define OD_ENTRY_H1011_restoreDefaultParameters &OD->list[10]
#define OD_ENTRY_H1012_COB_IDTimeStampObject &OD->list[11]
#define OD_ENTRY_H1014_COB_ID_EMCY &OD->list[12]
#define OD_ENTRY_H1015_inhibitTimeEMCY &OD->list[13]
#define OD_ENTRY_H1016_consumerHeartbeatTime &OD->list[14]
#define OD_ENTRY_H1017_producerHeartbeatTime &OD->list[15]
#define OD_ENTRY_H1018_identity &OD->list[16]
#define OD_ENTRY_H1019_synchronousCounterOverflowValue &OD->list[17]
#define OD_ENTRY_H1029_errorBehavior &OD->list[18]
#define OD_ENTRY_H1200_SDOServerParameter &OD->list[19]
#define OD_ENTRY_H1201_SDOServerParameter2 &OD->list[20]
#define OD_ENTRY_H1280_SDOClientParameter &OD->list[21]
#define OD_ENTRY_H1400_RPDOCommunicationParameter &OD->list[22]
#define OD_ENTRY_H1401_RPDOCommunicationParameter &OD->list[23]
#define OD_ENTRY_H1402_RPDOCommunicationParameter &OD->list[24]
#define OD_ENTRY_H1403_RPDOCommunicationParameter &OD->list[25]
#define OD_ENTRY_H1600_RPDOMappingParameter &OD->list[26]
#define OD_ENTRY_H1601_RPDOMappingParameter &OD->list[27]
#define OD_ENTRY_H1602_RPDOMappingParameter &OD->list[28]
#define OD_ENTRY_H1603_RPDOMappingParameter &OD->list[29]
#define OD_ENTRY_H1800_TPDOCommunicationParameter &OD->list[30]
#define OD_ENTRY_H1801_TPDOCommunicationParameter &OD->list[31]
#define OD_ENTRY_H1802_TPDOCommunicationParameter &OD->list[32]
#define OD_ENTRY_H1803_TPDOCommunicationParameter &OD->list[33]
#define OD_ENTRY_H1804_TPDOCommunicationParameter &OD->list[34]
#define OD_ENTRY_H1805_TPDOCommunicationParameter &OD->list[35]
#define OD_ENTRY_H1806_TPDOCommunicationParameter &OD->list[36]
#define OD_ENTRY_H1807_TPDOCommunicationParameter &OD->list[37]
#define OD_ENTRY_H1A00_TPDOMappingParameter &OD->list[38]
#define OD_ENTRY_H1A01_TPDOMappingParameter &OD->list[39]
#define OD_ENTRY_H1A02_TPDOMappingParameter &OD->list[40]
#define OD_ENTRY_H1A03_TPDOMappingParameter &OD->list[41]
#define OD_ENTRY_H1A04_TPDOMappingParameter &OD->list[42]
#define OD_ENTRY_H1A05_TPDOMappingParameter &OD->list[43]
#define OD_ENTRY_H1A06_TPDOMappingParameter &OD->list[44]
#define OD_ENTRY_H1A07_TPDOMappingParameter &OD->list[45]
#define OD_ENTRY_H1F80_NMTStartup &OD->list[46]
#define OD_ENTRY_H2000_firmwareBuild &OD->list[47]
#define OD_ENTRY_H2001_boardInfo &OD->list[48]
#define OD_ENTRY_H2002_componentVersions &OD->list[49]
#define OD_ENTRY_H2101_encoderStatus &OD->list[50]
#define OD_ENTRY_H2210_IMUPackedQuaternion &OD->list[51]
#define OD_ENTRY_H2211_IMUPackedAcceleration &OD->list[52]
#define OD_ENTRY_H2212_IMUPackedGyro &OD->list[53]
#define OD_ENTRY_H2300_temperatures &OD->list[54]
#define OD_ENTRY_H2301_power &OD->list[55]
#define OD_ENTRY_H2700_controlAuthority &OD->list[56]
#define OD_ENTRY_H2701_authorityPolicy &OD->list[57]
#define OD_ENTRY_H2800_parameterStoreMeta &OD->list[58]
#define OD_ENTRY_H4000_faultJournalHeader &OD->list[59]
#define OD_ENTRY_H4001_faultJournal &OD->list[60]
#define OD_ENTRY_H4002_faultJournalClear &OD->list[61]
#define OD_ENTRY_H4003_activeCauses &OD->list[62]
#define OD_ENTRY_H4200_threadStats &OD->list[63]
#define OD_ENTRY_H4201_timingHistogram &OD->list[64]
#define OD_ENTRY_H4202_CANStats &OD->list[65]
#define OD_ENTRY_H5FFE_ODLayoutHash &OD->list[66]
#define OD_ENTRY_H5FFF_manufacturerODVersion &OD->list[67]
#define OD_ENTRY_H6007_abortConnectionOptionCode &OD->list[68]
#define OD_ENTRY_H603F_errorCode &OD->list[69]
#define OD_ENTRY_H6040_controlword &OD->list[70]
#define OD_ENTRY_H6041_statusword &OD->list[71]
#define OD_ENTRY_H605A_quickStopOptionCode &OD->list[72]
#define OD_ENTRY_H605B_shutdownOptionCode &OD->list[73]
#define OD_ENTRY_H605C_disableOperationOptionCode &OD->list[74]
#define OD_ENTRY_H605D_haltOptionCode &OD->list[75]
#define OD_ENTRY_H605E_faultReactionOptionCode &OD->list[76]
#define OD_ENTRY_H6060_modesOfOperation &OD->list[77]
#define OD_ENTRY_H6061_modesOfOperationDisplay &OD->list[78]
#define OD_ENTRY_H6062_positionDemandValue &OD->list[79]
#define OD_ENTRY_H6064_positionActualValue &OD->list[80]
#define OD_ENTRY_H6065_followingErrorWindow &OD->list[81]
#define OD_ENTRY_H6066_followingErrorTimeOut &OD->list[82]
#define OD_ENTRY_H6067_positionWindow &OD->list[83]
#define OD_ENTRY_H6068_positionWindowTime &OD->list[84]
#define OD_ENTRY_H606B_velocityDemandValue &OD->list[85]
#define OD_ENTRY_H606C_velocityActualValue &OD->list[86]
#define OD_ENTRY_H606D_velocityWindow &OD->list[87]
#define OD_ENTRY_H606E_velocityWindowTime &OD->list[88]
#define OD_ENTRY_H6071_targetTorque &OD->list[89]
#define OD_ENTRY_H6072_maxTorque &OD->list[90]
#define OD_ENTRY_H6073_maxCurrent &OD->list[91]
#define OD_ENTRY_H6075_motorRatedCurrent &OD->list[92]
#define OD_ENTRY_H6076_motorRatedTorque &OD->list[93]
#define OD_ENTRY_H6077_torqueActualValue &OD->list[94]
#define OD_ENTRY_H6078_currentActualValue &OD->list[95]
#define OD_ENTRY_H6079_DCLinkCircuitVoltage &OD->list[96]
#define OD_ENTRY_H607A_targetPosition &OD->list[97]
#define OD_ENTRY_H607C_homeOffset &OD->list[98]
#define OD_ENTRY_H607D_softwarePositionLimit &OD->list[99]
#define OD_ENTRY_H607E_polarity &OD->list[100]
#define OD_ENTRY_H607F_maxProfileVelocity &OD->list[101]
#define OD_ENTRY_H6080_maxMotorSpeed &OD->list[102]
#define OD_ENTRY_H6081_profileVelocity &OD->list[103]
#define OD_ENTRY_H6083_profileAcceleration &OD->list[104]
#define OD_ENTRY_H6084_profileDeceleration &OD->list[105]
#define OD_ENTRY_H6085_quickStopDeceleration &OD->list[106]
#define OD_ENTRY_H6086_motionProfileType &OD->list[107]
#define OD_ENTRY_H6087_torqueSlope &OD->list[108]
#define OD_ENTRY_H608F_positionEncoderResolution &OD->list[109]
#define OD_ENTRY_H6091_gearRatio &OD->list[110]
#define OD_ENTRY_H6092_feedConstant &OD->list[111]
#define OD_ENTRY_H6098_homingMethod &OD->list[112]
#define OD_ENTRY_H6099_homingSpeeds &OD->list[113]
#define OD_ENTRY_H609A_homingAcceleration &OD->list[114]
#define OD_ENTRY_H60B0_positionOffset &OD->list[115]
#define OD_ENTRY_H60B1_velocityOffset &OD->list[116]
#define OD_ENTRY_H60B2_torqueOffset &OD->list[117]
#define OD_ENTRY_H60C2_interpolationTimePeriod &OD->list[118]
#define OD_ENTRY_H60E0_positiveTorqueLimitValue &OD->list[119]
#define OD_ENTRY_H60E1_negativeTorqueLimitValue &OD->list[120]
#define OD_ENTRY_H60E4_additionalPositionActualValue &OD->list[121]
#define OD_ENTRY_H60F4_followingErrorActualValue &OD->list[122]
#define OD_ENTRY_H60FD_digitalInputs &OD->list[123]
#define OD_ENTRY_H60FE_digitalOutputs &OD->list[124]
#define OD_ENTRY_H60FF_targetVelocity &OD->list[125]
#define OD_ENTRY_H6402_motorType &OD->list[126]
#define OD_ENTRY_H6403_motorCatalogueNumber &OD->list[127]
#define OD_ENTRY_H6404_motorManufacturer &OD->list[128]
#define OD_ENTRY_H6502_supportedDriveModes &OD->list[129]


/*******************************************************************************
    OD config structure
*******************************************************************************/
#ifdef CO_MULTIPLE_OD
#define OD_INIT_CONFIG(config) {\
    (config).CNT_NMT = OD_CNT_NMT;\
    (config).ENTRY_H1017 = OD_ENTRY_H1017;\
    (config).CNT_HB_CONS = OD_CNT_HB_CONS;\
    (config).CNT_ARR_1016 = OD_CNT_ARR_1016;\
    (config).ENTRY_H1016 = OD_ENTRY_H1016;\
    (config).CNT_EM = OD_CNT_EM;\
    (config).ENTRY_H1001 = OD_ENTRY_H1001;\
    (config).ENTRY_H1014 = OD_ENTRY_H1014;\
    (config).ENTRY_H1015 = OD_ENTRY_H1015;\
    (config).CNT_ARR_1003 = OD_CNT_ARR_1003;\
    (config).ENTRY_H1003 = OD_ENTRY_H1003;\
    (config).CNT_SDO_SRV = OD_CNT_SDO_SRV;\
    (config).ENTRY_H1200 = OD_ENTRY_H1200;\
    (config).CNT_SDO_CLI = OD_CNT_SDO_CLI;\
    (config).ENTRY_H1280 = OD_ENTRY_H1280;\
    (config).CNT_TIME = OD_CNT_TIME;\
    (config).ENTRY_H1012 = OD_ENTRY_H1012;\
    (config).CNT_SYNC = OD_CNT_SYNC;\
    (config).ENTRY_H1005 = OD_ENTRY_H1005;\
    (config).ENTRY_H1006 = OD_ENTRY_H1006;\
    (config).ENTRY_H1007 = OD_ENTRY_H1007;\
    (config).ENTRY_H1019 = OD_ENTRY_H1019;\
    (config).CNT_RPDO = OD_CNT_RPDO;\
    (config).ENTRY_H1400 = OD_ENTRY_H1400;\
    (config).ENTRY_H1600 = OD_ENTRY_H1600;\
    (config).CNT_TPDO = OD_CNT_TPDO;\
    (config).ENTRY_H1800 = OD_ENTRY_H1800;\
    (config).ENTRY_H1A00 = OD_ENTRY_H1A00;\
    (config).CNT_LEDS = 0;\
    (config).CNT_GFC = 0;\
    (config).ENTRY_H1300 = NULL;\
    (config).CNT_SRDO = 0;\
    (config).ENTRY_H1301 = NULL;\
    (config).ENTRY_H1381 = NULL;\
    (config).ENTRY_H13FE = NULL;\
    (config).ENTRY_H13FF = NULL;\
    (config).CNT_LSS_SLV = 0;\
    (config).CNT_LSS_MST = 0;\
    (config).CNT_GTWA = 0;\
    (config).CNT_TRACE = 0;\
}
#endif

#endif /* OD_H */
