"""Object dictionary identifiers for the Vortex V1 CANopen device.


Source: od/vortex_od.yaml, revision 1.0, OD hash 0xFF85661C.
Do not edit: change the YAML and run `python3 tools/odgen`.
"""

from __future__ import annotations

from typing import NamedTuple

OD_HASH = 0xFF85661C
REVISION = (1, 0)
REVISION_NUMBER = 0x00010000
OD_VERSION = 0x0100


class Entry(NamedTuple):
    index: int
    sub: int
    key: str
    name: str
    data_type: str
    access: str
    pdo: str
    default: object
    node_relative: bool
    low: object
    high: object
    persist: str
    units: str
    scale: float | None


IDX_DEVICE_TYPE = 0x1000
IDX_ERROR_REGISTER = 0x1001
IDX_PRE_DEFINED_ERROR_FIELD = 0x1003
IDX_COB_ID_SYNC = 0x1005
IDX_COMMUNICATION_CYCLE_PERIOD = 0x1006
IDX_SYNCHRONOUS_WINDOW_LENGTH = 0x1007
IDX_MANUFACTURER_DEVICE_NAME = 0x1008
IDX_MANUFACTURER_HARDWARE_VERSION = 0x1009
IDX_MANUFACTURER_SOFTWARE_VERSION = 0x100A
IDX_STORE_PARAMETERS = 0x1010
IDX_RESTORE_DEFAULT_PARAMETERS = 0x1011
IDX_COB_ID_TIME = 0x1012
IDX_COB_ID_EMCY = 0x1014
IDX_INHIBIT_TIME_EMCY = 0x1015
IDX_CONSUMER_HEARTBEAT_TIME = 0x1016
IDX_PRODUCER_HEARTBEAT_TIME = 0x1017
IDX_IDENTITY = 0x1018
IDX_SYNCHRONOUS_COUNTER_OVERFLOW_VALUE = 0x1019
IDX_ERROR_BEHAVIOR = 0x1029
IDX_SDO_SERVER_1 = 0x1200
IDX_SDO_SERVER_2 = 0x1201
IDX_SDO_CLIENT_1 = 0x1280
IDX_RPDO1_COMMUNICATION = 0x1400
IDX_RPDO2_COMMUNICATION = 0x1401
IDX_RPDO3_COMMUNICATION = 0x1402
IDX_RPDO4_COMMUNICATION = 0x1403
IDX_RPDO1_MAPPING = 0x1600
IDX_RPDO2_MAPPING = 0x1601
IDX_RPDO3_MAPPING = 0x1602
IDX_RPDO4_MAPPING = 0x1603
IDX_TPDO1_COMMUNICATION = 0x1800
IDX_TPDO2_COMMUNICATION = 0x1801
IDX_TPDO3_COMMUNICATION = 0x1802
IDX_TPDO4_COMMUNICATION = 0x1803
IDX_TPDO5_COMMUNICATION = 0x1804
IDX_TPDO6_COMMUNICATION = 0x1805
IDX_TPDO7_COMMUNICATION = 0x1806
IDX_TPDO8_COMMUNICATION = 0x1807
IDX_TPDO1_MAPPING = 0x1A00
IDX_TPDO2_MAPPING = 0x1A01
IDX_TPDO3_MAPPING = 0x1A02
IDX_TPDO4_MAPPING = 0x1A03
IDX_TPDO5_MAPPING = 0x1A04
IDX_TPDO6_MAPPING = 0x1A05
IDX_TPDO7_MAPPING = 0x1A06
IDX_TPDO8_MAPPING = 0x1A07
IDX_NMT_STARTUP = 0x1F80
IDX_FIRMWARE_BUILD = 0x2000
IDX_BOARD_INFO = 0x2001
IDX_COMPONENT_VERSIONS = 0x2002
IDX_ENCODER_STATUS = 0x2101
IDX_IMU_PACKED_QUATERNION = 0x2210
IDX_IMU_PACKED_ACCEL = 0x2211
IDX_IMU_PACKED_GYRO = 0x2212
IDX_TEMPERATURES = 0x2300
IDX_POWER = 0x2301
IDX_CONTROL_AUTHORITY = 0x2700
IDX_AUTHORITY_POLICY = 0x2701
IDX_PARAMETER_STORE_META = 0x2800
IDX_FAULT_JOURNAL_HEADER = 0x4000
IDX_FAULT_JOURNAL = 0x4001
IDX_FAULT_JOURNAL_CLEAR = 0x4002
IDX_ACTIVE_CAUSES = 0x4003
IDX_THREAD_STATS = 0x4200
IDX_TIMING_HISTOGRAM = 0x4201
IDX_CAN_STATS = 0x4202
IDX_OD_LAYOUT_HASH = 0x5FFE
IDX_MANUFACTURER_OD_VERSION = 0x5FFF
IDX_ABORT_CONNECTION_OPTION_CODE = 0x6007
IDX_ERROR_CODE = 0x603F
IDX_CONTROLWORD = 0x6040
IDX_STATUSWORD = 0x6041
IDX_QUICK_STOP_OPTION_CODE = 0x605A
IDX_SHUTDOWN_OPTION_CODE = 0x605B
IDX_DISABLE_OPERATION_OPTION_CODE = 0x605C
IDX_HALT_OPTION_CODE = 0x605D
IDX_FAULT_REACTION_OPTION_CODE = 0x605E
IDX_MODES_OF_OPERATION = 0x6060
IDX_MODES_OF_OPERATION_DISPLAY = 0x6061
IDX_POSITION_DEMAND_VALUE = 0x6062
IDX_POSITION_ACTUAL_VALUE = 0x6064
IDX_FOLLOWING_ERROR_WINDOW = 0x6065
IDX_FOLLOWING_ERROR_TIME_OUT = 0x6066
IDX_POSITION_WINDOW = 0x6067
IDX_POSITION_WINDOW_TIME = 0x6068
IDX_VELOCITY_DEMAND_VALUE = 0x606B
IDX_VELOCITY_ACTUAL_VALUE = 0x606C
IDX_VELOCITY_WINDOW = 0x606D
IDX_VELOCITY_WINDOW_TIME = 0x606E
IDX_TARGET_TORQUE = 0x6071
IDX_MAX_TORQUE = 0x6072
IDX_MAX_CURRENT = 0x6073
IDX_MOTOR_RATED_CURRENT = 0x6075
IDX_MOTOR_RATED_TORQUE = 0x6076
IDX_TORQUE_ACTUAL_VALUE = 0x6077
IDX_CURRENT_ACTUAL_VALUE = 0x6078
IDX_DC_LINK_CIRCUIT_VOLTAGE = 0x6079
IDX_TARGET_POSITION = 0x607A
IDX_HOME_OFFSET = 0x607C
IDX_SOFTWARE_POSITION_LIMIT = 0x607D
IDX_POLARITY = 0x607E
IDX_MAX_PROFILE_VELOCITY = 0x607F
IDX_MAX_MOTOR_SPEED = 0x6080
IDX_PROFILE_VELOCITY = 0x6081
IDX_PROFILE_ACCELERATION = 0x6083
IDX_PROFILE_DECELERATION = 0x6084
IDX_QUICK_STOP_DECELERATION = 0x6085
IDX_MOTION_PROFILE_TYPE = 0x6086
IDX_TORQUE_SLOPE = 0x6087
IDX_POSITION_ENCODER_RESOLUTION = 0x608F
IDX_GEAR_RATIO = 0x6091
IDX_FEED_CONSTANT = 0x6092
IDX_HOMING_METHOD = 0x6098
IDX_HOMING_SPEEDS = 0x6099
IDX_HOMING_ACCELERATION = 0x609A
IDX_POSITION_OFFSET = 0x60B0
IDX_VELOCITY_OFFSET = 0x60B1
IDX_TORQUE_OFFSET = 0x60B2
IDX_INTERPOLATION_TIME_PERIOD = 0x60C2
IDX_POSITIVE_TORQUE_LIMIT_VALUE = 0x60E0
IDX_NEGATIVE_TORQUE_LIMIT_VALUE = 0x60E1
IDX_ADDITIONAL_POSITION_ACTUAL_VALUE = 0x60E4
IDX_FOLLOWING_ERROR_ACTUAL_VALUE = 0x60F4
IDX_DIGITAL_INPUTS = 0x60FD
IDX_DIGITAL_OUTPUTS = 0x60FE
IDX_TARGET_VELOCITY = 0x60FF
IDX_MOTOR_TYPE = 0x6402
IDX_MOTOR_CATALOGUE_NUMBER = 0x6403
IDX_MOTOR_MANUFACTURER = 0x6404
IDX_SUPPORTED_DRIVE_MODES = 0x6502

ENTRIES: tuple[Entry, ...] = (
    Entry(0x1000, 0x00, 'device_type', 'Device type', 'U32', 'const', '', 131474, False, None, None, 'none', '', None),
    Entry(0x1001, 0x00, 'error_register', 'Error register', 'U8', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x1003, 0x00, 'pre_defined_error_field_number_of_errors', 'Number of errors', 'U8', 'rw', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x01, 'pre_defined_error_field_standard_error_field_1', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x02, 'pre_defined_error_field_standard_error_field_2', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x03, 'pre_defined_error_field_standard_error_field_3', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x04, 'pre_defined_error_field_standard_error_field_4', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x05, 'pre_defined_error_field_standard_error_field_5', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x06, 'pre_defined_error_field_standard_error_field_6', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x07, 'pre_defined_error_field_standard_error_field_7', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x08, 'pre_defined_error_field_standard_error_field_8', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x09, 'pre_defined_error_field_standard_error_field_9', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0A, 'pre_defined_error_field_standard_error_field_10', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0B, 'pre_defined_error_field_standard_error_field_11', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0C, 'pre_defined_error_field_standard_error_field_12', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0D, 'pre_defined_error_field_standard_error_field_13', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0E, 'pre_defined_error_field_standard_error_field_14', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x0F, 'pre_defined_error_field_standard_error_field_15', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1003, 0x10, 'pre_defined_error_field_standard_error_field_16', 'Standard error field', 'U32', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x1005, 0x00, 'cob_id_sync', 'COB-ID SYNC message', 'U32', 'rw', '', 128, False, None, None, 'comm', '', None),
    Entry(0x1006, 0x00, 'communication_cycle_period', 'Communication cycle period', 'U32', 'rw', '', 0, False, None, None, 'comm', 'µs', None),
    Entry(0x1007, 0x00, 'synchronous_window_length', 'Synchronous window length', 'U32', 'rw', '', 0, False, None, None, 'comm', 'µs', None),
    Entry(0x1008, 0x00, 'manufacturer_device_name', 'Manufacturer device name', 'VS', 'const', '', 'HardFOC Vortex V1', False, None, None, 'none', '', None),
    Entry(0x1009, 0x00, 'manufacturer_hardware_version', 'Manufacturer hardware version', 'VS', 'const', '', 'V1', False, None, None, 'none', '', None),
    Entry(0x100A, 0x00, 'manufacturer_software_version', 'Manufacturer software version', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x1010, 0x00, 'store_parameters_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 4, False, None, None, 'none', '', None),
    Entry(0x1010, 0x01, 'store_parameters_save_all', 'Save all parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1010, 0x02, 'store_parameters_save_communication', 'Save communication parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1010, 0x03, 'store_parameters_save_application', 'Save application parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1010, 0x04, 'store_parameters_save_calibration', 'Save calibration parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1011, 0x00, 'restore_default_parameters_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 4, False, None, None, 'none', '', None),
    Entry(0x1011, 0x01, 'restore_default_parameters_restore_all', 'Restore all default parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1011, 0x02, 'restore_default_parameters_restore_communication', 'Restore communication default parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1011, 0x03, 'restore_default_parameters_restore_application', 'Restore application default parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1011, 0x04, 'restore_default_parameters_restore_calibration', 'Restore calibration default parameters', 'U32', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1012, 0x00, 'cob_id_time', 'COB-ID time stamp object', 'U32', 'rw', '', 256, False, None, None, 'comm', '', None),
    Entry(0x1014, 0x00, 'cob_id_emcy', 'COB-ID EMCY', 'U32', 'rw', '', 128, True, None, None, 'comm', '', None),
    Entry(0x1015, 0x00, 'inhibit_time_emcy', 'Inhibit time EMCY', 'U16', 'rw', '', 10, False, None, None, 'comm', '100 µs', 0.0001),
    Entry(0x1016, 0x00, 'consumer_heartbeat_time_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 4, False, None, None, 'comm', '', None),
    Entry(0x1016, 0x01, 'consumer_heartbeat_time_consumer_heartbeat_time_1', 'Consumer heartbeat time', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1016, 0x02, 'consumer_heartbeat_time_consumer_heartbeat_time_2', 'Consumer heartbeat time', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1016, 0x03, 'consumer_heartbeat_time_consumer_heartbeat_time_3', 'Consumer heartbeat time', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1016, 0x04, 'consumer_heartbeat_time_consumer_heartbeat_time_4', 'Consumer heartbeat time', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1017, 0x00, 'producer_heartbeat_time', 'Producer heartbeat time', 'U16', 'rw', '', 100, False, None, None, 'comm', 'ms', None),
    Entry(0x1018, 0x00, 'identity_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x1018, 0x01, 'identity_vendor_id', 'Vendor-ID', 'U32', 'const', '', 0, False, None, None, 'none', '', None),
    Entry(0x1018, 0x02, 'identity_product_code', 'Product code', 'U32', 'const', '', 5636097, False, None, None, 'none', '', None),
    Entry(0x1018, 0x03, 'identity_revision_number', 'Revision number', 'U32', 'const', '', 65536, False, None, None, 'none', '', None),
    Entry(0x1018, 0x04, 'identity_serial_number', 'Serial number', 'U32', 'const', '', 0, False, None, None, 'none', '', None),
    Entry(0x1019, 0x00, 'synchronous_counter_overflow_value', 'Synchronous counter overflow value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1029, 0x00, 'error_behavior_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1029, 0x01, 'error_behavior_communication_error', 'Communication error', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1029, 0x02, 'error_behavior_profile_error', 'Profile error', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1200, 0x00, 'sdo_server_1_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 2, False, None, None, 'none', '', None),
    Entry(0x1200, 0x01, 'sdo_server_1_cob_id_client_to_server', 'COB-ID client to server (rx)', 'U32', 'ro', 't', 1536, True, None, None, 'none', '', None),
    Entry(0x1200, 0x02, 'sdo_server_1_cob_id_server_to_client', 'COB-ID server to client (tx)', 'U32', 'ro', 't', 1408, True, None, None, 'none', '', None),
    Entry(0x1201, 0x00, 'sdo_server_2_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1201, 0x01, 'sdo_server_2_cob_id_client_to_server', 'COB-ID client to server (rx)', 'U32', 'rw', '', 1728, True, None, None, 'comm', '', None),
    Entry(0x1201, 0x02, 'sdo_server_2_cob_id_server_to_client', 'COB-ID server to client (tx)', 'U32', 'rw', '', 1472, True, None, None, 'comm', '', None),
    Entry(0x1280, 0x00, 'sdo_client_1_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 3, False, None, None, 'none', '', None),
    Entry(0x1280, 0x01, 'sdo_client_1_cob_id_client_to_server', 'COB-ID client to server (tx)', 'U32', 'rw', 'tr', 2147483648, False, None, None, 'none', '', None),
    Entry(0x1280, 0x02, 'sdo_client_1_cob_id_server_to_client', 'COB-ID server to client (rx)', 'U32', 'rw', 'tr', 2147483648, False, None, None, 'none', '', None),
    Entry(0x1280, 0x03, 'sdo_client_1_node_id_of_the_sdo_server', 'Node-ID of the SDO server', 'U8', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x1400, 0x00, 'rpdo1_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'comm', '', None),
    Entry(0x1400, 0x01, 'rpdo1_communication_cob_id_used_by_rpdo', 'COB-ID used by RPDO', 'U32', 'rw', '', 512, True, None, None, 'comm', '', None),
    Entry(0x1400, 0x02, 'rpdo1_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 255, False, None, None, 'comm', '', None),
    Entry(0x1400, 0x05, 'rpdo1_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1401, 0x00, 'rpdo2_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'comm', '', None),
    Entry(0x1401, 0x01, 'rpdo2_communication_cob_id_used_by_rpdo', 'COB-ID used by RPDO', 'U32', 'rw', '', 768, True, None, None, 'comm', '', None),
    Entry(0x1401, 0x02, 'rpdo2_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1401, 0x05, 'rpdo2_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1402, 0x00, 'rpdo3_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'comm', '', None),
    Entry(0x1402, 0x01, 'rpdo3_communication_cob_id_used_by_rpdo', 'COB-ID used by RPDO', 'U32', 'rw', '', 1024, True, None, None, 'comm', '', None),
    Entry(0x1402, 0x02, 'rpdo3_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1402, 0x05, 'rpdo3_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1403, 0x00, 'rpdo4_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'comm', '', None),
    Entry(0x1403, 0x01, 'rpdo4_communication_cob_id_used_by_rpdo', 'COB-ID used by RPDO', 'U32', 'rw', '', 1280, True, None, None, 'comm', '', None),
    Entry(0x1403, 0x02, 'rpdo4_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1403, 0x05, 'rpdo4_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x00, 'rpdo1_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x01, 'rpdo1_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614807056, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x02, 'rpdo1_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1616904200, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x03, 'rpdo1_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x04, 'rpdo1_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x05, 'rpdo1_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x06, 'rpdo1_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x07, 'rpdo1_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1600, 0x08, 'rpdo1_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x00, 'rpdo2_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x01, 'rpdo2_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614807056, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x02, 'rpdo2_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1618608160, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x03, 'rpdo2_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x04, 'rpdo2_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x05, 'rpdo2_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x06, 'rpdo2_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x07, 'rpdo2_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1601, 0x08, 'rpdo2_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x00, 'rpdo3_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x01, 'rpdo3_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614807056, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x02, 'rpdo3_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1627324448, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x03, 'rpdo3_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x04, 'rpdo3_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x05, 'rpdo3_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x06, 'rpdo3_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x07, 'rpdo3_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1602, 0x08, 'rpdo3_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x00, 'rpdo4_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x01, 'rpdo4_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614807056, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x02, 'rpdo4_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1618018320, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x03, 'rpdo4_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x04, 'rpdo4_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x05, 'rpdo4_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x06, 'rpdo4_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x07, 'rpdo4_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1603, 0x08, 'rpdo4_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1800, 0x00, 'tpdo1_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1800, 0x01, 'tpdo1_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 1073742208, True, None, None, 'comm', '', None),
    Entry(0x1800, 0x02, 'tpdo1_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1800, 0x03, 'tpdo1_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 100, False, None, None, 'comm', '', None),
    Entry(0x1800, 0x05, 'tpdo1_communication_event_timer', 'Event timer', 'U16', 'rw', '', 100, False, None, None, 'comm', '', None),
    Entry(0x1800, 0x06, 'tpdo1_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1801, 0x00, 'tpdo2_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1801, 0x01, 'tpdo2_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 1073742464, True, None, None, 'comm', '', None),
    Entry(0x1801, 0x02, 'tpdo2_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1801, 0x03, 'tpdo2_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1801, 0x05, 'tpdo2_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1801, 0x06, 'tpdo2_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1802, 0x00, 'tpdo3_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1802, 0x01, 'tpdo3_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 1073742720, True, None, None, 'comm', '', None),
    Entry(0x1802, 0x02, 'tpdo3_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 1, False, None, None, 'comm', '', None),
    Entry(0x1802, 0x03, 'tpdo3_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1802, 0x05, 'tpdo3_communication_event_timer', 'Event timer', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1802, 0x06, 'tpdo3_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1803, 0x00, 'tpdo4_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1803, 0x01, 'tpdo4_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 1073742976, True, None, None, 'comm', '', None),
    Entry(0x1803, 0x02, 'tpdo4_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1803, 0x03, 'tpdo4_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1803, 0x05, 'tpdo4_communication_event_timer', 'Event timer', 'U16', 'rw', '', 100, False, None, None, 'comm', '', None),
    Entry(0x1803, 0x06, 'tpdo4_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1804, 0x00, 'tpdo5_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1804, 0x01, 'tpdo5_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 3221225920, True, None, None, 'comm', '', None),
    Entry(0x1804, 0x02, 'tpdo5_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1804, 0x03, 'tpdo5_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1804, 0x05, 'tpdo5_communication_event_timer', 'Event timer', 'U16', 'rw', '', 10, False, None, None, 'comm', '', None),
    Entry(0x1804, 0x06, 'tpdo5_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1805, 0x00, 'tpdo6_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1805, 0x01, 'tpdo6_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 3221226176, True, None, None, 'comm', '', None),
    Entry(0x1805, 0x02, 'tpdo6_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1805, 0x03, 'tpdo6_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1805, 0x05, 'tpdo6_communication_event_timer', 'Event timer', 'U16', 'rw', '', 10, False, None, None, 'comm', '', None),
    Entry(0x1805, 0x06, 'tpdo6_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1806, 0x00, 'tpdo7_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1806, 0x01, 'tpdo7_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 3221226432, True, None, None, 'comm', '', None),
    Entry(0x1806, 0x02, 'tpdo7_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1806, 0x03, 'tpdo7_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1806, 0x05, 'tpdo7_communication_event_timer', 'Event timer', 'U16', 'rw', '', 10, False, None, None, 'comm', '', None),
    Entry(0x1806, 0x06, 'tpdo7_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1807, 0x00, 'tpdo8_communication_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 6, False, None, None, 'comm', '', None),
    Entry(0x1807, 0x01, 'tpdo8_communication_cob_id_used_by_tpdo', 'COB-ID used by TPDO', 'U32', 'rw', '', 3221226688, True, None, None, 'comm', '', None),
    Entry(0x1807, 0x02, 'tpdo8_communication_transmission_type', 'Transmission type', 'U8', 'rw', '', 254, False, None, None, 'comm', '', None),
    Entry(0x1807, 0x03, 'tpdo8_communication_inhibit_time', 'Inhibit time', 'U16', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1807, 0x05, 'tpdo8_communication_event_timer', 'Event timer', 'U16', 'rw', '', 10, False, None, None, 'comm', '', None),
    Entry(0x1807, 0x06, 'tpdo8_communication_sync_start_value', 'SYNC start value', 'U8', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x00, 'tpdo1_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x01, 'tpdo1_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614872592, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x02, 'tpdo1_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1616969736, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x03, 'tpdo1_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x04, 'tpdo1_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x05, 'tpdo1_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x06, 'tpdo1_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x07, 'tpdo1_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A00, 0x08, 'tpdo1_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x00, 'tpdo2_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x01, 'tpdo2_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614872592, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x02, 'tpdo2_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1617166368, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x03, 'tpdo2_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x04, 'tpdo2_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x05, 'tpdo2_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x06, 'tpdo2_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x07, 'tpdo2_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A01, 0x08, 'tpdo2_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x00, 'tpdo3_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 2, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x01, 'tpdo3_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1614872592, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x02, 'tpdo3_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1617690656, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x03, 'tpdo3_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x04, 'tpdo3_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x05, 'tpdo3_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x06, 'tpdo3_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x07, 'tpdo3_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A02, 0x08, 'tpdo3_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x00, 'tpdo4_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 4, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x01, 'tpdo4_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 1618411536, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x02, 'tpdo4_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 1614741520, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x03, 'tpdo4_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 587202832, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x04, 'tpdo4_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 587268368, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x05, 'tpdo4_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x06, 'tpdo4_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x07, 'tpdo4_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A03, 0x08, 'tpdo4_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x00, 'tpdo5_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 4, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x01, 'tpdo5_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 571474192, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x02, 'tpdo5_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 571474448, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x03, 'tpdo5_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 571474704, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x04, 'tpdo5_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 571474960, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x05, 'tpdo5_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x06, 'tpdo5_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x07, 'tpdo5_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A04, 0x08, 'tpdo5_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x00, 'tpdo6_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 4, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x01, 'tpdo6_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 571539728, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x02, 'tpdo6_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 571539984, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x03, 'tpdo6_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 571540240, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x04, 'tpdo6_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 571540496, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x05, 'tpdo6_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x06, 'tpdo6_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x07, 'tpdo6_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A05, 0x08, 'tpdo6_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x00, 'tpdo7_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 5, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x01, 'tpdo7_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 571605264, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x02, 'tpdo7_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 571605520, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x03, 'tpdo7_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 571605776, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x04, 'tpdo7_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 571606024, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x05, 'tpdo7_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 571606280, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x06, 'tpdo7_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x07, 'tpdo7_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A06, 0x08, 'tpdo7_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x00, 'tpdo8_mapping_number_of_mapped_objects', 'Number of mapped application objects in PDO', 'U8', 'rw', '', 3, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x01, 'tpdo8_mapping_application_object_1', 'Application object 1', 'U32', 'rw', '', 553713936, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x02, 'tpdo8_mapping_application_object_2', 'Application object 2', 'U32', 'rw', '', 553714448, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x03, 'tpdo8_mapping_application_object_3', 'Application object 3', 'U32', 'rw', '', 553715472, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x04, 'tpdo8_mapping_application_object_4', 'Application object 4', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x05, 'tpdo8_mapping_application_object_5', 'Application object 5', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x06, 'tpdo8_mapping_application_object_6', 'Application object 6', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x07, 'tpdo8_mapping_application_object_7', 'Application object 7', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1A07, 0x08, 'tpdo8_mapping_application_object_8', 'Application object 8', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x1F80, 0x00, 'nmt_startup', 'NMT startup', 'U32', 'rw', '', 0, False, None, None, 'comm', '', None),
    Entry(0x2000, 0x00, 'firmware_build_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 9, False, None, None, 'none', '', None),
    Entry(0x2000, 0x01, 'firmware_build_app_version', 'Application version', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x02, 'firmware_build_hal_sha', 'HAL git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x03, 'firmware_build_core_sha', 'Core git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x04, 'firmware_build_tmc9660_driver_sha', 'TMC9660 driver git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x05, 'firmware_build_as5047u_driver_sha', 'AS5047U driver git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x06, 'firmware_build_bno08x_driver_sha', 'BNO08x driver git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x07, 'firmware_build_canopennode_sha', 'CANopenNode git SHA', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2000, 0x08, 'firmware_build_build_time', 'Build time', 'U32', 'const', '', 0, False, None, None, 'none', 's', None),
    Entry(0x2000, 0x09, 'firmware_build_idf_version', 'IDF version', 'VS', 'const', '', '', False, None, None, 'none', '', None),
    Entry(0x2001, 0x00, 'board_info_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'none', '', None),
    Entry(0x2001, 0x01, 'board_info_hw_revision', 'Hardware revision', 'U8', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2001, 0x02, 'board_info_mac', 'MAC address', 'OS', 'ro', '', b'\x00\x00\x00\x00\x00\x00', False, None, None, 'none', '', None),
    Entry(0x2001, 0x03, 'board_info_reset_reason', 'Reset reason', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2001, 0x04, 'board_info_uptime', 'Uptime', 'U32', 'ro', '', 0, False, None, None, 'none', 's', None),
    Entry(0x2001, 0x05, 'board_info_free_heap', 'Free heap', 'U32', 'ro', '', 0, False, None, None, 'none', 'B', None),
    Entry(0x2002, 0x00, 'component_versions_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x2002, 0x01, 'component_versions_tmc9660_firmware_id', 'TMC9660 firmware id', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2002, 0x02, 'component_versions_tmc9660_bootloader_id', 'TMC9660 bootloader id', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2002, 0x03, 'component_versions_bno08x_sh2_version', 'BNO08x SH-2 version', 'VS', 'ro', '', '', False, None, None, 'none', '', None),
    Entry(0x2002, 0x04, 'component_versions_as5047u_id', 'AS5047U id', 'U16', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2101, 0x00, 'encoder_status_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 8, False, None, None, 'none', '', None),
    Entry(0x2101, 0x01, 'encoder_status_angle_raw', 'Angle raw', 'U16', 'ro', 't', 0, False, None, None, 'none', 'counts', None),
    Entry(0x2101, 0x02, 'encoder_status_angle_compensated', 'Angle compensated', 'U16', 'ro', 't', 0, False, None, None, 'none', 'counts', None),
    Entry(0x2101, 0x03, 'encoder_status_velocity_raw', 'Velocity raw', 'I16', 'ro', 't', 0, False, None, None, 'none', 'counts/sample', None),
    Entry(0x2101, 0x04, 'encoder_status_multi_turn_position', 'Multi-turn position', 'I32', 'ro', 't', 0, False, None, None, 'none', 'counts', None),
    Entry(0x2101, 0x05, 'encoder_status_agc', 'AGC', 'U8', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2101, 0x06, 'encoder_status_magnitude', 'Magnitude', 'U16', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2101, 0x07, 'encoder_status_error_flags', 'Error flags', 'U16', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2101, 0x08, 'encoder_status_comm_errors', 'Communication errors', 'U32', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2210, 0x00, 'imu_packed_quaternion_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x2210, 0x01, 'imu_packed_quaternion_i', 'Quaternion i', 'I16', 'ro', 't', 0, False, None, None, 'none', '', 6.103515625e-05),
    Entry(0x2210, 0x02, 'imu_packed_quaternion_j', 'Quaternion j', 'I16', 'ro', 't', 0, False, None, None, 'none', '', 6.103515625e-05),
    Entry(0x2210, 0x03, 'imu_packed_quaternion_k', 'Quaternion k', 'I16', 'ro', 't', 0, False, None, None, 'none', '', 6.103515625e-05),
    Entry(0x2210, 0x04, 'imu_packed_quaternion_real', 'Quaternion real', 'I16', 'ro', 't', 0, False, None, None, 'none', '', 6.103515625e-05),
    Entry(0x2211, 0x00, 'imu_packed_accel_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x2211, 0x01, 'imu_packed_accel_x', 'Acceleration x', 'I16', 'ro', 't', 0, False, None, None, 'none', 'm/s²', 0.00390625),
    Entry(0x2211, 0x02, 'imu_packed_accel_y', 'Acceleration y', 'I16', 'ro', 't', 0, False, None, None, 'none', 'm/s²', 0.00390625),
    Entry(0x2211, 0x03, 'imu_packed_accel_z', 'Acceleration z', 'I16', 'ro', 't', 0, False, None, None, 'none', 'm/s²', 0.00390625),
    Entry(0x2211, 0x04, 'imu_packed_accel_sample_counter', 'Sample counter', 'U16', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2212, 0x00, 'imu_packed_gyro_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'none', '', None),
    Entry(0x2212, 0x01, 'imu_packed_gyro_x', 'Angular rate x', 'I16', 'ro', 't', 0, False, None, None, 'none', 'rad/s', 0.0009765625),
    Entry(0x2212, 0x02, 'imu_packed_gyro_y', 'Angular rate y', 'I16', 'ro', 't', 0, False, None, None, 'none', 'rad/s', 0.0009765625),
    Entry(0x2212, 0x03, 'imu_packed_gyro_z', 'Angular rate z', 'I16', 'ro', 't', 0, False, None, None, 'none', 'rad/s', 0.0009765625),
    Entry(0x2212, 0x04, 'imu_packed_gyro_calibration_status', 'Calibration status', 'U8', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2212, 0x05, 'imu_packed_gyro_accuracy', 'Accuracy', 'U8', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2300, 0x00, 'temperatures_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x2300, 0x01, 'temperatures_ntc', 'Power stage NTC', 'I16', 'ro', 't', 0, False, None, None, 'none', '°C', 0.1),
    Entry(0x2300, 0x02, 'temperatures_tmc_chip', 'TMC9660 chip', 'I16', 'ro', 't', 0, False, None, None, 'none', '°C', 0.1),
    Entry(0x2300, 0x03, 'temperatures_esp32_die', 'ESP32 die', 'I16', 'ro', 't', 0, False, None, None, 'none', '°C', 0.1),
    Entry(0x2300, 0x04, 'temperatures_motor', 'Motor', 'I16', 'ro', 't', 0, False, None, None, 'none', '°C', 0.1),
    Entry(0x2301, 0x00, 'power_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 3, False, None, None, 'none', '', None),
    Entry(0x2301, 0x01, 'power_supply_voltage', 'Supply voltage', 'U16', 'ro', 't', 0, False, None, None, 'none', 'mV', None),
    Entry(0x2301, 0x02, 'power_driver_voltage', 'Driver voltage', 'U16', 'ro', 't', 0, False, None, None, 'none', 'mV', None),
    Entry(0x2301, 0x03, 'power_power_good', 'Power good', 'BOOL', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x2700, 0x00, 'control_authority_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 2, False, None, None, 'none', '', None),
    Entry(0x2700, 0x01, 'control_authority_owner', 'Current owner', 'U8', 'ro', 't', 0, False, None, 3, 'none', '', None),
    Entry(0x2700, 0x02, 'control_authority_request', 'Request', 'U8', 'rw', '', 0, False, None, 3, 'none', '', None),
    Entry(0x2701, 0x00, 'authority_policy_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'app', '', None),
    Entry(0x2701, 0x01, 'authority_policy_enable_transports', 'Allowed enable transports', 'U8', 'rw', '', 1, False, None, 7, 'app', '', None),
    Entry(0x2701, 0x02, 'authority_policy_wireless_deadman', 'Wireless deadman', 'U16', 'rw', '', 200, False, 20, None, 'app', 'ms', None),
    Entry(0x2701, 0x03, 'authority_policy_require_master_liveness', 'Require master liveness', 'BOOL', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x2701, 0x04, 'authority_policy_wireless_service', 'Wireless service', 'BOOL', 'rw', '', 0, False, None, None, 'app', '', None),
    Entry(0x2800, 0x00, 'parameter_store_meta_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 5, False, None, None, 'none', '', None),
    Entry(0x2800, 0x01, 'parameter_store_meta_store_count', 'Store count', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2800, 0x02, 'parameter_store_meta_last_store_result', 'Last store result', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2800, 0x03, 'parameter_store_meta_stored_od_hash', 'OD hash at last store', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2800, 0x04, 'parameter_store_meta_records_applied', 'Records applied at restore', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x2800, 0x05, 'parameter_store_meta_records_dropped', 'Records dropped at restore', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4000, 0x00, 'fault_journal_header_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 4, False, None, None, 'none', '', None),
    Entry(0x4000, 0x01, 'fault_journal_header_count', 'Entry count', 'U16', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4000, 0x02, 'fault_journal_header_head', 'Head', 'U16', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4000, 0x03, 'fault_journal_header_capacity', 'Capacity', 'U16', 'ro', '', 64, False, None, None, 'none', '', None),
    Entry(0x4000, 0x04, 'fault_journal_header_overflow_count', 'Overflow count', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4001, 0x00, 'fault_journal', 'Fault journal', 'DOM', 'ro', '', None, False, None, None, 'none', '', None),
    Entry(0x4002, 0x00, 'fault_journal_clear', 'Fault journal clear', 'U32', 'wo', '', 0, False, None, None, 'none', '', None),
    Entry(0x4003, 0x00, 'active_causes_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 2, False, None, None, 'none', '', None),
    Entry(0x4003, 0x01, 'active_causes_active', 'Active causes', 'U32', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x4003, 0x02, 'active_causes_escalated', 'Escalated causes', 'U32', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x4200, 0x00, 'thread_stats_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 10, False, None, None, 'none', '', None),
    Entry(0x4200, 0x01, 'thread_stats_drive_rt_last', 'drive_rt last step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x02, 'thread_stats_drive_rt_max', 'drive_rt max step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x03, 'thread_stats_drive_rt_overruns', 'drive_rt overruns', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4200, 0x04, 'thread_stats_co_svc_last', 'co_svc last step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x05, 'thread_stats_co_svc_max', 'co_svc max step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x06, 'thread_stats_co_svc_overruns', 'co_svc overruns', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4200, 0x07, 'thread_stats_i2c_io_last', 'i2c_io last step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x08, 'thread_stats_i2c_io_max', 'i2c_io max step', 'U32', 'ro', '', 0, False, None, None, 'none', 'µs', None),
    Entry(0x4200, 0x09, 'thread_stats_i2c_io_overruns', 'i2c_io overruns', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4200, 0x0A, 'thread_stats_liveness_stale_mask', 'Liveness stale mask', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x00, 'timing_histogram_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 16, False, None, None, 'none', '', None),
    Entry(0x4201, 0x01, 'timing_histogram_jitter_le_10', 'Jitter up to 10 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x02, 'timing_histogram_jitter_le_20', 'Jitter up to 20 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x03, 'timing_histogram_jitter_le_50', 'Jitter up to 50 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x04, 'timing_histogram_jitter_le_100', 'Jitter up to 100 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x05, 'timing_histogram_jitter_le_200', 'Jitter up to 200 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x06, 'timing_histogram_jitter_le_500', 'Jitter up to 500 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x07, 'timing_histogram_jitter_le_1000', 'Jitter up to 1000 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x08, 'timing_histogram_jitter_gt_1000', 'Jitter above 1000 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x09, 'timing_histogram_wcet_le_100', 'Step up to 100 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0A, 'timing_histogram_wcet_le_150', 'Step up to 150 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0B, 'timing_histogram_wcet_le_200', 'Step up to 200 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0C, 'timing_histogram_wcet_le_250', 'Step up to 250 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0D, 'timing_histogram_wcet_le_300', 'Step up to 300 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0E, 'timing_histogram_wcet_le_500', 'Step up to 500 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x0F, 'timing_histogram_wcet_le_1000', 'Step up to 1000 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4201, 0x10, 'timing_histogram_wcet_gt_1000', 'Step above 1000 us', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x00, 'can_stats_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 10, False, None, None, 'none', '', None),
    Entry(0x4202, 0x01, 'can_stats_tec', 'TEC', 'U16', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x02, 'can_stats_rec', 'REC', 'U16', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x03, 'can_stats_state', 'Bus state', 'U8', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x04, 'can_stats_bus_off_count', 'Bus-off count', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x05, 'can_stats_rx_overruns', 'RX overruns', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x06, 'can_stats_tx_queue_full', 'TX queue full', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x07, 'can_stats_drops_can', 'Router drops CAN', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x08, 'can_stats_drops_wifi', 'Router drops Wi-Fi', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x09, 'can_stats_drops_espnow', 'Router drops ESP-NOW', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x4202, 0x0A, 'can_stats_drops_host', 'Router drops host', 'U32', 'ro', '', 0, False, None, None, 'none', '', None),
    Entry(0x5FFE, 0x00, 'od_layout_hash', 'OD layout hash', 'U32', 'const', '', 4286932508, False, None, None, 'none', '', None),
    Entry(0x5FFF, 0x00, 'manufacturer_od_version', 'Manufacturer OD version', 'U16', 'const', '', 256, False, None, None, 'none', '', None),
    Entry(0x6007, 0x00, 'abort_connection_option_code', 'Abort connection option code', 'I16', 'rw', '', 1, False, 0, 3, 'app', '', None),
    Entry(0x603F, 0x00, 'error_code', 'Error code', 'U16', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x6040, 0x00, 'controlword', 'Controlword', 'U16', 'rw', 'r', 0, False, None, None, 'none', '', None),
    Entry(0x6041, 0x00, 'statusword', 'Statusword', 'U16', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x605A, 0x00, 'quick_stop_option_code', 'Quick stop option code', 'I16', 'rw', '', 2, False, 0, 8, 'app', '', None),
    Entry(0x605B, 0x00, 'shutdown_option_code', 'Shutdown option code', 'I16', 'rw', '', 0, False, 0, 1, 'app', '', None),
    Entry(0x605C, 0x00, 'disable_operation_option_code', 'Disable operation option code', 'I16', 'rw', '', 1, False, 0, 1, 'app', '', None),
    Entry(0x605D, 0x00, 'halt_option_code', 'Halt option code', 'I16', 'rw', '', 1, False, 1, 4, 'app', '', None),
    Entry(0x605E, 0x00, 'fault_reaction_option_code', 'Fault reaction option code', 'I16', 'rw', '', 0, False, 0, 4, 'app', '', None),
    Entry(0x6060, 0x00, 'modes_of_operation', 'Modes of operation', 'I8', 'rw', 'r', 0, False, None, None, 'none', '', None),
    Entry(0x6061, 0x00, 'modes_of_operation_display', 'Modes of operation display', 'I8', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x6062, 0x00, 'position_demand_value', 'Position demand value', 'I32', 'ro', 't', 0, False, None, None, 'none', 'inc', None),
    Entry(0x6064, 0x00, 'position_actual_value', 'Position actual value', 'I32', 'ro', 't', 0, False, None, None, 'none', 'inc', None),
    Entry(0x6065, 0x00, 'following_error_window', 'Following error window', 'U32', 'rw', '', 4294967295, False, None, None, 'app', 'inc', None),
    Entry(0x6066, 0x00, 'following_error_time_out', 'Following error time out', 'U16', 'rw', '', 0, False, None, None, 'app', 'ms', None),
    Entry(0x6067, 0x00, 'position_window', 'Position window', 'U32', 'rw', '', 4294967295, False, None, None, 'app', 'inc', None),
    Entry(0x6068, 0x00, 'position_window_time', 'Position window time', 'U16', 'rw', '', 0, False, None, None, 'app', 'ms', None),
    Entry(0x606B, 0x00, 'velocity_demand_value', 'Velocity demand value', 'I32', 'ro', 't', 0, False, None, None, 'none', 'inc/s', None),
    Entry(0x606C, 0x00, 'velocity_actual_value', 'Velocity actual value', 'I32', 'ro', 't', 0, False, None, None, 'none', 'inc/s', None),
    Entry(0x606D, 0x00, 'velocity_window', 'Velocity window', 'U16', 'rw', '', 0, False, None, None, 'app', 'inc/s', None),
    Entry(0x606E, 0x00, 'velocity_window_time', 'Velocity window time', 'U16', 'rw', '', 0, False, None, None, 'app', 'ms', None),
    Entry(0x6071, 0x00, 'target_torque', 'Target torque', 'I16', 'rw', 'r', 0, False, None, None, 'none', '‰ of 0x6076', None),
    Entry(0x6072, 0x00, 'max_torque', 'Max torque', 'U16', 'rw', '', 1000, False, None, None, 'app', '‰', None),
    Entry(0x6073, 0x00, 'max_current', 'Max current', 'U16', 'rw', '', 1000, False, None, None, 'app', '‰', None),
    Entry(0x6075, 0x00, 'motor_rated_current', 'Motor rated current', 'U32', 'rw', '', 0, False, None, None, 'app', 'mA', None),
    Entry(0x6076, 0x00, 'motor_rated_torque', 'Motor rated torque', 'U32', 'rw', '', 0, False, None, None, 'app', 'mNm', None),
    Entry(0x6077, 0x00, 'torque_actual_value', 'Torque actual value', 'I16', 'ro', 't', 0, False, None, None, 'none', '‰', None),
    Entry(0x6078, 0x00, 'current_actual_value', 'Current actual value', 'I16', 'ro', 't', 0, False, None, None, 'none', '‰', None),
    Entry(0x6079, 0x00, 'dc_link_circuit_voltage', 'DC link circuit voltage', 'U32', 'ro', 't', 0, False, None, None, 'none', 'mV', None),
    Entry(0x607A, 0x00, 'target_position', 'Target position', 'I32', 'rw', 'r', 0, False, None, None, 'none', 'inc', None),
    Entry(0x607C, 0x00, 'home_offset', 'Home offset', 'I32', 'rw', '', 0, False, None, None, 'app', 'inc', None),
    Entry(0x607D, 0x00, 'software_position_limit_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'app', '', None),
    Entry(0x607D, 0x01, 'software_position_limit_min', 'Min position limit', 'I32', 'rw', '', 0, False, None, None, 'app', 'inc', None),
    Entry(0x607D, 0x02, 'software_position_limit_max', 'Max position limit', 'I32', 'rw', '', 0, False, None, None, 'app', 'inc', None),
    Entry(0x607E, 0x00, 'polarity', 'Polarity', 'U8', 'rw', '', 0, False, None, None, 'app', '', None),
    Entry(0x607F, 0x00, 'max_profile_velocity', 'Max profile velocity', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s', None),
    Entry(0x6080, 0x00, 'max_motor_speed', 'Max motor speed', 'U32', 'rw', '', 0, False, None, None, 'app', 'rpm', None),
    Entry(0x6081, 0x00, 'profile_velocity', 'Profile velocity', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s', None),
    Entry(0x6083, 0x00, 'profile_acceleration', 'Profile acceleration', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s²', None),
    Entry(0x6084, 0x00, 'profile_deceleration', 'Profile deceleration', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s²', None),
    Entry(0x6085, 0x00, 'quick_stop_deceleration', 'Quick stop deceleration', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s²', None),
    Entry(0x6086, 0x00, 'motion_profile_type', 'Motion profile type', 'I16', 'rw', '', 0, False, 0, 0, 'app', '', None),
    Entry(0x6087, 0x00, 'torque_slope', 'Torque slope', 'U32', 'rw', '', 0, False, None, None, 'app', '‰/s', None),
    Entry(0x608F, 0x00, 'position_encoder_resolution_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'app', '', None),
    Entry(0x608F, 0x01, 'position_encoder_resolution_encoder_increments', 'Encoder increments', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x608F, 0x02, 'position_encoder_resolution_motor_revolutions', 'Motor revolutions', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x6091, 0x00, 'gear_ratio_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'app', '', None),
    Entry(0x6091, 0x01, 'gear_ratio_motor_revolutions', 'Motor revolutions', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x6091, 0x02, 'gear_ratio_shaft_revolutions', 'Shaft revolutions', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x6092, 0x00, 'feed_constant_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'app', '', None),
    Entry(0x6092, 0x01, 'feed_constant_feed', 'Feed', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x6092, 0x02, 'feed_constant_shaft_revolutions', 'Shaft revolutions', 'U32', 'rw', '', 1, False, None, None, 'app', '', None),
    Entry(0x6098, 0x00, 'homing_method', 'Homing method', 'I8', 'rw', '', 0, False, None, None, 'app', '', None),
    Entry(0x6099, 0x00, 'homing_speeds_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'app', '', None),
    Entry(0x6099, 0x01, 'homing_speeds_switch_search', 'Speed during search for switch', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s', None),
    Entry(0x6099, 0x02, 'homing_speeds_zero_search', 'Speed during search for zero', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s', None),
    Entry(0x609A, 0x00, 'homing_acceleration', 'Homing acceleration', 'U32', 'rw', '', 0, False, None, None, 'app', 'inc/s²', None),
    Entry(0x60B0, 0x00, 'position_offset', 'Position offset', 'I32', 'rw', 'r', 0, False, None, None, 'none', 'inc', None),
    Entry(0x60B1, 0x00, 'velocity_offset', 'Velocity offset', 'I32', 'rw', 'r', 0, False, None, None, 'none', 'inc/s', None),
    Entry(0x60B2, 0x00, 'torque_offset', 'Torque offset', 'I16', 'rw', 'r', 0, False, None, None, 'none', '‰', None),
    Entry(0x60C2, 0x00, 'interpolation_time_period_highest_sub_index', 'Highest sub-index supported', 'U8', 'const', '', 2, False, None, None, 'none', '', None),
    Entry(0x60C2, 0x01, 'interpolation_time_period_value', 'Interpolation time period value', 'U8', 'rw', '', 1, False, None, None, 'none', '', None),
    Entry(0x60C2, 0x02, 'interpolation_time_period_index', 'Interpolation time index', 'I8', 'rw', '', -3, False, -6, -3, 'none', '', None),
    Entry(0x60E0, 0x00, 'positive_torque_limit_value', 'Positive torque limit value', 'U16', 'rw', '', 1000, False, None, None, 'app', '‰', None),
    Entry(0x60E1, 0x00, 'negative_torque_limit_value', 'Negative torque limit value', 'U16', 'rw', '', 1000, False, None, None, 'app', '‰', None),
    Entry(0x60E4, 0x00, 'additional_position_actual_value_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 1, False, None, None, 'none', '', None),
    Entry(0x60E4, 0x01, 'additional_position_actual_value_host_encoder', 'Host encoder position', 'I32', 'ro', 't', 0, False, None, None, 'none', 'counts', None),
    Entry(0x60F4, 0x00, 'following_error_actual_value', 'Following error actual value', 'I32', 'ro', 't', 0, False, None, None, 'none', 'inc', None),
    Entry(0x60FD, 0x00, 'digital_inputs', 'Digital inputs', 'U32', 'ro', 't', 0, False, None, None, 'none', '', None),
    Entry(0x60FE, 0x00, 'digital_outputs_highest_sub_index', 'Highest sub-index supported', 'U8', 'ro', '', 2, False, None, None, 'none', '', None),
    Entry(0x60FE, 0x01, 'digital_outputs_physical_outputs', 'Physical outputs', 'U32', 'rw', 'r', 0, False, None, None, 'none', '', None),
    Entry(0x60FE, 0x02, 'digital_outputs_bit_mask', 'Bit mask', 'U32', 'rw', 'r', 0, False, None, None, 'none', '', None),
    Entry(0x60FF, 0x00, 'target_velocity', 'Target velocity', 'I32', 'rw', 'r', 0, False, None, None, 'none', 'inc/s', None),
    Entry(0x6402, 0x00, 'motor_type', 'Motor type', 'U16', 'rw', '', 0, False, None, None, 'app', '', None),
    Entry(0x6403, 0x00, 'motor_catalogue_number', 'Motor catalogue number', 'VS', 'rw', '', '', False, None, None, 'app', '', None),
    Entry(0x6404, 0x00, 'motor_manufacturer', 'Motor manufacturer', 'VS', 'rw', '', '', False, None, None, 'app', '', None),
    Entry(0x6502, 0x00, 'supported_drive_modes', 'Supported drive modes', 'U32', 'const', '', 941, False, None, None, 'none', '', None),
)

BY_KEY: dict[str, Entry] = {e.key: e for e in ENTRIES}
BY_ADDRESS: dict[tuple[int, int], Entry] = {(e.index, e.sub): e for e in ENTRIES}
