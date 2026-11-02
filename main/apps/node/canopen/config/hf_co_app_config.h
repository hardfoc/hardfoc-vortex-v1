/**
 * @file hf_co_app_config.h
 * @brief CANopenNode stack configuration of the Vortex V1 device.
 *
 * @details hf-core's CO_driver_target.h includes this file before any stack
 *          module header (the directory is HF_CORE_CANOPENNODE_CONFIG_DIR),
 *          so these values replace the CANopenNode defaults. The firmware and
 *          the host node binary (host/node) build with the same file.
 *
 *          Differences from the defaults (design §2.1 G1, §10.1):
 *          - SDO server: segmented and block transfer (block needs the FIFO
 *            with alternate read and CRC-16, and a buffer of at least 889
 *            bytes, 127 segments of 7 bytes);
 *          - EMCY producer: configurable COB-ID (0x1014) and inhibit time
 *            (0x1015);
 *          - LSS slave: fastscan answers directly;
 *          - timerNext: CO_process() reports when it next needs to run, so
 *            co_svc sleeps until then or until a frame arrives.
 */
#ifndef HF_CO_APP_CONFIG_H
#define HF_CO_APP_CONFIG_H

#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT CO_CONFIG_FLAG_TIMERNEXT

#define CO_CONFIG_SDO_SRV                                                                                              \
  (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_SDO_SRV_BLOCK | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE                         \
   | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 1024

#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE | CO_CONFIG_FIFO_ALT_READ | CO_CONFIG_FIFO_CRC16_CCITT)
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE)

#define CO_CONFIG_EM                                                                                                   \
  (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_PROD_CONFIGURABLE | CO_CONFIG_EM_PROD_INHIBIT | CO_CONFIG_EM_HISTORY          \
   | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

#define CO_CONFIG_LSS (CO_CONFIG_LSS_SLAVE | CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)

#endif /* HF_CO_APP_CONFIG_H */
