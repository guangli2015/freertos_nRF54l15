/* errno is not a global variable, because that would make using it
   non-reentrant.  Instead, its address is returned by the function
   __errno.  */

#ifndef _PRJ_CONFIG_H_
#define _PRJ_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


//data_length.c
#define CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_TX 27
#define CONFIG_BLE_CONN_PARAMS_DATA_LENGTH_RX 27
//phy_mode.c
#define CONFIG_BLE_CONN_PARAMS_PHY 0x00
#define CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH 3
#ifdef __cplusplus
}
#endif
#endif /* _PRJ_CONFIG_H_ */
