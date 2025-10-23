/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @defgroup nrf_sdh_ble BLE support in SoftDevice Handler
 * @{
 * @ingroup  nrf_sdh
 * @brief    Declarations of types and functions required for BLE stack support.
 */

#ifndef NRF_SDH_BLE_H__
#define NRF_SDH_BLE_H__

#include <stdint.h>
#include <ble.h>
#include "nrf_section_iter.h"

#ifdef __cplusplus
extern "C" {
#endif
#define CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT 1
#define ARG_UNUSED(x) (void)(x)
/**
 * @brief Size of the buffer for a BLE event.
 */
#define NRF_SDH_BLE_EVT_BUF_SIZE BLE_EVT_LEN_MAX(CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE)

/**
 * @brief BLE stack event handler.
 */
typedef void (*nrf_sdh_ble_evt_handler_t)(const ble_evt_t *ble_evt, void *context);

/**
 * @brief BLE event observer.
 */
typedef struct  
{
	/**
	 * @brief BLE event handler.
	 */
	nrf_sdh_ble_evt_handler_t handler;
	/**
	 * @brief A parameter to the event handler.
	 */
	void *context;
} const nrf_sdh_ble_evt_observer;

/**@brief   Macro for registering @ref nrf_sdh_soc_evt_observer_t. Modules that want to be
 *          notified about SoC events must register the handler using this macro.
 *
 * @details This macro places the observer in a section named "sdh_soc_observers".
 *
 * @param[in]   _name       Observer name.
 * @param[in]   _prio       Priority of the observer event handler.
 *                          The smaller the number, the higher the priority.
 * @param[in]   _handler    BLE event handler.
 * @param[in]   _context    Parameter to the event handler.
 * @hideinitializer
 */
#define NRF_SDH_BLE_OBSERVER(_name, _handler, _context, _prio)                                      \
NRF_SECTION_SET_ITEM_REGISTER(sdh_ble_observers, _prio, static nrf_sdh_ble_evt_observer _name) =  \
{                                                                                                   \
    .handler   = _handler,                                                                          \
    .context = _context                                                                           \
}


/**
 * @brief Retrieve the starting address of the application's RAM.
 *
 * @param[out] app_ram_start The starting address of the application's RAM.
 *
 * @retval 0 On success.
 * @retval -EFAULT @p app_ram_start is @c NULL.
 */
int nrf_sdh_ble_app_ram_start_get(uint32_t *app_ram_start);

/**
 * @brief Enable the SoftDevice Bluetooth stack.
 *
 * @param[in] conn_tag Connection configuration tag.
 *
 * @retval 0 On success.
 */
int nrf_sdh_ble_enable(uint8_t conn_cfg_tag);

/**
 * @brief Get the assigned index for a connection handle.
 *
 * The returned value can be used for indexing into arrays where each element is associated
 * with a specific connection. Connection handles should never directly be used for indexing arrays.
 *
 * @param[in] conn_handle Connection handle.
 *
 * @returns An integer in the range from 0 to (CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT - 1) if the
 *          connection handle has been assigned to an index, otherwise -1.
 */
static inline int nrf_sdh_ble_idx_get(uint16_t conn_handle)
{
	/* Code size optimization when supporting only one connection. */
	if (CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT == 1) {
		ARG_UNUSED(conn_handle);
		return 0;
	}

	/* This is used when supporting multiple connections. */
	int _nrf_sdh_ble_idx_get(uint16_t conn_handle);
	return _nrf_sdh_ble_idx_get(conn_handle);
}

#ifdef __cplusplus
}
#endif

#endif /* NRF_SDH_BLE_H__ */

/** @} */
