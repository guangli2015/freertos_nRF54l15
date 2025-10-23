/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @defgroup nrf_sdh_soc SoC support in SoftDevice Handler
 * @{
 * @ingroup  nrf_sdh
 * @brief    Declarations of types and functions required for SoftDevice Handler SoC support.
 */

#ifndef NRF_SDH_SOC_H__
#define NRF_SDH_SOC_H__

#include <stdint.h>
#include "nrf_section_iter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SoftDevice SoC event handler.
 */
typedef void (*nrf_sdh_soc_evt_handler_t)(uint32_t evt_id, void *context);

/**
 * @brief SoftDevice SoC event observer.
 */
typedef struct  
{
	/**
	 * @brief SoC event handler.
	 */
	nrf_sdh_soc_evt_handler_t handler;
	/**
	 * @brief A parameter to the event handler.
	 */
	void *context;
} const nrf_sdh_soc_evt_observer;

/**@brief   Macro for registering @ref nrf_sdh_soc_evt_observer. Modules that want to be
 *          notified about SoC events must register the handler using this macro.
 *
 * @details This macro places the observer in a section named "sdh_soc_observers".
 *
 * @param[in]   _name       Observer name.
 * @param[in]   _prio       Priority of the observer event handler.
 *                          The smaller the number, the higher the priority.
 * @param[in]   _handler    SoC event handler.
 * @param[in]   _context    Parameter to the event handler.
 * @hideinitializer
 */
#define NRF_SDH_SOC_OBSERVER(_name, _handler, _context, _prio)                                      \
NRF_SECTION_SET_ITEM_REGISTER(sdh_soc_observers, _prio, static nrf_sdh_soc_evt_observer _name) =  \
{                                                                                                   \
    .handler   = _handler,                                                                          \
    .context = _context                                                                           \
}

#ifdef __cplusplus
}
#endif

#endif /* NRF_SDH_SOC_H__ */

/** @} */
