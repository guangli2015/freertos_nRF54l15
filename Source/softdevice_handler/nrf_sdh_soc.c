/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <nrf_sdh.h>
#include <nrf_sdh_soc.h>
#include <nrf_soc.h>
#include "log.h"

#define CONFIG_NRF_SDH_STR_TABLES 1
#define NRF_SDH_SOC_STACK_OBSERVER_PRIO 0
#define LOG_DBG
#define __ASSERT
//LOG_MODULE_DECLARE(nrf_sdh, CONFIG_NRF_SDH_LOG_LEVEL);
// Create section set "sdh_soc_observers".
#define NRF_SDH_SOC_OBSERVER_PRIO_LEVELS 2
NRF_SECTION_SET_DEF(sdh_soc_observers, nrf_sdh_soc_evt_observer, NRF_SDH_SOC_OBSERVER_PRIO_LEVELS);
static const char *tostr(uint32_t evt)
{
	switch (evt) {
	case NRF_EVT_HFCLKSTARTED:
		return "The HFCLK has started";
	case NRF_EVT_POWER_FAILURE_WARNING:
		return "A power failure warning has occurred";
	case NRF_EVT_FLASH_OPERATION_SUCCESS:
		return "Flash operation has completed successfully";
	case NRF_EVT_FLASH_OPERATION_ERROR:
		return "Flash operation has timed out with an error";
	case NRF_EVT_RADIO_BLOCKED:
		return "A radio timeslot was blocked";
	case NRF_EVT_RADIO_CANCELED:
		return "A radio timeslot was canceled by SoftDevice";
	case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
		return "A radio timeslot signal callback handler return was invalid";
	case NRF_EVT_RADIO_SESSION_IDLE:
		return "A radio timeslot session is idle";
	case NRF_EVT_RADIO_SESSION_CLOSED:
		return "A radio timeslot session is closed";
	case NRF_EVT_RAND_SEED_REQUEST:
		return "SoftDevice RNG needs to be seeded";
	default:
		return "Unknown";
	}
}
static uint8_t se=6;
static void softdevice_rng_seed(void)
{
#if 1
	uint32_t err = NRF_ERROR_INVALID_DATA;
	//psa_status_t status;
	uint8_t seed[SD_RAND_SEED_SIZE];
        memset(seed, 0, sizeof(seed));
        for(int i=0;i<SD_RAND_SEED_SIZE;i++)
        {
          seed[i] = (uint8_t)(0xA5 ^ i);  // 或者用硬件 RNG
        }
	//status = psa_generate_random(seed, sizeof(seed));
	//if (status == PSA_SUCCESS) {
		err = sd_rand_seed_set(seed);
                //err = NRF_SUCCESS;
		//memset(seed, 0, sizeof(seed));
		if (err == NRF_SUCCESS) {
			LOG_INF("SoftDevice RNG seeded\r\n");
			return;
		}
	//} else {
	//	LOG_ERR("Generate random failed, psa status %d", status);
	//}

	LOG_INF("Failed to seed SoftDevice RNG, nrf_error %#x\r\n", err);
#endif
}

static void soc_evt_poll(void *context)
{
	uint32_t err;
	uint32_t evt_id;

	while (true) {
		err = sd_evt_get(&evt_id);
		if (err != NRF_SUCCESS) {
			break;
		}

		if ((CONFIG_NRF_SDH_STR_TABLES)) {
			LOG_INF("SoC event: %s\r\n", tostr(evt_id));
		} else {
			LOG_INF("SoC event: 0x%x", evt_id);
		}

		if (evt_id == NRF_EVT_RAND_SEED_REQUEST) {
			softdevice_rng_seed();
		}

		/* Forward the event to SoC observers. 
		TYPE_SECTION_FOREACH(
			struct nrf_sdh_soc_evt_observer, nrf_sdh_soc_evt_observers, obs) {
			obs->handler(evt_id, obs->context);
		}*/

                        // Forward the event to SoC observers.
                nrf_section_iter_t  iter;
                for (nrf_section_iter_init(&iter, &sdh_soc_observers);
                     nrf_section_iter_get(&iter) != NULL;
                     nrf_section_iter_next(&iter))
                {
                    nrf_sdh_soc_evt_observer * p_observer;
                    nrf_sdh_soc_evt_handler_t    handler;

                    p_observer = (nrf_sdh_soc_evt_observer *) nrf_section_iter_get(&iter);
                    handler    = p_observer->handler;

                    handler(evt_id, p_observer->context);
                }
	}

	__ASSERT(err == NRF_ERROR_NOT_FOUND,
		 "Failed to receive SoftDevice event, nrf_error %#x", err);
}

/* Listen to SoftDevice events */
//NRF_SDH_STACK_EVT_OBSERVER(soc_evt_obs, soc_evt_poll, NULL, 0);
NRF_SDH_STACK_OBSERVER(m_nrf_sdh_soc_evts_poll, NRF_SDH_SOC_STACK_OBSERVER_PRIO) =
{
    .handler   = soc_evt_poll,
    .context = NULL,
};