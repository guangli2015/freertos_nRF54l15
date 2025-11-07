/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include  "log.h"
#define APP_BLE_OBSERVER_PRIO 3
#define TEST_CONFIG_STATE_OBSERVER_PRIO 0
#define TEST_CONFIG_SOC_OBSERVER_PRIO 0
#define CONFIG_NRF_SDH_BLE_CONN_TAG 99
static void on_ble_evt(const ble_evt_t *evt, void *ctx)
{
     LOG_INF("BLE event %d\r\n", evt->header.evt_id);
}
//NRF_SDH_BLE_OBSERVER(sdh_ble, on_ble_evt, NULL, 0);
NRF_SDH_BLE_OBSERVER(m_ble_observer, on_ble_evt, NULL, APP_BLE_OBSERVER_PRIO);
static void on_soc_evt(uint32_t evt, void *ctx)
{
    LOG_INF("SoC event\r\n");
	
}
//NRF_SDH_SOC_OBSERVER(sdh_soc, on_soc_evt, NULL, 0);
NRF_SDH_SOC_OBSERVER(m_soc_evt_observer, on_soc_evt, NULL,TEST_CONFIG_SOC_OBSERVER_PRIO);
static void on_state_change(enum nrf_sdh_state_evt state, void *ctx)
{
    LOG_INF("SoftDevice state has changed to %d\r\n", state);
        
}
//NRF_SDH_STATE_EVT_OBSERVER(sdh_state, on_state_change, NULL, 0);
//NRF_SDH_STATE_OBSERVER
NRF_SDH_STATE_OBSERVER(m_sd_state_observer, TEST_CONFIG_STATE_OBSERVER_PRIO) =
{
    .handler   = on_state_change,
    .context = NULL,
};
bool on_req_change(enum nrf_sdh_state_req request, void *context)
{
   LOG_INF("SoftDevice req has changed to %d\r\n", request);
}
NRF_SDH_REQUEST_OBSERVER(m_sd_req_observer, 0) =
{
    .handler   = on_req_change,
    .context = NULL,
};
int sftdevice_test(void)
{
	int err;

	LOG_INF("Hello SoftDevice sample started\r\n");

	err = nrf_sdh_enable_request();
	if (err) {
		
		goto idle;
	}

	LOG_INF("SoftDevice enabled\r\n");

	err = nrf_sdh_ble_enable(CONFIG_NRF_SDH_BLE_CONN_TAG);
	if (err) {
		
		goto idle;
	}

	LOG_INF("Bluetooth enabled\r\n");

	/*while (LOG_PROCESS()) {
	}

	k_busy_wait(2 * USEC_PER_SEC);*/
        

    volatile uint32_t i;
    for (i = 0; i < 12800000; i++) {
        __NOP();  // 防止优化掉循环
    }



	err = nrf_sdh_disable_request();
	if (err) {
		LOG_INF("Failed to disable SoftDevice, err %d\n", err);
		goto idle;
	}

	LOG_INF("SoftDevice disabled\r\n");
	LOG_INF("Bye\r\n");

idle:
#if 0
	while (true) {
		

		/* Wait for an event. */
		__WFE();

		/* Clear Event Register */
		__SEV();
		__WFE();
	}
#endif
	return 0;
}
