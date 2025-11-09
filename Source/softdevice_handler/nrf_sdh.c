/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <stdint.h>
#include <nrf_sdh.h>
#include <nrf_sdm.h>
#include "log.h"


#define CONFIG_NRF_SDH_CLOCK_LF_SRC NRF_CLOCK_LF_SRC_XTAL
#define CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV 0
#define CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 0
#define CONFIG_NRF_SDH_CLOCK_LF_ACCURACY 0
#if ((CONFIG_NRF_SDH_CLOCK_LF_SRC == NRF_CLOCK_LF_SRC_RC) &&                                       \
	(CONFIG_NRF_SDH_CLOCK_LF_ACCURACY != NRF_CLOCK_LF_ACCURACY_500_PPM))
#warning Please select NRF_CLOCK_LF_ACCURACY_500_PPM when using NRF_CLOCK_LF_SRC_RC
#endif
#define LOG_DBG
#define LOG_ERR
#define CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ 1
#define	EBUSY 16	/* Device or resource busy */
#define EALREADY 120		/* Socket already connected */
#define	EINVAL 22	/* Invalid argument */
#define NRF_SDH_REQ_OBSERVER_PRIO_LEVELS 2
#define NRF_SDH_STATE_OBSERVER_PRIO_LEVELS 2
#define NRF_SDH_STACK_OBSERVER_PRIO_LEVELS 2
// Create section "sdh_req_observers".
NRF_SECTION_SET_DEF(sdh_req_observers, nrf_sdh_state_req_observer, NRF_SDH_REQ_OBSERVER_PRIO_LEVELS);

// Create section "sdh_state_observers".
NRF_SECTION_SET_DEF(sdh_state_observers, nrf_sdh_state_evt_observer, NRF_SDH_STATE_OBSERVER_PRIO_LEVELS);

// Create section "sdh_stack_observers".
NRF_SECTION_SET_DEF(sdh_stack_observers, nrf_sdh_stack_evt_observer, NRF_SDH_STACK_OBSERVER_PRIO_LEVELS);



typedef struct {
    volatile bool value;
} atomic_t;

void atomic_set(atomic_t *a, bool val) {
    __disable_irq();
    a->value = val;
    __enable_irq();
}

static atomic_t sdh_enabled;	/* Whether the SoftDevice is enabled. */
static atomic_t sdh_suspended;	/* Whether this module is suspended. */
static atomic_t sdh_transition; /* Whether enable/disable process was started. */

static char *req_tostr(enum nrf_sdh_state_req r)
{
	switch (r) {
	case NRF_SDH_STATE_REQ_ENABLE:
		return "enable";
	case NRF_SDH_STATE_REQ_DISABLE:
		return "disable";
	default:
		return "unknown";
	};
}
static char *state_tostr(enum nrf_sdh_state_evt s)
{
	switch (s) {
	case NRF_SDH_STATE_EVT_ENABLE_PREPARE:
		return "enabling";
	case NRF_SDH_STATE_EVT_ENABLED:
		return "enabled";
	case NRF_SDH_STATE_EVT_DISABLE_PREPARE:
		return "disabling";
	case NRF_SDH_STATE_EVT_DISABLED:
		return "disabled";
	default:
		return "unknown";
	};
}

static int sdh_state_req_observer_notify(enum nrf_sdh_state_req req)
{
	/*if (IS_ENABLED(CONFIG_NRF_SDH_STR_TABLES)) {
		LOG_INF("State change request: %s", req_tostr(req));
	} else {
		LOG_INF("State change request: %#x", req);
	}*/
        nrf_section_iter_t iter;
        LOG_INF("State change request: %s\r\n", req_tostr(req));

        for (nrf_section_iter_init(&iter, &sdh_req_observers);
         nrf_section_iter_get(&iter) != NULL;
         nrf_section_iter_next(&iter))
    {
        nrf_sdh_state_req_observer    * p_observer;
        nrf_sdh_state_req_handler_t   handler;

        p_observer = (nrf_sdh_state_req_observer *) nrf_section_iter_get(&iter);
        handler    = p_observer->handler;

        if (handler(req, p_observer->context))
        {
            LOG_DBG("Notify observer 0x%x => ready\r\n", p_observer);
        }
        else
        {
            // Process is stopped.
            LOG_INF("Notify observer 0x%08x => blocking\n", p_observer);
            return -EBUSY;
        }
    }

	return 0;
}

 void sdh_state_evt_observer_notify(enum nrf_sdh_state_evt state)
{
        nrf_section_iter_t iter;
        LOG_DBG("State change: %s", state_tostr(state));

        for (nrf_section_iter_init(&iter, &sdh_state_observers);
         nrf_section_iter_get(&iter) != NULL;
         nrf_section_iter_next(&iter))
        {
          nrf_sdh_state_evt_observer    * p_observer;
          nrf_sdh_state_evt_handler_t   handler;

          p_observer = (nrf_sdh_state_evt_observer *) nrf_section_iter_get(&iter);
          handler    = p_observer->handler;

          handler(state, p_observer->context);
        }
}

__WEAK void softdevice_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	LOG_INF("SoftDevice fault! ID %#x, PC %#x, Info %#x", id, pc, info);


	switch (id) {
	case NRF_FAULT_ID_SD_ASSERT:
		LOG_INF("NRF_FAULT_ID_SD_ASSERT: SoftDevice assert");
		break;
	case NRF_FAULT_ID_APP_MEMACC:
		LOG_INF("NRF_FAULT_ID_APP_MEMACC: Application bad memory access");
		if (info == 0x00) {
			LOG_INF("Application tried to access SoftDevice RAM");
		} else {
			LOG_INF("Application tried to access SoftDevice peripheral at %#x", info);
		}
		break;
	}

	for (;;) {
		/* loop */
	}
}

int nrf_sdh_enable_request(void)
{
	int err;
	const nrf_clock_lf_cfg_t clock_lf_cfg = {
		.source = CONFIG_NRF_SDH_CLOCK_LF_SRC,
		.rc_ctiv = CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV,
		.rc_temp_ctiv = CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV,
		.accuracy = CONFIG_NRF_SDH_CLOCK_LF_ACCURACY
	};

	if (sdh_enabled.value) {
		return -EALREADY;
	}

	atomic_set(&sdh_transition, true);
       

	err = sdh_state_req_observer_notify(NRF_SDH_STATE_REQ_ENABLE);
	if (err) {
		/** TODO: should this be Success instead? */
		/* Leave sdh_transition to 1, so process can be continued */
		//__ASSERT(err == -EBUSY, "Unknown return value %d from sdh req observer", err);
		return -EBUSY;
	}

	atomic_set(&sdh_transition, false);


	/* Notify observers about starting SoftDevice enable process. */
	sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLE_PREPARE);

	err = sd_softdevice_enable(&clock_lf_cfg, softdevice_fault_handler);
	if (err) {
		LOG_INF("Failed to enable SoftDevice, nrf_error %#x\n", err);
		return -EINVAL;
	}

	atomic_set(&sdh_enabled, true);
	atomic_set(&sdh_suspended, false);


	/* Enable event interrupt, the priority has already been set by the stack. */
	NVIC_EnableIRQ((IRQn_Type)SD_EVT_IRQn);

	/* Notify observers about a finished SoftDevice enable process. */
	sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLED);

	return 0;
}

int nrf_sdh_disable_request(void)
{
	int err;

	if (!(sdh_enabled.value)) {
		return -EALREADY;
	}

	atomic_set(&sdh_transition, true);


	/* Notify observers about SoftDevice disable request. */
	err = sdh_state_req_observer_notify(NRF_SDH_STATE_REQ_DISABLE);
	if (err) {
		/** TODO: should this be Success instead? */
		/* Leave sdh_transition to 1, so process can be continued */
		//__ASSERT(err == -EBUSY, "Unknown return value %d from sdh req observer", err);
		return -EBUSY;
	}

	atomic_set(&sdh_transition, false);


	/* Notify observers about starting SoftDevice disable process. */
	sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_DISABLE_PREPARE);

	err = sd_softdevice_disable();
	if (err) {
		LOG_ERR("Failed to disable SoftDevice, nrf_error %#x", err);
		return -EINVAL;
	}

	atomic_set(&sdh_enabled, false);

	NVIC_DisableIRQ((IRQn_Type)SD_EVT_IRQn);

	/* Notify observers about a finished SoftDevice enable process. */
	sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_DISABLED);

	return 0;
}

int nrf_sdh_request_continue(void)
{
	if (!(sdh_transition.value)) {
		return -EINVAL;
	}

	if (sdh_enabled.value) {
		return nrf_sdh_disable_request();
	} else {
		return nrf_sdh_enable_request();
	}
}

bool nrf_sdh_is_enabled(void)
{
	return sdh_enabled.value;
}

void nrf_sdh_suspend(void)
{
	if (!(sdh_enabled.value) || (sdh_suspended.value)) {
		return;
	}

	NVIC_DisableIRQ((IRQn_Type)SD_EVT_IRQn);

	atomic_set(&sdh_suspended, true);
}

void nrf_sdh_resume(void)
{
	if ((!(sdh_suspended.value)) || (!(sdh_enabled.value))) {
		return;
	}

	/* Force calling ISR again to make sure we dispatch pending events */
	NVIC_SetPendingIRQ((IRQn_Type)SD_EVT_IRQn);
	NVIC_EnableIRQ((IRQn_Type)SD_EVT_IRQn);

	atomic_set(&sdh_suspended, false);
}

bool nrf_sdh_is_suspended(void)
{
	return (!(sdh_enabled.value)) || (sdh_suspended.value);
}

void nrf_sdh_evts_poll(void)
{
	/* Notify observers about pending SoftDevice event. */
	//TYPE_SECTION_FOREACH(struct nrf_sdh_stack_evt_observer, nrf_sdh_stack_evt_observers, obs) {
	//	obs->handler(obs->context);
	//}

            nrf_section_iter_t iter;

    // Notify observers about pending SoftDevice event.
    for (nrf_section_iter_init(&iter, &sdh_stack_observers);
         nrf_section_iter_get(&iter) != NULL;
         nrf_section_iter_next(&iter))
    {
        nrf_sdh_stack_evt_observer    * p_observer;
        nrf_sdh_stack_evt_handler_t   handler;

        p_observer = (nrf_sdh_stack_evt_observer *) nrf_section_iter_get(&iter);
        handler    = p_observer->handler;

        handler(p_observer->context);
    }
}

#if defined(CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ)

void SD_EVT_IRQHandler(void)
{
	nrf_sdh_evts_poll();
}

#elif defined(CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED)

static void sdh_events_poll(void *data, size_t len)
{
	(void)data;
	(void)len;

	nrf_sdh_evts_poll();
}

void SD_EVT_IRQHandler(void)
{
	int err;

	err = event_scheduler_defer(sdh_events_poll, NULL, 0);
	if (err) {
		LOG_WRN("Unable to schedule SoftDevice event, err %d", err);
	}
}

#elif defined(CONFIG_NRF_SDH_DISPATCH_MODEL_POLL)

#endif /* NRF_SDH_DISPATCH_MODEL */
#if 0
static void isr_handler(const void *arg)
{
	//ARG_UNUSED(arg);
	SD_EVT_IRQHandler();
}
#endif
 int sdh_irq_init(void)
{
	//IRQ_CONNECT(SD_EVT_IRQn, 4, isr_handler, NULL, 0);
	//irq_enable(SD_EVT_IRQn);
        NVIC_SetPriority(SD_EVT_IRQn, 4);
NVIC_EnableIRQ(SD_EVT_IRQn);
	return 0;
}

//SYS_INIT(sd_irq_init, POST_KERNEL, 0);
