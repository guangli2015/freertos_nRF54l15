/*
 * Copyright (c) 2012-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <nrfx_gpiote.h>
#include <nrfx_glue.h>
#include "err_num.h"
#include "bm_buttons.h"
#include "log.h"
#define LOG_DBG
#define LOG_ERR
#define LOG_WRN

#define IRQ_PRIO     3
#define BITS_PER_PIN 4
#define NUM_PINS     24




static const nrfx_gpiote_t gpiote20_instance = NRFX_GPIOTE_INSTANCE(20);
static const nrfx_gpiote_t gpiote30_instance = NRFX_GPIOTE_INSTANCE(30);

static inline const nrfx_gpiote_t *gpiote_get(uint32_t port)
{
	switch (port) {
	case 0:
		return &gpiote30_instance;
	case 1:
		return &gpiote20_instance;
	default:
		return NULL;
	}
}


static void gpiote_trigger_enable(nrfx_gpiote_pin_t pin, bool enable)
{

	const nrfx_gpiote_t *gpiote_inst = gpiote_get(NRF_PIN_NUMBER_TO_PORT(pin));

	nrfx_gpiote_trigger_enable(gpiote_inst, pin, enable);

}

static void gpiote_uninit(void)
{

	nrfx_gpiote_uninit(&gpiote20_instance);
	nrfx_gpiote_uninit(&gpiote30_instance);

}

static int gpiote_init(void)
{
	int err;


	if (!nrfx_gpiote_init_check(&gpiote20_instance)) {
		err = nrfx_gpiote_init(&gpiote20_instance, 7);
		if (err != NRFX_SUCCESS) {
			LOG_ERR("Failed to initialize gpiote20, err: 0x%08X", err);
			return -EIO;
		}

		//IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE_INST_GET(20)) + NRF_GPIOTE_IRQ_GROUP,
		//	    IRQ_PRIO, NRFX_GPIOTE_INST_HANDLER_GET(20), 0, 0);
	}

	if (!nrfx_gpiote_init_check(&gpiote30_instance)) {
		err = nrfx_gpiote_init(&gpiote30_instance, 7);
		if (err != NRFX_SUCCESS) {
			LOG_ERR("Failed to initialize gpiote30, err: 0x%08X", err);
			return -EIO;
		}

		//IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE_INST_GET(30)) + NRF_GPIOTE_IRQ_GROUP,
		//	    IRQ_PRIO, NRFX_GPIOTE_INST_HANDLER_GET(30), 0, 0);
	}

	return 0;
}

static int gpiote_input_configure(nrfx_gpiote_pin_t pin,
				  const nrfx_gpiote_input_pin_config_t *input_config)
{
	int err;


	const nrfx_gpiote_t *gpiote_inst = gpiote_get(NRF_PIN_NUMBER_TO_PORT(pin));

	err = nrfx_gpiote_input_configure(gpiote_inst, pin, input_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_input_configure, err: 0x%08X", err);
		return -EIO;
	}


	return 0;
}

struct bm_buttons_state {
	uint32_t pin_active;
	uint32_t detection_delay;
	struct bm_buttons_config const *configs;
	uint8_t num_configs;
	bool is_init;
	uint8_t pin_states[((NUM_PINS + 1) * BITS_PER_PIN) / 8];
};

static struct bm_buttons_state global;

static struct bm_buttons_config const *button_get(uint8_t pin)
{
	for (int i = 0; i < global.num_configs; i++) {
		struct bm_buttons_config const *config = &global.configs[i];

		if (pin == config->pin_number) {
			return config;
		}
	}

	return NULL;
}




static void user_event(uint8_t pin, enum bm_buttons_evt_type type)
{
	struct bm_buttons_config const *config = button_get(pin);

	if (config && config->handler) {
		LOG_DBG("Pin %d %s", pin, (type == BM_BUTTONS_PRESS) ? "pressed" : "released");
		config->handler(pin, type);
	}
}







static void gpiote_evt_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t action, void *ctx)
{
	LOG_INF("gpiote_evt_handler %d", pin);
       user_event(pin, BM_BUTTONS_PRESS);
}

int bm_buttons_init(struct bm_buttons_config const *configs, uint8_t num_configs,
		    uint32_t detection_delay)
{
	int err;

	if (global.is_init) {
		return -EPERM;
	}

	if (!configs) {
		return -EINVAL;
	}



	/* Timer needs to trigger two times before the button is detected as pressed/released.
	if (BM_TIMER_US_TO_TICKS(detection_delay) < 2 * BM_TIMER_MIN_TIMEOUT_TICKS) {
		return -EINVAL;
	} */

	err = gpiote_init();
	if (err) {
		return err;
	}

	global.configs = configs;
	global.num_configs = num_configs;
	global.detection_delay = detection_delay;

	const nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
	};

	const nrfx_gpiote_handler_config_t handler_config = {
		.handler = gpiote_evt_handler,
	};

	for (int i = 0; i < num_configs; i++) {
		const nrf_gpio_pin_pull_t pull_config = configs[i].pull_config;
		const nrfx_gpiote_input_pin_config_t input_config = {
			.p_pull_config = &pull_config,
			.p_trigger_config = &trigger_config,
			.p_handler_config = &handler_config,
		};

		const uint32_t pin_number = configs[i].pin_number;

		err = gpiote_input_configure(pin_number, &input_config);
		if (err) {
			return err;
		}
	}

	/*err = bm_timer_init(&global.timer, BM_TIMER_MODE_SINGLE_SHOT,
			      detection_delay_timeout_handler);
	if (err) {
		LOG_ERR("bm_timer_init failed, err: %d", err);
		return -EIO;
	}*/

	global.is_init = true;

	return 0;
}



int bm_buttons_enable(void)
{
	if (!global.is_init) {
		return -EPERM;
	}

	for (int i = 0; i < global.num_configs; i++) {
		gpiote_trigger_enable(global.configs[i].pin_number, true);
	}

	return 0;
}




