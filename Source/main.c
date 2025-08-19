/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <hal/nrf_gpio.h>
#include <nrfx_uarte.h>
/** @brief Macro for extracting absolute pin number from the relative pin and port numbers. */
#define NRF_PIN_PORT_TO_PIN_NUMBER(pin, port) (((pin) & 0x1F) | ((port) << 5))
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)

#ifndef BOARD_PIN_LED_0
#define BOARD_PIN_LED_0 NRF_PIN_PORT_TO_PIN_NUMBER(9, 2)
#endif
#ifndef BOARD_PIN_LED_1
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)
#endif
#ifndef BOARD_PIN_LED_2
#define BOARD_PIN_LED_2 NRF_PIN_PORT_TO_PIN_NUMBER(7, 2)
#endif
#ifndef BOARD_PIN_LED_3
#define BOARD_PIN_LED_3 NRF_PIN_PORT_TO_PIN_NUMBER(14, 1)
#endif

int app(void)
{
    int count = 1;
int err;

#if 0
    nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(BOARD_APP_UARTE_PIN_TX,
								     BOARD_APP_UARTE_PIN_RX);
    uarte_config.config.hwfc = NRF_UARTE_HWFC_ENABLED;
	uarte_config.cts_pin = BOARD_APP_UARTE_PIN_CTS;
	uarte_config.rts_pin = BOARD_APP_UARTE_PIN_RTS;
    uarte_config.interrupt_priority = 5;
    err = nrfx_uarte_init(&(uarte_inst),&uarte_config,uarte_event_handler);
    if (err != NRFX_SUCCESS) {
		return RT_ERROR;
	}

	const uint8_t out[] = "Hello world! I will echo the lines you enter:\r\n";

	err = nrfx_uarte_tx(&uarte_inst, out, sizeof(out), NRFX_UARTE_TX_BLOCKING);
	if (err != NRFX_SUCCESS) {
		//printk("UARTE TX failed, nrfx err %d\n", err);
		return RT_ERROR;
	}

    /* Start reception */
	err = nrfx_uarte_rx_enable(&(uarte_inst), 0);
	if (err != NRFX_SUCCESS) {
		//printk("UARTE RX failed, nrfx err %d\n", err);
        	return RT_ERROR;
	}
#endif
    //rt_pin_mode(RT_BSP_LED_PIN, PIN_MODE_OUTPUT);
nrf_gpio_cfg_output(BOARD_PIN_LED_0);
nrf_gpio_cfg_output(BOARD_PIN_LED_1);
nrf_gpio_cfg_output(BOARD_PIN_LED_2);
nrf_gpio_cfg_output(BOARD_PIN_LED_3);
    while (count++)
    {
        //rt_pin_write(RT_BSP_LED_PIN, PIN_HIGH);
        nrf_gpio_pin_write(BOARD_PIN_LED_0, 1);
        nrf_gpio_pin_write(BOARD_PIN_LED_1, 0);
        nrf_gpio_pin_write(BOARD_PIN_LED_2, 1);
        nrf_gpio_pin_write(BOARD_PIN_LED_3, 1);
        //rt_thread_mdelay(3000);
        //nrf_gpio_pin_write(BOARD_PIN_LED_0, 0);
        //nrf_gpio_pin_write(BOARD_PIN_LED_1, 0);
        //nrf_gpio_pin_write(BOARD_PIN_LED_2, 0);
       // nrf_gpio_pin_write(BOARD_PIN_LED_3, 0);
        //rt_pin_write(RT_BSP_LED_PIN, PIN_LOW);
        //rt_thread_mdelay(3000);
    }
    return 0;
}

void _start(void) {
app();
    
}
/*************************** End of file ****************************/
