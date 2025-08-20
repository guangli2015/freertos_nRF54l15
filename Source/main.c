/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/
#define NRFX_UARTE_ENABLED 1
#define NRFX_UARTE30_ENABLED 1
#define NRFX_UARTE00_ENABLED 1
#define NRFX_UARTE20_ENABLED 1


#define NRFX_UARTE21_ENABLED 1


#define NRFX_UARTE22_ENABLED 1

#define BOARD_APP_UARTE_PIN_TX NRF_PIN_PORT_TO_PIN_NUMBER(0, 0)


#define BOARD_APP_UARTE_PIN_RX NRF_PIN_PORT_TO_PIN_NUMBER(1, 0)

#define BOARD_APP_UARTE_PIN_RTS NRF_PIN_PORT_TO_PIN_NUMBER(2, 0)


#define BOARD_APP_UARTE_PIN_CTS NRF_PIN_PORT_TO_PIN_NUMBER(3, 0)
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
#if 1
static const nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(30);
/* Receive buffer used in UARTE ISR callback */
static uint8_t uarte_rx_buf[4];
static int buf_idx;

/* Handle data received from UARTE. */
static void uarte_rx_handler(char *data, size_t data_len)
{
	nrfx_err_t err;
	uint8_t c;
	static char rx_buf[128];
	static uint16_t rx_buf_idx;

	for (int i = 0; i < data_len; i++) {
		c = data[i];

		if (rx_buf_idx < sizeof(rx_buf)) {
			rx_buf[rx_buf_idx++] = c;
		}

		if ((c == '\n' || c == '\r') || (rx_buf_idx >= sizeof(rx_buf))) {
			if (rx_buf_idx == 0) {
				/* RX buffer is empty, nothing to send. */
				continue;
			}

			//printk("Echo data, len %d\n", rx_buf_idx);

			/* Add newline if not already output at the end */
			if ((rx_buf[rx_buf_idx - 1] != '\n') && (rx_buf_idx < sizeof(rx_buf))) {
				rx_buf[rx_buf_idx++] = '\n';
			}

			err = nrfx_uarte_tx(&uarte_inst, rx_buf, rx_buf_idx,
					    NRFX_UARTE_TX_BLOCKING);
			if (err != NRFX_SUCCESS) {
				//printk("nrfx_uarte_tx failed, nrfx_err %d\n", err);
			}

			rx_buf_idx = 0;
		}
	}
}
/* UARTE event handler */
static void uarte_event_handler(nrfx_uarte_event_t const *event, void *ctx)
{
	switch (event->type) {
	case NRFX_UARTE_EVT_RX_DONE:
		//printk("Received data from UARTE: %c\n", event->data.rx.p_buffer[0]);
		if (event->data.rx.length > 0) {
			uarte_rx_handler(event->data.rx.p_buffer, event->data.rx.length);
		}

		/* Provide new UARTE RX buffer. */
		nrfx_uarte_rx_enable(&uarte_inst, 0);
		break;
	case NRFX_UARTE_EVT_RX_BUF_REQUEST:
		nrfx_uarte_rx_buffer_set(&uarte_inst, &uarte_rx_buf[buf_idx], 1);

		buf_idx++;
		buf_idx = (buf_idx < sizeof(uarte_rx_buf)) ? buf_idx : 0;
		break;
	case NRFX_UARTE_EVT_ERROR:
		//printk("UARTE error %#x\n", event->data.error.error_mask);
		break;
	default:
		break;
	}
}
#endif
int main(void)
{
    int count = 1;
int err;

#if 1
    nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(BOARD_APP_UARTE_PIN_TX,
								     BOARD_APP_UARTE_PIN_RX);
    uarte_config.config.hwfc = NRF_UARTE_HWFC_ENABLED;
	uarte_config.cts_pin = BOARD_APP_UARTE_PIN_CTS;
	uarte_config.rts_pin = BOARD_APP_UARTE_PIN_RTS;
    uarte_config.interrupt_priority = 5;
    err = nrfx_uarte_init(&(uarte_inst),&uarte_config,uarte_event_handler);
    if (err != NRFX_SUCCESS) {
		return -1;
	}

	const uint8_t out[] = "Hello world! I will echo the lines you enter:\r\n";

	err = nrfx_uarte_tx(&uarte_inst, out, sizeof(out), NRFX_UARTE_TX_BLOCKING);
	if (err != NRFX_SUCCESS) {
		//printk("UARTE TX failed, nrfx err %d\n", err);
		return -1;
	}

    /* Start reception */
	err = nrfx_uarte_rx_enable(&(uarte_inst), 0);
	if (err != NRFX_SUCCESS) {
		//printk("UARTE RX failed, nrfx err %d\n", err);
        	return -1;
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
main();
    
}
/*************************** End of file ****************************/
