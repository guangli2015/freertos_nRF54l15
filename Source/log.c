
#include "FreeRTOS.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <nrfx_uarte.h>

#define LOG_BUFFER_SIZE 256
#define BOARD_APP_UARTE_PIN_TX NRF_PIN_PORT_TO_PIN_NUMBER(0, 0)


#define BOARD_APP_UARTE_PIN_RX NRF_PIN_PORT_TO_PIN_NUMBER(1, 0)

#define BOARD_APP_UARTE_PIN_RTS NRF_PIN_PORT_TO_PIN_NUMBER(2, 0)


#define BOARD_APP_UARTE_PIN_CTS NRF_PIN_PORT_TO_PIN_NUMBER(3, 0)

static const nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(30);

void log_inf(const char *fmt, ...) {
    char buf[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0 && len < LOG_BUFFER_SIZE) {
        nrfx_uarte_tx(&uarte_inst, (uint8_t *)buf, len, NRFX_UARTE_TX_BLOCKING);
    }
}

int log_init()
{
    int err;
    nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(BOARD_APP_UARTE_PIN_TX,
								     BOARD_APP_UARTE_PIN_RX);
    uarte_config.config.hwfc = NRF_UARTE_HWFC_ENABLED;
	uarte_config.cts_pin = BOARD_APP_UARTE_PIN_CTS;
	uarte_config.rts_pin = BOARD_APP_UARTE_PIN_RTS;
    uarte_config.interrupt_priority = 7;
    err = nrfx_uarte_init(&(uarte_inst),&uarte_config,NULL);
    return err;
}