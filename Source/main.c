/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/
/* FreeRTOS include. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
/*#define NRFX_UARTE_ENABLED 1
#define NRFX_UARTE30_ENABLED 1
#define NRFX_UARTE00_ENABLED 1
#define NRFX_UARTE20_ENABLED 1


#define NRFX_UARTE21_ENABLED 1


#define NRFX_UARTE22_ENABLED 1*/

#define BOARD_APP_UARTE_PIN_TX NRF_PIN_PORT_TO_PIN_NUMBER(0, 0)


#define BOARD_APP_UARTE_PIN_RX NRF_PIN_PORT_TO_PIN_NUMBER(1, 0)

#define BOARD_APP_UARTE_PIN_RTS NRF_PIN_PORT_TO_PIN_NUMBER(2, 0)


#define BOARD_APP_UARTE_PIN_CTS NRF_PIN_PORT_TO_PIN_NUMBER(3, 0)
#include <hal/nrf_gpio.h>
#include <nrfx_uarte.h>
#include "test_section.h"
#include <stdio.h>
#include <string.h>
#include  "log.h"
#include "irq_connect.h"

#include <nrfx_grtc.h>
#include <nrfx_clock.h>
#include "err_num.h"
#include "bm_buttons.h"
#include <stdbool.h>

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
#ifndef BOARD_PIN_BTN_0
#define BOARD_PIN_BTN_0 NRF_PIN_PORT_TO_PIN_NUMBER(13, 1)
#endif


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    /* Force an assert. */
    configASSERT( pcTaskName == 0 );
}

static void led_toggle_task_function (void * pvParameter)
{
   
    while (true)
    {
       // nrf_gpio_pin_write(BOARD_PIN_LED_0, 1);
          nrf_gpio_pin_toggle(BOARD_PIN_LED_1);
        /* Delay a task for a given number of ticks */
        vTaskDelay(3000);

        /* Tasks must be implemented to never return... */
    }
}
static TaskHandle_t m_softdevice_init_task; 
TaskHandle_t  led_toggle_task_handle;
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */



/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (TimerHandle_t xTimer )
{
   
    nrf_gpio_pin_toggle(BOARD_PIN_LED_2);
}
StaticTimer_t myTimerBuffer;

extern int sftdevice_test(void);
extern int softdevice_irq_init(void);
extern int ble_lbs_sample(void);
extern void nrf_sdh_freertos_init(void * p_context);

/* This function gets events from the SoftDevice and processes them. */
static void softdevice_init_task(void * pvParameter)
{
    LOG_INF("Enter softdevice_init\r\n");


    ble_lbs_sample();
   

}

int main(void)
{

  softdevice_irq_init();

  log_init();

  //nrf_sdh_freertos_init( NULL);
  nrf_gpio_cfg_output(BOARD_PIN_LED_0);
  nrf_gpio_cfg_output(BOARD_PIN_LED_1);
  nrf_gpio_cfg_output(BOARD_PIN_LED_2);
  nrf_gpio_cfg_output(BOARD_PIN_LED_3);

    BaseType_t xReturned1 = xTaskCreate(softdevice_init_task,
                                       "BLE_SD",
                                       512,
                                       NULL,
                                        3 ,
                                       &m_softdevice_init_task);
    if (xReturned1 != pdPASS)
    {
        LOG_INF("SoftDevice task not created\r\n");

    }

    xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_SECURE_STACK_SIZE + 200, NULL,  2, &led_toggle_task_handle);

        /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreateStatic( "LED1", 1000, pdTRUE, NULL, led_toggle_timer_callback,&myTimerBuffer);
    xTimerStart(led_toggle_timer_handle, 0);

        /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();


      while (true) {
		
		/* Wait for an event. */
		__WFE();

		/* Clear Event Register */
		__SEV();
		__WFE();
	}
    return 0;
}

void _start(void) {
main();
    
}

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that
 * is used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( 32 ) ) );

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ] __attribute__( ( aligned( 32 ) ) );

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/*************************** End of file ****************************/
