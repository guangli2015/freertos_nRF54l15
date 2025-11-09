/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nrf_sdh_freertos.h"
#include "nrf_sdh.h"

/* Group of FreeRTOS-related includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "log.h"


#define NRF_BLE_FREERTOS_SDH_TASK_STACK 512


static TaskHandle_t                 m_softdevice_task;  //!< Reference to SoftDevice FreeRTOS task.
static TaskHandle_t                 m_softdevice_init_task;  //!< Reference to SoftDevice FreeRTOS task.
static nrf_sdh_freertos_task_hook_t m_task_hook;        //!< A hook function run by the SoftDevice task before entering its loop.
extern int ble_lbs_sample(void);
#if 0
void SD_EVT_IRQHandler(void)
{
    BaseType_t yield_req = pdFALSE;

    vTaskNotifyGiveFromISR(m_softdevice_task, &yield_req);

    /* Switch the task if required. */
    portYIELD_FROM_ISR(yield_req);
}

extern int ble_lbs_sample(void);
/* This function gets events from the SoftDevice and processes them. */
static void softdevice_task(void * pvParameter)
{
    LOG_INF("Enter softdevice_task\r\n");

    if (m_task_hook != NULL)
    {
        m_task_hook(pvParameter);
    }
    
    while (true)
    {
        nrf_sdh_evts_poll();                    /* let the handlers run first, incase the EVENT occured before creating this task */

        (void) ulTaskNotifyTake(pdTRUE,         /* Clear the notification value before exiting (equivalent to the binary semaphore). */
                                portMAX_DELAY); /* Block indefinitely (INCLUDE_vTaskSuspend has to be enabled).*/
    }
}
#endif
/* This function gets events from the SoftDevice and processes them. */
static void softdevice_init_task(void * pvParameter)
{
    LOG_INF("Enter softdevice_init\r\n");

    if (m_task_hook != NULL)
    {
        m_task_hook(pvParameter);
    }
     ble_lbs_sample();
   
    while(1)
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context)
void nrf_sdh_freertos_init(void * p_context)
{
    LOG_INF("Creating  SoftDevice init task.\r\n");
#if 0
    m_task_hook = NULL;

    BaseType_t xReturned = xTaskCreate(softdevice_task,
                                       "BLE_init",
                                       NRF_BLE_FREERTOS_SDH_TASK_STACK,
                                       p_context,
                                       4  ,
                                       &m_softdevice_task);
    if (xReturned != pdPASS)
    {
        LOG_INF("SoftDevice task not created\r\n");

    }
#endif
    BaseType_t xReturned1 = xTaskCreate(softdevice_init_task,
                                       "BLE_SD",
                                       NRF_BLE_FREERTOS_SDH_TASK_STACK,
                                       p_context,
                                        3 ,
                                       &m_softdevice_init_task);
    if (xReturned1 != pdPASS)
    {
        LOG_INF("SoftDevice task not created\r\n");

    }
}
