/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <stdint.h>
#include "irq_connect.h"
#include <nrf_sdm.h>
extern void CLOCK_POWER_IRQHandler(void);
extern void RADIO_0_IRQHandler(void);
extern void TIMER10_IRQHandler(void);
extern void GRTC_3_IRQHandler(void);
extern void ECB00_IRQHandler(void);
extern void AAR00_CCM00_IRQHandler(void);
extern void SWI00_IRQHandler(void);

/* In irq_forward.s */
extern void SVC_Handler(void);
extern void CallSoftDeviceResetHandler(void);

uint32_t irq_forwarding_enabled_magic_number_holder;
uint32_t softdevice_vector_forward_address;

static void sd_enable_irq_forwarding(void)
{
	softdevice_vector_forward_address = 0x00162000;
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

	CallSoftDeviceResetHandler();
	irq_forwarding_enabled_magic_number_holder = IRQ_FORWARDING_ENABLED_MAGIC_NUMBER;
}

int softdevice_irq_init(void)
{
#define PRIO_HIGH 0	/* SoftDevice high priority interrupt */
#define PRIO_LOW 4	/* SoftDevice low priority interrupt */
#if 0
	/* IRQ_ZERO_LATENCY with CONFIG_ZERO_LATENCY_LEVELS equal to 1 (default) forces the priority
	 * level to 0, ignoring the specified priority.
	 * On `sd_softdevice_enable()`, the SoftDevice will override the necessary interrupts it
	 * uses internally with the priority levels it needs.
	 */
	IRQ_DIRECT_CONNECT(RADIO_0_IRQn, PRIO_HIGH, RADIO_0_IRQHandler, IRQ_ZERO_LATENCY);
	IRQ_DIRECT_CONNECT(TIMER10_IRQn, PRIO_HIGH, TIMER10_IRQHandler, IRQ_ZERO_LATENCY);
	IRQ_DIRECT_CONNECT(GRTC_3_IRQn, PRIO_HIGH, GRTC_3_IRQHandler, IRQ_ZERO_LATENCY);

	/* These are not zero latency. */
	IRQ_DIRECT_CONNECT(AAR00_CCM00_IRQn, PRIO_LOW, AAR00_CCM00_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(CLOCK_POWER_IRQn, PRIO_LOW, CLOCK_POWER_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(ECB00_IRQn, PRIO_LOW, ECB00_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(SWI00_IRQn, PRIO_LOW, SWI00_IRQHandler, 0);
#endif  
#if 0
    NVIC_SetPriority(RADIO_0_IRQn, 0);       // 等效于 PRIO_HIGH + IRQ_ZERO_LATENCY
    //NVIC_EnableIRQ(RADIO_0_IRQn);

    NVIC_SetPriority(TIMER10_IRQn, 0);
    //NVIC_EnableIRQ(TIMER10_IRQn);

    NVIC_SetPriority(GRTC_3_IRQn, 0);
    //NVIC_EnableIRQ(GRTC_3_IRQn);


    NVIC_SetPriority(AAR00_CCM00_IRQn, 4);   // 等效于 PRIO_LOW
    //NVIC_EnableIRQ(AAR00_CCM00_IRQn);

    NVIC_SetPriority(CLOCK_POWER_IRQn, 4);
    //NVIC_EnableIRQ(CLOCK_POWER_IRQn);

    NVIC_SetPriority(ECB00_IRQn, 4);
    //NVIC_EnableIRQ(ECB00_IRQn);

    NVIC_SetPriority(SWI00_IRQn, 4);
    //NVIC_EnableIRQ(SWI00_IRQn);
  

	NVIC_SetPriority(SVCall_IRQn, PRIO_LOW);
#endif
	sd_enable_irq_forwarding();

	return 0;
}

__attribute__((weak)) void C_HardFault_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_TIMER0_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_RTC0_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_SIGNALLING_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_RADIO_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_RNG_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_ECB_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_CCM_Handler(void)
{
	__asm__("SVC 255");
}

__attribute__((weak)) void C_POWER_CLOCK_Handler(void)
{
	__asm__("SVC 255");
}



//SYS_INIT(irq_init, PRE_KERNEL_1, 0);
