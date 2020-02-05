/*
 * hikebuddy_buttons.h
 * External-facing API for hikebuddy buttons (implements platform API for buttons)
 * RebbleOS
 *
 * Barry Carter <barry.carter@gmail.com>
 * Joshua Wise <joshua@joshuawise.com>
 * Saurabh Gandhi <gsaurabhr@gmail.com>
 *
 * See hikebuddy_buttons.c for a better description of this file.
 */

#ifndef __HIKEBUDDY_BUTTONS_PLATFORM_H
#define __HIKEBUDDY_BUTTONS_PLATFORM_H

#if defined(STM32F4XX)
#    include "stm32f4xx.h"
#elif defined(STM32F2XX)
#    include "stm32f2xx.h"
#else
// #    error "I have no idea what kind of stm32 this is; sorry"
#endif

#include <stdint.h>
#include "hikebuddy_buttons.h"

typedef struct {
    uint16_t gpio_pin;
    // GPIO_TypeDef *gpio_ptr;
    uint32_t exti_port;
    uint32_t exti_pinsource;
    uint32_t gpio_clock;
    uint32_t exti_irq;
} hikebuddy_button_t;

extern const hikebuddy_button_t platform_buttons[HW_BUTTON_MAX];

extern void hikebuddy_buttons_raw_isr();

#define HIKEBUDDY_BUTTONS_MK_IRQ_HANDLER(exti) \
    void EXTI ## exti ## _IRQHandler(void) \
    { \
        hikebuddy_buttons_raw_isr(); \
    }

#endif
