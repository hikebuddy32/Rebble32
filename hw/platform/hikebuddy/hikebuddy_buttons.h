/*
 * hikebuddy_buttons.h
 * External-facing API for esp32 buttons (implements platform API for buttons)
 * RebbleOS
 *
 * Barry Carter <barry.carter@gmail.com>
 * Joshua Wise <joshua@joshuawise.com>
 * Saurabh Gandhi <gsaurabhr@gmail.com>
 *
 * See hikebuddy_buttons.c for a better description of this file.
 */

#ifndef __HIKEBUDDY_BUTTONS_H
#define __HIKEBUDDY_BUTTONS_H

typedef enum hw_button {
    HW_BUTTON_BACK = 0,
    HW_BUTTON_UP,
    HW_BUTTON_SELECT,
    HW_BUTTON_DOWN,
    HW_BUTTON_HOME,
    HW_BUTTON_MAX
} hw_button_t;

typedef void (*hw_button_isr_t)(hw_button_t id);

void hw_button_init(void);
int hw_button_pressed(hw_button_t button_id);
void hw_button_set_isr(hw_button_isr_t isr);

#endif
