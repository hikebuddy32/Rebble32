#pragma once
/* hikebuddy_vibrate.c
 * GPIO vibrate implementation for Pebble Time (snowy)
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 * Modified by Saurabh Gandhi <gsaurabhr@gmail.com>
 */


typedef struct {
    uint16_t pin;
    uint32_t clock;
} vibrate_t;


void hw_vibrate_init(void);
void hw_vibrate_enable(uint8_t enabled);
