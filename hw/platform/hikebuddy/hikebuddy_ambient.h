#pragma once
/* hikebuddy_ambient.h
 * driver for the Ambient sensor
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 * Modified by Saurabh Gandhi <gsaurabhr@gmail.com>
 */

#include "platform.h"

void hw_ambient_init(void);
uint16_t hw_ambient_get(void);
