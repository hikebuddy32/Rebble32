/* snowy_vibrate.c
 * GPIO vibrate implementation for Pebble Time (snowy)
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 * Modified by Saurabh Gandhi <gsaurabhr@gmail.com>
 */

 #include <driver/gpio.h>
#include "esp32_power.h"
#include "stdio.h"
#include "string.h"
#include "hikebuddy_vibrate.h"

extern const vibrate_t hw_vibrate_config;

void hw_vibrate_init(void)
{
    esp32_power_request(ESP32_POWER_AHB1, hw_vibrate_config.clock);

  	gpio_config_t io_conf_vibr = {
      // .intr_type    = GPIO_INTR_ANYEDGE,
		  .mode         = GPIO_MODE_OUTPUT,
		  .pin_bit_mask = 1ULL<<hw_vibrate_config.pin,
  	};
    gpio_config(&io_conf_vibr);
    // GPIO_InitTypeDef GPIO_InitStructure_Vibr;
    //
    // // init the vibrator
    // GPIO_InitStructure_Vibr.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_InitStructure_Vibr.GPIO_Pin = hw_vibrate_config.pin;
    // GPIO_InitStructure_Vibr.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_InitStructure_Vibr.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure_Vibr.GPIO_OType = GPIO_OType_PP;
    // GPIO_Init(hw_vibrate_config.port, &GPIO_InitStructure_Vibr);

    esp32_power_release(ESP32_POWER_AHB1, hw_vibrate_config.clock);
}


void hw_vibrate_enable(uint8_t enabled)
{
    esp32_power_request(ESP32_POWER_AHB1, hw_vibrate_config.clock);

    gpio_set_level(hw_vibrate_config.pin, enabled);
    // if (enabled)
    //     GPIO_SetBits(hw_vibrate_config.port, hw_vibrate_config.pin);
    // else
    //     GPIO_ResetBits(hw_vibrate_config.port, hw_vibrate_config.pin);

    esp32_power_release(ESP32_POWER_AHB1, hw_vibrate_config.clock);
}
