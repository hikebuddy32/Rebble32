/* snowy_display.c
 * Sharp Memory LCD monochrome display
 *
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 * saurabh Gandhi <gsaurabhr@gmail.com>
 */


// #include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "display.h"
// #include "log.h"
#include "vibrate.h"
#include "hikebuddy_display.h"
// #include <stm32f4xx_spi.h>
// #include <stm32f4xx_tim.h>
#include "esp32_power.h"
#include "esp32_spi.h"
#include "platform_config.h"
#include "rebble_memory.h"
#include "resource.h"

/* display command types */
#define DISPLAY_CTYPE_NULL        0x00
#define DISPLAY_CTYPE_PARAM       0x01
#define DISPLAY_CTYPE_DISPLAY_OFF 0x02
#define DISPLAY_CTYPE_DISPLAY_ON  0x03
#define DISPLAY_CTYPE_SCENE       0x04
/* in full fat mode */
#define DISPLAY_CTYPE_FRAME       0x05


/* NOTE this is pinned to CCRAM on supported devices.
 * This does mean we can't (not that we could) dma from CCRAM to the SPI.
 * It's an unsupported hardware config for stm32 at least
 */
static uint8_t _frame_buffer[(DISPLAY_ROWS*(DISPLAY_COLS+16)+16)/8] MEM_REGION_DISPLAY;
static uint8_t _column_buffer[DISPLAY_ROWS];
static uint8_t _display_ready;
static uint8_t _display_curline;

void _hikebuddy_display_start_frame(uint8_t xoffset, uint8_t yoffset);\
void _hikebuddy_display_splash(uint8_t scene);
void _hikebuddy_display_full_init(void);
void _hikebuddy_display_send_frame(void);
void _hikebuddy_display_reset(uint8_t enabled);
void _hikebuddy_display_drawscene(uint8_t scene);
void _snowy_display_next_column(uint8_t col_index);
static void _spi_tx_done(void);

// typedef struct {
//     GPIO_TypeDef *port_display;
//     uint32_t clock_display;
//     uint16_t pin_reset;
//     uint16_t pin_cs;
//     // uint16_t pin_miso;
//     uint16_t pin_mosi;
//     uint16_t pin_sck;
//
//     /* inputs */
//     // uint16_t pin_reset_done;
//     // uint16_t pin_intn;
// } display_t;

/* Display configuration for the Pebble TIME */
// static const display_t display = {
//     .port_display    = GPIOG,
//     .clock_display   = RCC_AHB1Periph_GPIOG,
//     .pin_reset       = GPIO_Pin_15,
//     .pin_cs          = GPIO_Pin_8,
//     .pin_miso        = GPIO_Pin_12,
//     .pin_mosi        = GPIO_Pin_14,
//     .pin_sck         = GPIO_Pin_13,
//     .pin_reset_done  = GPIO_Pin_9,
//     .pin_intn        = GPIO_Pin_10,
// };

static const spi_bus_config_t bus_cfg={
  .mosi_io_num=PIN_LCD_MOSI,
  .sclk_io_num=PIN_LCD_CLK,
  .miso_io_num=-1,
  .quadwp_io_num=-1,
  .quadhd_io_num=-1,
  .max_transfer_sz=(DISPLAY_ROWS*(DISPLAY_COLS+16)+16)/8
};

static const spi_device_interface_config_t dev_cfg={
  .command_bits=8,
  .address_bits=0,
  .dummy_bits=0,
  .mode=0,
  .cs_ena_pretrans=0,
  .clock_speed_hz=SPIFREQ,
  .spics_io_num=PIN_LCD_CS,
  .queue_size=1,
  .flags=SPI_DEVICE_HALFDUPLEX|SPI_DEVICE_POSITIVE_CS|SPI_DEVICE_BIT_LSBFIRST|SPI_DEVICE_3WIRE,
  // .pre_cb=lcd_spi_pre_transfer_callback,
  // .post_cb=
};

/*
 * Initialise the hardware. This means all GPIOs and SPI for the display
 */
void hw_display_init(void)
{
    _display_ready = 0;

    /* init interupt pin, cs and reset */
    // stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    // stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOG);

    /* start SPI hardware */
    esp_err_t ret;
    ret=spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(HSPI_HOST, &dev_cfg, &spi);
    ESP_ERROR_CHECK(ret);

    /* Boot the display  */
    hw_display_start();

    // stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    // stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOG);

    return;
}

/* Public interface */

/*
 * When in bootloader mode, we can command the screen to power on
 * Of little use the end developers now
 */
void hw_display_on()
{
    // TODO first check that the lcd is initialized
    esp_err_t ret;
    // clear the screen (initialize pixel memory), then enable display
    ret = lcd_clrscr();
    vTaskDelay(50 / portTICK_PERIOD_MS); // wait for disp to stabilize
    ret = pca9555_set_output_state(PCA_0_EN_LCD, 1);
    ESP_LOGD(TAG, "enabled display");
}

/*
 * Start a frame render
 */
void hw_display_start_frame(uint8_t xoffset, uint8_t yoffset)
{
    _display_curline = 0;
}

uint8_t *hw_display_get_buffer(void)
{
    return _frame_buffer;
}

uint8_t hw_display_is_ready()
{
    return _display_ready;
}

uint8_t hw_display_process_isr(void)
{
    static spi_transaction_t fb_trans;
    fb_trans.flags = 0;
    fb_trans.cmd = MLCD_WR + vcom*MLCD_VCOM; // set or reset the vcom bit
    fb_trans.rxlength = 0;
    fb_trans.length = HEIGHT*(WIDTH+16)+8;
    fb_trans.tx_buffer = lcd_fb;
    // wait for previous transfer to complete (typically this
    // should be complete before the next write anyway)
    spi_transaction_t *rtrans;
    spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    // then send the next frame
    ret = spi_device_queue_trans(spi, &fb_trans, portMAX_DELAY);
    return 1;
}

/*
 * Reset the display
 * Needs work before it works (ahem) after first boot
 * as interrupts are still enabled and get all in the way
 */
void hw_display_reset(void)
{
    hw_display_start();
}

/*
 * Start the display init sequence. from cold start to full framebuffer
 */
void hw_display_start(void)
{
    _snowy_display_full_init();
}
