#include "ssd1306.h"
#include "setup.h"

uint8_t ssd1306_init_cmds[] = {
    CO_CMD,
    DISPLAYOFF,
    SETDISPLAYCLOCKDIV, 0xf0,
    SETMULTIPLEX, (uint8_t)(DISPLAY_HEIGHT - 1),
    SETDISPLAYOFFSET, 0x00,
    SETSTARTLINE,
    SETSEGREMAPDEC,
    CHARGEPUMP, 0x14,
    MEMORYMODE, 0x01,
    COMSCANDEC,
    SETCOMPINS, 0x02,
    SETCONTRAST, 0x22,  // 0xcf
    SETPRECHARGE, 0x00,
    SETVCOMDESELECT, 0x40,
    DISPLAYALLON_RESUME,
    NORMALDISPLAY,
    COLUMNADDR, 0, (uint8_t)(DISPLAY_WIDTH - 1),
    PAGEADDR, 0, (uint8_t)(DISPLAY_HEIGHT / 8 - 1),
    DISPLAYON,
};



void initOLED(void)
{
	// possibly checking some nrst pin?
	// make sure our i2c and gpio_af are properly set up.
	// clear a framebuffer
	// set nrst to high!
	// do an i2c write of the init commands
	HAL_I2C_Master_Transmit_DMA(&hi2c1, SSD1306_ADDR, ssd1306_init_cmds, sizeof(ssd1306_init_cmds));

}


