
#include "display.hpp"
#include "oSI2CDrv.hpp"
#include <stdio.h>
#include <string.h>

Display display;

const uint8_t Display::init_cmds[] = {
    Ssd1306::CONT_CMD,
    Ssd1306::DISPLAYOFF,
    Ssd1306::SETDISPLAYCLOCKDIV, 0xf0,
    Ssd1306::SETMULTIPLEX, static_cast<uint8_t>(DISPLAY_HEIGHT - 1),
    Ssd1306::SETDISPLAYOFFSET, 0x00,
    Ssd1306::SETSTARTLINE,
    Ssd1306::SETSEGREMAPDEC,
    Ssd1306::CHARGEPUMP, 0x14,
    Ssd1306::MEMORYMODE, 0x01,
    Ssd1306::COMSCANDEC,
    Ssd1306::SETCOMPINS, 0x02,
    Ssd1306::SETCONTRAST, 0xCF, 
    Ssd1306::SETPRECHARGE, 0x00,
    Ssd1306::SETVCOMDESELECT, 0x40,
    Ssd1306::DISPLAYALLON_RESUME,
    Ssd1306::NORMALDISPLAY,
    Ssd1306::COLUMNADDR, 0, static_cast<uint8_t>(DISPLAY_WIDTH - 1),
    Ssd1306::PAGEADDR, 0, static_cast<uint8_t>(DISPLAY_HEIGHT / 8 - 1),
    Ssd1306::DISPLAYON,
};


void Display::init()
{
    fb_cmds.frame.clear();
    oSI2CDrv::init(&hi2c1);
    oSI2CDrv::oSInit();
    // reset? 
    // HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_Set);
    oSI2CDrv::Transmit(DISP_ADDR, (uint8_t*)init_cmds, sizeof(init_cmds));
}


void Display::redraw()
{
    // oSI2CDrv::Transmit(0xAA, (uint8_t*)fb_cmds.dummy, 2);
    oSI2CDrv::Transmit(DISP_ADDR, fb_cmds.display_buffer_cmds, fb_cmds.size);
}


void Display::rotate(bool x, bool y) {
    uint8_t rotate_cmds[3] {
        Ssd1306::CONT_CMD,
        x ? Ssd1306::SETSEGREMAPINC : Ssd1306::SETSEGREMAPDEC,
        y ? Ssd1306::COMSCANINC : Ssd1306::COMSCANDEC,
    };
    oSI2CDrv::Transmit(DISP_ADDR, rotate_cmds, sizeof(rotate_cmds));
}

