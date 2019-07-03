
#include "display.hpp"
#include "oSI2CDrv.hpp"
#include <stdio.h>
#include <string.h>

const uint8_t Display::init_cmds[] = {
    Ssd1306::CONT_CMD,
    Ssd1306::DISPLAYOFF,
    Ssd1306::SETDISPLAYCLOCKDIV, 0xf0,
    Ssd1306::SETMULTIPLEX, static_cast<uint8_t>(DISP_HEIGHT - 1),
    Ssd1306::SETDISPLAYOFFSET, 0x00,
    Ssd1306::SETSTARTLINE,
    Ssd1306::SETSEGREMAPDEC,
    Ssd1306::CHARGEPUMP, 0x14,
    Ssd1306::MEMORYMODE, 0x01,
    Ssd1306::COMSCANDEC,
    Ssd1306::SETCOMPINS, 0x02,
    Ssd1306::SETCONTRAST, 0x22,  // 0xcf
    Ssd1306::SETPRECHARGE, 0x00,
    Ssd1306::SETVCOMDESELECT, 0x40,
    Ssd1306::DISPLAYALLON_RESUME,
    Ssd1306::NORMALDISPLAY,
    Ssd1306::COLUMNADDR, 0, static_cast<uint8_t>(DISP_WIDTH - 1),
    Ssd1306::PAGEADDR, 0, static_cast<uint8_t>(DISP_HEIGHT / 8 - 1),
    Ssd1306::DISPLAYON,
};



void Display::init()
{
    oSI2CDrv::init(&hi2c1);
    oSI2CDrv::oSInit();
    oSI2CDrv::Transmit(DISP_ADDR, (uint8_t*)Display::init_cmds, sizeof(Display::init_cmds));
    memset(sBuf.display_buffer, 0x00, 512);
}
void Display::redraw()
{
    oSI2CDrv::Transmit(DISP_ADDR, sBuf.display_buffer_cmds, sBuf.size);
}
