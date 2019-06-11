#ifndef ____SSD1306_H__
#define ____SSD1306_H__

#include <stdint.h>

#define SSD1306_ADDR         (0x3c)

#define CO_CMD               (0x00)
#define CMD                  (0x80)
#define CO_DATA              (0x40)
#define DATA                 (0xc0)

#define CHARGEPUMP           (0x8d)
#define COLUMNADDR           (0x21)
#define COMSCANDEC           (0xc8)
#define COMSCANINC           (0xc0)
#define DISPLAYALLON         (0xa5)
#define DISPLAYALLON_RESUME  (0xa4)
#define DISPLAYOFF           (0xae)
#define DISPLAYON            (0xaf)
#define EXTERNALVCC          (0x01)
#define INVERTDISPLAY        (0xa7)
#define MEMORYMODE           (0x20)
#define NORMALDISPLAY        (0xa6)
#define PAGEADDR             (0x22)
#define SETSEGREMAPINC       (0xa0)
#define SETSEGREMAPDEC       (0xa1)
#define SETCOMPINS           (0xda)
#define SETCONTRAST          (0x81)
#define SETDISPLAYCLOCKDIV   (0xd5)
#define SETDISPLAYOFFSET     (0xd3)
#define SETHIGHCOLUMN        (0x10)
#define SETLOWCOLUMN         (0x00)
#define SETSTARTPAGE         (0xb0)
#define SETMULTIPLEX         (0xa8)
#define SETPRECHARGE         (0xd9)
#define SETSTARTLINE         (0x40)
#define SETVCOMDESELECT      (0xdb)
#define SWITCHCAPVCC         (0x02)

#define DISPLAY_HEIGHT       (32)
#define DISPLAY_WIDTH        (128)













#endif //____SSD1306_H__
