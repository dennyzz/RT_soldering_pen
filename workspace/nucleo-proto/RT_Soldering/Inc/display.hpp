#ifndef __DISPLAY_HPP_
#define __DISPLAY_HPP_

#include "hardware.h"
#include "ssd1306.hpp"
#include <string.h>

#define DISP_WIDTH  (128)
#define DISP_HEIGHT (32)

class Display {
public:
	void init(); // init i2c comms and screen

	void redraw(); // draw the buffer out to screen

	inline uint8_t *get_buffer(){
		return (uint8_t*)sBuf.display_buffer;
	}

	inline void clear(){
		memset(sBuf.display_buffer, 0, sizeof(sBuf.display_buffer));
	}

    inline void draw_pixel(int x, int y) {
        if (x < 0  || x >= DISP_WIDTH || y < 0 || y >= DISP_HEIGHT) return;
        sBuf.display_buffer[x] |= 1 << y;
    }

    inline void clear_pixel(int x, int y) {
        if (x < 0  || x >= DISP_WIDTH || y < 0 || y >= DISP_HEIGHT) return;
        sBuf.display_buffer[x] &= ~(1 << y);
    }

	static const uint8_t DISP_ADDR = (0x3c<<1);
    static const uint8_t init_cmds[32];
private:

	struct screenBuffer {
		char dummy[3] = {'F', 'B', ':'};
		uint8_t display_buffer_cmds[1] = {Ssd1306::CONT_DATA,};
		uint32_t display_buffer[DISP_WIDTH];
		uint32_t size = sizeof(display_buffer) + sizeof(display_buffer_cmds);
	}sBuf;

};

#endif // __DISPLAY_HPP_
