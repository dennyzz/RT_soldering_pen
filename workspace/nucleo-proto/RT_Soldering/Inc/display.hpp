#ifndef __DISPLAY_HPP_
#define __DISPLAY_HPP_

#include "hardware.h"
#include "ssd1306.hpp"
#include <string.h>
#include "framebuffer.hpp"

class Display {

	typedef uint32_t HEIGHT_TYPE;
public:
	static const int DISPLAY_WIDTH = 128;
	static const int DISPLAY_HEIGHT = 32;

	typedef FrameBuffer<DISPLAY_WIDTH, DISPLAY_HEIGHT, HEIGHT_TYPE> FrameBuff;

private:

	static const uint8_t DISP_ADDR = (0x3c<<1);
    static const uint8_t init_cmds[32];

	struct FbCmds {
		char dummy[3] = {'F', 'B', ':'};
		uint8_t display_buffer_cmds[1] = {Ssd1306::CONT_DATA,};
		FrameBuff frame;
		static constexpr size_t size = sizeof(display_buffer_cmds) + sizeof(frame);
	}fb_cmds;


public:

	void init(); // init i2c comms and display

	void redraw(); // draw the buffer out to display

    void rotate(bool x, bool y);

	inline FrameBuff &get_fb(){
		return fb_cmds.frame;
	}
};

extern Display display;

#endif // __DISPLAY_HPP_
