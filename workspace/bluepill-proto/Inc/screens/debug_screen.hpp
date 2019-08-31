#ifndef _DEBUGSCREEN_H_
#define _DEBUGSCREEN_H_
#include "font.hpp"

namespace screen {

class Debug : public Screen {

private: 
// Display::FrameBuff &fb = display.get_fb();
int i = 0;
int x = 10;

public:
    // Debug(GUIManager &screen_manager) : screen_manager(screen_manager) {}
	void tick()
	{
		i++;
	}
    void draw() {
    	char buffer[16];
		fb.clear();
		sprintf(buffer, "%3d", i);
		x = fb.draw_text(50, 10, buffer, Font::num22);
		fb.draw_text(x, 10, "\260F", Font::num7);
		display.redraw();
    }
};

}

#endif