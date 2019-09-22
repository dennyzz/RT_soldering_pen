#ifndef _DEBUGSCREEN_H_
#define _DEBUGSCREEN_H_
#include "font.hpp"

namespace screen {

class Debug : public Screen {
	// Display::FrameBuff &fb = display.get_fb();
private: 
int i = 0;
int x = 10;

public:
    Debug(screen::ScreenHolder &_screen_holder) : Screen(_screen_holder) {}
	void update()
	{
		printf("Debug::update()\n");
		i++;
	}

    void draw() 
    {
		printf("Debug::draw()\n");
    	char buffer[16];
		fb.clear();
		sprintf(buffer, "%3d", i);
		x = fb.draw_text(0, 10, buffer, Font::num22);
		fb.draw_text(x, 10, "\260F", Font::num7);
		display.redraw();
    }
};

}

#endif
