#ifndef _DEBUGSCREEN_H_
#define _DEBUGSCREEN_H_
#include "font.hpp"

namespace screen {

class Splash : public Screen {

private:
	Display::FrameBuff &fb = display.get_fb();
	bool loaded = false;
	char buffer[16];
	void loadFromFlash(void){
		// image will be stored at 2nd to last KB in flash memory or page
		// should be some kind of header bytes maybe "IMG" in ascii?  + 512 bytes of image
		// so should be 516 bytes for the image
	};

public:
    Splash(Screen_Holder &screen_holder) : screen_holder(screen_holder) {}
	
	void update()
	{

	};

	// draw functions assumes cleared fb, and adds all graphics to buffer
    void draw() {
		if(!loaded)
		{
			//TODO: grab the image from flash and send to buffer
			// instead of loading from flash for now we generate a dummy
			sprintf(buffer, "dummy text");
			fb.draw_text(5, 5, buffer, Font::num22);
			loaded = true;
		}
	};

    bool button_up(int act) { return false; };
    bool button_dn(int act) { return false; };
    bool button_both(int act) { return false; };
};

}

#endif