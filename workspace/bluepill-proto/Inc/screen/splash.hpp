#ifndef _SPLASHSCREEN_H_
#define _SPLASHSCREEN_H_
#include "font.hpp"
#include "screen/screen.hpp"
#include "hardware.h"

namespace screen {

class Splash : public Screen {

private:
	bool loaded = false;
	int i = 0;
	char buffer[20];
	void loadFromFlash(void){
		// image will be stored at 2nd to last KB in flash memory or page
		// should be some kind of header bytes maybe "IMG" in ascii?  + 512 bytes of image
		// so should be 516 bytes for the image
	};

public:
    Splash(screen::ScreenHolder &screen_holder) : Screen(screen_holder) {}
	~Splash() {}
	
	void update()
	{
		// printf("Splash::update()\n");
		i++;
		if(i > 40)
		{
			// reset self
			i = 0;
			loaded = false;
			change_screen(ScreenId::DEBUG);
		}
	};

	// draw functions assumes cleared fb, and adds all graphics to buffer
    void draw() {
		if(!loaded)
		{
			fb.clear();
			// printf("Splash::draw()\n");
			//TODO: grab the image from flash and send to buffer
			// instead of loading from flash for now we generate a dummy
			sprintf(buffer, "Splash screen");
			fb.draw_text(5, 5, buffer, Font::sans8);
			sprintf(buffer, "%3d", i);
			fb.draw_text(84, 2, buffer, Font::num22);
			// loaded = true;
		}
	};

    bool button_up(int act) { return false; };
    bool button_dw(int act) { return false; };
    bool button_both(int act) { return false; };
};

}

#endif
