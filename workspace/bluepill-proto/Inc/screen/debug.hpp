#ifndef _DEBUGSCREEN_H_
#define _DEBUGSCREEN_H_
#include "font.hpp"
#include "screen/screen.hpp"
#include "hardware.h"

namespace screen {

class Debug : public Screen {

int i = 0;
int x = 10;
int y = 0;
char buffer[32];
Heater_struct &heating;

public:
    Debug(screen::ScreenHolder &_screen_holder, Heater_struct &heating) : Screen(_screen_holder), heating(heating) {}
//    Debug(screen::ScreenHolder &_screen_holder) : Screen(_screen_holder) {}
	void update()
	{
		i++;
	}

    void draw() 
    {
		fb.clear();
		draw_adc();
		display.redraw();
    }
private:
	void draw_adc()
	{
		sprintf(buffer, "Vin:%4d", (int)heating.vin);
		x = fb.draw_text(0, 0, buffer, Font::sans8);
		sprintf(buffer, "I1:%4d", (int)heating.i1);
		x = fb.draw_text(0, 10, buffer, Font::sans8);
		sprintf(buffer, "I2:%4d", (int)heating.i2);
		x = fb.draw_text(0, 20, buffer, Font::sans8);
		sprintf(buffer, "Ttip:%4d", (int)heating.ttip);
		x = fb.draw_text(64, 0, buffer, Font::sans8);
		sprintf(buffer, "Tamb:%4d", (int)heating.tamb);
		x = fb.draw_text(64, 10, buffer, Font::sans8);
	}
	void draw_counter()
	{
		sprintf(buffer, "%3d", i);
		x = fb.draw_text(0, 10, buffer, Font::num22);
		fb.draw_text(x, 10, "\260F", Font::num7);
	}
};

}

#endif
