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
	void draw_raw_adc()
	{
		sprintf(buffer, "Vin:%5d", (int)heating.vin);
		x = fb.draw_text(0, 0, buffer, Font::sans8);
		sprintf(buffer, "I1 :%5d", (int)heating.i1);
		x = fb.draw_text(0, 10, buffer, Font::sans8);
		sprintf(buffer, "I2 :%5d", (int)heating.i2);
		x = fb.draw_text(0, 20, buffer, Font::sans8);
		sprintf(buffer, "Ttip:%5d", (int)heating.ttip);
		x = fb.draw_text(64, 0, buffer, Font::sans8);
		sprintf(buffer, "Tamb:%5d", (int)heating.tamb);
		x = fb.draw_text(64, 10, buffer, Font::sans8);
		sprintf(buffer, "Tint:%5d", (int)heating.tint);
		x = fb.draw_text(64, 20, buffer, Font::sans8);
	}
	void draw_adc()
	{
		uint16_t unit = heating.vin / 1000;
		uint16_t decimal = heating.vin % 1000;
		sprintf(buffer, "Vin:%2d.%03dV", (int)unit, (int)decimal);
		x = fb.draw_text(0, 0, buffer, Font::sans8);

		sprintf(buffer, "I1 :%4dmA", (int)heating.i1);
		x = fb.draw_text(0, 10, buffer, Font::sans8);

		sprintf(buffer, "I2 :%4dmA", (int)heating.i2);
		x = fb.draw_text(0, 20, buffer, Font::sans8);

		unit = heating.ttip / 10;
		decimal = heating.ttip % 10;
		sprintf(buffer, "Ttip:%3d.%1dC",(int)unit, (int)decimal);
		x = fb.draw_text(50, 0, buffer, Font::sans8);

		unit = heating.tamb / 10;
		decimal = heating.tamb % 10;
		sprintf(buffer, "Tamb:%3d.%1dC",(int)unit, (int)decimal);
		x = fb.draw_text(50, 10, buffer, Font::sans8);

		unit = heating.tint / 10;
		decimal = heating.tint % 10;
		sprintf(buffer, "Tint:%3d.%1dC",(int)unit, (int)decimal);
		x = fb.draw_text(50, 20, buffer, Font::sans8);
		
		sprintf(buffer, ":%4d", (int)heating.vint);
		x = fb.draw_text(100, 20, buffer, Font::sans8);
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
