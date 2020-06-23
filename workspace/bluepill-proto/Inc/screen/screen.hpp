#ifndef ___SCREEN_HPP_
#define ___SCREEN_HPP_

#include "display.hpp"
#include "button.hpp"

namespace screen {
class Screen;

enum ScreenId {
    SPLASH = 0, 
    DEBUG, 
    MAIN, 
    INFO, 
    MAX
	// MAIN = 0,
	// INFO, 
	// COUNT, 
	// DEBUG,
 //    MAX
};

class ScreenHolder {
	ScreenId screen_id = ScreenId::SPLASH;
	Screen **screen_list;

public:
    ScreenHolder(Screen **list) :
        screen_list(list) {}

    void set(ScreenId id) {
        screen_id = id;
    }

    Screen *get() {
        return screen_list[static_cast<int>(screen_id)];
    }

};

/* base class for all Screen displays */
class Screen {
    ScreenHolder &screen_holder;
protected:
	Display::FrameBuff &fb = display.get_fb();

	void change_screen(ScreenId id) {
		screen_holder.set(id);
	}

public:
    Screen(ScreenHolder &init_screen_holder) : screen_holder(init_screen_holder) {}
    virtual ~Screen() {}

    virtual bool button_up(Button::Action act) { return false; };
    virtual bool button_dw(Button::Action act) { return false; };
    virtual bool button_both(Button::Action act) { return false; };
    virtual void update() = 0;
    virtual void draw() = 0;
};

}
#endif // ___SCREEN_HPP_
