#ifndef ___SCREEN_HPP_
#define ___SCREEN_HPP_

#include "display.hpp"

namespace screen {
class Screen;

enum ScreenId {
	MAIN = 0,
	INFO, 
	COUNT, 
	DEBUG,
    MAX
};

class GUIManager {
	ScreenId screen_id = ScreenId::DEBUG;
	Screen **screen_list;

public:
    GUIManager(Screen **list) :
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
protected:
	Display::FrameBuff &fb = display.get_fb();
    GUIManager &screen_manager;
	void change_screen(ScreenId id) {
		screen_manager.set(id);
	}

public:
    Screen(GUIManager &screen_manager) : screen_manager(screen_manager) {}

    virtual bool button_up(int act) { return false; };
    virtual bool button_dn(int act) { return false; };
    virtual bool button_both(int act) { return false; };
    virtual void tick() = 0;
    virtual void draw() = 0;
};

}
#endif // ___SCREEN_HPP_
