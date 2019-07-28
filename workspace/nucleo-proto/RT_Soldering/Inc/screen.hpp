#ifndef ___SCREEN_HPP_
#define ___SCREEN_HPP_

namespace screen {

class Screen;

enum class ScreenId {
	MAIN,
	INFO, 
	COUNT, 
	DEBUG,
};

class GUIManager {
	ScreenId screen_id = ScreenId::MAIN;
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

}

/* base class for all Screen displays */
class Screen {

	board::Display::Fb &fb = board::display.get_fb();
    GUIManager &screen_manager;
protected:
	void change_screen(ScreenId id) {
		screen_manager.set(id);
	}

public:
    Screen(GUIManager &screen_manager) : screen_manager(screen_manager) {}

    virtual bool button_up(int act) { return false; };
    virtual bool button_dn(int act) { return false; };
    virtual bool button_both(int act) { return false; };
    virtual void draw() = 0;

};

}
#endif // ___SCREEN_HPP_
