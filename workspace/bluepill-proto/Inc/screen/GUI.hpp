#ifndef ___GUI_HPP__
#define ___GUI_HPP__

#include "display.hpp"
#include "screen/screen.hpp"
#include "screen/debug.hpp"
#include "screen/splash.hpp"
namespace screen{

class GUI {
    screen::ScreenHolder screen_holder;

    // instantiate all screens
    screen::Splash screen_main;
    screen::Splash screen_menu;
    screen::Debug screen_debug;
    screen::Splash screen_splash;

    // screen::Menu screen_menu;

    // Settings &_settings;

    // add all screens to the holder
    screen::Screen *screens[static_cast<int>(screen::ScreenId::MAX)] = {
        &screen_main,
        &screen_menu,
        &screen_debug,
        &screen_splash,
    };

    // static const unsigned BUTTONS_SAMPLE_TICKS = board::Clock::CORE_FREQ / 1000 * 10;  // ticks
    // unsigned _buttons_sample_ticks = 0;
    // lib::Button _button_up;
    // lib::Button _button_dw;
    // lib::Button _button_both;

    // bool rotation_last = false;

    // void _buttons_process_fast(unsigned delta_ticks) {
    //     _buttons_sample_ticks += delta_ticks;
    //     if (_buttons_sample_ticks < BUTTONS_SAMPLE_TICKS) return;
    //     _buttons_sample_ticks -= BUTTONS_SAMPLE_TICKS;
    //     bool btn_state_up;
    //     bool btn_state_dw;
    //     if (_settings.get_left_handed()) {
    //         btn_state_up = board::Buttons::get_instance().is_pressed_dw();
    //         btn_state_dw = board::Buttons::get_instance().is_pressed_up();
    //     } else {
    //         btn_state_up = board::Buttons::get_instance().is_pressed_up();
    //         btn_state_dw = board::Buttons::get_instance().is_pressed_dw();
    //     }
    //     _button_up.process(btn_state_up, btn_state_dw, 10);
    //     _button_dw.process(btn_state_dw, btn_state_up, 10);
    //     _button_both.process(btn_state_up && btn_state_dw, false, 10);
    // }

    // void _buttons_process() {
    //     lib::Button::Action btn_action_up = _button_up.get_status();
    //     lib::Button::Action btn_action_dw = _button_dw.get_status();
    //     lib::Button::Action btn_action_both = _button_both.get_status();
    //     if (_screen_holder.get()->button_up(btn_action_up)) _button_up.block();
    //     if (_screen_holder.get()->button_dw(btn_action_dw)) _button_dw.block();
    //     if (_screen_holder.get()->button_both(btn_action_both)) _button_both.block();
    // }

    Display::FrameBuff &fb = display.get_fb();

public:

    // GUI(Heating &heating, Settings &settings) :
    GUI() : 
        screen_holder(screens),
        screen_main(screen_holder),
        screen_menu(screen_holder),
        screen_debug(screen_holder),
		screen_splash(screen_holder) {}
        // _screen_main(_screen_holder, heating, settings),
        // _screen_menu(_screen_holder, heating, settings),
        // _settings(settings) {}

    void init(void)
    {
        display.init();   
    }
    void process(unsigned delta_ticks) {
        //_buttons_process_fast(delta_ticks);
        screen_holder.get()->update();
    }

    void draw() {
        // if (board::I2c::get_instance().is_busy()) return;
        // bool rotation = _settings.get_left_handed();
        // if (rotation_last != rotation) {
        //     rotation_last = rotation;
        //     board::display.rotate(rotation, rotation);
        // }

        fb.clear();
        screen_holder.get()->draw();
        display.redraw();
    }

};

}

#endif //___GUI_HPP__
