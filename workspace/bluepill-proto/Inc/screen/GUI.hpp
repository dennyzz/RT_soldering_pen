#ifndef ___GUI_HPP__
#define ___GUI_HPP__

#include "display.hpp"
#include "screen/screen.hpp"
#include "screen/debug.hpp"
#include "screen/splash.hpp"
#include "button.hpp"

namespace screen{

class GUI {
    screen::ScreenHolder screen_holder;

    // instantiate all screens
    screen::Splash screen_main;
    screen::Splash screen_menu;
    screen::Debug screen_debug;
    screen::Splash screen_splash;

    // screen::Menu screen_menu;
    Heater_struct &heater;
    // Settings &settings;

    // add all screens to the holder
    screen::Screen *screens[static_cast<int>(screen::ScreenId::MAX)] = {
        &screen_debug,
        &screen_main,
        &screen_menu,
        &screen_splash,
    };

    static const unsigned BUTTON_SAMPLE_MIN_TICKS = 10;  // ticks
    unsigned buttons_sample_ticks = 0;
    Button button_up;
    Button button_dw;
    Button button_both;

    bool rotation_last = false;

    void buttons_process_raw(unsigned delta_ticks) {
        buttons_sample_ticks += delta_ticks;
        if (buttons_sample_ticks < BUTTON_SAMPLE_MIN_TICKS) return;
        bool btn_state_up;
        bool btn_state_dw;
        // if (_settings.get_left_handed()) {
        //     btn_state_up = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BTN_DWN_GPIO_Port, BTN_DWN_Pin));
        //     btn_state_dw = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin));
        // } else {
            btn_state_up = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin));
            btn_state_dw = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BTN_DWN_GPIO_Port, BTN_DWN_Pin));
        // }
        button_up.process(btn_state_up, btn_state_dw, buttons_sample_ticks);
        button_dw.process(btn_state_dw, btn_state_up, buttons_sample_ticks);
        button_both.process(btn_state_up && btn_state_dw, false, buttons_sample_ticks);
        buttons_sample_ticks = 0;
    }

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
    GUI(Heater_struct &heating) : 
        screen_holder(screens),
        screen_main(screen_holder),
        screen_menu(screen_holder),
        screen_debug(screen_holder, heating),
		screen_splash(screen_holder), 
        heater(heating) {}
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
