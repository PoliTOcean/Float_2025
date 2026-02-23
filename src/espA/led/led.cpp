#include "led.h"
#include "../config.h"

/*
 *******************************************************************************
 * led.cpp
 *******************************************************************************
 */

LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);

LEDController::LEDController(uint8_t rPin, uint8_t gPin, uint8_t bPin)
    : _led(rPin, gPin, bPin, RGBLed::COMMON_CATHODE) {}

void LEDController::setState(LEDState state) {
    if (_state == state) return; // Nothing to do
    _state = state;

    switch (state) {
        case LEDState::OFF:
            _led.off();
            break;
        case LEDState::INIT:
        case LEDState::IDLE:
            _led.setColor(0, 255, 0);               // Solid green
            break;
        case LEDState::IDLE_WITH_DATA:
            _led.flash(RGBLed::GREEN, 500);          // Fast green blink
            break;
        case LEDState::LOW_BATTERY:
            _led.setColor(255, 0, 0);               // Solid red
            break;
        case LEDState::ERROR:
            _led.flash(RGBLed::RED, 500);            // Fast red blink
            break;
        case LEDState::PROFILE:
            _led.setColor(RGBLed::BLUE);            // Solid blue
            break;
        case LEDState::AUTO_MODE:
            _led.flash(RGBLed::YELLOW, 750);         // Yellow blink
            break;
        case LEDState::HOMING:
            _led.flash(RGBLed::MAGENTA, 500);        // Magenta blink
            break;
        case LEDState::MOTOR_MOVING:
            _led.setColor(255, 0, 255);             // Solid purple
            break;
        case LEDState::PID_CONTROL:
            _led.flash(RGBLed::CYAN, 600);           // Cyan blink
            break;
        case LEDState::COMMUNICATION:
            _led.setColor(RGBLed::WHITE);           // Solid white
            break;
        case LEDState::OTA_MODE:
            _led.flash(RGBLed::ORANGE, 500);         // Orange blink
            break;
    }
}

void LEDController::update() {
    _led.update();
}