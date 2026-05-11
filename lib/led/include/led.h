#pragma once

#include <Arduino.h>
#include <RGBLed.h>

/*
 *******************************************************************************
 * led.h
 * RGB LED state machine.
 * Each state maps to a colour and optional blink pattern.
 *******************************************************************************
 */

enum class LEDState : uint8_t {
    OFF,
    INIT,
    IDLE,
    IDLE_WITH_DATA,
    LOW_BATTERY,
    ERROR,
    PROFILE,
    AUTO_MODE,
    HOMING,
    MOTOR_MOVING,
    PID_CONTROL,
    COMMUNICATION,
    OTA_MODE,
};

class LEDController {
public:
    LEDController(uint8_t rPin, uint8_t gPin, uint8_t bPin);

    // Apply a new state (idempotent — safe to call repeatedly)
    void setState(LEDState state);

    // Must be called every loop iteration to drive blink animations
    void update();

    LEDState currentState() const { return _state; }

private:
    RGBLed   _led;
    LEDState _state = LEDState::OFF;
};

// Singleton — defined in led.cpp, used throughout the project
extern LEDController ledController;