#include "RGBLed.h"

// Color definitions
const uint8_t RGBLed::BLACK[3] = {0, 0, 0};
const uint8_t RGBLed::RED[3] = {255, 0, 0};
const uint8_t RGBLed::GREEN[3] = {0, 255, 0};
const uint8_t RGBLed::BLUE[3] = {0, 0, 255};
const uint8_t RGBLed::YELLOW[3] = {255, 255, 0};
const uint8_t RGBLed::CYAN[3] = {0, 255, 255};
const uint8_t RGBLed::MAGENTA[3] = {255, 0, 255};
const uint8_t RGBLed::WHITE[3] = {255, 255, 255};
const uint8_t RGBLed::ORANGE[3] = {255, 165, 0};

RGBLed::RGBLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin, CommonType type) {
    _redPin = redPin;
    _greenPin = greenPin;
    _bluePin = bluePin;
    _type = type;
    _brightness = 255;
    _flashing = false;
    _flashState = false;
    _lastFlashTime = 0;
    
    pinMode(_redPin, OUTPUT);
    pinMode(_greenPin, OUTPUT);
    pinMode(_bluePin, OUTPUT);
    
    off();
}

void RGBLed::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    _flashing = false;
    writeColor(red, green, blue);
}

void RGBLed::setColor(const uint8_t color[3]) {
    setColor(color[0], color[1], color[2]);
}

void RGBLed::setColor(uint32_t color) {
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;
    setColor(red, green, blue);
}

void RGBLed::off() {
    _flashing = false;
    writeColor(0, 0, 0);
}

void RGBLed::brightness(uint8_t value) {
    _brightness = value;
}

void RGBLed::flash(const uint8_t color[3], uint16_t period) {
    _flashColor[0] = color[0];
    _flashColor[1] = color[1];
    _flashColor[2] = color[2];
    _flashPeriod = period;
    _flashing = true;
    _lastFlashTime = millis();
    _flashState = true;
    writeColor(_flashColor[0], _flashColor[1], _flashColor[2]);
}

void RGBLed::flash(uint32_t color, uint16_t period) {
    uint8_t colorArray[3];
    colorArray[0] = (color >> 16) & 0xFF; // Red
    colorArray[1] = (color >> 8) & 0xFF;  // Green
    colorArray[2] = color & 0xFF;         // Blue
    flash(colorArray, period);
}

void RGBLed::update() {
    if (_flashing) {
        unsigned long currentTime = millis();
        if (currentTime - _lastFlashTime >= _flashPeriod / 2) {
            _flashState = !_flashState;
            _lastFlashTime = currentTime;
            
            if (_flashState) {
                writeColor(_flashColor[0], _flashColor[1], _flashColor[2]);
            } else {
                writeColor(0, 0, 0);
            }
        }
    }
}

uint8_t RGBLed::adjustBrightness(uint8_t value) {
    return (uint8_t)((uint16_t)value * _brightness / 255);
}

void RGBLed::writeColor(uint8_t red, uint8_t green, uint8_t blue) {
    red = adjustBrightness(red);
    green = adjustBrightness(green);
    blue = adjustBrightness(blue);
    
    if (_type == COMMON_ANODE) {
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
    }
    
    analogWrite(_redPin, red);
    analogWrite(_greenPin, green);
    analogWrite(_bluePin, blue);
}
