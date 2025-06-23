#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h>

class RGBLed {
public:
    enum CommonType {
        COMMON_CATHODE,
        COMMON_ANODE
    };
    
    // Common colors
    static const uint8_t BLACK[3];
    static const uint8_t RED[3];
    static const uint8_t GREEN[3];
    static const uint8_t BLUE[3];
    static const uint8_t YELLOW[3];
    static const uint8_t CYAN[3];
    static const uint8_t MAGENTA[3];
    static const uint8_t WHITE[3];
    static const uint8_t ORANGE[3];
    
    RGBLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin, CommonType type = COMMON_CATHODE);
    
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setColor(const uint8_t color[3]);
    void setColor(uint32_t color); // For hex colors
    void off();
    void brightness(uint8_t value);
    void flash(const uint8_t color[3], uint16_t period);
    void flash(uint32_t color, uint16_t period);
    void update();
    
private:
    uint8_t _redPin;
    uint8_t _greenPin;
    uint8_t _bluePin;
    CommonType _type;
    uint8_t _brightness;
    
    // Flash variables
    uint8_t _flashColor[3];
    uint16_t _flashPeriod;
    unsigned long _lastFlashTime;
    bool _flashState;
    bool _flashing;
    
    uint8_t adjustBrightness(uint8_t value);
    void writeColor(uint8_t red, uint8_t green, uint8_t blue);
};

#endif
