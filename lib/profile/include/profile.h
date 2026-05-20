#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * profile.h
 * Depth profile execution: PID-controlled descent, bottom hold, ascent.
 * Also owns the EEPROM logging of sensor data.
 *******************************************************************************
 */

class ProfileManager {
public:
    ProfileManager();

    // Reset EEPROM read/write pointers (call before starting a new profile)
    void resetEEPROM();

    // Core measurement/control loop.
    //   targetDepth : desired depth in metres, or TARGET_SURFACE / TARGET_BOTTOM
    //   holdTimeSec : how long to remain stable at target before completing
    //   timeoutSec  : absolute time limit for the entire phase
    void measure(float targetDepth, float holdTimeSec, float timeoutSec);

    // Transmit all buffered EEPROM records to ESPB over ESP-NOW
    void sendStoredData();

    // Wipe EEPROM data and reset pointers
    void clearEEPROM();

    uint16_t writePtr() const { return _writePtr; }
    uint16_t readPtr()  const { return _readPtr; }

private:
    uint16_t _writePtr = 0;
    uint16_t _readPtr  = 0;

    void _logReading(float pressure, float temperature);
};

// Singleton
extern ProfileManager profileManager;