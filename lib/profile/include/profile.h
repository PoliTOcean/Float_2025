#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * profile.h
 * Depth profile execution: PID-controlled descent, bottom hold, ascent.
 * Also owns mission logging and stored data replay.
 *******************************************************************************
 */

class ProfileManager {
public:
    ProfileManager();

    // Reset EEPROM read/write pointers (call before starting a new profile)
    void resetEEPROM();

    // Set active profile id and start the mission clock if needed.
    void beginProfile(uint8_t profileId);

    // Log and transmit the pre-descent defined data packet.
    void logDeploymentPacket();

    // Core measurement/control loop.
    //   targetDepth : desired depth in metres, or TARGET_SURFACE / TARGET_BOTTOM
    //   holdTimeSec : how long to remain stable at target before completing
    //   timeoutSec  : absolute time limit for the entire phase
    void measure(float targetDepth, float holdTimeSec, float timeoutSec);

    // Transmit all buffered flash CSV records to ESPB over ESP-NOW
    void sendStoredData();

    // Wipe legacy EEPROM data, reset pointers, and clear the flash CSV log
    void clearEEPROM();

    uint16_t writePtr() const { return _writePtr; }
    uint16_t readPtr()  const { return _readPtr; }

private:
    uint16_t _writePtr = 0;
    uint16_t _readPtr  = 0;
    uint8_t  _activeProfileId = 0;
    unsigned long _missionStartMs = 0;
    bool _missionClockRunning = false;

    void _logReading(float pressure, float temperature);
    void _logProfileReading(const char* phase);
    float _missionTimeS() const;
};

// Singleton
extern ProfileManager profileManager;
