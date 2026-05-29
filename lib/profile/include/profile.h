#pragma once

#include <Arduino.h>
#include "config.h"

/*
 *******************************************************************************
 * profile.h
 * Depth profile execution: PID-controlled descent, bottom hold, ascent.
 * Also owns mission logging and stored data replay.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

struct RuntimeProfileConfig {
    uint8_t profileCount = PROFILE_MAX_COUNT;
    float deepTargetM = TARGET_DEPTH;
    float shallowTopTargetM = TARGET_SHALLOW_TOP_DEPTH;
    float depthToleranceM = DEPTH_MAX_ERROR;
    float holdTimeS = STAT_TIME;
    float pidTimeoutS = TIMEOUT_PID_TIME;
    float ascentTimeoutS = TIMEOUT_ASCENT;
    float surfaceOffsetM = SURFACE_TARGET_OFFSET_M;
};

class ProfileManager {
public:
    ProfileManager();

    // Load runtime profile settings from NVS, falling back to config.h defaults.
    void beginConfig();

    const RuntimeProfileConfig& config() const { return _config; }
    float shallowBottomTargetM() const;
    bool setConfig(const RuntimeProfileConfig& config);
    bool validateConfig(const RuntimeProfileConfig& config) const;
    void formatConfigJson(char* buffer, size_t bufferSize) const;

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
    RuntimeProfileConfig _config;

    void _logReading(float pressure, float temperature);
    void _logProfileReading(const char* phase);
    float _missionTimeS() const;
    void _saveConfig();
    void _applyConfigToSubsystems();
};

// Singleton
extern ProfileManager profileManager;
