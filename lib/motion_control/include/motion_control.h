#pragma once

#include <Arduino.h>
#include "motor.h"
#include "tof.h"

/*
 *******************************************************************************
 * motion_control.h
 * Firmware-level motion control that coordinates motor, TOF, LED and debug.
 *******************************************************************************
 */

class MotionController {
public:
    MotionController(MotorController& motor, TofSensor& tof);

    bool homeWithTof(float stopPressureKpa = 0.0f, bool* pressureStop = nullptr, uint8_t* pressureStopSamples = nullptr);
    bool waitForMotor(uint32_t timeoutMs);
    bool moveToMax(uint32_t timeoutMs = 0, float stopPressureKpa = 0.0f, bool* pressureStop = nullptr, uint8_t* pressureStopSamples = nullptr);
    bool moveToWithTimeout(long targetPosition, uint32_t timeoutMs, bool keepOutputsEnabled = false);
    bool manualStepTest(long steps, uint32_t speed);
    bool balance(uint32_t holdMs);

    bool motionAllowed();
    bool emergencyStopActive() const { return _emergencyStop; }
    void clearEmergencyStop();
    void emergencyStop(const char* reason);
    void serviceEmergencyStop();
    bool remoteStopRequested();

private:
    MotorController& _motor;
    TofSensor& _tof;
    bool _emergencyStop = false;

    float readPressureKpa();
    bool tofMaxExtensionStopReached(unsigned long nowMs,
                                    unsigned long& lastTofSampleMs,
                                    const char* context);
    bool pressureStopReached(float stopPressureKpa, uint8_t* pressureStopSamples = nullptr);
    bool waitWithPressureStop(uint32_t waitMs, float stopPressureKpa, uint8_t* pressureStopSamples = nullptr);
};

// Singleton defined by the main firmware.
extern MotionController motionController;
