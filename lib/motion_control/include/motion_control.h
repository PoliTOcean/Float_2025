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

    bool homeWithTof();
    bool waitForMotor(uint32_t timeoutMs);
    bool moveToWithTimeout(long targetPosition, uint32_t timeoutMs);
    bool manualStepTest(long steps, uint32_t speed);
    bool balance(uint32_t holdMs);

    bool motionAllowed();
    bool emergencyStopActive() const { return _emergencyStop; }
    void clearEmergencyStop();
    void emergencyStop(const char* reason);
    void serviceEmergencyStop();

private:
    MotorController& _motor;
    TofSensor& _tof;
    bool _emergencyStop = false;
};
