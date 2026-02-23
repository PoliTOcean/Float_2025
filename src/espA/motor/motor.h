#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

/*
 *******************************************************************************
 * motor.h
 * Stepper motor control: homing, safe absolute/relative moves, endstop handling.
 *
 * Coordinate system (after homing):
 *   0                  = home / fully retracted (lower endstop side)
 *   MOTOR_MAX_STEPS    = fully extended (upper endstop side)
 *   Safe range         = [MOTOR_ENDSTOP_MARGIN, MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN]
 *******************************************************************************
 */

class MotorController {
public:
    MotorController();

    // Call once in setup() — configures pins and AccelStepper
    void begin();

    // Drive to lower endstop and set position = 0.
    // Returns true on success, false on timeout or unexpected endstop hit.
    bool home();

    // Move to an absolute step position within the safe range.
    // Returns true if the target was reached, false if interrupted by an endstop.
    bool moveTo(long targetPosition);

    // Move a relative number of steps from the current position.
    bool moveSteps(long steps);

    // Current position according to AccelStepper
    long position();

    bool isHomed()        const { return _homed; }
    bool emergencyStop()  const { return _emergencyStop; }

    // Call every loop iteration (and inside blocking move loops via yield)
    // to process interrupt flags and drive the stepper.
    // is_homing: true only during the homing sequence (enables lower endstop anywhere)
    void processEndstops(bool isHoming = false);

    // Exposed so the profile / main loop can call stepper.run() if needed
    AccelStepper stepper;

private:
    bool _homed         = false;
    bool _emergencyStop = false;

    // Latched endstop states (cleared only inside handleEndstopHit)
    bool _upperHit      = false;
    bool _lowerHit      = false;

    // Back off from whichever endstop was hit, then clear the latch
    void _handleEndstopHit();

    void _disableOutputs();
    void _enableOutputs();
};

// ---------------------------------------------------------------------------
// Interrupt flags — set by ISRs, consumed by processEndstops()
// ---------------------------------------------------------------------------
extern volatile bool g_upperEndstopTriggered;
extern volatile bool g_lowerEndstopTriggered;

void IRAM_ATTR upperEndstopISR();
void IRAM_ATTR lowerEndstopISR();

// Singleton
extern MotorController motor;