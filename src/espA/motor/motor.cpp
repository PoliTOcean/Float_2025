#include "motor.h"
#include "../config.h"
#include "../led/led.h"
#include <DebugSerial.h>

/*
 *******************************************************************************
 * motor.cpp
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
// ISR flags (file-scope, consumed by processEndstops)
// ---------------------------------------------------------------------------
volatile bool g_upperEndstopTriggered = false;
volatile bool g_lowerEndstopTriggered = false;

void IRAM_ATTR upperEndstopISR() { g_upperEndstopTriggered = true; }
void IRAM_ATTR lowerEndstopISR() { g_lowerEndstopTriggered = true; }

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------
MotorController motor;

// ---------------------------------------------------------------------------
MotorController::MotorController()
    : stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}

// ---------------------------------------------------------------------------
void MotorController::begin() {
    // DRV8825 power pins
    pinMode(PIN_EN,        OUTPUT);
    pinMode(PIN_DRV_SLEEP, OUTPUT);
    digitalWrite(PIN_EN,        HIGH); // Disabled at startup
    digitalWrite(PIN_DRV_SLEEP, HIGH); // Driver awake

    // Endstop inputs
    pinMode(PIN_ENDSTOP_UP,   INPUT_PULLUP);
    pinMode(PIN_ENDSTOP_DOWN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_UP),   upperEndstopISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_DOWN), lowerEndstopISR, FALLING);

    // AccelStepper configuration
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);
    stepper.setEnablePin(PIN_EN);
    stepper.setPinsInverted(/*dir*/true, /*step*/false, /*enable*/true); // EN is active-low
    _disableOutputs();
}

// ---------------------------------------------------------------------------
bool MotorController::home() {
    Debug.println("Motor: starting homing sequence");
    ledController.setState(LEDState::HOMING);

    _homed         = false;
    _upperHit      = false;
    _lowerHit      = false;
    _emergencyStop = false;
    g_upperEndstopTriggered = false;
    g_lowerEndstopTriggered = false;

    _enableOutputs();
    stepper.setMaxSpeed(MOTOR_HOMING_SPEED);
    stepper.setAcceleration(MOTOR_HOMING_SPEED);
    stepper.move(-static_cast<long>(MOTOR_MAX_STEPS) * 2); // Overshoot to guarantee endstop contact

    unsigned long startMs = millis();

    while (!_lowerHit && !_emergencyStop) {
        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor: homing timed out");
            stepper.stop();
            _disableOutputs();
            return false;
        }
        stepper.run();
        processEndstops(/*isHoming=*/true);
        ledController.update();
        yield();
    }

    delay(100); // Let the motor settle

    if (!_lowerHit) {
        // Emergency stop was triggered by something other than the lower endstop
        Debug.println("Motor: homing aborted — unexpected emergency stop");
        if (_upperHit) _handleEndstopHit();
        _disableOutputs();
        return false;
    }

    // Back off and define home position
    _handleEndstopHit();                  // Backs off ENDSTOP_MARGIN steps
    stepper.setCurrentPosition(0);        // Zero = home
    _emergencyStop = false;

    // Restore normal operating speeds
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);

    _homed = true;
    _disableOutputs();
    Debug.println("Motor: homing complete — position set to 0");
    return true;
}

// ---------------------------------------------------------------------------
bool MotorController::moveTo(long targetPosition) {
    if (!_homed) {
        Debug.println("Motor: moveTo() called before homing — ignoring");
        return false;
    }

    // Clamp to safe range
    targetPosition = constrain(targetPosition,
                                MOTOR_ENDSTOP_MARGIN,
                                MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);

    _upperHit      = false;
    _lowerHit      = false;
    _emergencyStop = false;
    g_upperEndstopTriggered = false;
    g_lowerEndstopTriggered = false;

    _enableOutputs();
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);
    stepper.moveTo(targetPosition);

    Debug.printf("Motor: moving to %ld\n", targetPosition);

    while (stepper.distanceToGo() != 0 && !_emergencyStop) {
        stepper.run();
        processEndstops(/*isHoming=*/false);
        ledController.update();
        yield();
    }

    if (_emergencyStop) {
        Debug.println("Motor: emergency stop during moveTo");
        _handleEndstopHit();
        _disableOutputs();
        return false;
    }

    _disableOutputs();
    Debug.printf("Motor: reached position %ld\n", stepper.currentPosition());
    return true;
}

// ---------------------------------------------------------------------------
bool MotorController::moveSteps(long steps) {
    return moveTo(stepper.currentPosition() + steps);
}

// ---------------------------------------------------------------------------
long MotorController::position() {
    return stepper.currentPosition();
}

// ---------------------------------------------------------------------------
void MotorController::processEndstops(bool isHoming) {
    const long pos    = stepper.currentPosition();
    const long target = stepper.targetPosition();
    const bool movingUp   = target > pos;
    const bool movingDown = target < pos;

    // --- Upper endstop ---
    // Only relevant when moving up and close to the upper limit
    const bool nearUpper = pos >= static_cast<long>(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_WINDOW);
    if (movingUp && nearUpper && g_upperEndstopTriggered) {
        g_upperEndstopTriggered = false;
        if (!_upperHit) {
            Debug.println("Motor: upper endstop hit");
            _upperHit      = true;
            _emergencyStop = true;
            stepper.setCurrentPosition(stepper.targetPosition()); // Halt immediately
        }
    } else {
        // Clear stale flag if we're not in the checking window
        g_upperEndstopTriggered = false;
    }

    // --- Lower endstop ---
    // Relevant during homing (always) or when moving down near the lower limit
    const bool nearLower = pos <= static_cast<long>(MOTOR_ENDSTOP_WINDOW);
    const bool checkLower = isHoming || (movingDown && nearLower);
    if (checkLower && g_lowerEndstopTriggered) {
        g_lowerEndstopTriggered = false;
        if (!_lowerHit) {
            Debug.println("Motor: lower endstop hit");
            _lowerHit      = true;
            _emergencyStop = true;
            stepper.setCurrentPosition(stepper.targetPosition());
        }
    } else {
        g_lowerEndstopTriggered = false;
    }
}

// ---------------------------------------------------------------------------
void MotorController::_handleEndstopHit() {
    _enableOutputs();
    constexpr unsigned long BACKOFF_TIMEOUT_MS = 2000;

    if (_upperHit) {
        stepper.setAcceleration(MOTOR_MAX_SPEED);
        stepper.move(-MOTOR_ENDSTOP_MARGIN);

        unsigned long t0 = millis();
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            ledController.update();
            yield();
            if (millis() - t0 > BACKOFF_TIMEOUT_MS) {
                Debug.println("Motor: timeout backing off upper endstop");
                stepper.stop();
                break;
            }
        }
        stepper.setCurrentPosition(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
        Debug.printf("Motor: backed off upper endstop → pos %ld\n", stepper.currentPosition());
        _upperHit = false;
    }

    if (_lowerHit) {
        stepper.setAcceleration(MOTOR_HOMING_SPEED);
        stepper.move(MOTOR_ENDSTOP_MARGIN);

        unsigned long t0 = millis();
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            ledController.update();
            yield();
            if (millis() - t0 > BACKOFF_TIMEOUT_MS) {
                Debug.println("Motor: timeout backing off lower endstop");
                stepper.stop();
                break;
            }
        }
        stepper.setCurrentPosition(MOTOR_ENDSTOP_MARGIN);
        Debug.printf("Motor: backed off lower endstop → pos %ld\n", stepper.currentPosition());
        _lowerHit = false;
    }

    _disableOutputs();
}

// ---------------------------------------------------------------------------
void MotorController::_enableOutputs()  { stepper.enableOutputs(); }
void MotorController::_disableOutputs() { stepper.disableOutputs(); }