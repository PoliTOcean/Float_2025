#include "motion_control.h"
#include "config.h"
#include "led.h"
#include "DebugSerial.h"

MotionController::MotionController(MotorController& motor, TofSensor& tof)
    : _motor(motor),
      _tof(tof) {}

void MotionController::clearEmergencyStop() {
    _emergencyStop = false;
}

void MotionController::emergencyStop(const char* reason) {
    _emergencyStop = true;
    _motor.stop();
    _motor.disableOutputs();
    Debug.printf("Motor emergency stop: %s\n", reason);
    ledController.setState(LEDState::ERROR);
}

void MotionController::serviceEmergencyStop() {
    if (!_emergencyStop) {
        return;
    }

    _motor.stop();
    _motor.disableOutputs();
    ledController.setState(LEDState::ERROR);
}

bool MotionController::motionAllowed() {
    if (!_emergencyStop) {
        return true;
    }

    Debug.println("Motor command blocked: emergency stop active");
    serviceEmergencyStop();
    return false;
}

bool MotionController::waitForMotor(uint32_t timeoutMs) {
    const unsigned long startMs = millis();

    while (_motor.distanceToGo() != 0) {
        if (timeoutMs > 0 && millis() - startMs > timeoutMs) {
            emergencyStop("movement timeout");
            return false;
        }

        _motor.run();
        ledController.update();
        yield();
    }

    return true;
}

bool MotionController::homeWithTof() {
    Debug.println("Motor homing: starting with TOF");
    clearEmergencyStop();
    ledController.setState(LEDState::HOMING);

    _motor.clearPosition();
    _motor.enableOutputs();
    _motor.setMaxSpeed(MOTOR_HOMING_SPEED);
    _motor.setAcceleration(MOTOR_HOMING_SPEED);
    _motor.startMoveSteps(-static_cast<long>(MOTOR_MAX_STEPS) * 2);

    const unsigned long startMs = millis();
    bool homeDetected = false;

    while (_motor.distanceToGo() != 0 && !homeDetected) {
        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor homing: timed out");
            emergencyStop("homing timeout");
            return false;
        }

        float distanceMm = 0.0f;
        if (_tof.readActiveMinDistanceMm(distanceMm)) {
            Debug.printf("Motor homing: TOF distance = %.1f mm\n", distanceMm);

            if (distanceMm < TOF_HOMING_THRESHOLD) {
                Debug.printf("Motor homing: threshold reached (%.1f < %.1f mm)\n",
                             distanceMm, TOF_HOMING_THRESHOLD);
                homeDetected = true;
            }
        }

        _motor.run();
        ledController.update();
        yield();
    }

    if (!homeDetected) {
        Debug.println("Motor homing: no TOF detection");
        emergencyStop("homing finished without TOF detection");
        return false;
    }

    delay(100);

    _motor.startMoveSteps(MOTOR_ENDSTOP_MARGIN);
    if (!waitForMotor(2000)) {
        Debug.println("Motor homing: timeout during backoff");
        return false;
    }

    _motor.setCurrentPosition(0);
    _motor.setMaxSpeed(MOTOR_MAX_SPEED);
    _motor.setAcceleration(MOTOR_MAX_ACCELERATION);
    _motor.disableOutputs();
    clearEmergencyStop();

    Debug.println("Motor homing: complete, position set to 0");
    return true;
}

bool MotionController::moveToWithTimeout(long targetPosition, uint32_t timeoutMs) {
    if (!motionAllowed()) {
        return false;
    }

    if (!_motor.isPositionKnown()) {
        emergencyStop("move requested before motor position is known");
        return false;
    }

    _motor.enableOutputs();
    _motor.startMoveTo(targetPosition);

    const bool success = waitForMotor(timeoutMs);
    if (success) {
        _motor.disableOutputs();
    }

    return success;
}

bool MotionController::manualStepTest(long steps, uint32_t speed) {
    if (!motionAllowed()) {
        return false;
    }

    Debug.printf("Test: moving %ld steps at %u steps/s\n", steps, speed);

    _motor.setMaxSpeed(speed);
    _motor.setAcceleration(MOTOR_MAX_ACCELERATION);
    _motor.enableOutputs();
    _motor.startMoveSteps(steps);

    const bool success = waitForMotor(0);

    _motor.setMaxSpeed(MOTOR_MAX_SPEED);
    _motor.setAcceleration(MOTOR_MAX_ACCELERATION);
    if (success) {
        _motor.disableOutputs();
        Debug.printf("Test complete — pos %ld\n", _motor.position());
    }

    return success;
}

bool MotionController::balance(uint32_t holdMs) {
    if (!motionAllowed()) {
        return false;
    }

    Debug.println("Balance: extending");
    if (!moveToWithTimeout(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN, 0)) {
        return false;
    }

    delay(holdMs);

    Debug.println("Balance: retracting");
    return moveToWithTimeout(MOTOR_ENDSTOP_MARGIN, 0);
}
