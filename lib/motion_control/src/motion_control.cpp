#include "motion_control.h"
#include "config.h"
#include "led.h"
#include "sensors.h"
#include "comms.h"
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

bool MotionController::remoteStopRequested() {
    if (comms.lastCommand().command != CMD_STOP) {
        return false;
    }

    comms.clearCommand();
    emergencyStop("remote stop");
    comms.sendMessage(CMD13_ACK, 1000);
    return true;
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
    unsigned long lastTofSampleMs = 0;

    while (_motor.distanceToGo() != 0) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (timeoutMs > 0 && millis() - startMs > timeoutMs) {
            emergencyStop("movement timeout");
            return false;
        }

        _motor.run();
        if (tofMaxExtensionStopReached(nowMs, lastTofSampleMs, "movement")) {
            return false;
        }

        ledController.update();
        yield();
    }

    return true;
}

float MotionController::readPressureKpa() {
    sensors.read();
    return sensors.pressure() / 1000.0f;
}

bool MotionController::tofMaxExtensionStopReached(unsigned long nowMs,
                                                  unsigned long& lastTofSampleMs,
                                                  const char* context) {
    if (!_tof.isInitialized() || TOF_MAX_STOP_DISTANCE_MM <= 0.0f) {
        return false;
    }

    if (_motor.distanceToGo() <= 0) {
        return false;
    }

    if (nowMs - lastTofSampleMs < MOTOR_HOMING_TOF_PERIOD_MS) {
        return false;
    }
    lastTofSampleMs = nowMs;

    float distanceMm = 0.0f;
    if (!_tof.readDistanceMm(distanceMm) || distanceMm < TOF_MAX_STOP_DISTANCE_MM) {
        return false;
    }

    Debug.printf("%s: TOF max extension stop reached (%.1f >= %.1f mm)\n",
                 context,
                 distanceMm,
                 TOF_MAX_STOP_DISTANCE_MM);
    emergencyStop("TOF max extension limit");
    return true;
}

bool MotionController::pressureStopReached(float stopPressureKpa, uint8_t* pressureStopSamples) {
    if (stopPressureKpa <= 0.0f) {
        return false;
    }

    const float pressureKpa = readPressureKpa();
    if (pressureKpa > stopPressureKpa) {
        if (pressureStopSamples != nullptr) {
            (*pressureStopSamples)++;
            if (*pressureStopSamples < BALANCE_STOP_PRESSURE_SAMPLES) {
                return false;
            }
        }

        _motor.stop();
        _motor.disableOutputs();
        Debug.printf("Balance: pressure stop %.2f kPa > %.2f kPa\n",
                     pressureKpa,
                     stopPressureKpa);
        return true;
    }

    if (pressureStopSamples != nullptr) {
        *pressureStopSamples = 0;
    }

    return false;
}

bool MotionController::waitWithPressureStop(uint32_t waitMs, float stopPressureKpa, uint8_t* pressureStopSamples) {
    const unsigned long startMs = millis();

    while (millis() - startMs < waitMs) {
        if (remoteStopRequested()) {
            return true;
        }

        if (pressureStopReached(stopPressureKpa, pressureStopSamples)) {
            return true;
        }

        ledController.update();
        delay(BALANCE_PRESSURE_SAMPLE_PERIOD_MS);
    }

    return false;
}

bool MotionController::homeWithTof(float stopPressureKpa, bool* pressureStop, uint8_t* pressureStopSamples) {
    if (pressureStop != nullptr) {
        *pressureStop = false;
    }

    Debug.println("Motor homing: starting with TOF");
    clearEmergencyStop();
    ledController.setState(LEDState::HOMING);

    _motor.clearPosition();
    _motor.enableOutputs();
    _motor.setMaxSpeed(MOTOR_HOMING_SPEED);
    _motor.setAcceleration(MOTOR_HOMING_SPEED);
    _motor.startMoveSteps(-static_cast<long>(MOTOR_MAX_STEPS) * 2);

    const unsigned long startMs = millis();
    unsigned long lastTofSampleMs = 0;
    unsigned long lastPressureSampleMs = 0;
    bool homeDetected = false;

    while (_motor.distanceToGo() != 0 && !homeDetected) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastPressureSampleMs >= BALANCE_PRESSURE_SAMPLE_PERIOD_MS) {
            lastPressureSampleMs = nowMs;
            if (pressureStopReached(stopPressureKpa, pressureStopSamples)) {
                if (pressureStop != nullptr) {
                    *pressureStop = true;
                }
                return true;
            }
        }

        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor homing: timed out");
            emergencyStop("homing timeout");
            return false;
        }

        _motor.run();

        if (nowMs - lastTofSampleMs >= MOTOR_HOMING_TOF_PERIOD_MS) {
            lastTofSampleMs = nowMs;

            float distanceMm = 0.0f;
            if (_tof.readDistanceMm(distanceMm)) {
                if (distanceMm < TOF_HOMING_THRESHOLD) {
                    Debug.printf("Motor homing: threshold reached (%.1f < %.1f mm)\n",
                                 distanceMm, TOF_HOMING_THRESHOLD);
                    homeDetected = true;
                    _motor.stop();
                }
            }
        }

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

bool MotionController::moveToWithTimeout(long targetPosition,
                                         uint32_t timeoutMs,
                                         bool keepOutputsEnabled) {
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
    if (success && !keepOutputsEnabled) {
        _motor.disableOutputs();
    }

    return success;
}

bool MotionController::moveToMax(uint32_t timeoutMs,
                                 float stopPressureKpa,
                                 bool* pressureStop,
                                 uint8_t* pressureStopSamples) {
    if (pressureStop != nullptr) {
        *pressureStop = false;
    }

    if (!motionAllowed()) {
        return false;
    }

    if (!_motor.isPositionKnown()) {
        emergencyStop("moveToMax requested before motor position is known");
        return false;
    }

    const long targetPosition = static_cast<long>(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
    _motor.enableOutputs();
    _motor.startMoveTo(targetPosition);

    const unsigned long startMs = millis();
    unsigned long lastTofSampleMs = 0;
    unsigned long lastPressureSampleMs = 0;

    while (_motor.distanceToGo() != 0) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastPressureSampleMs >= BALANCE_PRESSURE_SAMPLE_PERIOD_MS) {
            lastPressureSampleMs = nowMs;
            if (pressureStopReached(stopPressureKpa, pressureStopSamples)) {
                if (pressureStop != nullptr) {
                    *pressureStop = true;
                }
                return true;
            }
        }

        if (timeoutMs > 0 && millis() - startMs > timeoutMs) {
            emergencyStop("moveToMax timeout");
            return false;
        }

        _motor.run();

        if (tofMaxExtensionStopReached(nowMs, lastTofSampleMs, "moveToMax")) {
            return false;
        }

        ledController.update();
        yield();
    }

    _motor.disableOutputs();
    return true;
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

    const float baselinePressureKpa = readPressureKpa();
    const float stopPressureKpa = baselinePressureKpa + BALANCE_STOP_PRESSURE_DELTA_KPA;
    uint8_t pressureStopSamples = 0;

    Debug.printf("Balance: baseline=%.2f kPa stop=%.2f kPa delta=%.2f kPa\n",
                 baselinePressureKpa,
                 stopPressureKpa,
                 BALANCE_STOP_PRESSURE_DELTA_KPA);

    while (motionAllowed()) {
        if (remoteStopRequested()) {
            return false;
        }

        bool pressureStop = false;

        if (pressureStopReached(stopPressureKpa, &pressureStopSamples)) {
            return true;
        }

        Debug.println("Balance: extending");
        if (!moveToMax(MOTOR_HOMING_TIMEOUT, stopPressureKpa, &pressureStop, &pressureStopSamples)) {
            return false;
        }
        if (pressureStop) {
            return true;
        }

        if (waitWithPressureStop(holdMs, stopPressureKpa, &pressureStopSamples)) {
            return true;
        }

        Debug.println("Balance: homing");
        if (!homeWithTof(stopPressureKpa, &pressureStop, &pressureStopSamples)) {
            return false;
        }
        if (pressureStop) {
            return true;
        }

        if (waitWithPressureStop(holdMs, stopPressureKpa, &pressureStopSamples)) {
            return true;
        }
    }

    return false;
}
