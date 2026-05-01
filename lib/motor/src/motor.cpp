#include "motor.h"
#include "config.h"

/*
 *******************************************************************************
 * motor.cpp
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
MotorController::MotorController()
    : _stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}

// ---------------------------------------------------------------------------
void MotorController::begin() {
    pinMode(PIN_EN,        OUTPUT);
    pinMode(PIN_DRV_SLEEP, OUTPUT);
    pinMode(PIN_DRV_RST,   OUTPUT);

    // Driver awake and reset not asserted before configuring AccelStepper.
    digitalWrite(PIN_DRV_SLEEP, HIGH);
    digitalWrite(PIN_DRV_RST,   HIGH);
    digitalWrite(PIN_EN,        HIGH); // HIGH = disabled on DRV8825

    _stepper.setPinsInverted(/*dir*/true, /*step*/false, /*enable*/true);
    _stepper.setEnablePin(PIN_EN);
    _stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    _stepper.setAcceleration(MOTOR_MAX_ACCELERATION);
    _stepper.disableOutputs();
}

// ---------------------------------------------------------------------------
void MotorController::setCurrentPosition(long position) {
    _stepper.setCurrentPosition(position);
    _positionKnown = true;
}

// ---------------------------------------------------------------------------
void MotorController::clearPosition() {
    _positionKnown = false;
}

// ---------------------------------------------------------------------------
bool MotorController::moveTo(long targetPosition) {
    if (!_positionKnown) {
        return false;
    }

    enableOutputs();
    startMoveTo(targetPosition);

    while (distanceToGo() != 0) {
        run();
        yield();
    }

    disableOutputs();
    return true;
}

// ---------------------------------------------------------------------------
bool MotorController::moveSteps(long steps) {
    if (!_positionKnown) {
        return false;
    }

    return moveTo(_stepper.currentPosition() + steps);
}

// ---------------------------------------------------------------------------
void MotorController::startMoveTo(long targetPosition) {
    _stepper.moveTo(_clampTarget(targetPosition));
}

// ---------------------------------------------------------------------------
void MotorController::startMoveSteps(long steps) {
    _stepper.move(steps);
}

// ---------------------------------------------------------------------------
void MotorController::run() {
    _stepper.run();
}

// ---------------------------------------------------------------------------
void MotorController::stop() {
    _stepper.stop();
}

// ---------------------------------------------------------------------------
long MotorController::distanceToGo() {
    return _stepper.distanceToGo();
}

// ---------------------------------------------------------------------------
void MotorController::setMaxSpeed(float speed) {
    _stepper.setMaxSpeed(speed);
}

// ---------------------------------------------------------------------------
void MotorController::setAcceleration(float acceleration) {
    _stepper.setAcceleration(acceleration);
}

// ---------------------------------------------------------------------------
void MotorController::enableOutputs() {
    _stepper.enableOutputs();
}

// ---------------------------------------------------------------------------
void MotorController::disableOutputs() {
    _stepper.disableOutputs();
}

// ---------------------------------------------------------------------------
long MotorController::position() {
    return _stepper.currentPosition();
}

// ---------------------------------------------------------------------------
long MotorController::_clampTarget(long targetPosition) const {
    return constrain(targetPosition,
                     static_cast<long>(MOTOR_ENDSTOP_MARGIN),
                     static_cast<long>(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN));
}
