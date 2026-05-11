#include "motor.h"
#include "config.h"

/*
 *******************************************************************************
 * motor.cpp
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
MotorController::MotorController() {}

// ---------------------------------------------------------------------------
void MotorController::begin() {
    pinMode(PIN_EN,        OUTPUT);
    pinMode(PIN_DRV_SLEEP, OUTPUT);
    pinMode(PIN_DRV_RST,   OUTPUT);

    // Driver awake and reset not asserted before configuring FastAccelStepper.
    digitalWrite(PIN_DRV_SLEEP, HIGH);
    digitalWrite(PIN_DRV_RST,   HIGH);
    digitalWrite(PIN_EN,        HIGH); // HIGH = disabled on DRV8825

    _engine.init();
    _stepper = _engine.stepperConnectToPin(PIN_STEP);
    if (!_stepper) {
        return;
    }

    _stepper->setDirectionPin(PIN_DIR, true, 200);
    _stepper->setEnablePin(PIN_EN, true);
    _stepper->setAutoEnable(false);

    setMaxSpeed(MOTOR_MAX_SPEED);
    setAcceleration(MOTOR_MAX_ACCELERATION);
    disableOutputs();
}

// ---------------------------------------------------------------------------
void MotorController::setCurrentPosition(long position) {
    if (!_isReady()) {
        return;
    }

    _stepper->forceStopAndNewPosition(static_cast<int32_t>(position));
    _targetPosition = position;
    _positionKnown = true;
}

// ---------------------------------------------------------------------------
void MotorController::clearPosition() {
    _positionKnown = false;
}

// ---------------------------------------------------------------------------
bool MotorController::moveTo(long targetPosition) {
    if (!_positionKnown || !_isReady()) {
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
    if (!_positionKnown || !_isReady()) {
        return false;
    }

    return moveTo(position() + steps);
}

// ---------------------------------------------------------------------------
void MotorController::startMoveTo(long targetPosition) {
    _startRawMoveTo(_clampTarget(targetPosition));
}

// ---------------------------------------------------------------------------
void MotorController::startMoveSteps(long steps) {
    _startRawMoveTo(position() + steps);
}

// ---------------------------------------------------------------------------
void MotorController::run() {}

// ---------------------------------------------------------------------------
void MotorController::stop() {
    if (!_isReady()) {
        return;
    }

    const long currentPosition = position();
    _stepper->forceStopAndNewPosition(static_cast<int32_t>(currentPosition));
    _targetPosition = currentPosition;
}

// ---------------------------------------------------------------------------
long MotorController::distanceToGo() {
    if (!_isReady() || !_stepper->isRunning()) {
        return 0;
    }

    return _targetPosition - position();
}

// ---------------------------------------------------------------------------
void MotorController::setMaxSpeed(float speed) {
    if (!_isReady()) {
        _maxSpeed = static_cast<uint32_t>(max(1.0f, speed));
        return;
    }

    _maxSpeed = static_cast<uint32_t>(max(1.0f, speed) + 0.5f);
    _stepper->setSpeedInHz(_maxSpeed);
}

// ---------------------------------------------------------------------------
void MotorController::setAcceleration(float acceleration) {
    if (!_isReady()) {
        _acceleration = static_cast<int32_t>(max(1.0f, acceleration));
        return;
    }

    _acceleration = static_cast<int32_t>(max(1.0f, acceleration) + 0.5f);
    _stepper->setAcceleration(_acceleration);
}

// ---------------------------------------------------------------------------
void MotorController::enableOutputs() {
    if (_isReady()) {
        _stepper->enableOutputs();
    }
}

// ---------------------------------------------------------------------------
void MotorController::disableOutputs() {
    if (_isReady()) {
        _stepper->disableOutputs();
    }
}

// ---------------------------------------------------------------------------
long MotorController::position() {
    if (!_isReady()) {
        return 0;
    }

    return _stepper->getCurrentPosition();
}

// ---------------------------------------------------------------------------
long MotorController::_clampTarget(long targetPosition) const {
    return constrain(targetPosition,
                     static_cast<long>(MOTOR_ENDSTOP_MARGIN),
                     static_cast<long>(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN));
}

// ---------------------------------------------------------------------------
bool MotorController::_isReady() const {
    return _stepper != nullptr;
}

// ---------------------------------------------------------------------------
bool MotorController::_startRawMoveTo(long targetPosition) {
    if (!_isReady()) {
        return false;
    }

    _targetPosition = targetPosition;
    return _stepper->moveTo(static_cast<int32_t>(targetPosition)) == MOVE_OK;
}
