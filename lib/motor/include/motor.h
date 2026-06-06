#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>

/*
 *******************************************************************************
 * motor.h
 * Stepper motor controller: driver setup, movement primitives, and position.
 *
 * Higher-level procedures such as homing, sensor checks, LEDs, and debug output
 * belong to the firmware layer that coordinates multiple controllers.
 *
 * Coordinate system, once the firmware has established a reference:
 *   0                       = reference/home position (pistone inserito, vuota)
 *   uToMotorPos(1.0f)       = fully extended (siringa piena d'acqua)
 *   Safe range (symmetric)  = [-(MAX-margin), +(MAX-margin)]
 *   See include/config.h: uToMotorPos() respects MOTOR_INVERT_LOGICAL.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

class MotorController {
public:
    MotorController();

    // Configure motor driver pins and FastAccelStepper.
    void begin();

    // Mark the current mechanical position as known.
    void setCurrentPosition(long position);
    void clearPosition();
    bool isPositionKnown() const { return _positionKnown; }

    // Blocking moves. Absolute moves require a known reference position.
    bool moveTo(long targetPosition);
    bool moveSteps(long steps);

    // Non-blocking movement primitives for firmware-level procedures.
    void startMoveTo(long targetPosition);
    void startMoveSteps(long steps);
    // Jog relativo SENZA clamp ai fine corsa software. Da usare SOLO per il
    // rientro meccanico manuale (manual keyboard) quando il pistone è
    // disallineato e serve uscire dal range nominale. Il firmware di missione
    // non deve usarlo: vedi startMoveSteps/startMoveTo (clampati).
    void startJogStepsUnclamped(long steps);
    void run();
    void stop();
    long distanceToGo();

    // Motion configuration and driver outputs.
    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);
    void enableOutputs();
    void disableOutputs();

    // Current position according to FastAccelStepper.
    long position();

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper* _stepper = nullptr;
    bool _positionKnown = false;
    long _targetPosition = 0;
    uint32_t _maxSpeed = 0;
    int32_t _acceleration = 0;

    long _clampTarget(long targetPosition) const;
    bool _isReady() const;
    bool _startRawMoveTo(long targetPosition);
};

// Singleton
extern MotorController motor;
