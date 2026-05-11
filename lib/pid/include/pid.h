#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * pid.h
 * Discrete PID controller for depth regulation.
 * Uses depth-rate derivative (dDepth/dt) instead of d(error)/dt to avoid
 * derivative kick on setpoint changes.
 *******************************************************************************
 */

class PIDController {
public:
    // Gains are mutable at runtime (command 8 / UPDATE_PID)
    float Kp;
    float Ki;
    float Kd;

    explicit PIDController(float kp, float ki, float kd);

    // Reset integrator and history — call before each new PID session
    void reset();

    // Compute one PID step.
    // Returns a step count (positive = sink deeper, negative = rise).
    float compute(float targetDepth, float currentDepth);

private:
    float         _integral   = 0.0f;
    float         _lastDepth  = 0.0f;
    float         _lastError  = 0.0f;
    unsigned long _lastTimeMs = 0;
};

// Singleton — defined in pid.cpp
extern PIDController pidController;