#include "pid.h"
#include "../config.h"
#include <DebugSerial.h>

/*
 *******************************************************************************
 * pid.cpp
 *******************************************************************************
 */

PIDController pidController(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd) {}

void PIDController::reset() {
    _integral   = 0.0f;
    _lastDepth  = 0.0f;
    _lastError  = 0.0f;
    _lastTimeMs = millis();
}

float PIDController::compute(float targetDepth, float currentDepth) {
    unsigned long now = millis();
    float dt = (now - _lastTimeMs) / 1000.0f;
    if (dt <= 0.0f) dt = PERIOD_MEASUREMENT / 1000.0f;

    const float error = targetDepth - currentDepth;

    // --- Proportional ---
    const float proportional = Kp * error;

    // --- Integral with anti-windup ---
    _integral += error * dt;
    _integral = constrain(_integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    const float integral = Ki * _integral;

    // --- Derivative on measurement (avoids setpoint-change kick) ---
    const float depthRate  = (_lastDepth - currentDepth) / dt;
    const float derivative = Kd * depthRate;

    float output = proportional + integral + derivative;
    output = constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

    // Update state for next iteration
    _lastError  = error;
    _lastDepth  = currentDepth;
    _lastTimeMs = now;

    Debug.printf("PID: target=%.2f cur=%.2f err=%.2f P=%.2f I=%.2f D=%.2f out=%.2f\n",
                 targetDepth, currentDepth, error,
                 proportional, integral, derivative, output);

    return output;
}