#include "pid.h"
#include "config.h"
#include "DebugSerial.h"

/*
 *******************************************************************************
 * pid.cpp
 * Depth PID implementation with output normalized to [0, 1] (syringe travel
 * fraction). Filtered derivative on measurement (IIR LPF) and conditional
 * anti-windup. Gains are expressed per metre of error, portable across
 * syringe geometries.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */


PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd),
      alphaD(PID_ALPHA_D_DEFAULT),
      periodMs(PID_PERIOD_DEFAULT_MS),
      integralLimit(PID_INTEGRAL_LIMIT),
      minRetargetFrac(PID_MIN_RETARGET_FRAC),
      uNeutral(PID_U_NEUTRAL) {}

void PIDController::reset() {
    _integral     = 0.0f;
    _dFilt        = 0.0f;
    _lastDepth    = 0.0f;
    _lastTimeMs   = millis();
    _hasLastDepth = false;
}

float PIDController::computeNormalized(float targetDepth, float currentDepth) {
    const unsigned long now = millis();
    float dt = (now - _lastTimeMs) / 1000.0f;
    // Clamp robusto a primo tick / glitch (millis() rollover o reset)
    if (dt <= 0.0f || dt > 1.0f) dt = periodMs / 1000.0f;

    const float error = targetDepth - currentDepth;

    // --- Proportional ---
    const float P = Kp * error;

    // --- Derivative on measurement, LPF IIR ---
    const float dRaw = _hasLastDepth ? (currentDepth - _lastDepth) / dt : 0.0f;
    _dFilt = alphaD * dRaw + (1.0f - alphaD) * _dFilt;
    // d(error)/dt = -dDepth/dt (target costante a regime)
    const float D = -Kd * _dFilt;

    // --- Integral (preview con vecchio accumulo, decisione di aggiornamento sotto) ---
    const float I = Ki * _integral;

    const float uRaw = uNeutral + P + I + D;
    const float uSat = constrain(uRaw, 0.0f, 1.0f);

    // Conditional integration: aggiorna solo se non saturati nel verso dell'errore
    const bool satHigh = (uRaw > 1.0f);
    const bool satLow  = (uRaw < 0.0f);
    if (!((satHigh && error > 0.0f) || (satLow && error < 0.0f))) {
        _integral += error * dt;
        _integral = constrain(_integral, -integralLimit, integralLimit);
    }

    _lastDepth    = currentDepth;
    _lastTimeMs   = now;
    _hasLastDepth = true;

    Debug.printf("PID: tgt=%.2f cur=%.2f e=%.3f P=%.3f I=%.3f D=%.3f u=%.3f%s\n",
                 targetDepth, currentDepth, error,
                 P, I, D, uSat,
                 (satHigh || satLow) ? " SAT" : "");

    return uSat;
}
