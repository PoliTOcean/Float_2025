#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * pid.h
 * PID di profondità con output normalizzato in [0, 1] (frazione di corsa
 * siringa). Derivata su misura filtrata con LPF IIR, anti-windup conditional.
 * Guadagni espressi in "frazione di corsa per metro di errore" — portabili tra
 * geometrie diverse della siringa/motore.
 *******************************************************************************
 */

class PIDController {
public:
    // Mutabili a runtime via CMD_UPDATE_PID (8)
    float Kp;
    float Ki;
    float Kd;

    // Mutabili a runtime via CMD_UPDATE_PID_EXT (14)
    float    alphaD;     // LPF coefficient sulla derivata, in (0, 1]
    uint16_t periodMs;   // Periodo del tick PID nel loop (ms)

    // Offset costante di kick-start; somma direttamente all'output normalizzato
    float uNeutral;

    PIDController(float kp, float ki, float kd);

    // Da chiamare prima di ogni nuova sessione PID
    void reset();

    // Calcola il prossimo target normalizzato in [0, 1]
    //   targetDepth, currentDepth: in metri
    // Ritorna: posizione siringa desiderata come frazione di corsa
    float computeNormalized(float targetDepth, float currentDepth);

private:
    float         _integral     = 0.0f;
    float         _dFilt        = 0.0f;
    float         _lastDepth    = 0.0f;
    unsigned long _lastTimeMs   = 0;
    bool          _hasLastDepth = false;
};

// Singleton — definito in pid.cpp
extern PIDController pidController;
