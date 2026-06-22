#pragma once

#include <Arduino.h>
#include <MS5837.h>
#include <INA.h>
#include "config.h"

/*
 *******************************************************************************
 * sensors.h
 * Wraps the Bar02 pressure sensor and INA219 power monitor.
 * Exposes depth calculation and battery voltage reading.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

class SensorManager {
public:
    SensorManager();

    // Call once in setup() — blocks until both sensors are found
    void begin();

    // Read all sensor values (call before depth() / temperature())
    void read();

    // Legacy/reference depth used by the current controller: bottom of float.
    float depth();

    // Raw pressure-sensor depth calculation using Stevino's principle.
    float sensorDepth();

    // Overload: calculate raw pressure-sensor depth from a pressure value (Pa)
    float depthFromPressure(float pressurePa) const;

    // Corrected MATE judge references, based on configurable sensor offsets.
    float bottomDepth();
    float topDepth();
    float referenceDepthForPhase(const char* phase);

    // Surface target: the top of the float should sit this many metres below
    // the water surface when "floating". Runtime-tunable.
    float surfaceTargetOffset() const { return _surfaceTargetOffsetM; }
    void  setSurfaceTargetOffset(float meters);
    // Depth target for the float to reach the desired surface offset, expressed
    // in the same reference as topDepth(). Equivalent to surfaceTargetOffset().
    float surfaceTargetDepth() const { return _surfaceTargetOffsetM; }

    // Last raw pressure reading (Pa)
    float pressure();

    // Last temperature reading (°C)
    float temperature();

    // Battery bus voltage (mV) — reads from INA219 on demand
    uint32_t batteryMilliVolts();

    // -----------------------------------------------------------------------
    // SIMULATORE (HIL da banco): quando attivo, sensorDepth()/pressure() — e
    // quindi depth()/bottomDepth()/topDepth() che li usano — restituiscono una
    // quota SIMULATA da un modello fisico mosso dalla posizione reale del motore.
    // Il motore si muove davvero: PID_HOLD/PID_STEP/measure()/GO girano invariati.
    // -----------------------------------------------------------------------
    void  simEnable(bool on);
    bool  simEnabled() const { return _simEnabled; }
    void  simConfigure(float uNeutral, float accelGain, float dragQuad, float poolDepth);
    void  simReset(float sensorDepthM = 0.0f);
    float simSensorDepth() const { return _simZ; }
    void  simFormatStatus(char* buffer, size_t bufferSize) const;

private:
    MS5837    _bar02;
    INA_Class _ina;
    int8_t    _inaDeviceIndex = -1;  // -1 = not found yet

    float     _atmPressurePa  = 0.0f; // Reference pressure set at startup
    float     _surfaceTargetOffsetM = SURFACE_TARGET_OFFSET_M;

    // --- Stato simulatore ---
    bool          _simEnabled   = false;
    float         _simZ         = 0.0f; // quota sensore simulata [m]
    float         _simV         = 0.0f; // velocità verticale [m/s] (+ = giù/affonda)
    unsigned long _simLastMs    = 0;    // 0 = primo step (inizializza dt)
    float         _simUNeutral  = SIM_U_NEUTRAL;
    float         _simAccelGain = SIM_ACCEL_GAIN;
    float         _simDragQuad  = SIM_DRAG_QUAD;
    float         _simPoolDepth = SIM_POOL_DEPTH;
    void  _simStep();

    void _initPressureSensor();
    void _initPowerMonitor();
};

// Singleton — defined in sensors.cpp
extern SensorManager sensors;
