#pragma once

#include <Arduino.h>
#include <MS5837.h>
#include <INA.h>

/*
 *******************************************************************************
 * sensors.h
 * Wraps the Bar02 pressure sensor and INA219 power monitor.
 * Exposes depth calculation and battery voltage reading.
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

    // Last raw pressure reading (Pa)
    float pressure();

    // Last temperature reading (°C)
    float temperature();

    // Battery bus voltage (mV) — reads from INA219 on demand
    uint32_t batteryMilliVolts();

private:
    MS5837    _bar02;
    INA_Class _ina;
    int8_t    _inaDeviceIndex = -1;  // -1 = not found yet

    float     _atmPressurePa  = 0.0f; // Reference pressure set at startup

    void _initPressureSensor();
    void _initPowerMonitor();
};

// Singleton — defined in sensors.cpp
extern SensorManager sensors;
