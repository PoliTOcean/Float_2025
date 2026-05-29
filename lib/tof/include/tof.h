#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>
#include "config.h"

/*
 *******************************************************************************
 * tof.h
 * VL53L7CX multi-zone Time-of-Flight sensor controller.
 *
 * The VL53L7CX is an 8x8 / 4x4 multi-zone ranging sensor. For homing and
 * endstop detection we configure it in 4x4 mode and aggregate the enabled
 * zones into a single distance by taking the minimum valid range. This favours
 * detecting the nearest obstacle, which is the correct behaviour for the
 * syringe carriage approach. The enabled zone mask is configured in config.h.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

struct TofMeasurement {
    float distanceMm = 0.0f;       // Aggregated raw distance with configured offset removed
    float rawDistanceMm = 0.0f;    // Aggregated raw distance (min of valid zones)
    uint8_t rangeStatus = 0;       // Status of the selected (nearest) zone
    uint8_t validZoneCount = 0;    // Number of zones reporting a valid range
    bool valid = false;
};

class TofSensor {
public:
    TofSensor(TwoWire& wire, uint8_t lpnPin, uint8_t gpio1Pin);

    bool begin();
    bool readMeasurement(TofMeasurement& measurement);
    bool readDistanceMm(float& distanceMm);

    bool isInitialized() const { return _initialized; }

private:
    static constexpr uint8_t  RANGING_FREQUENCY_HZ = 15;   // 4x4 supports up to 60Hz; 15Hz is plenty for homing
    static constexpr uint8_t  RESOLUTION_ZONES    = TOF_MATRIX_ZONE_COUNT; // 4x4

    bool _isValidZoneStatus(uint8_t status) const;

    TwoWire& _wire;
    uint8_t _lpnPin;
    VL53L7CX _sensor;
    bool _initialized = false;
};
