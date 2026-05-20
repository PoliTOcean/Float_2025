#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

/*
 *******************************************************************************
 * tof.h
 * VL53L4CD Time-of-Flight sensor controller.
 *******************************************************************************
 */

struct TofMeasurement {
    float distanceMm = 0.0f;
    float rawDistanceMm = 0.0f;
    uint8_t rangeStatus = 0;
    uint16_t ambientRateKcps = 0;
    uint16_t ambientPerSpadKcps = 0;
    uint16_t signalRateKcps = 0;
    uint16_t signalPerSpadKcps = 0;
    uint16_t numberOfSpad = 0;
    uint16_t sigmaMm = 0;
    bool valid = false;
};

class TofSensor {
public:
    TofSensor(TwoWire& wire, uint8_t xshutPin, uint8_t gpio1Pin);

    bool begin();
    bool readMeasurement(TofMeasurement& measurement);
    bool readDistanceMm(float& distanceMm);

    bool isInitialized() const { return _initialized; }

private:
    static constexpr uint32_t RANGE_TIMING_BUDGET_MS = 30;
    static constexpr uint32_t RANGE_INTER_MEASUREMENT_MS = 0;

    bool _isValidResult(const VL53L4CD_Result_t& result) const;

    TwoWire& _wire;
    uint8_t _xshutPin;
    VL53L4CD _sensor;
    bool _initialized = false;
};
