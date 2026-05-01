#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

/*
 *******************************************************************************
 * tof.h
 * VL53L7CX Time-of-Flight sensor controller.
 *******************************************************************************
 */

class TofSensor {
public:
    TofSensor(TwoWire& wire, uint8_t lpnPin, uint8_t resetPin);

    bool begin();
    bool readCenterDistanceMm(float& distanceMm);
    bool setActiveZones(const uint8_t* zones, uint8_t count);
    bool readActiveDistanceMm(float& distanceMm);
    bool readActiveMinDistanceMm(float& distanceMm);

    bool isInitialized() const { return _initialized; }

private:
    static constexpr uint8_t MAX_ACTIVE_ZONES = VL53L7CX_RESOLUTION_4X4;

    bool _readData(VL53L7CX_ResultsData& data);
    bool _isValidZoneReading(const VL53L7CX_ResultsData& data, uint8_t zone) const;
    uint8_t _collectActiveDistances(const VL53L7CX_ResultsData& data,
                                    float* distances,
                                    uint8_t maxCount) const;

    TwoWire& _wire;
    uint8_t _lpnPin;
    uint8_t _resetPin;
    VL53L7CX _sensor;
    bool _initialized = false;
    uint8_t _activeZones[MAX_ACTIVE_ZONES] = {5, 6, 9, 10};
    uint8_t _activeZoneCount = 4;
};
