#include "tof.h"

#include "config.h"

TofSensor::TofSensor(TwoWire& wire, uint8_t lpnPin, uint8_t gpio1Pin)
    : _wire(wire),
      _lpnPin(lpnPin),
      _sensor(&_wire, _lpnPin) {
    (void)gpio1Pin;
}

bool TofSensor::begin() {
    _sensor.begin();

    if (_sensor.init_sensor() != 0) {
        return false;
    }

    _wire.setClock(1000000);

    if (_sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4) != VL53L7CX_STATUS_OK) {
        return false;
    }

    if (_sensor.vl53l7cx_set_ranging_frequency_hz(RANGING_FREQUENCY_HZ) != VL53L7CX_STATUS_OK) {
        return false;
    }

    if (_sensor.vl53l7cx_start_ranging() != VL53L7CX_STATUS_OK) {
        return false;
    }

    _initialized = true;
    return true;
}

bool TofSensor::readMeasurement(TofMeasurement& measurement) {
    if (!_initialized) {
        return false;
    }

    uint8_t ready = 0;
    if (_sensor.vl53l7cx_check_data_ready(&ready) != VL53L7CX_STATUS_OK || ready == 0) {
        return false;
    }

    VL53L7CX_ResultsData results;
    if (_sensor.vl53l7cx_get_ranging_data(&results) != VL53L7CX_STATUS_OK) {
        return false;
    }

    int32_t bestRawMm = INT32_MAX;
    uint8_t bestStatus = 255;
    uint8_t validCount = 0;

    for (uint8_t i = 0; i < RESOLUTION_ZONES; ++i) {
        if ((TOF_ZONE_ENABLE_MASK & (static_cast<uint16_t>(1U) << i)) == 0) {
            continue;
        }

        const uint8_t status = results.target_status[i];
        const int16_t dist = results.distance_mm[i];

        if (!_isValidZoneStatus(status) || dist <= 0) {
            continue;
        }

        ++validCount;
        if (dist < bestRawMm) {
            bestRawMm = dist;
            bestStatus = status;
        }
    }

    measurement.validZoneCount = validCount;

    if (validCount == 0) {
        measurement.rawDistanceMm = 0.0f;
        measurement.distanceMm = 0.0f;
        measurement.rangeStatus = bestStatus;
        measurement.valid = false;
        return true;
    }

    measurement.rawDistanceMm = static_cast<float>(bestRawMm);
    measurement.distanceMm = measurement.rawDistanceMm - TOF_DISTANCE_RAW_OFFSET_MM;
    if (measurement.distanceMm < 0.0f) {
        measurement.distanceMm = 0.0f;
    }
    measurement.rangeStatus = bestStatus;
    measurement.valid = true;
    return true;
}

bool TofSensor::readDistanceMm(float& distanceMm) {
    TofMeasurement measurement;
    if (!readMeasurement(measurement) || !measurement.valid) {
        return false;
    }

    distanceMm = measurement.distanceMm;
    return true;
}

bool TofSensor::_isValidZoneStatus(uint8_t status) const {
    // Per ST guidance: status 5 is the nominal "range valid" code; 9 is
    // "range valid with large pulse" — both are acceptable for our use.
    return status == 5 || status == 9;
}
