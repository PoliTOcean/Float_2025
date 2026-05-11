#include "tof.h"

TofSensor::TofSensor(TwoWire& wire, uint8_t lpnPin, uint8_t resetPin)
    : _wire(wire),
      _lpnPin(lpnPin),
      _resetPin(resetPin),
      _sensor(&_wire, _lpnPin, _resetPin) {}

bool TofSensor::begin() {
    pinMode(_lpnPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);

    digitalWrite(_lpnPin, LOW);
    digitalWrite(_resetPin, LOW);
    delay(100);
    digitalWrite(_lpnPin, HIGH);
    digitalWrite(_resetPin, HIGH);
    delay(100);

    _wire.setClock(1000000);

    if (_sensor.begin() != VL53L7CX_STATUS_OK) {
        return false;
    }

    if (_sensor.init_sensor() != VL53L7CX_STATUS_OK) {
        return false;
    }

    if (_sensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4) != VL53L7CX_STATUS_OK) {
        return false;
    }

    _sensor.vl53l7cx_set_ranging_frequency_hz(30);

    if (_sensor.vl53l7cx_start_ranging() != VL53L7CX_STATUS_OK) {
        return false;
    }

    _initialized = true;
    return true;
}

bool TofSensor::setActiveZones(const uint8_t* zones, uint8_t count) {
    if (zones == nullptr || count == 0 || count > MAX_ACTIVE_ZONES) {
        return false;
    }

    uint8_t uniqueCount = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (zones[i] >= VL53L7CX_RESOLUTION_4X4) {
            return false;
        }

        bool alreadyPresent = false;
        for (uint8_t j = 0; j < uniqueCount; j++) {
            if (_activeZones[j] == zones[i]) {
                alreadyPresent = true;
                break;
            }
        }

        if (!alreadyPresent) {
            _activeZones[uniqueCount++] = zones[i];
        }
    }

    if (uniqueCount == 0) {
        return false;
    }

    _activeZoneCount = uniqueCount;
    return true;
}

bool TofSensor::_readData(VL53L7CX_ResultsData& data) {
    if (!_initialized) {
        return false;
    }

    uint8_t ready = 0;
    if (_sensor.vl53l7cx_check_data_ready(&ready) != VL53L7CX_STATUS_OK || !ready) {
        return false;
    }

    if (_sensor.vl53l7cx_get_ranging_data(&data) != VL53L7CX_STATUS_OK) {
        return false;
    }

    return true;
}

bool TofSensor::_isValidZoneReading(const VL53L7CX_ResultsData& data, uint8_t zone) const {
    if (zone >= VL53L7CX_RESOLUTION_4X4 || data.nb_target_detected[zone] == 0) {
        return false;
    }

    const uint8_t targetIndex = VL53L7CX_NB_TARGET_PER_ZONE * zone;
    const uint8_t status = data.target_status[targetIndex];
    return (status == 5 || status == 9) && data.distance_mm[targetIndex] > 0;
}

uint8_t TofSensor::_collectActiveDistances(const VL53L7CX_ResultsData& data,
                                           float* distances,
                                           uint8_t maxCount) const {
    uint8_t count = 0;

    for (uint8_t i = 0; i < _activeZoneCount && count < maxCount; i++) {
        const uint8_t zone = _activeZones[i];
        if (_isValidZoneReading(data, zone)) {
            const uint8_t targetIndex = VL53L7CX_NB_TARGET_PER_ZONE * zone;
            distances[count++] = static_cast<float>(data.distance_mm[targetIndex]);
        }
    }

    return count;
}

bool TofSensor::readCenterDistanceMm(float& distanceMm) {
    VL53L7CX_ResultsData data;
    if (!_readData(data) || !_isValidZoneReading(data, 5)) {
        return false;
    }

    const uint8_t targetIndex = VL53L7CX_NB_TARGET_PER_ZONE * 5;
    distanceMm = static_cast<float>(data.distance_mm[targetIndex]);
    return true;
}

bool TofSensor::readActiveDistanceMm(float& distanceMm) {
    VL53L7CX_ResultsData data;
    if (!_readData(data)) {
        return false;
    }

    float distances[MAX_ACTIVE_ZONES];
    const uint8_t count = _collectActiveDistances(data, distances, MAX_ACTIVE_ZONES);
    if (count == 0) {
        return false;
    }

    for (uint8_t i = 1; i < count; i++) {
        const float value = distances[i];
        uint8_t j = i;
        while (j > 0 && distances[j - 1] > value) {
            distances[j] = distances[j - 1];
            j--;
        }
        distances[j] = value;
    }

    if ((count % 2) == 1) {
        distanceMm = distances[count / 2];
    } else {
        distanceMm = (distances[(count / 2) - 1] + distances[count / 2]) * 0.5f;
    }

    return true;
}

bool TofSensor::readActiveMinDistanceMm(float& distanceMm) {
    VL53L7CX_ResultsData data;
    if (!_readData(data)) {
        return false;
    }

    float distances[MAX_ACTIVE_ZONES];
    const uint8_t count = _collectActiveDistances(data, distances, MAX_ACTIVE_ZONES);
    if (count == 0) {
        return false;
    }

    distanceMm = distances[0];
    for (uint8_t i = 1; i < count; i++) {
        if (distances[i] < distanceMm) {
            distanceMm = distances[i];
        }
    }

    return true;
}
