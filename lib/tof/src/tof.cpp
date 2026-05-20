#include "tof.h"

#include "config.h"

TofSensor::TofSensor(TwoWire& wire, uint8_t xshutPin, uint8_t gpio1Pin)
    : _wire(wire),
      _xshutPin(xshutPin),
      _sensor(&_wire, _xshutPin) {
    (void)gpio1Pin;
}

bool TofSensor::begin() {
    if (_sensor.begin() != VL53L4CD_ERROR_NONE) {
        return false;
    }

    _wire.setClock(1000000);

    if (_sensor.InitSensor() != VL53L4CD_ERROR_NONE) {
        return false;
    }

    if (_sensor.VL53L4CD_SetRangeTiming(RANGE_TIMING_BUDGET_MS,
                                        RANGE_INTER_MEASUREMENT_MS) != VL53L4CD_ERROR_NONE) {
        return false;
    }

    if (_sensor.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
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
    if (_sensor.VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE || ready == 0) {
        return false;
    }

    if (_sensor.VL53L4CD_ClearInterrupt() != VL53L4CD_ERROR_NONE) {
        return false;
    }

    VL53L4CD_Result_t result;
    if (_sensor.VL53L4CD_GetResult(&result) != VL53L4CD_ERROR_NONE) {
        return false;
    }

    measurement.rawDistanceMm = static_cast<float>(result.distance_mm);
    measurement.distanceMm = measurement.rawDistanceMm - TOF_DISTANCE_OFFSET_MM;
    if (measurement.distanceMm < 0.0f) {
        measurement.distanceMm = 0.0f;
    }
    measurement.rangeStatus = result.range_status;
    measurement.ambientRateKcps = result.ambient_rate_kcps;
    measurement.ambientPerSpadKcps = result.ambient_per_spad_kcps;
    measurement.signalRateKcps = result.signal_rate_kcps;
    measurement.signalPerSpadKcps = result.signal_per_spad_kcps;
    measurement.numberOfSpad = result.number_of_spad;
    measurement.sigmaMm = result.sigma_mm;
    measurement.valid = _isValidResult(result);
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

bool TofSensor::_isValidResult(const VL53L4CD_Result_t& result) const {
    return result.range_status == 0 && result.distance_mm > 0;
}
