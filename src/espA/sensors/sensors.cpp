#include "sensors.h"
#include "../config.h"
#include "../led/led.h"
#include <Wire.h>
#include <DebugSerial.h>

/*
 *******************************************************************************
 * sensors.cpp
 *******************************************************************************
 */

SensorManager sensors;

SensorManager::SensorManager() {}

// ---------------------------------------------------------------------------
void SensorManager::begin() {
    Wire.begin();
    _initPowerMonitor();
    _initPressureSensor();
}

// ---------------------------------------------------------------------------
void SensorManager::_initPowerMonitor() {
    // Retry up to 3 times with a 2 s gap — INA219 can be slow to appear
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        uint8_t found = _ina.begin(5, 100000); // 5 A max, 0.1 Ω shunt
        for (uint8_t i = 0; i < found; i++) {
            if (strcmp(_ina.getDeviceName(i), "INA219") == 0) {
                _inaDeviceIndex = static_cast<int8_t>(i);
                _ina.setMode(INA_MODE_CONTINUOUS_BUS, i);
                Debug.printf("INA219 found at index %d\n", i);
                return;
            }
        }
        Debug.println("INA219 not found — retrying in 2 s...");
        delay(2000);
    }

    // Fatal: cannot operate without power monitoring
    Debug.println("CRITICAL: INA219 not found after 3 attempts");
    ledController.setState(LEDState::ERROR);
    while (true) {
        ledController.update();
        yield();
    }
}

// ---------------------------------------------------------------------------
void SensorManager::_initPressureSensor() {
    _bar02.setModel(MS5837::MS5837_02BA); // Bar02

    unsigned long lastWarning = 0;
    while (!_bar02.init()) {
        if (millis() - lastWarning > 5000) {
            lastWarning = millis();
            Debug.println("Bar02 init failed — check SDA/SCL (White=SDA, Green=SCL)");
        }
        ledController.setState(LEDState::ERROR);
        ledController.update();
        yield();
    }

    _bar02.setFluidDensity(WATER_DENSITY_FRESH);
    _bar02.read();
    _atmPressurePa = _bar02.pressure(MS5837::Pa);
    Debug.printf("Bar02 ready — atm pressure: %.2f Pa\n", _atmPressurePa);
}

// ---------------------------------------------------------------------------
void SensorManager::read() {
    _bar02.read();
}

float SensorManager::depth() {
    return depthFromPressure(_bar02.pressure(MS5837::Pa));
}

float SensorManager::depthFromPressure(float pressurePa) const {
    return (pressurePa - _atmPressurePa) / (WATER_DENSITY_FRESH * GRAVITY)
           + FLOAT_LENGTH;
}

float SensorManager::pressure() {
    return _bar02.pressure(MS5837::Pa);
}

float SensorManager::temperature() {
    return _bar02.temperature();
}

uint32_t SensorManager::batteryMilliVolts() {
    if (_inaDeviceIndex < 0) return 0;
    _ina.waitForConversion(_inaDeviceIndex);
    return _ina.getBusMilliVolts(_inaDeviceIndex);
}