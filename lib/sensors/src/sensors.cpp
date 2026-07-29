#include "sensors.h"
#include "config.h"
#include "led.h"
#include "motor.h"
#include <Wire.h>
#include <cstring>
#include <DebugSerial.h>

/*
 *******************************************************************************
 * sensors.cpp
 * Bar02 pressure sensor + INA219 power monitor wrappers. Computes depth from
 * raw pressure using Stevino's principle and exposes top/bottom-of-float
 * references plus the runtime-tunable surface target offset.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */


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
    if (_simEnabled) {
        _simStep();
        return;
    }
    _bar02.read();
}

float SensorManager::depth() {
    return bottomDepth();
}

float SensorManager::sensorDepth() {
    if (_simEnabled) return _simZ;
    return depthFromPressure(_bar02.pressure(MS5837::Pa));
}

float SensorManager::depthFromPressure(float pressurePa) const {
    return (pressurePa - _atmPressurePa) / (WATER_DENSITY_FRESH * GRAVITY);
}

float SensorManager::bottomDepth() {
    return sensorDepth() + SENSOR_TO_BOTTOM_M;
}

float SensorManager::topDepth() {
    return sensorDepth() - SENSOR_TO_TOP_M;
}

void SensorManager::setSurfaceTargetOffset(float meters) {
    if (meters < 0.0f) meters = 0.0f;
    _surfaceTargetOffsetM = meters;
    Debug.printf("Surface target offset set to %.3f m\n", _surfaceTargetOffsetM);
}

float SensorManager::referenceDepthForPhase(const char* phase) {
    // Profondità RIPORTATA (pacchetti/grafico): sempre riferita alla CIMA del
    // float, così parte da ~0 in superficie ed è un riferimento unico e continuo.
    // Il CONTROLLO resta riferito al FONDO (depth() = bottomDepth()): il PID porta
    // comunque il fondo a 2.5 m e la cima a 40 cm. L'offset cima→fondo va
    // comunicato al giudice per l'hold profondo (regolamento Task 4).
    (void)phase;
    return topDepth();
}

float SensorManager::pressure() {
    if (_simEnabled) {
        // Pressione coerente con la quota simulata del sensore (Stevino), così i
        // log/pacchetti mostrano un kPa plausibile e depthFromPressure() tornerebbe _simZ.
        return _atmPressurePa + WATER_DENSITY_FRESH * GRAVITY * _simZ;
    }
    return _bar02.pressure(MS5837::Pa);
}

// ---------------------------------------------------------------------------
// SIMULATORE
// ---------------------------------------------------------------------------
void SensorManager::simEnable(bool on) {
    _simEnabled = on;
    if (on) {
        _simZ = 0.0f;      // parte in superficie (sensore a quota 0)
        _simV = 0.0f;
        _simLastMs = 0;    // forza l'inizializzazione del dt al primo step
    }
    Debug.printf("Float SIM %s\n", on ? "ON (barometro simulato, motore reale)" : "OFF");
}

void SensorManager::simConfigure(float uNeutral, float accelGain, float dragQuad, float poolDepth) {
    _simUNeutral  = constrain(uNeutral, 0.0f, 1.0f);
    _simAccelGain = (accelGain > 0.0f) ? accelGain : _simAccelGain;
    _simDragQuad  = (dragQuad  >= 0.0f) ? dragQuad  : _simDragQuad;
    _simPoolDepth = (poolDepth > 0.0f) ? poolDepth : _simPoolDepth;
    Debug.printf("SIM cfg: uNeutral=%.3f accelGain=%.4f dragQuad=%.3f pool=%.2f m\n",
                 _simUNeutral, _simAccelGain, _simDragQuad, _simPoolDepth);
}

void SensorManager::simReset(float sensorDepthM) {
    _simZ = (sensorDepthM < 0.0f) ? 0.0f : sensorDepthM;
    _simV = 0.0f;
    _simLastMs = 0;
}

void SensorManager::simFormatStatus(char* buffer, size_t bufferSize) const {
    if (buffer == nullptr || bufferSize == 0) return;
    snprintf(buffer, bufferSize,
             "SIM %s | z=%.3f m v=%.3f m/s | uNeutral=%.3f accelGain=%.4f dragQuad=%.3f pool=%.2f m",
             _simEnabled ? "ON" : "OFF",
             _simZ, _simV, _simUNeutral, _simAccelGain, _simDragQuad, _simPoolDepth);
}

void SensorManager::_simStep() {
    const unsigned long now = millis();
    if (_simLastMs == 0) { _simLastMs = now; return; } // primo campione: solo inizializza
    float dt = (now - _simLastMs) / 1000.0f;
    _simLastMs = now;
    if (dt <= 0.0f) return;
    if (dt > SIM_MAX_DT_S) dt = SIM_MAX_DT_S; // evita salti d'integrazione se il loop si ferma

    // u reale dalla posizione del motore: include il ritardo di corsa del motore,
    // così la taratura PID vede la stessa lentezza meccanica del float vero.
    const float u = motorPosToU(motor.position());
    const float aBuoy = _simAccelGain * (u - _simUNeutral); // u>neutral => +a => affonda
    const float aDrag = -_simDragQuad * _simV * fabsf(_simV); // drag quadratico, frena
    _simV += (aBuoy + aDrag) * dt;
    _simZ += _simV * dt;

    // Vincolo superficie: il sensore non emerge sopra il pelo (z>=0).
    if (_simZ < 0.0f) { _simZ = 0.0f; if (_simV < 0.0f) _simV = 0.0f; }
    // Vincolo fondo vasca: il FONDO del float tocca il fondo (z = pool - lunghezza).
    const float zMax = _simPoolDepth - SENSOR_TO_BOTTOM_M;
    if (zMax > 0.0f && _simZ > zMax) { _simZ = zMax; if (_simV > 0.0f) _simV = 0.0f; }
}

float SensorManager::temperature() {
    return _bar02.temperature();
}

uint32_t SensorManager::batteryMilliVolts() {
    if (_inaDeviceIndex < 0) return 0;
    _ina.waitForConversion(_inaDeviceIndex);
    return _ina.getBusMilliVolts(_inaDeviceIndex);
}
