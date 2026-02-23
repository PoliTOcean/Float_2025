#include "profile.h"
#include "../config.h"
#include "../led/led.h"
#include "../motor/motor.h"
#include "../pid/pid.h"
#include "../sensors/sensors.h"
#include "../comms/comms.h"
#include <EEPROM.h>
#include <float_common.h>
#include <DebugSerial.h>

/*
 *******************************************************************************
 * profile.cpp
 *******************************************************************************
 */

ProfileManager profileManager;

ProfileManager::ProfileManager() {}

// ---------------------------------------------------------------------------
void ProfileManager::resetEEPROM() {
    _writePtr = 0;
    _readPtr  = 0;
}

// ---------------------------------------------------------------------------
void ProfileManager::_logReading(float pressure, float temperature) {
    if (_writePtr + sizeof(sensor_data) > EEPROM_SIZE) {
        _writePtr = 0; // Wrap around
    }
    sensor_data rec;
    rec.pressure    = pressure;
    rec.temperature = temperature;
    EEPROM.put(_writePtr, rec);
    _writePtr += sizeof(sensor_data);
    EEPROM.commit();
}

// ---------------------------------------------------------------------------
void ProfileManager::measure(float targetDepth, float holdTimeSec, float timeoutSec) {
    Debug.printf("Profile phase: target=%.2f hold=%.0fs timeout=%.0fs\n",
                 targetDepth, holdTimeSec, timeoutSec);

    const bool isSurfaceTarget = (targetDepth == TARGET_SURFACE);
    const bool isBottomTarget  = (targetDepth == TARGET_BOTTOM);
    const bool isPIDPhase      = !isSurfaceTarget && !isBottomTarget;

    // --- LED and initial motor positioning ---
    if (isPIDPhase) {
        ledController.setState(LEDState::PID_CONTROL);
        pidController.reset();
        // Pre-position syringe to kick-start descent
        motor.moveTo(500);
    } else {
        ledController.setState(LEDState::PROFILE);
    }

    unsigned long phaseStart     = millis();
    unsigned long lastMeasMs     = 0;
    unsigned long lastWriteMs    = 0;
    bool          motorCommanded = false; // For simple (non-PID) phases
    float         lastDepth      = 0.0f;
    int           stableCount    = 0;

    // -----------------------------------------------------------------------
    while (true) {
        ledController.update();
        yield();

        // Abort if phase timeout exceeded
        if (millis() - phaseStart > static_cast<unsigned long>(timeoutSec * 1000UL)) {
            Debug.println("Profile phase: timeout");
            break;
        }

        // --- Measurement tick ---
        if (millis() - lastMeasMs < PERIOD_MEASUREMENT) continue;
        lastMeasMs = millis();

        sensors.read();
        const float currentDepth = sensors.depth();

        // ---- Simple phases: drive motor to endpoint once, then wait ----
        if (!isPIDPhase) {
            if (!motorCommanded) {
                if (isBottomTarget) {
                    motor.moveTo(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
                } else {
                    motor.moveTo(MOTOR_ENDSTOP_MARGIN); // Surface
                }
                motorCommanded = true;
            }

            // Bottom hold: wait until depth stops changing
            if (isBottomTarget) {
                if (currentDepth > FLOAT_LENGTH + 0.1f &&
                    fabsf(currentDepth - lastDepth) < DEPTH_EPSILON) {
                    stableCount++;
                } else {
                    stableCount = 0;
                }
                if (stableCount >= static_cast<int>(holdTimeSec * 1000.0f / PERIOD_MEASUREMENT)) {
                    Debug.println("Profile: stationary at bottom — phase complete");
                    break;
                }
            }

            // Surface hold: wait until depth ≈ FLOAT_LENGTH
            if (isSurfaceTarget) {
                if (fabsf(currentDepth - TARGET_SURFACE) < DEPTH_EPSILON) {
                    stableCount++;
                } else {
                    stableCount = 0;
                }
                if (stableCount >= static_cast<int>(holdTimeSec * 1000.0f / PERIOD_MEASUREMENT)) {
                    Debug.println("Profile: stationary at surface — phase complete");
                    break;
                }
            }

            lastDepth = currentDepth;
            continue;
        }

        // ---- PID phase ----
        const float pidOutput = pidController.compute(targetDepth, currentDepth);
        if (fabsf(pidOutput) > 1.0f) {
            motor.moveSteps(static_cast<long>(pidOutput));
        }

        // --- EEPROM write tick (also checks hold condition for PID phase) ---
        if (millis() - lastWriteMs >= PERIOD_EEPROM_WRITE) {
            lastWriteMs = millis();

            // Mark PID-phase records with temperature sentinel
            _logReading(sensors.pressure(), 100.0f);

            if (fabsf(currentDepth - targetDepth) < DEPTH_MAX_ERROR) {
                stableCount++;
                // holdTimeSec × (1000 / WRITE_PERIOD) = required ticks at target
                const int requiredTicks =
                    static_cast<int>(holdTimeSec * 1000.0f / PERIOD_EEPROM_WRITE) + 1;
                if (stableCount >= requiredTicks) {
                    Debug.println("Profile: PID hold complete — target depth sustained");
                    break;
                }
            } else {
                stableCount = 0;
            }
        }

        lastDepth = currentDepth;
    }
    // -----------------------------------------------------------------------

    motor.stepper.stop();
    motor.stepper.disableOutputs();
    Debug.println("Profile phase finished");
}

// ---------------------------------------------------------------------------
void ProfileManager::sendStoredData() {
    Debug.println("Transmitting stored profile data...");

    char     line[OUTPUT_LEN];
    uint16_t packetIndex    = 0;
    uint16_t savedReadPtr   = _readPtr; // Remember position so data can be resent

    while (_readPtr + sizeof(sensor_data) <= _writePtr) {
        sensor_data rec;
        EEPROM.get(_readPtr, rec);
        _readPtr += sizeof(sensor_data);

        snprintf(line, OUTPUT_LEN,
                 "{\"company_number\":\"EX10\","
                 "\"pressure\":\"%.2f\","
                 "\"depth\":\"%.2f\","
                 "\"temperature\":\"%.2f\","
                 "\"mseconds\":\"%d\"}",
                 rec.pressure,
                 sensors.depthFromPressure(rec.pressure),
                 rec.temperature,
                 packetIndex * PERIOD_EEPROM_WRITE);

        delay(50);
        comms.sendMessage(line, 100);
        packetIndex++;
    }

    _readPtr = savedReadPtr; // Allow the data to be re-sent if needed

    comms.sendMessage("STOP_DATA", 100);
    Debug.printf("Data transmission complete — %d packets sent\n", packetIndex);
}

// ---------------------------------------------------------------------------
void ProfileManager::clearEEPROM() {
    _writePtr = 0;
    _readPtr  = 0;
    for (int i = 0; i < 100; i++) EEPROM.write(i, 0);
    EEPROM.commit();
    Debug.println("EEPROM cleared");
}