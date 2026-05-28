#include "profile.h"
#include "config.h"
#include "led.h"
#include "motor.h"
#include "motion_control.h"
#include "pid.h"
#include "sensors.h"
#include "comms.h"
#include "flash_storage.h"
#include <EEPROM.h>
#include "float_common.h"
#include "DebugSerial.h"

/*
 *******************************************************************************
 * profile.cpp
 *******************************************************************************
 */


ProfileManager::ProfileManager() {}

namespace {
bool sendPacketFromStorage(const char* message, uint32_t timeoutMs) {
    return comms.sendMessage(message, timeoutMs);
}
}

// ---------------------------------------------------------------------------
void ProfileManager::resetEEPROM() {
    _writePtr = 0;
    _readPtr  = 0;
}

// ---------------------------------------------------------------------------
void ProfileManager::beginProfile(uint8_t profileId) {
    _activeProfileId = profileId;
    if (!_missionClockRunning) {
        _missionStartMs = millis();
        _missionClockRunning = true;
    }
}

// ---------------------------------------------------------------------------
void ProfileManager::logDeploymentPacket() {
    beginProfile(0);
    sensors.read();
    _logProfileReading("deployed");

    char packet[OUTPUT_LEN];
    snprintf(packet, OUTPUT_LEN,
             "{\"company_number\":\"%s\","
             "\"profile_id\":%u,"
             "\"time_s\":%.2f,"
             "\"pressure_kpa\":%.2f,"
             "\"depth_m\":%.2f,"
             "\"phase\":\"%s\","
             "\"sensor_depth_m\":%.2f}",
             COMPANY_NUMBER,
             static_cast<unsigned>(_activeProfileId),
             _missionTimeS(),
             sensors.pressure() / 1000.0f,
             sensors.referenceDepthForPhase("deployed"),
             "deployed",
             sensors.sensorDepth());

    comms.sendMessage(packet, 1000);
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
void ProfileManager::_logProfileReading(const char* phase) {
    flashStorage.appendRecord(COMPANY_NUMBER,
                              _activeProfileId,
                              _missionTimeS(),
                              sensors.pressure() / 1000.0f,
                              sensors.referenceDepthForPhase(phase),
                              phase,
                              sensors.sensorDepth());
}

// ---------------------------------------------------------------------------
float ProfileManager::_missionTimeS() const {
    if (!_missionClockRunning) return 0.0f;
    return static_cast<float>(millis() - _missionStartMs) / 1000.0f;
}

// ---------------------------------------------------------------------------
void ProfileManager::measure(float targetDepth, float holdTimeSec, float timeoutSec) {
    Debug.printf("Profile phase: target=%.2f hold=%.0fs timeout=%.0fs\n",
                 targetDepth, holdTimeSec, timeoutSec);

    const bool isSurfaceTarget = (targetDepth == TARGET_SURFACE);
    const bool isBottomTarget  = (targetDepth == TARGET_BOTTOM);
    const bool isPIDPhase      = !isSurfaceTarget && !isBottomTarget;
    const bool isDeepTarget    = fabsf(targetDepth - TARGET_DEPTH) < 0.001f;
    const bool isShallowTarget = fabsf(targetDepth - TARGET_SHALLOW_BOTTOM_DEPTH) < 0.001f;

    // --- LED and initial motor positioning ---
    if (isPIDPhase) {
        ledController.setState(LEDState::PID_CONTROL);
        pidController.reset();
        if (isDeepTarget) {
            // Pre-position syringe to kick-start the deep descent only.
            motionController.moveToWithTimeout(500, 0);
        }
    } else {
        ledController.setState(LEDState::PROFILE);
    }

    unsigned long phaseStart     = millis();
    unsigned long lastMeasMs     = 0;
    unsigned long lastWriteMs    = 0;
    unsigned long lastProfileWriteMs = 0;
    bool          motorCommanded = false; // For simple (non-PID) phases
    float         lastDepth      = 0.0f;
    int           stableCount    = 0;

    // -----------------------------------------------------------------------
    while (true) {
        ledController.update();
        yield();

        if (motionController.remoteStopRequested()) {
            Debug.println("Profile phase: remote stop");
            break;
        }

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
        const char* phase = "descending";

        if (isPIDPhase) {
            if (isShallowTarget) {
                phase = (fabsf(currentDepth - targetDepth) < DEPTH_MAX_ERROR)
                        ? "hold_40cm"
                        : "ascending";
            } else {
                phase = (fabsf(currentDepth - targetDepth) < DEPTH_MAX_ERROR)
                        ? "hold_2_5m"
                        : "descending";
            }
        } else if (isSurfaceTarget) {
            phase = (fabsf(currentDepth - TARGET_SURFACE) < DEPTH_EPSILON)
                    ? "hold_40cm"
                    : "ascending";
        } else if (isBottomTarget) {
            phase = "descending";
        }

        if (millis() - lastProfileWriteMs >= PROFILE_LOG_PERIOD_MS) {
            lastProfileWriteMs = millis();
            _logProfileReading(phase);
        }

        // ---- Simple phases: drive motor to endpoint once, then wait ----
        if (!isPIDPhase) {
            if (!motorCommanded) {
                if (isBottomTarget) {
                    motionController.moveToMax();
                } else {
                    motionController.moveToWithTimeout(MOTOR_ENDSTOP_MARGIN, 0); // Surface
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
        long pidSteps = static_cast<long>(pidOutput);
        if (pidSteps != 0 && fabsf(targetDepth - currentDepth) > DEPTH_EPSILON) {
            if (labs(pidSteps) < PID_MIN_MOVE_STEPS) {
                pidSteps = (pidSteps > 0) ? PID_MIN_MOVE_STEPS : -PID_MIN_MOVE_STEPS;
            }
            motionController.moveToWithTimeout(motor.position() + pidSteps, 0, true);
        }

        // --- EEPROM write tick (also checks hold condition for PID phase) ---
        if (millis() - lastWriteMs >= PERIOD_EEPROM_WRITE) {
            lastWriteMs = millis();

            // Mark PID-phase records with temperature sentinel
            _logReading(sensors.pressure(), 100.0f);

            if (fabsf(currentDepth - targetDepth) < DEPTH_MAX_ERROR) {
                stableCount++;
                // Seven 5-second packets span the required 30-second hold.
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

    motor.stop();
    motor.disableOutputs();
    Debug.println("Profile phase finished");
}

// ---------------------------------------------------------------------------
void ProfileManager::sendStoredData() {
    Debug.println("Transmitting stored profile data from flash...");

    if (flashStorage.transmitDataPackets(sendPacketFromStorage, 100)) {
        comms.sendMessage("STOP_DATA", 100);
        Debug.println("Flash data transmission complete");
        return;
    }

    comms.sendMessage("STOP_DATA", 100);
    Debug.println("Flash data unavailable; no stored packets sent");
}

// ---------------------------------------------------------------------------
void ProfileManager::clearEEPROM() {
    _writePtr = 0;
    _readPtr  = 0;
    for (int i = 0; i < 100; i++) EEPROM.write(i, 0);
    EEPROM.commit();
    flashStorage.clearLog();
    _missionClockRunning = false;
    _missionStartMs = 0;
    _activeProfileId = 0;
    Debug.println("EEPROM cleared; flash log reset");
}
