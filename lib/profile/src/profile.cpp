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
#include <Preferences.h>
#include "float_common.h"
#include "DebugSerial.h"

/*
 *******************************************************************************
 * profile.cpp
 * Depth profile state machine: simple descent/ascent endpoints and PID-driven
 * hold phases. Owns the flash CSV mission log and replay of stored packets to
 * the control station.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */


ProfileManager::ProfileManager() {}

namespace {
constexpr uint32_t PROFILE_CONFIG_MAGIC = 0x50464C54UL; // "PFLT"
constexpr uint16_t PROFILE_CONFIG_VERSION = 1;
constexpr char PROFILE_CONFIG_NAMESPACE[] = "float_profile";
constexpr char PROFILE_CONFIG_KEY[] = "cfg";

struct StoredProfileConfig {
    uint32_t magic;
    uint16_t version;
    RuntimeProfileConfig config;
};

bool sendPacketFromStorage(const char* message, uint32_t timeoutMs) {
    return comms.sendMessage(message, timeoutMs);
}
}

// ---------------------------------------------------------------------------
void ProfileManager::beginConfig() {
    RuntimeProfileConfig loaded;
    bool hasValidStoredConfig = false;

    Preferences preferences;
    if (preferences.begin(PROFILE_CONFIG_NAMESPACE, true)) {
        if (preferences.getBytesLength(PROFILE_CONFIG_KEY) == sizeof(StoredProfileConfig)) {
            StoredProfileConfig stored;
            preferences.getBytes(PROFILE_CONFIG_KEY, &stored, sizeof(stored));
            if (stored.magic == PROFILE_CONFIG_MAGIC &&
                stored.version == PROFILE_CONFIG_VERSION &&
                validateConfig(stored.config)) {
                loaded = stored.config;
                hasValidStoredConfig = true;
            }
        }
        preferences.end();
    }

    _config = loaded;
    _applyConfigToSubsystems();

    if (!hasValidStoredConfig) {
        _saveConfig();
        Debug.println("Profile config: using config.h defaults");
    } else {
        Debug.println("Profile config: loaded from NVS");
    }
}

// ---------------------------------------------------------------------------
float ProfileManager::shallowBottomTargetM() const {
    return _config.shallowTopTargetM + SENSOR_TO_BOTTOM_M + SENSOR_TO_TOP_M;
}

// ---------------------------------------------------------------------------
bool ProfileManager::setConfig(const RuntimeProfileConfig& config) {
    if (!validateConfig(config)) {
        Debug.println("Profile config rejected: invalid values");
        return false;
    }

    _config = config;
    _applyConfigToSubsystems();
    _saveConfig();
    Debug.println("Profile config updated");
    return true;
}

// ---------------------------------------------------------------------------
bool ProfileManager::validateConfig(const RuntimeProfileConfig& config) const {
    const float shallowBottomM =
        config.shallowTopTargetM + SENSOR_TO_BOTTOM_M + SENSOR_TO_TOP_M;

    return config.profileCount >= 1 && config.profileCount <= 10 &&
           isfinite(config.deepTargetM) &&
           isfinite(config.shallowTopTargetM) &&
           isfinite(config.depthToleranceM) &&
           isfinite(config.holdTimeS) &&
           isfinite(config.pidTimeoutS) &&
           isfinite(config.ascentTimeoutS) &&
           isfinite(config.surfaceOffsetM) &&
           config.deepTargetM >= 0.0f && config.deepTargetM <= 5.0f &&
           config.shallowTopTargetM >= 0.0f && config.shallowTopTargetM <= 5.0f &&
           config.depthToleranceM >= 0.005f && config.depthToleranceM <= 1.0f &&
           config.holdTimeS >= 1.0f && config.holdTimeS <= 600.0f &&
           config.pidTimeoutS >= 5.0f && config.pidTimeoutS <= 900.0f &&
           config.ascentTimeoutS >= 5.0f && config.ascentTimeoutS <= 900.0f &&
           config.surfaceOffsetM >= 0.0f && config.surfaceOffsetM <= 5.0f &&
           shallowBottomM < config.deepTargetM;
}

// ---------------------------------------------------------------------------
void ProfileManager::formatConfigJson(char* buffer, size_t bufferSize) const {
    if (buffer == nullptr || bufferSize == 0) return;

    snprintf(buffer, bufferSize,
             "{\"profile_count\":%u,"
             "\"deep_target_m\":%.3f,"
             "\"shallow_top_m\":%.3f,"
             "\"shallow_bottom_m\":%.3f,"
             "\"depth_tolerance_m\":%.3f,"
             "\"hold_s\":%.1f,"
             "\"pid_timeout_s\":%.1f,"
             "\"ascent_timeout_s\":%.1f,"
             "\"surface_offset_m\":%.3f}",
             static_cast<unsigned>(_config.profileCount),
             _config.deepTargetM,
             _config.shallowTopTargetM,
             shallowBottomTargetM(),
             _config.depthToleranceM,
             _config.holdTimeS,
             _config.pidTimeoutS,
             _config.ascentTimeoutS,
             _config.surfaceOffsetM);
}

// ---------------------------------------------------------------------------
void ProfileManager::_saveConfig() {
    StoredProfileConfig stored = {
        PROFILE_CONFIG_MAGIC,
        PROFILE_CONFIG_VERSION,
        _config,
    };

    Preferences preferences;
    if (!preferences.begin(PROFILE_CONFIG_NAMESPACE, false)) {
        Debug.println("Profile config: NVS open failed");
        return;
    }

    preferences.putBytes(PROFILE_CONFIG_KEY, &stored, sizeof(stored));
    preferences.end();
}

// ---------------------------------------------------------------------------
void ProfileManager::_applyConfigToSubsystems() {
    sensors.setSurfaceTargetOffset(_config.surfaceOffsetM);
}

// ---------------------------------------------------------------------------
void ProfileManager::resetEEPROM() {
    _writePtr = 0;
    _readPtr  = 0;
    // Azzera il flash log all'inizio di una nuova missione: non più al boot,
    // così il log di un test fallito sopravvive al power-cycle ed è leggibile
    // con DUMP_LOG finché non si avvia un nuovo profilo.
    flashStorage.clearLog();
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
             "\"sensor_depth_m\":%.2f,"
             "\"syringe_u\":%.4f}",
             COMPANY_NUMBER,
             static_cast<unsigned>(_activeProfileId),
             _missionTimeS(),
             sensors.pressure() / 1000.0f,
             sensors.referenceDepthForPhase("deployed"),
             "deployed",
             sensors.sensorDepth(),
             motorPosToU(motor.position()));

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
                              sensors.sensorDepth(),
                              motorPosToU(motor.position()));
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
    const bool isDeepTarget    = fabsf(targetDepth - _config.deepTargetM) < 0.001f;
    const bool isShallowTarget = fabsf(targetDepth - shallowBottomTargetM()) < 0.001f;

    // --- LED and initial motor positioning ---
    if (isPIDPhase) {
        ledController.setState(LEDState::PID_CONTROL);
        pidController.reset();
        if (isDeepTarget) {
            // Pre-position syringe to kick-start the deep descent only.
            // PID_DESCENT_KICK_U → spinta iniziale per "affondare", contenuta
            // per non far superare il target prima che il PID prenda il controllo.
            motionController.moveToWithTimeout(uToMotorPos(PID_DESCENT_KICK_U), 0);
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
    // Per la fase PID: ultimo target assoluto comandato al motore (in step).
    // Inizializzato al pre-position (PID_DESCENT_KICK_U) per isDeepTarget,
    // altrimenti alla posizione corrente — letta dopo il primo sensors.read().
    long          lastCommandedTarget = isDeepTarget ? uToMotorPos(PID_DESCENT_KICK_U) : motor.position();

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
        // Fase PID gira al ritmo configurabile pidController.periodMs (default 50 ms,
        // modificabile via PID_CONFIG_SET). Fasi simple restano a PERIOD_MEASUREMENT.
        const uint16_t measPeriodMs =
            isPIDPhase ? pidController.periodMs : PERIOD_MEASUREMENT;
        if (millis() - lastMeasMs < measPeriodMs) continue;
        lastMeasMs = millis();

        sensors.read();
        const float currentDepth = sensors.depth();
        const char* phase = "descending";

        if (isPIDPhase) {
            if (isShallowTarget) {
                phase = (fabsf(currentDepth - targetDepth) < _config.depthToleranceM)
                        ? "hold_40cm"
                        : "ascending";
            } else {
                phase = (fabsf(currentDepth - targetDepth) < _config.depthToleranceM)
                        ? "hold_2_5m"
                        : "descending";
            }
        } else if (isSurfaceTarget) {
            // Riferimento: top del float a `surfaceTargetOffset` sotto il pelo.
            // bottomDepth atteso = FLOAT_LENGTH + offset.
            const float surfaceRefDepth = FLOAT_LENGTH + sensors.surfaceTargetOffset();
            phase = (fabsf(currentDepth - surfaceRefDepth) < DEPTH_EPSILON)
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
                    // Bottom: u=1 logico → siringa piena → affonda
                    motionController.moveToWithTimeout(uToMotorPos(1.0f), 0);
                } else {
                    // Surface: u=0 logico → siringa vuota → galleggia
                    motionController.moveToWithTimeout(uToMotorPos(0.0f), 0);
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

            // Surface hold: wait until depth ≈ FLOAT_LENGTH + surfaceTargetOffset
            if (isSurfaceTarget) {
                const float surfaceRefDepth = FLOAT_LENGTH + sensors.surfaceTargetOffset();
                if (fabsf(currentDepth - surfaceRefDepth) < DEPTH_EPSILON) {
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
        // Output PID = posizione assoluta della siringa, frazione di corsa in [0, 1].
        // Comando motore NON bloccante: startMoveTo aggiorna il target di FastAccelStepper
        // al volo, anche se il motore sta ancora viaggiando dal tick precedente.
        // Clamp dell'output PID a [PID_U_MIN, PID_U_MAX]: tiene la siringa
        // lontana dagli estremi meccanici che coincidono con le soglie TOF,
        // così il controllo non si auto-ferma in emergency stop al limite.
        const float u = constrain(
            pidController.computeNormalized(targetDepth, currentDepth),
            PID_U_MIN, PID_U_MAX);
        const long usableSteps =
            (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
        const long posTarget = uToMotorPos(u);
        const long deadbandSteps =
            (long)(pidController.minRetargetFrac * (float)usableSteps);
        if (labs(posTarget - lastCommandedTarget) >= deadbandSteps) {
            motor.enableOutputs();
            motor.startMoveTo(posTarget);
            lastCommandedTarget = posTarget;
        }

        // --- EEPROM write tick (also checks hold condition for PID phase) ---
        if (millis() - lastWriteMs >= PERIOD_EEPROM_WRITE) {
            lastWriteMs = millis();

            // Mark PID-phase records with temperature sentinel
            _logReading(sensors.pressure(), 100.0f);

            if (fabsf(currentDepth - targetDepth) < _config.depthToleranceM) {
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
