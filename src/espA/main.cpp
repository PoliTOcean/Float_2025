/*
 *******************************************************************************
 *                      FLOAT board — ESP32-A (ESPA)  v11.0
 *
 * main.cpp
 * Entry point: setup() + loop() with the main state machine.
 * All subsystem logic lives in the dedicated modules under src/.
 *
 * Module map:
 *   config.h              — pin definitions, tuning constants
 *   led/led.h             — RGB LED state machine
 *   motor/motor.h         — stepper motor controller
 *   tof/tof.h             - VL53L7CX Time-of-Flight sensor controller
 *   motion_control.h      — homing, safe movement, and emergency stop
 *   pid/pid.h             — depth PID controller
 *   sensors/sensors.h     — Bar02 pressure sensor + INA219 power monitor
 *   comms/comms.h         — ESP-NOW messaging + OTA
 *   profile/profile.h     — depth profile execution + flash CSV logging
 *
 * Maintainers: Colabella Davide, Benevenga Filippo
 * Past contributors: Fachechi Gino Marco, Gullotta Salvatore
 * Company   : Team PoliTOcean @ Politecnico di Torino
 * Board pkg : esp32 by Espressif Systems v2.0.17
 *******************************************************************************
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <float_common.h>
#include <DebugSerial.h>

#include <config.h>
#include "led.h"
#include "motor.h"
#include "tof.h"
#include "motion_control.h"
#include "pid.h"
#include "sensors.h"
#include "comms.h"
#include "profile.h"
#include "flash_storage.h"
#include "runtime_config.h"

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static uint8_t  g_status          = CMD_IDLE;
static bool     g_autoModeActive  = false;
static bool     g_autoCommitted   = false;
static bool     g_autoMissionDone = false;
static bool     g_idle            = false;
static bool     g_debugModeActive = false;

// Make debug_mode_active reachable by DebugSerial / comms (extern linkage)
bool debug_mode_active = false;

// Global instances of main classes
LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);
MotionController motionController(motor, tofSensor);
SensorManager sensors;
PIDController pidController(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
ProfileManager profileManager;
// CommsManager comms; // Uncomment if you need this instance here

// Forward declarations — PID tuning helpers (test via seriale USB diretta)
static void servicePidTuningSerial();
static void runSyringeSet(float uNorm, float durationS);
static void runPidHold(float depthTarget, float durationS);
static void runPidStep(float depthTarget);
static bool runVerticalProfiles(uint8_t& completedProfiles);
static void attemptAutoRecovery();
// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    delay(100);
    Serial.begin(115200);
    Debug.println("=== Float ESPA v10.0 — starting ===");

    // --- LED (first, so we can signal errors immediately) ---
    ledController.setState(LEDState::INIT);

    // --- EEPROM ---
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Debug.println("CRITICAL: EEPROM init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // --- Communication (ESP-NOW) ---
    comms.begin();

    // --- DebugSerial ---
    // Wraps Serial so Debug.println() can optionally forward over ESP-NOW
    Debug.begin(&debug_mode_active,
                [](const char* msg, uint32_t timeout) -> uint8_t {
                    return comms.sendMessage(msg, timeout) ? 1 : 0;
                });
    Debug.println("DebugSerial ready");

    // --- Runtime PID / balance / motor settings ---
    runtimeConfig.begin();

    // --- Runtime mission profile ---
    profileManager.beginConfig();

    // --- Internal flash mission log ---
    // LittleFS è persistente al ciclo di alimentazione: al boot il log della
    // sessione precedente è ancora presente. Lo dumpiamo qui su Serial come
    // comodità (se il monitor è già connesso), ma NON lo azzeriamo: dopo un
    // test fallito spesso il monitor si collega in ritardo, quindi il log deve
    // sopravvivere al power-cycle e restare leggibile con il comando DUMP_LOG.
    // L'azzeramento avviene solo all'inizio di una nuova missione (resetEEPROM)
    // o su comando esplicito (CMD_CLEAR_EEPROM). Stampa diretta su Serial (non
    // Debug) per un CSV pulito, indipendente da debug_mode_active.
    if (flashStorage.begin()) {
        Serial.println("===== FLASH LOG DUMP (previous session) BEGIN =====");
        if (!flashStorage.printLogTo(Serial)) {
            Serial.println("(no previous log or flash unavailable)");
        }
        Serial.println("===== FLASH LOG DUMP END =====");
        Debug.println("Flash log ready (use DUMP_LOG to re-read)");
    } else {
        Debug.println("WARNING: flash log unavailable; stored data disabled");
    }

    // --- Sensors (Bar02 + INA219) ---
    // Both _initPressureSensor() and _initPowerMonitor() will block and
    // show LED_ERROR if the hardware is not found.
    sensors.begin();


    // Two short green blinks to confirm both sensors are alive
    ledController.setState(LEDState::INIT);
    delay(500);
    ledController.setState(LEDState::OFF);
    delay(500);
    ledController.setState(LEDState::INIT);
    delay(500);
    ledController.setState(LEDState::OFF);
    delay(500);

    // --- Motor + homing ---
    motor.begin();
    runtimeConfig.applyMotorConfig();

    Debug.println("Initializing TOF sensor...");
    if (!tofSensor.begin()) {
        Debug.println("CRITICAL: TOF sensor init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }
    Debug.println("TOF sensor ready");

    //motor_selftest();
    Debug.println("Starting motor homing...");
    if (!motionController.homeWithTof()) {
        Debug.println("CRITICAL: motor homing failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // Initialise outgoing packet battery field above threshold
    comms.status_to_send.charge = BATT_THRESH + 1;

    Debug.println("=== Initialisation complete — float ready ===");
    ledController.setState(LEDState::IDLE);
}

// ---------------------------------------------------------------------------
// LOOP — main state machine
// ---------------------------------------------------------------------------
void loop() {
    ledController.update();
    motionController.serviceEmergencyStop();
    servicePidTuningSerial();

    // -----------------------------------------------------------------------
    switch (g_status) {

    // -----------------------------------------------------------------------
    case CMD_IDLE:
    {
        g_autoCommitted = false;

        // Update battery reading in the outgoing packet
        comms.status_to_send.charge = sensors.batteryMilliVolts();

        // Low battery overrides LED
        if (comms.status_to_send.charge < BATT_THRESH) {
            ledController.setState(LEDState::LOW_BATTERY);
        }

        if (!g_idle) comms.clearCommand();

        // Send heartbeat / acknowledgement
        bool sent;
        if (profileManager.writePtr() > profileManager.readPtr()) {
            ledController.setState(LEDState::IDLE_WITH_DATA);
            sent = comms.sendMessage(IDLE_W_DATA_ACK, 5000);
        } else {
            ledController.setState(LEDState::IDLE);
            sent = comms.sendMessage(IDLE_ACK, 5000);
        }

        if (sent) {
            g_idle = true;
            unsigned long t0 = millis();
            while (millis() - t0 < PERIOD_CONN_CHECK &&
                   comms.lastCommand().command == CMD_IDLE) {
                ledController.update();
                servicePidTuningSerial();
                delay(10);
            }

            g_status = comms.lastCommand().command;
            if (g_status != CMD_IDLE) {
                g_idle = false;
                ledController.setState(LEDState::COMMUNICATION);
            }
        } else if (!g_autoMissionDone && g_autoModeActive) {
            // No comms — activate autonomous mode
            Debug.println("No comms — entering auto mode");
            ledController.setState(LEDState::AUTO_MODE);
            g_status       = CMD_GO;
            g_autoCommitted = true;
        }
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_GO: // Execute a full depth profile
    {
        bool ack = g_autoCommitted ? true : comms.sendMessage(CMD1_ACK, 1000);

        // Il comando GO resta inchiodato in _received perché sott'acqua non
        // arrivano nuovi pacchetti: lo consumiamo subito così non riparte da
        // solo al ritorno in IDLE e non dipendiamo da g_idle per il clear.
        comms.clearCommand();

        // true se un profilo è stato interrotto da emergency stop (es. safety
        // TOF): serve a decidere se tentare l'auto-recovery a fine missione.
        bool aborted = false;
        // Dichiarato qui (scope esterno) perché serve anche al blocco aborted.
        uint8_t completedProfiles = 0;

        if (ack && motionController.motionAllowed()) {
            aborted = runVerticalProfiles(completedProfiles);
            if (g_autoCommitted &&
                completedProfiles >= profileManager.config().profileCount) {
                g_autoMissionDone = true;
            }
        }

        // Auto-recovery: senza telemetria, un emergency stop lascerebbe il float
        // bloccato sul fondo col LED rosso e ogni comando successivo rifiutato da
        // motionAllowed(). Il record emergency_stop è già su flash; qui un homing
        // cancella lo stop e riporta la siringa a galleggiamento, pronto per un GO.
        if (aborted) {
            attemptAutoRecovery();
        }

        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SEND_DATA: // Stream buffered flash data to control station
    {
        Debug.println("Sending stored sensor data");
        profileManager.sendStoredData();
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_BALANCE: // Drive syringe to full extension then retraction
    {
        if (comms.sendMessage(CMD3_ACK, 1000)) {
            motionController.balance();
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_CLEAR_EEPROM:
    {
        if (comms.sendMessage(CMD4_ACK, 1000)) {
            profileManager.clearEEPROM();
            g_autoMissionDone = false;
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_AUTO_MODE: // Toggle autonomous profile execution
    {
        if (comms.sendMessage(CMD5_ACK, 1000)) {
            g_autoModeActive = !g_autoModeActive;
            if (g_autoModeActive) {
                g_autoMissionDone = false;
            }
            Debug.printf("Auto mode: %s\n", g_autoModeActive ? "ON" : "OFF");
            ledController.setState(g_autoModeActive ? LEDState::AUTO_MODE : LEDState::IDLE);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SEND_PACKAGE: // Send a single live sensor snapshot
    {
        sensors.read();

        char packet[OUTPUT_LEN];
        snprintf(packet, OUTPUT_LEN,
                 "{\"company_number\":\"%s\","
                 "\"time_s\":%.2f,"
                 "\"pressure_kpa\":%.2f,"
                 "\"depth_m\":%.2f,"
                 "\"phase\":\"%s\","
                 "\"sensor_depth_m\":%.2f,"
                 "\"syringe_u\":%.4f}",
                 COMPANY_NUMBER,
                 static_cast<float>(millis()) / 1000.0f,
                 sensors.pressure() / 1000.0f,
                 sensors.referenceDepthForPhase("live"),
                 "live",
                 sensors.sensorDepth(),
                 motorPosToU(motor.position()));

        comms.sendMessage(packet, 1000);
        Debug.println("Live snapshot sent");
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_OTA: // Start OTA update server (blocks for 5 min)
    {
        if (comms.sendMessage(CMD7_ACK, 1000)) {
            ledController.setState(LEDState::OTA_MODE);
            comms.runOTASession(); // Blocking — handles its own ESP-NOW teardown/restore
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PID_CONFIG_SET:
    {
        const output_message cmd = comms.lastCommand();
        const PidConfigPayload& payload = cmd.payload.pidConfig;
        RuntimePidConfig nextConfig;
        nextConfig.kp = payload.kp;
        nextConfig.ki = payload.ki;
        nextConfig.kd = payload.kd;
        nextConfig.periodMs = static_cast<uint16_t>(payload.periodMs);
        nextConfig.alphaD = payload.alphaD;
        nextConfig.integralLimit = payload.integralLimit;
        nextConfig.minRetargetFrac = payload.minRetargetFrac;
        nextConfig.uNeutral = payload.uNeutral;

        const bool updated = runtimeConfig.setPidConfig(nextConfig);
        comms.sendMessage(updated ? CMD8_ACK : CMD8_ERR, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PID_CONFIG_GET:
    {
        char packet[OUTPUT_LEN];
        runtimeConfig.formatPidConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_TEST_STEPS: // Manual stepper test
    {
        if (comms.sendMessage(CMD10_ACK, 1000)) {
            long steps = comms.lastCommand().payload.testSteps.steps;
            motionController.manualStepTest(steps, runtimeConfig.motor().testSpeed);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_DEBUG_MODE: // Toggle remote serial forwarding
    {
        if (comms.sendMessage(CMD11_ACK, 1000)) {
            debug_mode_active    = !debug_mode_active;
            g_debugModeActive    = debug_mode_active;
            Debug.printf("Debug mode: %s\n", debug_mode_active ? "ON" : "OFF");
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_HOME: // Remote-triggered homing
    {
        if (comms.sendMessage(CMD12_ACK, 1000)) {
            Debug.println("Remote homing requested");
            if (!motionController.homeWithTof()) {
                Debug.println("ERROR: remote homing failed");
                ledController.setState(LEDState::ERROR);
            } else {
                Debug.println("Remote homing complete");
            }
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_STOP: // Remote emergency stop
    {
        motionController.emergencyStop("remote stop");
        comms.sendMessage(CMD13_ACK, 1000);
        comms.clearCommand();
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SYRINGE_SET: // Test: posiziona siringa a u in [0,1] per N secondi
    {
        if (comms.sendMessage(CMD15_ACK, 1000)) {
            const output_message cmd = comms.lastCommand();
            const SyringeSetPayload& payload = cmd.payload.syringeSet;
            const float u   = payload.uNorm;
            const float dur = payload.durationS;
            runSyringeSet(u, dur);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PID_HOLD: // Test: PID a quota fissa per N secondi
    {
        if (comms.sendMessage(CMD16_ACK, 1000)) {
            const output_message cmd = comms.lastCommand();
            const PidHoldPayload& payload = cmd.payload.pidHold;
            const float depth = payload.depthM;
            const float dur   = payload.durationS;
            runPidHold(depth, dur);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PID_STEP: // Test: step response PID a quota X per 60 s
    {
        if (comms.sendMessage(CMD17_ACK, 1000)) {
            const float depth = comms.lastCommand().payload.pidStep.depthM;
            runPidStep(depth);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SET_SURFACE_OFFSET: // Imposta target di galleggiamento (m sotto pelo)
    {
        if (comms.sendMessage(CMD18_ACK, 1000)) {
            sensors.setSurfaceTargetOffset(comms.lastCommand().payload.surfaceOffset.meters);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PROFILE_SET:
    {
        const output_message cmd = comms.lastCommand();
        const ProfileSetPayload& payload = cmd.payload.profileSet;
        RuntimeProfileConfig nextConfig;
        nextConfig.profileCount      = payload.profileCount;
        nextConfig.descentTargetM    = payload.descentTargetM;
        nextConfig.ascentTargetM     = payload.ascentTargetM;
        nextConfig.depthToleranceM   = payload.depthToleranceM;
        nextConfig.holdTimeS         = payload.holdTimeS;
        nextConfig.descentTimeoutS   = payload.descentTimeoutS;
        nextConfig.ascentTimeoutS    = payload.ascentTimeoutS;
        nextConfig.surfaceRestOffsetM = payload.surfaceRestOffsetM;

        const bool updated = profileManager.setConfig(nextConfig);
        comms.sendMessage(updated ? CMD19_ACK : CMD19_ERR, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_PROFILE_GET:
    {
        char packet[OUTPUT_LEN];
        profileManager.formatConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_BALANCE_CONFIG_SET:
    {
        const output_message cmd = comms.lastCommand();
        const BalanceConfigPayload& payload = cmd.payload.balanceConfig;
        RuntimeBalanceConfig nextConfig;
        nextConfig.holdMs = payload.holdMs;
        nextConfig.stopPressureDeltaKpa = payload.stopPressureDeltaKpa;
        nextConfig.stopPressureSamples = payload.stopPressureSamples;
        nextConfig.samplePeriodMs = payload.samplePeriodMs;

        const bool updated = runtimeConfig.setBalanceConfig(nextConfig);
        comms.sendMessage(updated ? CMD21_ACK : CMD21_ERR, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_BALANCE_CONFIG_GET:
    {
        char packet[OUTPUT_LEN];
        runtimeConfig.formatBalanceConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_MOTOR_CONFIG_SET:
    {
        const output_message cmd = comms.lastCommand();
        const MotorConfigPayload& payload = cmd.payload.motorConfig;
        RuntimeMotorConfig nextConfig;
        nextConfig.maxSpeed = payload.maxSpeed;
        nextConfig.maxAcceleration = payload.maxAcceleration;
        nextConfig.homingSpeed = payload.homingSpeed;
        nextConfig.testSpeed = payload.testSpeed;

        const bool updated = runtimeConfig.setMotorConfig(nextConfig);
        comms.sendMessage(updated ? CMD23_ACK : CMD23_ERR, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_MOTOR_CONFIG_GET:
    {
        char packet[OUTPUT_LEN];
        runtimeConfig.formatMotorConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    default:
        Debug.printf("Unknown command: %d\n", g_status);
        g_status = CMD_IDLE;
        break;
    }

    delay(10);
}

// ---------------------------------------------------------------------------
// PID TUNING — comandi via seriale USB diretta
//
// Comandi accettati (uno per riga, terminato da \n):
//   PID_CONFIG_SET <kp> <ki> <kd> <period_ms> <alpha_d> <integral_limit>
//                  <min_retarget_frac> <u_neutral>
//                                      — aggiorna e salva configurazione PID
//   PID_CONFIG_GET                    — stampa configurazione PID corrente
//   SYRINGE_SET <u_norm> <dur_s>      — siringa a posizione normalizzata [0,1]
//                                       per N secondi, log depth ogni 100 ms
//   PID_HOLD <depth_m> <dur_s>        — PID a quota X per N secondi, log a 5 Hz
//   PID_STEP <depth_m>                — step response: PID a quota X per
//                                       max 60 s (esci a regime), log a 10 Hz
//   SURFACE_OFFSET <m>                — target di galleggiamento: il top del
//                                       float sta a <m> sotto il pelo (default 0.10)
//   SIM_ON / SIM_OFF                  — simulatore barometro on/off. Il motore si
//                                       muove DAVVERO; la quota è simulata da un
//                                       modello fisico mosso dalla siringa. Poi usa
//                                       PID_STEP/PID_HOLD/GO per tarare il PID a secco.
//   SIM_GET                           — stato e parametri del simulatore
//   SIM_CONFIG <uNeutral> <accelGain> <dragQuad> <poolDepth>
//                                      — ritara la fisica del simulatore a runtime
//   GO                                — lancia la missione completa (profili +
//                                       sosta) da seriale, senza GUI/ESPB. Con SIM
//                                       attivo = test end-to-end al banco a secco.
//
// Tutto il logging finisce su Serial (USB), formato CSV per facile import.
// ---------------------------------------------------------------------------
static String g_serialLineBuf;

static void servicePidTuningSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            String line = g_serialLineBuf;
            g_serialLineBuf = "";
            line.trim();
            if (line.length() == 0) return;

            // Tokenize semplice (solo separatore spazio)
            const char* cstr = line.c_str();
            char buf[192];
            strncpy(buf, cstr, sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = '\0';
            char* tok = strtok(buf, " ");
            if (!tok) return;

            if (strcmp(tok, "PID_CONFIG_SET") == 0) {
                char* values[8] = {};
                for (char*& value : values) {
                    value = strtok(nullptr, " ");
                    if (!value) {
                        Debug.println("ERR: PID_CONFIG_SET <kp> <ki> <kd> <period_ms> <alpha_d> <integral_limit> <min_retarget_frac> <u_neutral>");
                        return;
                    }
                }
                RuntimePidConfig config;
                config.kp = atof(values[0]);
                config.ki = atof(values[1]);
                config.kd = atof(values[2]);
                config.periodMs = static_cast<uint16_t>(atof(values[3]));
                config.alphaD = atof(values[4]);
                config.integralLimit = atof(values[5]);
                config.minRetargetFrac = atof(values[6]);
                config.uNeutral = atof(values[7]);
                Debug.println(runtimeConfig.setPidConfig(config) ? "OK PID_CONFIG_SET" : "ERR PID_CONFIG_SET invalid");
            } else if (strcmp(tok, "PID_CONFIG_GET") == 0) {
                char packet[OUTPUT_LEN];
                runtimeConfig.formatPidConfigJson(packet, sizeof(packet));
                Debug.println(packet);
            } else if (strcmp(tok, "SYRINGE_SET") == 0) {
                char* a = strtok(nullptr, " ");
                char* b = strtok(nullptr, " ");
                if (!a || !b) { Debug.println("ERR: SYRINGE_SET <u_norm> <dur_s>"); return; }
                runSyringeSet(atof(a), atof(b));
            } else if (strcmp(tok, "PID_HOLD") == 0) {
                char* a = strtok(nullptr, " ");
                char* b = strtok(nullptr, " ");
                if (!a || !b) { Debug.println("ERR: PID_HOLD <depth_m> <dur_s>"); return; }
                runPidHold(atof(a), atof(b));
            } else if (strcmp(tok, "PID_STEP") == 0) {
                char* a = strtok(nullptr, " ");
                if (!a) { Debug.println("ERR: PID_STEP <depth_m>"); return; }
                runPidStep(atof(a));
            } else if (strcmp(tok, "SURFACE_OFFSET") == 0) {
                char* a = strtok(nullptr, " ");
                if (!a) { Debug.println("ERR: SURFACE_OFFSET <m>"); return; }
                sensors.setSurfaceTargetOffset(atof(a));
                Debug.printf("OK SURFACE_OFFSET %.3f m\n", sensors.surfaceTargetOffset());
            } else if (strcmp(tok, "SIM_ON") == 0) {
                sensors.simEnable(true);
                Debug.println("OK SIM_ON (barometro simulato, motore reale)");
            } else if (strcmp(tok, "SIM_OFF") == 0) {
                sensors.simEnable(false);
                Debug.println("OK SIM_OFF");
            } else if (strcmp(tok, "SIM_GET") == 0) {
                char buf[176];
                sensors.simFormatStatus(buf, sizeof(buf));
                Debug.println(buf);
            } else if (strcmp(tok, "SIM_CONFIG") == 0) {
                char* values[4] = {};
                for (char*& value : values) {
                    value = strtok(nullptr, " ");
                    if (!value) {
                        Debug.println("ERR: SIM_CONFIG <uNeutral> <accelGain> <dragQuad> <poolDepth>");
                        return;
                    }
                }
                sensors.simConfigure(atof(values[0]), atof(values[1]),
                                     atof(values[2]), atof(values[3]));
                Debug.println("OK SIM_CONFIG");
            } else if (strcmp(tok, "GO") == 0) {
                // Lancia la missione completa (profileCount profili + sosta)
                // direttamente da seriale, senza GUI/ESPB: utile col SIM per il
                // test end-to-end al banco. Bloccante fino a fine missione;
                // per fermarla prima resetta ESPA.
                if (!motionController.motionAllowed()) {
                    Debug.println("ERR: GO — motion not allowed (serve homing ok)");
                    return;
                }
                Debug.println("# GO: missione completa (profili + sosta). Reset ESPA per abortire.");
                uint8_t completed = 0;
                const bool aborted = runVerticalProfiles(completed);
                if (aborted) attemptAutoRecovery();
                Debug.printf("# GO done: %u/%u profili%s\n",
                             completed, profileManager.config().profileCount,
                             aborted ? " (ABORT)" : "");
            } else if (strcmp(tok, "DUMP_LOG") == 0) {
                // Dump del flash log su richiesta: risolve il caso in cui il
                // serial monitor si collega dopo il dump automatico nel setup().
                // NON azzera il log, così può essere riletto più volte.
                Serial.println("===== FLASH LOG DUMP (on demand) BEGIN =====");
                if (!flashStorage.printLogTo(Serial)) {
                    Serial.println("(no log or flash unavailable)");
                }
                Serial.println("===== FLASH LOG DUMP END =====");
            } else {
                Debug.printf("ERR: unknown cmd '%s'\n", tok);
            }
            return;
        }
        if (g_serialLineBuf.length() < 191) g_serialLineBuf += c;
    }
}

// Comanda direttamente la siringa a u in [0,1] e logga la profondità per
// caratterizzare la dinamica del float (costanti di tempo, guadagno DC).
// Bypassa il PID: utile per stimare guadagni iniziali.
static void runSyringeSet(float uNorm, float durationS) {
    uNorm = constrain(uNorm, 0.0f, 1.0f);
    if (durationS < 0.5f || durationS > 300.0f) {
        Debug.println("ERR: durationS in [0.5, 300]");
        return;
    }
    const long posTarget = uToMotorPos(uNorm);

    Debug.printf("# SYRINGE_SET u=%.3f target_steps=%ld dur=%.1fs\n",
                  uNorm, posTarget, durationS);
    Debug.println("# t_ms,depth_m,motor_pos");
    motor.enableOutputs();
    motor.startMoveTo(posTarget);

    const unsigned long t0 = millis();
    unsigned long lastLog = 0;
    unsigned long lastTofSampleMs = 0;
    while (millis() - t0 < (unsigned long)(durationS * 1000.0f)) {
        if (Serial.available()) { Debug.println("# aborted"); break; }
        // Supervisione TOF su ogni movimento: fondo corsa esteso (tappo) = stop
        // pulito; oltre il limite superiore = emergency (già scattato dentro).
        const TofGuard guard = motionController.tofGuard(millis(), lastTofSampleMs, "syringe");
        if (guard == TofGuard::ExtendLimit) {
            motor.stop();
            Debug.println("# extension limit (TOF)");
        } else if (guard == TofGuard::Emergency) {
            Debug.println("# aborted (TOF)");
            break;
        }
        if (millis() - lastLog >= 100) {
            lastLog = millis();
            sensors.read();
            Debug.printf("%lu,%.3f,%ld\n",
                          millis() - t0, sensors.depth(), motor.position());
        }
        ledController.update();
        yield();
    }
    motor.stop();
    motor.disableOutputs();
    Debug.println("# done");
}

// Loop PID condiviso da PID_HOLD e PID_STEP: tiene la quota target per
// durationMs, ricalcolando il setpoint a pidController.periodMs e loggando il
// CSV ogni logPeriodMs. La supervisione TOF, il deadband e la saturazione al
// fondo corsa sono identici fra i due comandi: vivono qui per non divergere.
static void runPidLoop(float depthTarget, unsigned long durationMs, unsigned long logPeriodMs) {
    pidController.reset();
    motor.enableOutputs();

    const long usable = (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
    const long deadbandSteps = (long)(pidController.minRetargetFrac * (float)usable);
    long lastCommandedTarget = motor.position();

    const unsigned long t0 = millis();
    unsigned long lastTick = 0;
    unsigned long lastLog  = 0;
    unsigned long lastTofSampleMs = 0;
    bool atExtensionLimit = false;  // pistone fermo al fondo corsa (tappo): non ricomandare verso l'estensione
    while (millis() - t0 < durationMs) {
        if (Serial.available()) { Debug.println("# aborted"); break; }
        if (motionController.remoteStopRequested()) { Debug.println("# remote stop"); break; }

        // Supervisione TOF su ogni movimento. ExtendLimit = saturazione normale a
        // piena estensione: NON aborte, ferma e inibisce ulteriore estensione
        // finché il PID non chiede di risalire. Emergency = anomalia → abort.
        const TofGuard guard = motionController.tofGuard(millis(), lastTofSampleMs, "pid");
        if (guard == TofGuard::ExtendLimit) {
            if (!atExtensionLimit) {
                motor.stop();
                lastCommandedTarget = motor.position();
                atExtensionLimit = true;
            }
        } else if (guard == TofGuard::Emergency) {
            Debug.println("# aborted (TOF)");
            break;
        }

        if (millis() - lastTick >= pidController.periodMs) {
            lastTick = millis();
            sensors.read();
            const float depth = sensors.depth();
            const float u = pidController.computeNormalized(depthTarget, depth);
            const long posTarget = uToMotorPos(u);
            // In saturazione al fondo corsa accetta solo target che fanno
            // RISALIRE (verso home = pos più alta); ignora richieste di ulteriore
            // estensione, che riaprirebbero il tappo.
            const bool retreating = posTarget > motor.position();
            if ((!atExtensionLimit || retreating) &&
                labs(posTarget - lastCommandedTarget) >= deadbandSteps) {
                motor.startMoveTo(posTarget);
                lastCommandedTarget = posTarget;
                atExtensionLimit = false;
            }
            if (millis() - lastLog >= logPeriodMs) {
                lastLog = millis();
                Debug.printf("%lu,%.3f,%.3f,%.3f,%.3f,%ld\n",
                              millis() - t0, depth, depthTarget,
                              depthTarget - depth, u, motor.position());
            }
        }
        ledController.update();
        yield();
    }
    motor.stop();
    motor.disableOutputs();
    Debug.println("# done");
}

// Hold PID a quota fissa per N secondi, log a 5 Hz con CSV completo.
// Versione "tarable" di un profile PID phase, senza pre-position né hold check.
static void runPidHold(float depthTarget, float durationS) {
    if (depthTarget < 0.1f || depthTarget > 5.0f) {
        Debug.println("ERR: depthTarget in [0.1, 5.0] m");
        return;
    }
    if (durationS < 1.0f || durationS > 600.0f) {
        Debug.println("ERR: durationS in [1, 600]");
        return;
    }

    Debug.printf("# PID_HOLD target=%.3fm dur=%.1fs Kp=%.4f Ki=%.4f Kd=%.4f "
                  "period=%u alpha=%.3f\n",
                  depthTarget, durationS,
                  pidController.Kp, pidController.Ki, pidController.Kd,
                  pidController.periodMs, pidController.alphaD);
    Debug.println("# t_ms,depth_m,target_m,error_m,u_norm,motor_pos");

    runPidLoop(depthTarget, (unsigned long)(durationS * 1000.0f), 200);
}

// Step response: cambio istantaneo del setpoint, esci a 60 s.
// Identico a PID_HOLD ma con durata fissa e log a 10 Hz per catturare la rampa.
static void runPidStep(float depthTarget) {
    if (depthTarget < 0.1f || depthTarget > 5.0f) {
        Debug.println("ERR: depthTarget in [0.1, 5.0] m");
        return;
    }
    Debug.printf("# PID_STEP target=%.3fm Kp=%.4f Ki=%.4f Kd=%.4f "
                  "period=%u alpha=%.3f\n",
                  depthTarget,
                  pidController.Kp, pidController.Ki, pidController.Kd,
                  pidController.periodMs, pidController.alphaD);
    Debug.println("# t_ms,depth_m,target_m,error_m,u_norm,motor_pos");

    runPidLoop(depthTarget, 60000UL, 100);
}

// ---------------------------------------------------------------------------
// Missione completa: profileCount profili (discesa→hold→risalita→hold) + sosta
// finale. Riusata dal case CMD_GO (GUI/ESP-NOW/auto) e dal comando seriale
// diretto "GO" (test al banco / simulatore). Ritorna true se abortita da
// emergency stop; scrive in completedProfiles il numero di profili conclusi.
static bool runVerticalProfiles(uint8_t& completedProfiles) {
    completedProfiles = 0;
    bool aborted = false;
    const RuntimeProfileConfig& profileConfig = profileManager.config();
    Debug.println("Mission: starting vertical profiles");

    profileManager.resetEEPROM();
    profileManager.logDeploymentPacket();

    while (completedProfiles < profileConfig.profileCount && motionController.motionAllowed()) {
        profileManager.beginProfile(completedProfiles + 1);
        Debug.printf("Profile %d: PID descent to %.2f m bottom reference\n",
                     completedProfiles + 1, profileConfig.descentTargetM);
        profileManager.measure(profileConfig.descentTargetM,
                               profileConfig.holdTimeS,
                               profileConfig.descentTimeoutS);
        if (!motionController.motionAllowed()) { aborted = true; break; }

        delay(500);

        Debug.printf("Profile %d: PID ascent to %.2f m top reference\n",
                     completedProfiles + 1, profileConfig.ascentTargetM);
        profileManager.measure(profileManager.ascentTargetBottomM(),
                               profileConfig.holdTimeS,
                               profileConfig.ascentTimeoutS);
        if (!motionController.motionAllowed()) { aborted = true; break; }

        motor.disableOutputs();
        completedProfiles++;
        Debug.printf("Profile %d complete\n", completedProfiles);
        delay(500);
    }

    // Sosta finale: tieni la CIMA del float a surfaceRestOffsetM sotto il pelo
    // (antenna sommersa) finché non arriva il recupero o scade la finestra —
    // evita di rompere la superficie (penalità) in attesa dell'ROV.
    if (!aborted && motionController.motionAllowed()) {
        Debug.printf("Mission: surface rest, top at %.2f m below surface\n",
                     profileConfig.surfaceRestOffsetM);
        profileManager.measure(profileManager.restTargetBottomM(),
                               profileConfig.holdTimeS,
                               REST_WINDOW_S);
        if (!motionController.motionAllowed()) aborted = true;
    }

    return aborted;
}

// Auto-recovery su abort: l'homing cancella l'emergency stop e riporta la siringa
// in posizione nota (galleggiamento), così il float risale e resta pronto.
static void attemptAutoRecovery() {
    Debug.println("Profile aborted by emergency stop — attempting auto-recovery");
    if (motionController.homeWithTof()) {
        Debug.println("Auto-recovery homing complete — float ready");
    } else {
        Debug.println("Auto-recovery homing FAILED — float remains in error");
    }
}
