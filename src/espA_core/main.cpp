/*
 *******************************************************************************
 *            FLOAT board — ESP32-A (ESPA)  —  CORE-DRIVEN build
 *
 * main.cpp  (env: espA_core)
 * Scheduler HAL attorno al control core Model-Based (lib/float_core).
 *
 * Differenza rispetto a src/espA/main.cpp (firmware di gara, INTATTO):
 *   - la logica di missione NON vive qui: e' in float_core_step() (tick-based,
 *     pura, la stessa che il modello Simulink/Stateflow riprodurra' in MIL).
 *   - questo file fa SOLO da HAL: legge i sensori -> costruisce CoreInputs ->
 *     chiama float_core_step() -> attua motore/LED/comms/flash dai CoreOutputs.
 *   - i comandi mission-critical (GO/HOME/STOP) diventano eventi per il core;
 *     i comandi di manutenzione (config get/set, dati) sono gestiti dall'HAL
 *     solo quando il core e' IDLE.
 *
 * Non ancora portati in questo build sperimentale (restano in espA): OTA,
 * balance, bench PID (SYRINGE/HOLD/STEP), auto-mode, simulatore seriale.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
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

#include "float_core.h"

// ---------------------------------------------------------------------------
// Singleton (definiti qui perche' src/espA/main.cpp non viene compilato in
// questo environment: stesse istanze del firmware di gara).
// ---------------------------------------------------------------------------
LEDController   ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor       tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);
MotionController motionController(motor, tofSensor);
SensorManager   sensors;
PIDController   pidController(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
ProfileManager  profileManager;

bool debug_mode_active = false;

// ---------------------------------------------------------------------------
// Stato del core + config + memoria di edge-detection per l'attuatore.
// ---------------------------------------------------------------------------
static CoreState  g_core;
static CoreConfig g_cfg;

static unsigned long g_lastTickMs   = 0;
static unsigned long g_missionStart = 0;
static CoreMode      g_lastCoreMode = CORE_MODE_INIT;
static CoreMotorMode g_lastMotorMode = CORE_MOTOR_HOLD;
static long          g_lastIssuedTarget = LONG_MIN;
static int           g_lastJogDir   = 0;

// ---------------------------------------------------------------------------
// Costruzione della CoreConfig da config.h + runtime NVS.
// Richiamata al boot e dopo ogni comando che cambia la configurazione.
// ---------------------------------------------------------------------------
static void buildCoreConfig() {
    const RuntimeProfileConfig& pc  = profileManager.config();
    const RuntimePidConfig&     pid = runtimeConfig.pid();

    g_cfg.profile_count          = pc.profileCount;
    g_cfg.descent_target_m       = pc.descentTargetM;
    g_cfg.ascent_target_bottom_m = profileManager.ascentTargetBottomM();
    g_cfg.rest_target_bottom_m   = profileManager.restTargetBottomM();
    g_cfg.depth_tolerance_m      = pc.depthToleranceM;
    g_cfg.hold_time_s            = pc.holdTimeS;
    g_cfg.descent_timeout_s      = pc.descentTimeoutS;
    g_cfg.ascent_timeout_s       = pc.ascentTimeoutS;
    g_cfg.rest_window_s          = REST_WINDOW_S;

    g_cfg.kp = pid.kp;  g_cfg.ki = pid.ki;  g_cfg.kd = pid.kd;
    g_cfg.alpha_d          = pid.alphaD;
    g_cfg.integral_limit   = pid.integralLimit;
    g_cfg.u_neutral        = pid.uNeutral;
    g_cfg.min_retarget_frac = pid.minRetargetFrac;
    g_cfg.pid_period_s     = pid.periodMs / 1000.0f;
    g_cfg.descent_kick_u   = PID_DESCENT_KICK_U;
    g_cfg.u_min = PID_U_MIN;  g_cfg.u_max = PID_U_MAX;

    g_cfg.usable_steps         = (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
    g_cfg.endstop_margin_steps = MOTOR_ENDSTOP_MARGIN;
    g_cfg.max_steps            = MOTOR_MAX_STEPS;

    g_cfg.tof_safe_min_mm            = TOF_SAFE_RANGE_MIN_MM;
    g_cfg.tof_safe_max_mm            = TOF_SAFE_RANGE_MAX_MM;
    g_cfg.tof_safety_stop_samples    = TOF_SAFETY_STOP_SAMPLES;
    g_cfg.tof_homing_approach_mm     = TOF_HOMING_APPROACH_MM;
    g_cfg.tof_homing_threshold_mm    = TOF_HOMING_THRESHOLD;
    g_cfg.tof_homing_confirm_samples = TOF_HOMING_CONFIRM_SAMPLES;
    g_cfg.homing_timeout_s           = MOTOR_HOMING_TIMEOUT / 1000.0f;
}

// ---------------------------------------------------------------------------
// Attuazione motore (contratto latch: GOTO aggancia, HOLD insegue, STOP ferma,
// JOG moto continuo). FastAccelStepper e' timer-driven: startMoveTo/Steps sono
// fire-and-forget, quindi HOLD non richiede nessuna azione (il motore prosegue
// verso il target agganciato). Edge-detection per non ri-comandare ogni tick.
// ---------------------------------------------------------------------------
static void applyMotor(const CoreOutputs& out) {
    if (out.motor_zero_here) {
        motor.setCurrentPosition(0);
        g_lastIssuedTarget = 0;
    }

    switch (out.motor_mode) {
    case CORE_MOTOR_GOTO:
        if (out.motor_enable) motor.enableOutputs();
        if (out.motor_target != g_lastIssuedTarget || g_lastMotorMode != CORE_MOTOR_GOTO) {
            motor.startMoveTo(out.motor_target);
            g_lastIssuedTarget = out.motor_target;
        }
        break;

    case CORE_MOTOR_JOG: {
        const int dir = (out.motor_target < 0) ? -1 : +1;
        if (g_lastMotorMode != CORE_MOTOR_JOG || dir != g_lastJogDir) {
            motor.enableOutputs();
            motor.startMoveSteps(dir * 2L * (long)MOTOR_MAX_STEPS);
            g_lastJogDir = dir;
        }
        break;
    }

    case CORE_MOTOR_STOP:
        if (g_lastMotorMode != CORE_MOTOR_STOP) motor.stop();
        if (!out.motor_enable) motor.disableOutputs();
        break;

    case CORE_MOTOR_HOLD:
    default:
        // Il motore prosegue autonomamente verso il target agganciato.
        if (!out.motor_enable && motor.distanceToGo() == 0) motor.disableOutputs();
        break;
    }
    g_lastMotorMode = out.motor_mode;
}

// ---------------------------------------------------------------------------
static void applyLed(const CoreOutputs& out, const CoreInputs& in) {
    LEDState s = LEDState::OFF;
    switch (out.led) {
    case CORE_LED_INIT:           s = LEDState::INIT;           break;
    case CORE_LED_IDLE:           s = LEDState::IDLE;           break;
    case CORE_LED_IDLE_WITH_DATA: s = LEDState::IDLE_WITH_DATA; break;
    case CORE_LED_ERROR:          s = LEDState::ERROR;          break;
    case CORE_LED_PROFILE:        s = LEDState::PROFILE;        break;
    case CORE_LED_HOMING:         s = LEDState::HOMING;         break;
    case CORE_LED_PID_CONTROL:    s = LEDState::PID_CONTROL;    break;
    case CORE_LED_COMMUNICATION:  s = LEDState::COMMUNICATION;  break;
    case CORE_LED_OFF:
    default:                      s = LEDState::OFF;            break;
    }
    // Low-battery override in idle (come nel firmware di gara).
    if (g_core.mode == CORE_MODE_IDLE && in.battery_mv < BATT_THRESH) {
        s = LEDState::LOW_BATTERY;
    }
    ledController.setState(s);
}

// ---------------------------------------------------------------------------
static void applyAck(const CoreOutputs& out, const CoreInputs& in) {
    comms.status_to_send.charge = in.battery_mv;
    switch (out.ack_event) {
    case CORE_ACK_GO_RECVD:     comms.sendMessage(CMD1_ACK, 1000);       break;
    case CORE_ACK_HOME_RECVD:   comms.sendMessage(CMD12_ACK, 1000);      break;
    case CORE_ACK_STOP_RECVD:   comms.sendMessage(CMD13_ACK, 1000);      break;
    case CORE_ACK_IDLE:         comms.sendMessage(IDLE_ACK, 1000);       break;
    case CORE_ACK_IDLE_W_DATA:  comms.sendMessage(IDLE_W_DATA_ACK, 1000); break;
    case CORE_ACK_NONE:
    default: break;
    }
}

// ---------------------------------------------------------------------------
static const char* phaseTag(CorePhase p) {
    switch (p) {
    case CORE_PHASE_DESCEND_HOLD: return "descending";
    case CORE_PHASE_ASCEND_HOLD:  return "ascending";
    case CORE_PHASE_SURFACE_REST: return "resting";
    default:                      return "profile";
    }
}

// Traduce l'evento di log del core in un record sul flash CSV (come
// ProfileManager::_logProfileReading, ma guidato dal core).
static void applyLog(const CoreOutputs& out) {
    const char* tag = "profile";
    uint8_t profileId = (uint8_t)(out.completed_profiles + 1);

    switch (out.log_kind) {
    case CORE_LOG_DEPLOY:       tag = "deployed";     profileId = 0;
                                g_missionStart = millis();            break;
    case CORE_LOG_PHASE_START:  tag = "phase_start";                  break;
    case CORE_LOG_PROFILE:      tag = phaseTag(out.phase);            break;
    case CORE_LOG_EXIT_HOLD_OK: tag = "exit_hold_ok";                 break;
    case CORE_LOG_EXIT_TIMEOUT: tag = "exit_timeout";                 break;
    case CORE_LOG_EXIT_REMOTE:  tag = "exit_remote_stop";             break;
    case CORE_LOG_EMERGENCY:    tag = "emergency_stop"; profileId = 0; break;
    case CORE_LOG_HOMING:       tag = "homing";        profileId = 0; break;
    case CORE_LOG_NONE:
    default: return;
    }

    const float t = g_missionStart ? (millis() - g_missionStart) / 1000.0f : 0.0f;
    flashStorage.appendRecord(COMPANY_NUMBER, profileId, t,
                              sensors.pressure() / 1000.0f,
                              sensors.referenceDepthForPhase(tag), tag,
                              sensors.sensorDepth(),
                              motorPosToU(motor.position()));
}

// ---------------------------------------------------------------------------
// Comandi di manutenzione (gestiti solo quando il core e' IDLE). Riusano i
// sottosistemi esistenti. I comandi mission-critical (GO/HOME/STOP) NON passano
// di qui: sono eventi per il core.
// ---------------------------------------------------------------------------
static void dispatchMaintenance(const output_message& cmd) {
    char packet[OUTPUT_LEN];
    switch (cmd.command) {
    case CMD_SEND_DATA:
        profileManager.sendStoredData();
        break;

    case CMD_SEND_PACKAGE:
        sensors.read();
        snprintf(packet, OUTPUT_LEN,
                 "{\"company_number\":\"%s\",\"time_s\":%.2f,\"pressure_kpa\":%.2f,"
                 "\"depth_m\":%.2f,\"phase\":\"live\",\"sensor_depth_m\":%.2f,"
                 "\"syringe_u\":%.4f}",
                 COMPANY_NUMBER, millis() / 1000.0f, sensors.pressure() / 1000.0f,
                 sensors.referenceDepthForPhase("live"), sensors.sensorDepth(),
                 motorPosToU(motor.position()));
        comms.sendMessage(packet, 1000);
        break;

    case CMD_CLEAR_EEPROM:
        if (comms.sendMessage(CMD4_ACK, 1000)) profileManager.clearEEPROM();
        break;

    case CMD_SET_SURFACE_OFFSET:
        if (comms.sendMessage(CMD18_ACK, 1000))
            sensors.setSurfaceTargetOffset(cmd.payload.surfaceOffset.meters);
        break;

    case CMD_DEBUG_MODE:
        if (comms.sendMessage(CMD11_ACK, 1000)) debug_mode_active = !debug_mode_active;
        break;

    case CMD_PID_CONFIG_SET: {
        const PidConfigPayload& p = cmd.payload.pidConfig;
        RuntimePidConfig c;
        c.kp = p.kp; c.ki = p.ki; c.kd = p.kd;
        c.periodMs = (uint16_t)p.periodMs; c.alphaD = p.alphaD;
        c.integralLimit = p.integralLimit; c.minRetargetFrac = p.minRetargetFrac;
        c.uNeutral = p.uNeutral;
        const bool ok = runtimeConfig.setPidConfig(c);
        buildCoreConfig();
        comms.sendMessage(ok ? CMD8_ACK : CMD8_ERR, 1000);
        break;
    }
    case CMD_PID_CONFIG_GET:
        runtimeConfig.formatPidConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        break;

    case CMD_PROFILE_SET: {
        const ProfileSetPayload& p = cmd.payload.profileSet;
        RuntimeProfileConfig c;
        c.profileCount = p.profileCount; c.descentTargetM = p.descentTargetM;
        c.ascentTargetM = p.ascentTargetM; c.depthToleranceM = p.depthToleranceM;
        c.holdTimeS = p.holdTimeS; c.descentTimeoutS = p.descentTimeoutS;
        c.ascentTimeoutS = p.ascentTimeoutS; c.surfaceRestOffsetM = p.surfaceRestOffsetM;
        const bool ok = profileManager.setConfig(c);
        buildCoreConfig();
        comms.sendMessage(ok ? CMD19_ACK : CMD19_ERR, 1000);
        break;
    }
    case CMD_PROFILE_GET:
        profileManager.formatConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        break;

    case CMD_BALANCE_CONFIG_SET: {
        const BalanceConfigPayload& p = cmd.payload.balanceConfig;
        RuntimeBalanceConfig c;
        c.holdMs = p.holdMs; c.stopPressureDeltaKpa = p.stopPressureDeltaKpa;
        c.stopPressureSamples = p.stopPressureSamples; c.samplePeriodMs = p.samplePeriodMs;
        const bool ok = runtimeConfig.setBalanceConfig(c);
        comms.sendMessage(ok ? CMD21_ACK : CMD21_ERR, 1000);
        break;
    }
    case CMD_BALANCE_CONFIG_GET:
        runtimeConfig.formatBalanceConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        break;

    case CMD_MOTOR_CONFIG_SET: {
        const MotorConfigPayload& p = cmd.payload.motorConfig;
        RuntimeMotorConfig c;
        c.maxSpeed = p.maxSpeed; c.maxAcceleration = p.maxAcceleration;
        c.homingSpeed = p.homingSpeed; c.testSpeed = p.testSpeed;
        const bool ok = runtimeConfig.setMotorConfig(c);
        buildCoreConfig();
        comms.sendMessage(ok ? CMD23_ACK : CMD23_ERR, 1000);
        break;
    }
    case CMD_MOTOR_CONFIG_GET:
        runtimeConfig.formatMotorConfigJson(packet, sizeof(packet));
        comms.sendMessage(packet, 1000);
        break;

    default:
        Debug.printf("espA_core: cmd %d non supportato in questo build\n", cmd.command);
        break;
    }
}

// ---------------------------------------------------------------------------
// TEST DA BANCO — comandi seriali USB diretti.
// Con SIM attivo: motore/TOF/homing REALI, quota SIMULATA -> missione completa
// a secco sul chip vero, traccia confrontabile con l'harness MIL nativo.
//
//   SIM_ON | SIM_OFF | SIM_GET | SIM_CONFIG <uNeutral> <accelGain> <dragQuad> <pool>
//   GO | HOME | STOP            — inietta l'evento nel core (one-shot)
//   TELEM_ON | TELEM_OFF        — telemetria CSV su Serial
//   DUMP_LOG                    — dump del flash log
// ATTENZIONE: con GO il pistone si estende DAVVERO (u fino a 0.92). Verifica che
// il meccanismo sia libero di muoversi prima di lanciare la missione al banco.
// ---------------------------------------------------------------------------
static volatile CoreEvent g_serialEvent = CORE_EVT_NONE;
static bool          g_telem       = false;
static unsigned long g_telemLastMs = 0;
static CoreMode      g_telemLastMode  = CORE_MODE_INIT;
static CorePhase     g_telemLastPhase = CORE_PHASE_NONE;
static String        g_serialBuf;

static const char* coreModeName(CoreMode m) {
    switch (m) {
    case CORE_MODE_INIT:      return "INIT";
    case CORE_MODE_HOMING:    return "HOMING";
    case CORE_MODE_IDLE:      return "IDLE";
    case CORE_MODE_MISSION:   return "MISSION";
    case CORE_MODE_SAFE_STOP: return "SAFE_STOP";
    default:                  return "?";
    }
}
static const char* corePhaseName(CorePhase p) {
    switch (p) {
    case CORE_PHASE_PRE_DESCENT:  return "pre_descent";
    case CORE_PHASE_DESCEND_HOLD: return "descend_hold";
    case CORE_PHASE_ASCEND_HOLD:  return "ascend_hold";
    case CORE_PHASE_SURFACE_REST: return "surface_rest";
    case CORE_PHASE_DONE:         return "done";
    default:                      return "-";
    }
}
static float coreTargetForPhase(CorePhase p) {
    switch (p) {
    case CORE_PHASE_DESCEND_HOLD: return g_cfg.descent_target_m;
    case CORE_PHASE_ASCEND_HOLD:  return g_cfg.ascent_target_bottom_m;
    case CORE_PHASE_SURFACE_REST: return g_cfg.rest_target_bottom_m;
    default:                      return 0.0f;
    }
}

// Telemetria: una riga ~1 Hz + a ogni cambio di modo/fase. Stesse colonne del
// MIL nativo (tools/mil) per confronto diretto on-target vs simulazione.
static void serviceTelemetry(const CoreInputs& in, const CoreOutputs& out) {
    if (!g_telem) return;
    const unsigned long now = millis();
    const bool changed = (g_core.mode != g_telemLastMode) || (out.phase != g_telemLastPhase);
    if (!changed && (now - g_telemLastMs) < 1000) return;
    g_telemLastMs   = now;
    g_telemLastMode = g_core.mode;
    g_telemLastPhase = out.phase;
    Serial.printf("%.2f,%s,%s,%.3f,%.3f,%.4f,%ld,%.1f\n",
                  now / 1000.0f, coreModeName(g_core.mode), corePhaseName(out.phase),
                  in.depth_m, coreTargetForPhase(out.phase),
                  motorPosToU(in.motor_pos), in.motor_pos,
                  in.tof_valid ? in.tof_mm : -1.0f);
}

static void serviceSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (g_serialBuf.length() < 120) g_serialBuf += c; continue; }

        String line = g_serialBuf; g_serialBuf = ""; line.trim();
        if (line.length() == 0) return;
        char buf[128];
        strncpy(buf, line.c_str(), sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
        char* tok = strtok(buf, " ");
        if (!tok) return;

        if      (!strcmp(tok, "GO"))   { g_serialEvent = CORE_EVT_GO;   Serial.println("# GO"); }
        else if (!strcmp(tok, "HOME")) { g_serialEvent = CORE_EVT_HOME; Serial.println("# HOME"); }
        else if (!strcmp(tok, "STOP")) { g_serialEvent = CORE_EVT_STOP; Serial.println("# STOP"); }
        else if (!strcmp(tok, "SIM_ON"))  { sensors.simEnable(true);  Serial.println("# SIM ON (motore/TOF reali, quota simulata)"); }
        else if (!strcmp(tok, "SIM_OFF")) { sensors.simEnable(false); Serial.println("# SIM OFF"); }
        else if (!strcmp(tok, "SIM_GET")) { char s[176]; sensors.simFormatStatus(s, sizeof(s)); Serial.println(s); }
        else if (!strcmp(tok, "SIM_CONFIG")) {
            char* v[4] = {};
            for (auto& x : v) { x = strtok(nullptr, " "); if (!x) { Serial.println("# ERR: SIM_CONFIG <uNeutral> <accelGain> <dragQuad> <pool>"); return; } }
            sensors.simConfigure(atof(v[0]), atof(v[1]), atof(v[2]), atof(v[3]));
        }
        else if (!strcmp(tok, "TELEM_ON"))  { g_telem = true;  Serial.println("t_s,mode,phase,depth_m,target_m,u,motor_pos,tof_mm"); }
        else if (!strcmp(tok, "TELEM_OFF")) { g_telem = false; }
        else if (!strcmp(tok, "DUMP_LOG"))  {
            Serial.println("===== FLASH LOG DUMP BEGIN =====");
            if (!flashStorage.printLogTo(Serial)) Serial.println("(no log)");
            Serial.println("===== FLASH LOG DUMP END =====");
        }
        else { Serial.printf("# ERR: comando sconosciuto '%s'\n", tok); }
        return;
    }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    delay(100);
    Serial.begin(115200);
    Debug.println("=== Float ESPA (core-driven) — starting ===");

    ledController.setState(LEDState::INIT);

    if (!EEPROM.begin(EEPROM_SIZE)) {
        Debug.println("CRITICAL: EEPROM init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    comms.begin();
    Debug.begin(&debug_mode_active,
                [](const char* msg, uint32_t timeout) -> uint8_t {
                    return comms.sendMessage(msg, timeout) ? 1 : 0;
                });

    runtimeConfig.begin();
    profileManager.beginConfig();

    if (flashStorage.begin()) {
        Serial.println("===== FLASH LOG DUMP (previous session) BEGIN =====");
        if (!flashStorage.printLogTo(Serial))
            Serial.println("(no previous log or flash unavailable)");
        Serial.println("===== FLASH LOG DUMP END =====");
    } else {
        Debug.println("WARNING: flash log unavailable");
    }

    sensors.begin();

    motor.begin();
    runtimeConfig.applyMotorConfig();

    Debug.println("Initializing TOF sensor...");
    if (!tofSensor.begin()) {
        Debug.println("CRITICAL: TOF sensor init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // A differenza di espA, l'homing NON e' fatto qui: lo guida il core
    // (INIT -> HOMING) al primo tick, via i CoreOutputs.
    buildCoreConfig();
    float_core_init(&g_core);
    comms.status_to_send.charge = BATT_THRESH + 1;

    Debug.println("=== Init complete — core scheduler running ===");
}

// ---------------------------------------------------------------------------
// LOOP — scheduler HAL a passo fisso (pid_period). Nessuna logica di missione:
// e' tutta in float_core_step().
// ---------------------------------------------------------------------------
void loop() {
    ledController.update();
    serviceSerial(); // reattivo ogni iterazione, indipendente dal tick del core

    const unsigned long now = millis();
    const unsigned long periodMs = runtimeConfig.pid().periodMs;
    if (g_lastTickMs != 0 && (now - g_lastTickMs) < periodMs) {
        delay(1);
        return;
    }
    const float dt = (g_lastTickMs == 0) ? (periodMs / 1000.0f)
                                         : (now - g_lastTickMs) / 1000.0f;
    g_lastTickMs = now;

    // --- Ingressi: sensori + comando ---
    sensors.read();
    float tofMm = 0.0f;
    const bool tofValid = tofSensor.isInitialized() && tofSensor.readDistanceMm(tofMm);

    output_message cmd = comms.lastCommand();
    CoreEvent event = CORE_EVT_NONE;
    if      (cmd.command == CMD_GO)   event = CORE_EVT_GO;
    else if (cmd.command == CMD_HOME) event = CORE_EVT_HOME;
    else if (cmd.command == CMD_STOP) event = CORE_EVT_STOP;
    if (event != CORE_EVT_NONE) comms.clearCommand(); // consuma l'evento mission

    // Evento da seriale (test da banco): one-shot, se non c'e' gia' un comando radio.
    if (event == CORE_EVT_NONE && g_serialEvent != CORE_EVT_NONE) {
        event = g_serialEvent;
        g_serialEvent = CORE_EVT_NONE;
    }

    CoreInputs in;
    in.dt_s                 = dt;
    in.depth_m              = sensors.depth();
    in.pressure_kpa         = sensors.pressure() / 1000.0f;
    in.tof_mm               = tofValid ? tofMm : 0.0f;
    in.tof_valid            = tofValid;
    in.motor_pos            = motor.position();
    in.motor_position_known = motor.isPositionKnown();
    in.motor_busy           = motor.distanceToGo() != 0;
    in.battery_mv           = sensors.batteryMilliVolts();
    in.has_stored_data      = profileManager.writePtr() > profileManager.readPtr();
    in.event                = event;
    in.cfg                  = &g_cfg;

    // --- Un passo del core ---
    CoreOutputs out;
    float_core_step(&in, &g_core, &out);

    // --- Velocita' motore per modo (homing vs missione) ---
    if (g_core.mode != g_lastCoreMode) {
        if (g_core.mode == CORE_MODE_HOMING) {
            motor.setMaxSpeed(runtimeConfig.motor().homingSpeed);
            motor.setAcceleration(runtimeConfig.motor().homingSpeed);
        } else {
            motor.setMaxSpeed(runtimeConfig.motor().maxSpeed);
            motor.setAcceleration(runtimeConfig.motor().maxAcceleration);
        }
        g_lastCoreMode = g_core.mode;
    }

    // --- Attuazione ---
    applyMotor(out);
    applyLed(out, in);
    applyAck(out, in);
    if (out.log_now) applyLog(out);
    serviceTelemetry(in, out);

    // --- Comandi di manutenzione: solo a core IDLE ---
    if (g_core.mode == CORE_MODE_IDLE &&
        cmd.command != CMD_IDLE && event == CORE_EVT_NONE) {
        dispatchMaintenance(cmd);
        comms.clearCommand();
    }
}
