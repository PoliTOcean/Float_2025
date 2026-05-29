#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * config.h
 * Centralized configuration: pin definitions, tuning constants, network params.
 * Team PoliTOcean @ Politecnico di Torino
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
// HARDWARE PIN DEFINITIONS
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_DIR           = 32;   // IO32: DIR  (dalla PCB)
constexpr uint8_t PIN_STEP          = 33;   // IO33: STEP (dalla PCB)
constexpr uint8_t PIN_EN            = 27;   // IO27: ENABLE ✓
constexpr uint8_t PIN_DRV_SLEEP     = 25;   // IO25: SLEEP (dalla PCB)
constexpr uint8_t PIN_DRV_RST       = 26;   // IO26: RESET (dalla PCB)

// Note: PIN_ENDSTOP_DOWN and PIN_ENDSTOP_UP removed — replaced with TOF sensor
constexpr uint8_t PIN_LED_R         = 19;   // Red LED channel
constexpr uint8_t PIN_LED_G         = 18;   // Green LED channel
constexpr uint8_t PIN_LED_B         = 5;    // Blue LED channel

// ---------------------------------------------------------------------------
// MOTOR CONSTANTS
// ---------------------------------------------------------------------------
constexpr uint16_t MOTOR_STEPS_PER_REV   = 200;   // Motor steps per revolution (motor specific 360/1.8)
constexpr uint8_t  MOTOR_MICROSTEP       = 1;     // Microstepping (4 = quarter step)
constexpr float    MOTOR_GEAR_RATIO      = 26.85124f; // Gearbox ratio (26 + 103/121)
constexpr float    MOTOR_SCREW_PITCH_MM  = 2.0f;  // Distance between adjacent thread crests (mm)
constexpr uint8_t  MOTOR_SCREW_STARTS    = 4;     // Number of thread starts/principles

constexpr float MOTOR_SCREW_LEAD_MM =
    MOTOR_SCREW_PITCH_MM * MOTOR_SCREW_STARTS;
constexpr float MOTOR_REVS_PER_MM =
    MOTOR_GEAR_RATIO / MOTOR_SCREW_LEAD_MM;
constexpr float MOTOR_STEPS_PER_MM =
    MOTOR_STEPS_PER_REV * MOTOR_MICROSTEP * MOTOR_REVS_PER_MM;

constexpr float    MOTOR_TRAVEL_MM       = 35.0f; // Normal commanded syringe travel (mm)
constexpr uint32_t MOTOR_MAX_STEPS       = static_cast<uint32_t>(MOTOR_TRAVEL_MM *
																 MOTOR_STEPS_PER_MM + 0.5f);
constexpr uint32_t MOTOR_MAX_SPEED       = 1800;  // Normal operating speed (steps/s)
constexpr uint32_t MOTOR_MAX_ACCELERATION = 1800; // Normal acceleration/deceleration (steps/s^2)
constexpr uint32_t MOTOR_HOMING_SPEED    = 1800;   // Homing speed (steps/s)
constexpr uint16_t MOTOR_ENDSTOP_MARGIN  = 10;    // Safety margin from endstops (steps)

// Geometria reale: home (motor_pos=0) = siringa retratta, vuota → float galleggia.
// MOTOR_MAX_STEPS = siringa estesa, piena d'acqua → float affonda.
// Convenzione "logica" del PID/profile: u=0 → galleggia (siringa vuota),
//                                       u=1 → affonda (siringa piena).
// La geometria coincide con la convenzione logica: nessuna inversione necessaria.
constexpr bool MOTOR_INVERT_LOGICAL = false;
inline long uToMotorPos(float u) {
    const long usable = (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
    const float uPhys = MOTOR_INVERT_LOGICAL ? (1.0f - u) : u;
    return (long)MOTOR_ENDSTOP_MARGIN + (long)(uPhys * (float)usable);
}
constexpr uint32_t MOTOR_HOMING_TIMEOUT  = 30000;  // Homing timeout (ms)
constexpr uint16_t MOTOR_HOMING_TOF_PERIOD_MS = 50; // TOF polling period during homing (ms)

// TOF (Time-of-Flight) sensor - VL53L7CX
constexpr uint8_t  TOF_XSHUT_PIN         = 16;    // LPn (sensor enable) pin
constexpr uint8_t  TOF_GPIO1_PIN         = 15;    // Optional INT pin, unused in polling mode
// VL53L7CX 4x4 zone mask: bit 0..15 maps directly to driver zone index
// results.distance_mm[i] / results.target_status[i]. 1 = enabled, 0 = ignored.
constexpr uint8_t  TOF_MATRIX_ZONE_COUNT = 16;
constexpr uint16_t TOF_ZONE_ENABLE_MASK  = 0x0660; // Central zones: 5, 6, 9, 10
constexpr float    TOF_DISTANCE_RAW_OFFSET_MM = 6.0f; // Raw distance is this much higher than real distance
constexpr float    TOF_HOMING_THRESHOLD     = 75.0f; // Homing stop distance: stop when TOF reads ABOVE this (siringa retratta = lontana dal TOF) (mm)
constexpr float    TOF_HOMING_APPROACH_MM   = 50.0f; // Approach phase: move toward TOF until reading BELOW this, then start homing (mm)
constexpr float    TOF_SAFE_RANGE_MIN_MM    = 40.0f; // Safety range lower bound: siringa estesa, troppo vicina al TOF (mm)
constexpr float    TOF_SAFE_RANGE_MAX_MM    = 80.0f; // Safety range upper bound: siringa retratta, troppo lontana dal TOF (mm). Deve stare sopra TOF_HOMING_THRESHOLD con margine per il rumore (≥10 mm) altrimenti moveToMax fa emergency stop subito dopo l'homing.

// ---------------------------------------------------------------------------
// BALANCE / PURGE CONTROL
// ---------------------------------------------------------------------------
constexpr float    BALANCE_STOP_PRESSURE_DELTA_KPA = 5.0f; // Stop balance when Bar02 rises above baseline by this amount
constexpr uint8_t  BALANCE_STOP_PRESSURE_SAMPLES = 3; // Consecutive above-threshold samples required
constexpr uint16_t BALANCE_PRESSURE_SAMPLE_PERIOD_MS = 50; // Bar02 polling period during balance

// ---------------------------------------------------------------------------
// TIMING CONSTANTS  (ms unless noted)
// ---------------------------------------------------------------------------
constexpr uint16_t PERIOD_MEASUREMENT   = 100;   // Between depth readings
constexpr uint16_t PERIOD_CONN_CHECK    = 500;   // Between idle acknowledgements

#ifdef POOL_TEST_PROFILE
constexpr uint16_t PERIOD_EEPROM_WRITE   = 2000; // Faster hold checks for shallow pool tests
constexpr uint16_t PROFILE_LOG_PERIOD_MS = 500;  // Denser flash log for short pool runs
constexpr uint16_t DATA_PACKET_PERIOD_MS = 2000; // Denser replay packets for short pool runs
#else
constexpr uint16_t PERIOD_EEPROM_WRITE   = 5000; // Between EEPROM writes / hold checks
constexpr uint16_t PROFILE_LOG_PERIOD_MS = 1000; // Between flash profile writes
constexpr uint16_t DATA_PACKET_PERIOD_MS = 5000; // Packet cadence shown to judges
#endif

// ---------------------------------------------------------------------------
// PID TUNING (output normalizzato in [0,1] = frazione di corsa siringa)
// ---------------------------------------------------------------------------
// Kp/Ki/Kd mutabili a runtime via CMD_UPDATE_PID (8); periodMs e alphaD via
// CMD_UPDATE_PID_EXT (14). Espressi in "frazione di corsa per metro di errore",
// portabili tra siringhe — se cambia MOTOR_MAX_STEPS, i guadagni restano validi.
constexpr uint16_t PID_PERIOD_DEFAULT_MS  = 50;    // Default tick PID (ms)
constexpr float    PID_KP_DEFAULT         = 0.17f; // frazione_corsa / m
constexpr float    PID_KI_DEFAULT         = 0.0f;  // frazione_corsa / (m·s)
constexpr float    PID_KD_DEFAULT         = 0.13f; // frazione_corsa / (m/s)
constexpr float    PID_INTEGRAL_LIMIT     = 5.0f;  // m·s (bound conservativo)
constexpr float    PID_ALPHA_D_DEFAULT    = 0.25f; // LPF IIR coeff per derivata
constexpr float    PID_U_NEUTRAL          = 0.011f;// kick-start offset (~500/47100)
constexpr float    PID_MIN_RETARGET_FRAC  = 0.001f;// dead-band ri-comando (frazione corsa)

// ---------------------------------------------------------------------------
// FLOAT PHYSICAL / MISSION CONSTANTS
// ---------------------------------------------------------------------------
constexpr float    FLOAT_LENGTH        = 0.51f;  // Bottom-to-sensor height (m)
constexpr float    SENSOR_TO_BOTTOM_M  = FLOAT_LENGTH; // Pressure sensor to bottom reference
constexpr float    SENSOR_TO_TOP_M     = 0.0f;   // Pressure sensor to top reference; calibrate on hardware
constexpr float    DEPTH_EPSILON       = 0.01f;  // "Stationary" tolerance (m)

#ifdef POOL_TEST_PROFILE
constexpr float    POOL_TEST_WATER_DEPTH = 0.70f; // Reference only: assumed test pool depth (m)
constexpr uint8_t  PROFILE_MAX_COUNT     = 1;     // One cycle keeps shallow-pool tests shorter and safer
constexpr float    DEPTH_MAX_ERROR       = 0.025f; // Narrow tolerance because pool targets are close together
constexpr float    TARGET_DEPTH          = 0.63f; // Deep hold: bottom reference (m), ~7 cm above a 70 cm floor
constexpr float    TARGET_SHALLOW_TOP_DEPTH = 0.06f; // Shallow hold: top reference (m)
constexpr float    STAT_TIME             = 8.0f;  // Short pool hold; actual check cadence is PERIOD_EEPROM_WRITE
constexpr float    TIMEOUT_PID_TIME      = 45.0f; // Max PID phase time (s)
constexpr float    TIMEOUT_ASCENT        = 45.0f; // Max ascent + shallow hold time (s)
#else
constexpr uint8_t  PROFILE_MAX_COUNT   = 2;      // Profiles before auto-stop
constexpr float    DEPTH_MAX_ERROR     = 0.33f;  // MATE depth tolerance (m)
constexpr float    TARGET_DEPTH        = 2.50f;  // Deep hold: bottom reference (m)
constexpr float    TARGET_SHALLOW_TOP_DEPTH = 0.40f; // Shallow hold: top reference (m)
constexpr float    STAT_TIME           = 30.0f;  // MATE hold time at target (s)
constexpr float    TIMEOUT_PID_TIME    = 180.0f; // Max PID phase time (s)
constexpr float    TIMEOUT_ASCENT      = 120.0f; // Max ascent + shallow hold time (s)
#endif

constexpr float    TARGET_SHALLOW_BOTTOM_DEPTH =
    TARGET_SHALLOW_TOP_DEPTH + SENSOR_TO_BOTTOM_M + SENSOR_TO_TOP_M;

// Sentinel values passed to measure() as targetDepth
constexpr float    TARGET_SURFACE      = FLOAT_LENGTH; // Legacy surface sentinel
constexpr int8_t   TARGET_BOTTOM       = -1;           // Descend to pool floor

// ---------------------------------------------------------------------------
// SENSOR CONSTANTS
// ---------------------------------------------------------------------------
constexpr float    WATER_DENSITY_FRESH = 997.0f;   // kg/m³
constexpr float    GRAVITY             = 9.80665f;

// ---------------------------------------------------------------------------
// NETWORK / OTA
// ---------------------------------------------------------------------------
constexpr char     WIFI_SSID[]         = "PIPO";
constexpr char     WIFI_PASSWORD[]     = "politocean";

// constexpr uint8_t  MAC_ESPB[6]        = {0xEC, 0xE3, 0x34, 0xCE, 0x59, 0x1C};
constexpr uint8_t MAC_ESPB[6] = {0x88, 0x57, 0x21, 0x84, 0x8C, 0xE8};
constexpr uint8_t MAC_ESPA[6] = {0x88, 0x57, 0x21, 0x84, 0x83, 0x8C};
constexpr uint8_t ESPNOW_CHANNEL = 1;

// ---------------------------------------------------------------------------
// EEPROM / DATA
// ---------------------------------------------------------------------------
constexpr char     COMPANY_NUMBER[]     = "EX10";
constexpr char     FLASH_LOG_PATH[]     = "/mission/current_profile.csv";

// EEPROM_SIZE and sensor_data struct come from float_common.h
