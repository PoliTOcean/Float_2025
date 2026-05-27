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

constexpr float    MOTOR_TRAVEL_MM       = 40.0f; // Total syringe travel (mm)
constexpr uint32_t MOTOR_MAX_STEPS       = static_cast<uint32_t>(MOTOR_TRAVEL_MM *
																 MOTOR_STEPS_PER_MM + 0.5f);
constexpr uint32_t MOTOR_MAX_SPEED       = 1500;  // Normal operating speed (steps/s); tested stable up to 2140 steps/s
constexpr uint32_t MOTOR_MAX_ACCELERATION = 1500;   // Normal acceleration/deceleration (steps/s^2); tested stable up to 2140 steps/s^2
constexpr uint32_t MOTOR_HOMING_SPEED    = 1500;   // Homing speed (steps/s)
constexpr uint16_t MOTOR_ENDSTOP_MARGIN  = 10;    // Safety margin from endstops (steps)
constexpr uint32_t MOTOR_HOMING_TIMEOUT  = 30000;  // Homing timeout (ms)
constexpr uint16_t MOTOR_HOMING_TOF_PERIOD_MS = 50; // TOF polling period during homing (ms)

// TOF (Time-of-Flight) sensor - VL53L7CX
constexpr uint8_t  TOF_XSHUT_PIN         = 16;    // LPn (sensor enable) pin
constexpr uint8_t  TOF_GPIO1_PIN         = 15;    // Optional INT pin, unused in polling mode
constexpr float    TOF_DISTANCE_OFFSET_MM = 24.0f; // Measured raw offset: raw distance - real distance
constexpr float    TOF_HOMING_THRESHOLD  = 40.0f; // Distance threshold for homing (mm)
constexpr float    TOF_MAX_STOP_MARGIN_MM = 2.0f; // Extra margin beyond homing distance + syringe travel
constexpr float    TOF_MAX_STOP_DISTANCE_MM =
    TOF_HOMING_THRESHOLD + MOTOR_TRAVEL_MM + TOF_MAX_STOP_MARGIN_MM;

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
// PID TUNING
// ---------------------------------------------------------------------------
// These are mutable at runtime via command 8 (UPDATE_PID), so they live in
// pid.cpp as extern variables — only defaults are declared here.
constexpr float PID_KP_DEFAULT        = 10.0f;
constexpr float PID_KI_DEFAULT        = 0.0f;
constexpr float PID_KD_DEFAULT        = 350.0f;
constexpr float PID_OUTPUT_LIMIT      = 80.0f;   // Max output magnitude (steps)
constexpr float PID_INTEGRAL_LIMIT    = 5.0f;    // Anti-windup clamp

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
