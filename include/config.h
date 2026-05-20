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

constexpr float    MOTOR_TRAVEL_MM       = 45.0f; // Total syringe travel (mm)
constexpr uint32_t MOTOR_MAX_STEPS       = static_cast<uint32_t>(MOTOR_TRAVEL_MM *
																 MOTOR_STEPS_PER_MM + 0.5f);
constexpr uint32_t MOTOR_MAX_SPEED       = 1500;  // Normal operating speed (steps/s); tested stable up to 2140 steps/s
constexpr uint32_t MOTOR_MAX_ACCELERATION = 1500;   // Normal acceleration/deceleration (steps/s^2); tested stable up to 2140 steps/s^2
constexpr uint32_t MOTOR_HOMING_SPEED    = 1500;   // Homing speed (steps/s)
constexpr uint16_t MOTOR_ENDSTOP_MARGIN  = 10;    // Safety margin from endstops (steps)
constexpr uint32_t MOTOR_HOMING_TIMEOUT  = 30000;  // Homing timeout (ms)
constexpr uint16_t MOTOR_HOMING_TOF_PERIOD_MS = 50; // TOF polling period during homing (ms)

// TOF (Time-of-Flight) sensor - VL53L4CD
constexpr uint8_t  TOF_XSHUT_PIN         = 16;    // Sensor shutdown pin
constexpr uint8_t  TOF_GPIO1_PIN         = 15;    // Optional interrupt pin, unused in polling mode
constexpr float    TOF_DISTANCE_OFFSET_MM = 24.0f; // Measured raw offset: raw distance - real distance
constexpr float    TOF_HOMING_THRESHOLD  = 40.0f; // Distance threshold for homing (mm)
constexpr float    TOF_MAX_STOP_DISTANCE_CM = 0.0f; // Max-extension TOF stop distance (cm, <=0 disabled until calibrated)

// ---------------------------------------------------------------------------
// TIMING CONSTANTS  (ms unless noted)
// ---------------------------------------------------------------------------
constexpr uint16_t PERIOD_MEASUREMENT   = 100;   // Between depth readings
constexpr uint16_t PERIOD_EEPROM_WRITE  = 5000;  // Between EEPROM writes
constexpr uint16_t PERIOD_CONN_CHECK    = 500;   // Between idle acknowledgements

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
constexpr uint8_t  PROFILE_MAX_COUNT   = 2;      // Profiles before auto-stop
constexpr float    FLOAT_LENGTH        = 0.51f;  // Bottom-to-sensor height (m)
constexpr float    DEPTH_MAX_ERROR     = 0.49f;  // "At target" tolerance (m)
constexpr float    DEPTH_EPSILON       = 0.01f;  // "Stationary" tolerance (m)
constexpr float    TARGET_DEPTH        = 2.50f;  // PID setpoint (m)
constexpr float    STAT_TIME           = 45.0f;  // Hold time at target (s)
constexpr float    TIMEOUT_PID_TIME    = 180.0f; // Max PID phase time (s)
constexpr float    TIMEOUT_ASCENT      = 50.0f;  // Max ascent time (s)

// Sentinel values passed to measure() as targetDepth
constexpr float    TARGET_SURFACE      = FLOAT_LENGTH; // Ascend to surface
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

constexpr uint8_t  MAC_ESPB[6]        = {0xEC, 0xE3, 0x34, 0xCE, 0x59, 0x1C};

// ---------------------------------------------------------------------------
// EEPROM / DATA
// ---------------------------------------------------------------------------
// EEPROM_SIZE and sensor_data struct come from float_common.h
