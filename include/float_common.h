#ifndef FLOAT_COMMON_H
#define FLOAT_COMMON_H

#include <Arduino.h>
#include <string.h>

/*
 *******************************************************************************
 * float_common.h
 * Shared command codes, ACK strings, and ESP-NOW message structs used by both
 * ESPA and ESPB.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

// Communication protocol defines
#define OUTPUT_LEN      250-sizeof(uint16_t) // Length of the output on the MAC layer
#define EEPROM_SIZE     512                 // EEPROM allocation size in bytes
#define BATT_THRESH     12000                // Battery threshold in mV for low battery warning

// Command codes shared by GUI/NEXUS, ESPB and ESPA.
// Append new commands at the end to keep older tools compatible.
enum FloatCommand : uint8_t {
  CMD_IDLE         = 0,
  CMD_GO           = 1,
  CMD_SEND_DATA    = 2,
  CMD_BALANCE      = 3,
  CMD_CLEAR_EEPROM = 4,
  CMD_AUTO_MODE    = 5,
  CMD_SEND_PACKAGE = 6,
  CMD_OTA          = 7,
  CMD_PID_CONFIG_SET = 8,
  CMD_RESERVED_9   = 9,
  CMD_TEST_STEPS   = 10,
  CMD_DEBUG_MODE   = 11,
  CMD_HOME         = 12,
  CMD_STOP         = 13,
  CMD_PID_CONFIG_GET = 14,
  CMD_SYRINGE_SET  = 15,
  CMD_PID_HOLD     = 16,
  CMD_PID_STEP     = 17,
  CMD_SET_SURFACE_OFFSET = 18,
  CMD_PROFILE_SET  = 19,
  CMD_PROFILE_GET  = 20,
  CMD_BALANCE_CONFIG_SET = 21,
  CMD_BALANCE_CONFIG_GET = 22,
  CMD_MOTOR_CONFIG_SET = 23,
  CMD_MOTOR_CONFIG_GET = 24,
};

// List of messages for the ESPA acknowledgements: CS has to be aware of these 
#define IDLE_ACK        "FLOAT_IDLE"        
#define IDLE_W_DATA_ACK "FLOAT_IDLE_W_DATA"
#define CMD1_ACK        "GO_RECVD"
#define CMD3_ACK        "CMD3_RECVD"
#define CMD4_ACK        "CMD4_RECVD"
#define CMD5_ACK        "SWITCH_AM_RECVD"
#define CMD7_ACK        "TRY_UPLOAD_RECVD"
#define CMD8_ACK        "PID_CONFIG_SET_RECVD"
#define CMD8_ERR        "PID_CONFIG_SET_ERR"
#define CMD10_ACK       "TEST_STEPS_RECVD"
#define CMD11_ACK       "DEBUG_MODE_RECVD"
#define CMD12_ACK       "HOME_RECVD"
#define CMD13_ACK       "STOP_RECVD"
#define CMD15_ACK       "SYRINGE_SET_RECVD"
#define CMD16_ACK       "PID_HOLD_RECVD"
#define CMD17_ACK       "PID_STEP_RECVD"
#define CMD18_ACK       "SURFACE_OFF_RECVD"
#define CMD19_ACK       "PROFILE_SET_RECVD"
#define CMD19_ERR       "PROFILE_SET_ERR"
#define CMD21_ACK       "BALANCE_CONFIG_SET_RECVD"
#define CMD21_ERR       "BALANCE_CONFIG_SET_ERR"
#define CMD23_ACK       "MOTOR_CONFIG_SET_RECVD"
#define CMD23_ERR       "MOTOR_CONFIG_SET_ERR"

// Sensor data structure
typedef struct sensor_data {
  float pressure;
  float temperature;
} sensor_data;

// I/O STRUCTS for ESP-NOW communication
// Must match between ESPA and ESPB structures
typedef struct input_message {
  uint16_t charge;
  char message[OUTPUT_LEN];
} input_message;

struct EmptyPayload {
  uint8_t reserved;
};

struct PidConfigPayload {
  float kp;
  float ki;
  float kd;
  float periodMs;
  float alphaD;
  float integralLimit;
  float minRetargetFrac;
  float uNeutral;
};

struct BalanceConfigPayload {
  uint32_t holdMs;
  float stopPressureDeltaKpa;
  uint8_t stopPressureSamples;
  uint16_t samplePeriodMs;
};

struct MotorConfigPayload {
  uint32_t maxSpeed;
  uint32_t maxAcceleration;
  uint32_t homingSpeed;
  uint32_t testSpeed;
};

struct TestStepsPayload {
  int32_t steps;
};

struct SyringeSetPayload {
  float uNorm;
  float durationS;
};

struct PidHoldPayload {
  float depthM;
  float durationS;
};

struct PidStepPayload {
  float depthM;
};

struct SurfaceOffsetPayload {
  float meters;
};

struct ProfileSetPayload {
  uint8_t profileCount;
  float deepTargetM;
  float shallowTopTargetM;
  float depthToleranceM;
  float holdTimeS;
  float pidTimeoutS;
  float ascentTimeoutS;
  float surfaceOffsetM;
};

union FloatCommandPayload {
  EmptyPayload empty;
  PidConfigPayload pidConfig;
  BalanceConfigPayload balanceConfig;
  MotorConfigPayload motorConfig;
  TestStepsPayload testSteps;
  SyringeSetPayload syringeSet;
  PidHoldPayload pidHold;
  PidStepPayload pidStep;
  SurfaceOffsetPayload surfaceOffset;
  ProfileSetPayload profileSet;
};

typedef struct output_message {
  FloatCommand command = CMD_IDLE;
  FloatCommandPayload payload;
} output_message;

inline output_message makeOutputMessage(FloatCommand command) {
  output_message message;
  memset(&message, 0, sizeof(message));
  message.command = command;
  return message;
}

static_assert(sizeof(output_message) <= 250, "output_message must fit in one ESP-NOW packet");

// Shared logical LED states. ESPA maps them to the RGB LED; ESPB maps the
// subset it can represent on the built-in single-colour LED.
enum class LEDState : uint8_t {
  OFF,             // Off / disabled
  INIT,            // Green solid or boot blinks - Initializing
  IDLE,            // Green solid / ESPB solid on - Ready/idle
  IDLE_WITH_DATA,  // Green fast blink - Idle with stored profile data
  LOW_BATTERY,     // Red solid - Battery voltage below threshold
  ERROR,           // Red fast blink / ESPB fast blink - Error state
  PROFILE,         // Blue solid - Running non-PID profile phase
  AUTO_MODE,       // Yellow blink - Auto mode active
  HOMING,          // Purple blink - Motor homing
  MOTOR_MOVING,    // Purple solid - Motor moving
  PID_CONTROL,     // Cyan blink - PID depth control active
  COMMUNICATION,   // White solid - Communicating
  OTA_MODE,        // Orange blink - OTA update mode
};

#endif
