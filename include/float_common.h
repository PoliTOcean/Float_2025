#ifndef FLOAT_COMMON_H
#define FLOAT_COMMON_H

#include <Arduino.h>

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
  CMD_UPDATE_PID   = 8,
  CMD_SET_SPEED    = 9,
  CMD_TEST_STEPS   = 10,
  CMD_DEBUG_MODE   = 11,
  CMD_HOME         = 12,
  CMD_STOP         = 13,
  CMD_UPDATE_PID_EXT = 14,
  CMD_SYRINGE_SET  = 15,
  CMD_PID_HOLD     = 16,
  CMD_PID_STEP     = 17,
};

// List of messages for the ESPA acknowledgements: CS has to be aware of these 
#define IDLE_ACK        "FLOAT_IDLE"        
#define IDLE_W_DATA_ACK "FLOAT_IDLE_W_DATA"
#define CMD1_ACK        "GO_RECVD"
#define CMD3_ACK        "CMD3_RECVD"
#define CMD4_ACK        "CMD4_RECVD"
#define CMD5_ACK        "SWITCH_AM_RECVD"
#define CMD7_ACK        "TRY_UPLOAD_RECVD"
#define CMD8_ACK        "CHNG_PARMS_RECVD"
#define CMD9_ACK        "TEST_FREQ_RECVD"
#define CMD10_ACK       "TEST_STEPS_RECVD"
#define CMD11_ACK       "DEBUG_MODE_RECVD"
#define CMD12_ACK       "HOME_RECVD"
#define CMD13_ACK       "STOP_RECVD"
#define CMD14_ACK       "CHNG_PID_EXT_RECVD"
#define CMD15_ACK       "SYRINGE_SET_RECVD"
#define CMD16_ACK       "PID_HOLD_RECVD"
#define CMD17_ACK       "PID_STEP_RECVD"

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

typedef struct output_message {
  float params[3];
  int32_t steps;
  uint16_t freq;
  uint8_t command = CMD_IDLE;
} output_message;

// MAC addresses - UPDATE THESE TO YOUR ACTUAL MAC ADDRESSES
extern uint8_t espA_mac[6];
extern uint8_t espB_mac[6];

// LED States for better status indication
enum FloatLEDState {
  LED_OFF,
  LED_INIT,           // Green solid - Initializing
  LED_IDLE,           // Green solid - Ready/Idle
  LED_IDLE_DATA,      // Green fast blink - Idle with data
  LED_LOW_BATTERY,    // Red solid - Low battery
  LED_ERROR,          // Red fast blink - Error state
  LED_PROFILE,        // Blue blink - Running profile
  LED_AUTO_MODE,      // Yellow blink - Auto mode active
  LED_HOMING,         // Purple blink - Motor homing
  LED_MOTOR_MOVING,   // Purple solid - Motor moving
  LED_PID_CONTROL,    // Cyan blink - PID active
  LED_COMMUNICATION,  // White blink - Communicating
  LED_OTA_MODE        // Orange blink - OTA update mode
};

#endif
