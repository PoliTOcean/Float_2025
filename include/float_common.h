#ifndef FLOAT_COMMON_H
#define FLOAT_COMMON_H

#include <Arduino.h>

// Communication protocol defines
#define OUTPUT_LEN      250-sizeof(uint16_t) // Length of the output on the MAC layer
#define EEPROM_SIZE     4096                 // EEPROM allocation size in bytes
#define BATT_THRESH     11500                // Battery threshold in mV for low battery warning

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
  uint8_t command = 0;
} output_message;

// MAC addresses - UPDATE THESE TO YOUR ACTUAL MAC ADDRESSES
extern uint8_t espA_mac[6];
extern uint8_t espB_mac[6];

// LED States for better status indication
enum FloatLEDState {
  LED_OFF,
  LED_INIT,           // Green blink - Initializing
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
