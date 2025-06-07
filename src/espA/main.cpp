/*
*********************************************************************************************************
*                            Code of the ESP32 on the FLOAT board (ESPA)
*
* Filename: main.cpp (formerly EspA.ino)
* Version: 10.0.0
* Developers: Fachechi Gino Marco, Gullotta Salvatore
* Company: Team PoliTOcean @ Politecnico di Torino
* Arduino board package: esp32 by Espressif Systems, v2.0.17
*
* Description:
* Enhanced version with AccelStepper, improved PID control, RGBLed library,
* endstop safety, homing functionality, and ElegantOTA support.
********************************************************************************************************* 
*/

/** INCLUDES **/
#include <Arduino.h>
#include <float_common.h>
#include <MS5837.h>            // Library for pressure sensor (Bar02) control over I2C
// #include <esp_now.h>           // Esp_now library for WiFi connections establishing and management - REMOVED
#include <WiFi.h>              // Layer for proper initialization and setting of ESP32 WiFi module
#include <WiFiClient.h>        // For TCP client handling
#include <Wire.h>              // Library for I2C connection protocol interface
#include <EEPROM.h>            // Library for EEPROM read/write operations
#include <ElegantOTA.h>        // OTA firmware updating (new library)
#include <WebServer.h>         // Web server for OTA (ElegantOTA uses this)
#include <INA.h>               // Library for programming of the INA220 current and voltage monitor
#include <AccelStepper.h>      // Advanced stepper motor control
#include <RGBLed.h>            // RGB LED control library
#include <DebugSerial.h>       // Debug serial library for ESP-NOW forwarding

/** HARDWARE PIN DEFINITIONS **/
const uint8_t  DIR               = 25;         // Pin for motor direction driving
const uint8_t  STEP              = 26;         // Pin for motor step pulses
const uint8_t  EN                = 27;         // Active-low pin for motor enabling
const uint8_t  BUTTON_DOWN       = 16;         // Lower endstop (home position)
const uint8_t  BUTTON_UP         = 17;         // Upper endstop 
const uint8_t  R_PIN             = 19;         // Pin of the RED LED
const uint8_t  G_PIN             = 18;         // Pin of the GREEN LED
const uint8_t  B_PIN             = 5;          // Pin of the BLUE LED
const uint8_t DRV8825_SLEEP = 32;           // Pin for DRV8825 sleep mode
const uint8_t DRV8825_RST = 35;             // Pin for DRV8825 reset

/** MOTOR AND CONTROL CONSTANTS **/
const uint16_t MAX_STEPS         = 1700;       // Number of motor steps for full range
const uint32_t MAX_SPEED         = 1200;        // Maximum motor speed in steps/sec
const uint32_t HOMING_SPEED      = 600;        // Homing speed in steps/sec
const uint16_t ENDSTOP_MARGIN    = 30;         // Safety margin from endstops in steps
const uint32_t HOMING_TIMEOUT    = 5000;      // Homing timeout in milliseconds
const uint16_t ENDSTOP_ACTIVE_STEPS = 150;    // Only check endstops when motor is within this many steps of them

/** TIMING CONSTANTS **/
const uint16_t MEAS_PERIOD       = 100;        // Period between measurements in ms
const uint16_t WRITE_PERIOD      = 2000;       // Period between EEPROM writes in ms
// const uint16_t CONN_CHECK_PERIOD = 500;        // Period between acknowledgements in ms - REPURPOSED for status send
const uint16_t STATUS_SEND_PERIOD = 2000;      // Period for sending status in idle mode
const uint16_t CLIENT_TIMEOUT_FOR_AUTO_MODE = 10000; // ms to wait before auto mode if client disconnects

/** PID CONTROL CONSTANTS **/
float    Kp                = 10.0;       // Proportional gain (increased for underwater response)
float    Ki                = 0.0;        // Integral gain (for steady-state error)
float    Kd                = 250.0;       // Derivative gain (for stability)
const float    PID_OUTPUT_LIMIT  = 80.0;      // Maximum PID output in steps
const float    PID_INTEGRAL_LIMIT = 5.0;     // Anti-windup limit for integral term

/** FLOAT SPECIFIC CONSTANTS **/
const uint8_t  MAX_PROFILES      = 2;          // Number of profiles for maximum points
const int8_t   MAX_TARGET        = -1;         // Encodes the pool bottom as target when given as parameter to the measure() function
const float    FLOAT_LENGTH      = 0.51;       // Length of the FLOAT, measured form the very bottom to the pressure sensor top, expressed in m
const float    MAX_ERROR         = 0.4;        // Error span in which the FLOAT can be considered at target depth during a profile, expressed in m
const float    EPSILON           = 0.01;       // Error span in which two consecutive measures are considered equal, expressed in m
const float    TARGET_DEPTH      = 2.00;        // Target depth to be met and mantained when sinking, expressed in m
const float    STAT_TIME         = 50;          // Time period in which the FLOAT has to maintain TARGET_DEPTH, expressed in s 
const float    MAX_PID_TIME      = 120;          

/** NETWORK CONSTANTS **/
const char*    SSID              = "FLOAT_AP";     // ESP_A WiFi network name
const char*    PASSWORD          = "politocean"; // ESP_A WiFi password
const uint16_t TCP_PORT          = 8888;         // Port for TCP server

/** GLOBAL OBJECTS **/
INA_Class           INA;           // Current/voltage monitor
MS5837              sensor;        // Pressure sensor
// esp_now_peer_info_t peerInfo;      // ESP-NOW peer info - REMOVED
WebServer           http_server(80);    // Web server for OTA (used by ElegantOTA)
WiFiServer          tcpServer(TCP_PORT); // TCP server for commands
WiFiClient          tcpClient;           // Current connected TCP client
AccelStepper        stepper(AccelStepper::DRIVER, STEP, DIR); // Stepper motor controller
RGBLed              led(R_PIN, G_PIN, B_PIN, RGBLed::COMMON_CATHODE); // RGB LED controller

/** MAC ADDRESSES **/
// uint8_t espA_mac[6] = {0x5C, 0x01, 0x3B, 0x2B, 0xA8, 0x00}; // This ESP32 MAC - Informational
// uint8_t broadcastAddress[] = {0x5C, 0x01, 0x3B, 0x2C, 0xE0, 0x68}; // ESPB MAC - REMOVED

/** GLOBAL VARIABLES **/
uint8_t  profile_count    = 0;     // Number of completed profiles
uint8_t  auto_mode_active = 0;     // Auto mode status
// uint8_t  idle             = 0;  // 1 if in idle phase, 0 otherwise - Less relevant, managed by client connection
uint8_t  auto_committed   = 0;     // Auto mode commit flag
uint8_t  deviceNumber     = -1;    // INA220 device number
// int8_t   send_result      = -1;    // Message sending result - REMOVED (TCP has different semantics)
int8_t   status           = 0;     // Main state machine status
uint16_t eeprom_read_ptr  = 0;     // EEPROM read pointer
uint16_t eeprom_write_ptr = 0;     // EEPROM write pointer
uint16_t current_step     = 0;     // Current motor position
uint32_t test_speed       = MAX_SPEED;    // Default test speed in steps/sec
float    atm_pressure     = 0;     // Atmospheric pressure reference
float    depth            = 0;     // Current depth measurement
unsigned long client_last_seen_time = 0; // For auto mode trigger on disconnect

/** ENDSTOP AND SAFETY VARIABLES **/
// Remove all ISR-related variables
bool upper_endstop_hit = false;
bool lower_endstop_hit = false;
volatile bool emergency_stop = false;
bool motor_homed = false;

// Add interrupt flags
volatile bool upper_interrupt_triggered = false;
volatile bool lower_interrupt_triggered = false;

// Add debouncing variables for polling
unsigned long upper_button_pressed_time = 0;
unsigned long lower_button_pressed_time = 0;

/** PID CONTROL VARIABLES **/
float pid_integral = 0.0;
float pid_last_error = 0.0;
float pid_last_depth = 0.0;
unsigned long pid_last_time = 0;

/** LED STATE MANAGEMENT **/
FloatLEDState current_led_state = LED_INIT;
unsigned long led_last_update = 0;

/** DEBUG MODE VARIABLES **/
bool debug_mode_active = false;

/** COMMUNICATION STRUCTURES **/
// input_message  status_to_send;    // Data to send to espB (charge and messages) - Will send strings directly
// output_message command_received;  // Commands received from espB - Will parse from TCP strings

// Use output_message struct for holding parsed command parameters
output_message command_params; // Stores parameters for commands like PARAMS, TEST_FREQ, TEST_STEPS


/** FUNCTION DECLARATIONS **/
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); // REMOVED
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len); // REMOVED
void setLEDState(FloatLEDState state);
void updateLED();
bool homeMotor();
bool safeMoveTo(long targetPosition);
bool safeMoveSteps(long steps);
float calculatePID(float targetDepth, float currentDepth);
void handleEndstopHit();
uint8_t send_message(const char* message, uint32_t timeout_ms); // Timeout less relevant for TCP stream
float f_depth();
void measure(float targetDepth, float time);
void processEndstops(bool is_homing);
void parseCommand(String cmd_str); // New function for parsing TCP commands

/** INTERRUPT SERVICE ROUTINES **/
void IRAM_ATTR upperEndstopISR() {
  upper_interrupt_triggered = true;
}

void IRAM_ATTR lowerEndstopISR() {
  lower_interrupt_triggered = true;
}

/** ENDSTOP POLLING PROCESSING **/
void processEndstops(bool is_homing) {
  unsigned long current_time = millis();
  long current_position = stepper.currentPosition();
  long target_position = stepper.targetPosition();
  bool motor_moving_up = (target_position > current_position);
  bool motor_moving_down = (target_position < current_position);
  
  // --- Upper Endstop Processing ---
  bool check_upper = motor_moving_up && (current_position >= (MAX_STEPS - ENDSTOP_ACTIVE_STEPS));
  
  if (check_upper && upper_interrupt_triggered) { // Use interrupt flag instead of digitalRead
    upper_interrupt_triggered = false; // Clear the flag
    if (!upper_endstop_hit) { // Process only if not already considered hit
      Debug.println("Debounced: Upper Endstop HIT!");
      upper_endstop_hit = true;
      emergency_stop = true;
      stepper.setCurrentPosition(stepper.targetPosition()); // Stop motor immediately
    }
  } else if (!check_upper || !upper_interrupt_triggered) {
    upper_interrupt_triggered = false; // Clear flag if not in checking zone
  }

  // --- Lower Endstop Processing ---
  bool check_lower = is_homing || (motor_moving_down && (current_position <= ENDSTOP_ACTIVE_STEPS));
  
  if (check_lower && lower_interrupt_triggered) { // Use interrupt flag instead of digitalRead
    lower_interrupt_triggered = false; // Clear the flag
    if (!lower_endstop_hit) {
      Debug.println("Debounced: Lower Endstop HIT!");
      lower_endstop_hit = true;
      emergency_stop = true;
      stepper.setCurrentPosition(stepper.targetPosition());
    }
  } else if (!check_lower || !lower_interrupt_triggered) {
    // Button not pressed or not checking, reset debouncing
    lower_interrupt_triggered = false; // Clear flag if not in checking zone
  }
}

/** LED CONTROL FUNCTIONS **/
void setLEDState(FloatLEDState state) {
  current_led_state = state;
  led_last_update = millis();
  
  switch(state) {
    case LED_OFF:
      led.off();
      break;
    case LED_INIT:
      led.setColor(0, 255, 0); // Green
      break;
    case LED_IDLE:
      led.setColor(0, 255, 0); // Green solid
      break;
    case LED_IDLE_DATA:
      led.flash(RGBLed::GREEN, 500); // Green fast blink
      break;
    case LED_LOW_BATTERY:
      led.setColor(255, 0, 0); // Red solid
      break;
    case LED_ERROR:
      led.flash(RGBLed::RED, 500); // Red fast blink
      break;
    case LED_PROFILE:
      led.setColor(RGBLed::BLUE); // Blue solid
      break;
    case LED_AUTO_MODE:
      led.flash(RGBLed::YELLOW, 750); // Yellow blink
      break;
    case LED_HOMING:
      led.flash(RGBLed::MAGENTA, 500); // Purple blink
      break;
    case LED_MOTOR_MOVING:
      led.setColor(255, 0, 255); // Purple solid
      break;
    case LED_PID_CONTROL:
      led.flash(RGBLed::CYAN, 600); // Cyan blink
      break;
    case LED_COMMUNICATION:
      led.flash(RGBLed::WHITE, 500); // White blink
      break;
    case LED_OTA_MODE:
      led.flash(RGBLed::ORANGE, 500); // Orange blink
      break;
  }
}

void updateLED() {
  // Handle battery status override
  // Read battery charge for LED update if needed (original code had status_to_send.charge)
  // For simplicity, let's assume battery check for LED is handled if status_to_send.charge is updated elsewhere
  // or we read it here if critical.
  // if (INA.getBusMilliVolts(deviceNumber) < BATT_THRESH && current_led_state != LED_LOW_BATTERY) {
  //   setLEDState(LED_LOW_BATTERY);
  //   return;
  // }
  
  // Update LED animation
  led.update();
}

/** MOTOR CONTROL AND SAFETY FUNCTIONS **/
bool homeMotor() {
  Debug.println("Starting homing sequence...");
  setLEDState(LED_HOMING);
  
  motor_homed = false;
  // Reset all endstop states before starting a move
  upper_endstop_hit = false;
  lower_endstop_hit = false;
  emergency_stop = false;
  
  // Enable motor
  stepper.enableOutputs();
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.setAcceleration(HOMING_SPEED / 2);
  
  // Move towards lower endstop (home position)
  Debug.println("Homing: Moving towards lower endstop...");
  stepper.move(-MAX_STEPS * 2); // Move more than full range to ensure hitting endstop
  
  unsigned long startTime = millis();
  
  // Wait for lower_endstop_hit or timeout or other emergency
  while (!lower_endstop_hit && (millis() - startTime) < HOMING_TIMEOUT && !emergency_stop) {
    stepper.run();
    processEndstops(true); // Pass true for homing mode
    updateLED();
    yield();
  }
  
  // If loop exited due to timeout or non-lower_endstop_hit emergency, ensure motor is stopped
  if (!lower_endstop_hit && !emergency_stop) { 
      stepper.stop(); 
  }
  
  delay(100); // Allow motor to settle
  
  if (lower_endstop_hit) {
    Debug.println("Homing: Lower endstop found and debounced.");
    handleEndstopHit(); // Back off from the endstop, sets position to ENDSTOP_MARGIN

    stepper.setCurrentPosition(0); // Define this backed-off position as HOME (0)
    current_step = 0;
    motor_homed = true;
    emergency_stop = false; // Homing successful, clear emergency for next operations
    
    Debug.println("Homing completed successfully");
    stepper.disableOutputs();
    return true;
  } else {
    Debug.println("Homing failed - timeout or other emergency stop");
    if (emergency_stop && upper_endstop_hit) {
        Debug.println("Homing failed: Upper endstop hit unexpectedly.");
        handleEndstopHit(); // Handle the unexpected upper hit (backs off, sets position)
    }
    // emergency_stop remains true if it was set by an endstop
    stepper.disableOutputs();
    motor_homed = false;
    return false;
  }
}

bool safeMoveTo(long targetPosition) {
  if (!motor_homed) {
    Debug.println("Motor not homed - cannot move safely");
    return false;
  }
  
  // Clamp target position to safe range
  if (targetPosition < ENDSTOP_MARGIN) {
    targetPosition = ENDSTOP_MARGIN;
  }
  if (targetPosition > (MAX_STEPS - ENDSTOP_MARGIN)) {
    targetPosition = MAX_STEPS - ENDSTOP_MARGIN;
  }
  
  // Reset all endstop states before starting a move
  upper_endstop_hit = false;
  lower_endstop_hit = false;
  emergency_stop = false;
  
  stepper.enableOutputs();
  stepper.setMaxSpeed(MAX_SPEED); // Ensure normal operating speed
  stepper.setAcceleration(MAX_SPEED / 2);
  stepper.moveTo(targetPosition);
  
  Debug.printf("safeMoveTo: Moving to %ld\n", targetPosition);
  // Run until target reached or emergency stop
  while (stepper.distanceToGo() != 0 && !emergency_stop) {
    stepper.run();
    processEndstops(false); // Use normal mode (not homing)
    updateLED();
    yield();
  }
  
  // If loop exited due to emergency_stop, an endstop was hit and debounced
  if (emergency_stop) {
    Debug.println("safeMoveTo: Emergency stop triggered by endstop.");
    handleEndstopHit(); // Back off from the endstop, updates position
    current_step = stepper.currentPosition(); // Update current_step after backoff
    // stepper.disableOutputs();
    return false; // Movement failed
  }
  
  // If loop exited because distanceToGo is 0 (successful move)
  current_step = stepper.currentPosition();
  // stepper.disableOutputs();
  Debug.printf("Motor moved to position: %ld\n", current_step);
  return true;
}

bool safeMoveSteps(long steps) {
  long newPosition = stepper.currentPosition() + steps;
  return safeMoveTo(newPosition);
}

void handleEndstopHit() {
  Debug.println("handleEndstopHit: Processing debounced endstop.");
  stepper.enableOutputs(); // Ensure motor is enabled for backing off

  if (upper_endstop_hit) {
    Debug.println("handleEndstopHit: Upper endstop - backing away.");

    stepper.setAcceleration(MAX_SPEED / 2); 
    stepper.move(-ENDSTOP_MARGIN); // Relative move
    unsigned long backoffStartTime = millis();
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      updateLED(); // Keep LED updated
      yield();
      if (millis() - backoffStartTime > 2000) { // Timeout for backoff
          Debug.println("handleEndstopHit: Timeout backing off upper endstop!");
          stepper.stop();
          break;
      }
    }
    stepper.setCurrentPosition(MAX_STEPS - ENDSTOP_MARGIN); // Define position after backoff
    current_step = stepper.currentPosition();
    Debug.printf("handleEndstopHit: Backed off upper. New pos: %d\n", current_step);
    
    upper_endstop_hit = false; // Clear the state now that we've handled it
  }
  
  if (lower_endstop_hit) {
    Debug.println("handleEndstopHit: Lower endstop - backing away.");

    stepper.setAcceleration(HOMING_SPEED / 2); // Use homing accel for consistency
    stepper.move(ENDSTOP_MARGIN);
    unsigned long backoffStartTime = millis();
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      updateLED();
      yield();
      if (millis() - backoffStartTime > 2000) { // Timeout for backoff
          Debug.println("handleEndstopHit: Timeout backing off lower endstop!");
          stepper.stop();
          break;
      }
    }
    stepper.setCurrentPosition(ENDSTOP_MARGIN); 
    current_step = stepper.currentPosition();
    Debug.printf("handleEndstopHit: Backed off lower. New pos: %d\n", current_step);

    lower_endstop_hit = false; // Clear the state
  }
  // stepper.disableOutputs(); // Disable motor after backing off
}

/** IMPROVED PID CONTROL **/
float calculatePID(float targetDepth, float currentDepth) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - pid_last_time) / 1000.0; // Convert to seconds
  
  if (deltaTime <= 0) deltaTime = MEAS_PERIOD / 1000.0;
  
  // Calculate error
  float error = targetDepth - currentDepth;
  
  // Proportional term
  float proportional = Kp * error;
  
  // Integral term with anti-windup
  pid_integral += error * deltaTime;
  if (pid_integral > PID_INTEGRAL_LIMIT) pid_integral = PID_INTEGRAL_LIMIT;
  if (pid_integral < -PID_INTEGRAL_LIMIT) pid_integral = -PID_INTEGRAL_LIMIT;
  float integral = Ki * pid_integral;
  
  // Derivative term (based on depth change for better underwater performance)
  float depth_derivative = (pid_last_depth - currentDepth) / deltaTime;
  float derivative = Kd * depth_derivative;
  
  // Calculate total output
  float output = proportional + integral + derivative;
  
  // Limit output
  if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
  if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;
  
  // Update for next iteration
  pid_last_error = error;
  pid_last_depth = currentDepth;
  pid_last_time = currentTime;
  
  // Debug output
  Debug.printf("PID: Target=%.2f, Current=%.2f, Error=%.2f, Output=%.2f\n", 
                targetDepth, currentDepth, error, output);
  
  return output;
}

/** COMMUNICATION FUNCTIONS **/
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // REMOVED
//   send_result = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
// }

// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { // REMOVED
//   memcpy(&command_received, incomingData, sizeof(command_received));
// }

uint8_t send_message(const char* message, uint32_t timeout_ms) {
  // timeout_ms is largely ignored for TCP stream, but kept for compatibility with DebugSerial
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(message);
    // Serial.printf("TCP SENT: %s\n", message); // Optional: for local debugging
    return 1; // Success
  }
  return 0; // Failure (no client or not connected)
}

/*
*********************************************************************************************************
*                                           f_depth()
*
* Description : Utility function that uses Stevino rule to calculate a depth value starting from 
*               the pressure measured at setup and the last pressure value given by 
*               the pressure sensor. It also adjusts the depth value by adding the FLOAT
*               length to it, in order to mock the position of the pressure sensor, which is
*               at the top of the robot.
*
* Argument(s) : none
*
* Return(s)   : The depth value calculated, expressed in meters.
*********************************************************************************************************
*/
float f_depth () {
  depth = (sensor.pressure(MS5837::Pa)-atm_pressure)/(997*9.80665); // Stevino rule to calculate depth. 
//                                                                     Density: 997Kg/m^3 for fresh water, 1029Kg/m^3 for sea water
  return depth + FLOAT_LENGTH;                                      // Adds the FLOAT length to the result
}

float f_depth(float pressure) {
  return (pressure - atm_pressure) / (997 * 9.80665) + FLOAT_LENGTH;
}

/** PARSE COMMAND FUNCTION **/
void parseCommand(String cmd_str) {
    cmd_str.trim();
    Debug.printf("Received command: %s\n", cmd_str.c_str());

    // Reset previous command parameters
    memset(&command_params, 0, sizeof(command_params));
    int new_status = 0; // Default to idle if command is not recognized

    if (cmd_str.equalsIgnoreCase("GO")) new_status = 1;
    else if (cmd_str.equalsIgnoreCase("SEND_DATA")) new_status = 2;
    else if (cmd_str.equalsIgnoreCase("BALANCE")) new_status = 3;
    else if (cmd_str.equalsIgnoreCase("CLEAR_EEPROM")) new_status = 4;
    else if (cmd_str.equalsIgnoreCase("SWITCH_AUTO_MODE")) new_status = 5;
    else if (cmd_str.equalsIgnoreCase("SEND_PACKAGE")) new_status = 6;
    else if (cmd_str.equalsIgnoreCase("OTA_UPDATE")) new_status = 7;
    else if (cmd_str.startsWith("PARAMS")) {
        new_status = 8;
        // sscanf is safer with C-strings
        int parsed_count = sscanf(cmd_str.c_str(), "PARAMS %f %f %f", &command_params.params[0], &command_params.params[1], &command_params.params[2]);
        if (parsed_count != 3) {
            Debug.println("Invalid PARAMS format. Expected: PARAMS Kp Ki Kd");
            new_status = 0; // Revert to idle on parse error
        }
    } else if (cmd_str.startsWith("TEST_FREQ")) {
        new_status = 9;
        int parsed_count = sscanf(cmd_str.c_str(), "TEST_FREQ %u", &command_params.freq);
        if (parsed_count != 1) {
            Debug.println("Invalid TEST_FREQ format. Expected: TEST_FREQ value");
            new_status = 0;
        }
    } else if (cmd_str.startsWith("TEST_STEPS")) {
        new_status = 10;
        int parsed_count = sscanf(cmd_str.c_str(), "TEST_STEPS %ld", &command_params.steps);
         if (parsed_count != 1) {
            Debug.println("Invalid TEST_STEPS format. Expected: TEST_STEPS value");
            new_status = 0;
        }
    } else if (cmd_str.equalsIgnoreCase("DEBUG_MODE")) new_status = 11;
    else if (cmd_str.equalsIgnoreCase("HOME")) new_status = 12;
    else if (cmd_str.equalsIgnoreCase("STATUS_REQ")) {
        // Client requests status. Idle loop (status 0) sends it periodically.
        // We can force an immediate send if needed, or just let idle loop handle it.
        // For now, this command just ensures we stay/go to idle.
        new_status = 0; 
    }
    else {
        Debug.printf("Unknown command: %s\n", cmd_str.c_str());
        new_status = 0; // Go to idle
    }
    
    if (new_status != 0) { // If a valid command is to be processed
        status = new_status;
        auto_committed = 0; // Reset auto_committed flag for new commands from client
        setLEDState(LED_COMMUNICATION); // Indicate command processing
    } else if (status != 0 && new_status == 0) { // If command was invalid or STATUS_REQ, go to idle
        status = 0;
    }
    // If status was already 0 and new_status is 0, it remains 0.
}

/** MAIN MEASUREMENT AND CONTROL FUNCTION **/
void measure(float targetDepth, float time) {
  Debug.printf("Starting measurement: target=%.2f, time=%.2f\n", targetDepth, time);
  
  if (targetDepth == FLOAT_LENGTH || targetDepth == MAX_TARGET) {
    setLEDState(LED_PROFILE);
  } else {
    setLEDState(LED_PID_CONTROL);
    // Reset PID controller
    pid_integral = 0.0;
    pid_last_error = 0.0;
    pid_last_depth = 0.0;
    pid_last_time = millis();
  }
  
  uint64_t start_time = millis();
  uint64_t last_measurement = 0;
  uint64_t last_write = 0;
  bool motor_stopped = false;
  
  while (true) {
    updateLED();
    
    // Take measurement every MEAS_PERIOD
    if (millis() - last_measurement >= MEAS_PERIOD) {
      sensor.read();
      depth = f_depth();
      last_measurement = millis();
      
      // Handle special target cases
      if (targetDepth == FLOAT_LENGTH || targetDepth == MAX_TARGET) {
        if (!motor_stopped) {
          if (targetDepth == MAX_TARGET) {
            // Move to maximum depth (full extension)
            safeMoveTo(MAX_STEPS - ENDSTOP_MARGIN);
          } else {
            // Move to surface (home position)
            safeMoveTo(ENDSTOP_MARGIN);
          }
          motor_stopped = true;
        }
      } else {
        // PID control for target depth
        float pidOutput = calculatePID(targetDepth, depth);
        
        if (abs(pidOutput) > 1.0) { // Only move if output is significant
          long steps = (long)pidOutput;
          if (safeMoveSteps(steps)) {
            current_step = stepper.currentPosition();
          }
        }
      }
      
      // Write to EEPROM periodically
      if (millis() - last_write >= WRITE_PERIOD) {
        // Write sensor data to EEPROM
        sensor_data data;
        data.pressure = sensor.pressure(MS5837::Pa);
        data.temperature = sensor.temperature();
        if (targetDepth != MAX_TARGET && targetDepth != FLOAT_LENGTH)
          data.temperature = 100;
        
        // Write to EEPROM (implementation depends on your data format)
        EEPROM.put(eeprom_write_ptr, data);
        eeprom_write_ptr += sizeof(sensor_data);
        if (eeprom_write_ptr >= EEPROM_SIZE - sizeof(sensor_data)) {
          eeprom_write_ptr = 0; // Wrap around
        }
        EEPROM.commit();
        
        last_write = millis();
      }
      
      // Check for depth stability (stationary detection)
      static float last_depth_check = 0;
      static int stable_count = 0;

      if (targetDepth == MAX_TARGET) {  // case for max depth
        if (abs(depth - last_depth_check) < EPSILON && depth>FLOAT_LENGTH+0.1) {
          stable_count++;
          if (stable_count >= (time * 1000 / MEAS_PERIOD)) {
            Debug.println("Float is stationary at bottom - profile complete");
            break;
          }
        } else {
          stable_count = 0;
        }
      }
      else if (targetDepth == FLOAT_LENGTH) {
        if (abs(depth - targetDepth) < EPSILON) {
          stable_count++;
          if (stable_count >= (time * 1000 / MEAS_PERIOD)) {
            Debug.println("Float is stationary at surface - profile complete");
            break;
          }
        }
      }
      else {                                            // case for target depth
        if (abs(depth - targetDepth) < MAX_ERROR) {
          stable_count++;
          if (stable_count >= (time * 1000 / MEAS_PERIOD)) {
            Debug.println("Profile at target depth complete");
            break;
          }
        }
      }

      last_depth_check = depth;
      if (millis()-start_time > MAX_PID_TIME*1000)  //timeout
        break; 
    }
    
    // Allow other tasks to run
    yield();
  }
  
  // Stop motor and return to idle LED state
  stepper.stop();
  stepper.disableOutputs();
  
  Debug.println("Measurement completed");
}

/** SETUP FUNCTION **/
void setup() {
  delay(100);
  Serial.begin(115200); 
  Serial.println("Float ESPA v10.0 - Starting initialization...");
  
  // Initialize LED early for status feedback
  setLEDState(LED_INIT);
  
  // Initialize motor pins
  pinMode(EN, OUTPUT);
  pinMode(DRV8825_SLEEP, OUTPUT);
  pinMode(DRV8825_RST, OUTPUT);
  digitalWrite(EN, HIGH); // Start with motor disabled
  digitalWrite(DRV8825_SLEEP, HIGH); // Enable driver by default
  digitalWrite(DRV8825_RST, HIGH);   // Keep driver out of reset by default

  
  // Initialize endstop pins with interrupts
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  
  // Attach interrupts for falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(BUTTON_UP), upperEndstopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DOWN), lowerEndstopISR, FALLING);
  
  // Initialize stepper motor
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_SPEED / 2);
  stepper.setEnablePin(EN);
  stepper.setPinsInverted(true, false, true); // Enable pin is active low
  stepper.disableOutputs();
  
  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
    setLEDState(LED_ERROR);
    while(1) delay(1000);
  }
  eeprom_write_ptr = 0;
  eeprom_read_ptr = 0;
  
  // Initialize WiFi for AP and TCP Server
  Serial.printf("Setting up AP: %s\n", SSID);
  WiFi.softAP(SSID, PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("AP IP address: %s\n", apIP.toString().c_str());
  Serial.printf("TCP Server started on port %d\n", TCP_PORT);
  tcpServer.begin();
  
  // Initialize ESP-NOW - REMOVED
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  //   setLEDState(LED_ERROR);
  //   while(1) delay(1000);
  // }
  
  // esp_now_register_send_cb(OnDataSent); // REMOVED
  // esp_now_register_recv_cb(OnDataRecv); // REMOVED
  
  // Add peer - REMOVED
  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 0;
  // peerInfo.encrypt = false;
  
  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add peer");
  //   setLEDState(LED_ERROR);
  //   while(1) delay(1000);
  // }

  // Initialize DebugSerial library
  Debug.begin(&debug_mode_active, send_message); // Pass the new TCP send_message
  Debug.println("Debug serial initialized - ready for remote debugging over TCP");
  
  // Initialize I2C and find INA220
  Wire.begin();
  
  for (uint8_t j = 0; j < 3; j++) {
    uint8_t devicesFound = INA.begin(5, 100000); // 5A max, 0.1 ohm shunt
    for (uint8_t i = 0; i < devicesFound; i++) {
      if (strcmp(INA.getDeviceName(i), "INA219") == 0) {
        deviceNumber = i;
        INA.setMode(INA_MODE_CONTINUOUS_BUS, deviceNumber);
        break;
      }
    }
    if (deviceNumber != UINT8_MAX) break;
    Debug.println("No INA found. Waiting 2s and retrying...");
    delay(2000);
  }
  
  if (deviceNumber == UINT8_MAX) {
    Debug.println("Failed to find INA220");
    setLEDState(LED_ERROR);
    while(1) delay(1000);
  }
  
  // Initialize pressure sensor
  sensor.setModel(1);
  uint64_t sensor_timeout = millis();
  while (!sensor.init()) {
    if (millis() - sensor_timeout > 5000) {
      sensor_timeout = millis();
      Debug.println("Sensor init failed! Check SDA/SCL connections");
      Debug.println("Blue Robotics Bar02: White=SDA, Green=SCL");
    }
    setLEDState(LED_ERROR);
    updateLED();
    delay(100);
  }
  
  sensor.setFluidDensity(997); // Freshwater density
  sensor.read();
  atm_pressure = sensor.pressure(MS5837::Pa);
  Debug.printf("Atmospheric pressure: %.2f Pa\n", atm_pressure);
  
  // Perform motor homing
  Debug.println("Starting motor homing...");
  if (!homeMotor()) {
    Debug.println("CRITICAL: Motor homing failed!");
    setLEDState(LED_ERROR);
    while(1) {
      updateLED();
      delay(1000);
    }
  }
  
  Debug.println("Initialization complete - Float ready!");
}

/** MAIN LOOP **/
void loop() {
  // Handle ElegantOTA client connections if OTA is active.
  // Case 7's loop will call http_server.handleClient() and ElegantOTA.loop().
  // So, no need for ElegantOTA.loop() here at the top level of the main loop.

  // Handle TCP Client Connection & Data
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      if (tcpClient) {
        Debug.println("Old client disconnected.");
        tcpClient.stop();
      }
      tcpClient = newClient;
      Debug.printf("New client connected from: %s\n", tcpClient.remoteIP().toString().c_str());
      client_last_seen_time = millis(); // Update last seen time
      // status = 0; // Ensure idle state on new connection if not already
    }
  }

  if (tcpClient && tcpClient.connected()) {
    client_last_seen_time = millis(); // Update last seen time while client is connected
    if (tcpClient.available() > 0) {
      String command_str = tcpClient.readStringUntil('\n');
      if (command_str.length() > 0) {
        parseCommand(command_str); // This updates global 'status'
      }
    }
  } else { // No client or client disconnected
    if (status != 0 && status != 7) { // If not idle or in OTA mode
        // Debug.println("Client disconnected, returning to idle.");
    }
    // status = 0; // Go to idle if client disconnects (handled by auto mode check too)
    // command_params.command = 0; // Clear any pending command

    // Auto mode activation on prolonged client disconnection
    if (auto_mode_active && profile_count < MAX_PROFILES && status == 0) { // Only if idle
        if (millis() - client_last_seen_time > CLIENT_TIMEOUT_FOR_AUTO_MODE) {
            Debug.println("Client disconnected for too long - activating auto mode");
            status = 1; // Go to profile execution
            auto_committed = 1; // Mark as auto-triggered
            // client_last_seen_time = millis(); // Reset timer or handle state change
        }
    }
    if (status == 0 && !auto_committed) { // If truly idle and not about to run auto_mode
         // setLEDState(LED_ERROR); // Indicate connection issue if desired, or specific "disconnected" LED
    }
  }
  
  updateLED();

  // Process endstops, especially if motor is idle, to catch unexpected hits.
  // Movement functions also call this, but this is a safety net.
  if (stepper.distanceToGo() == 0 && !emergency_stop) { // If motor is supposed to be idle
      processEndstops(false); // Normal mode, not homing
      if (emergency_stop) { // An endstop was hit while idle
          Debug.println("Endstop hit while motor was idle!");
          handleEndstopHit(); // Back off
          setLEDState(LED_ERROR); 
      }
  }
  
  switch (status) {
    case 0: // Idle
      {
        static unsigned long last_status_send_time = 0;
        // uint8_t result; // Not used in the same way
        // auto_committed = 0; // Reset if entering idle, unless set by auto mode trigger
        
        if (tcpClient && tcpClient.connected()) {
            if (millis() - last_status_send_time > STATUS_SEND_PERIOD) {
                INA.waitForConversion(deviceNumber);
                uint16_t current_charge = INA.getBusMilliVolts(deviceNumber);
                
                char status_msg[128];
                const char* auto_status_str = auto_mode_active ? "ON" : "OFF";
                const char* data_status_str = (eeprom_write_ptr > eeprom_read_ptr) ? "IDLE_W_DATA" : "IDLE";

                if (eeprom_write_ptr > eeprom_read_ptr) {
                    setLEDState(LED_IDLE_DATA);
                } else {
                    setLEDState(LED_IDLE);
                }
                sprintf(status_msg, "STATUS:%s|CHARGE:%u|AUTO:%s|PROFILE_COUNT:%d", 
                        data_status_str, current_charge, auto_status_str, profile_count);
                send_message(status_msg, 0);
                last_status_send_time = millis();
            }
        } else {
            // If no client, LED might indicate error or waiting state
            // Auto mode is handled outside the switch if client is disconnected
            if (!auto_mode_active || profile_count >= MAX_PROFILES) { // If auto mode cannot run
                 setLEDState(LED_ERROR); // e.g. Red blink for no client and no auto mode
            } else if (auto_mode_active && status == 0) { // Eligible for auto mode but not yet triggered
                 setLEDState(LED_AUTO_MODE); // Blinking yellow waiting for timeout
            }
        }
        // If a command is received via TCP, parseCommand will change 'status'
        // and the loop will break out of case 0 in the next iteration.
      }
      break;
      
    case 1: // GO - Execute profile
      {
        // uint8_t result; // Not used for TCP ACK in the same way
        
        if (auto_committed) {
          setLEDState(LED_AUTO_MODE);
          Debug.println("Auto mode: Starting profile execution");
          // result = 1; // Auto mode doesn't need acknowledgment from a client
        } else {
          send_message("ACK:GO_RECEIVED", 0); // Send ACK to client
          // result = 1; // Assume client got it if send_message succeeded (client connected)
        }
        
        // if (result) { // Assuming command should proceed
        Debug.println("Starting profile execution");

        eeprom_read_ptr = 0; 
        eeprom_write_ptr = 0; 
        
        Debug.println("Phase 0: Descending to target depth");
        measure(TARGET_DEPTH, STAT_TIME); 
        delay(500); 
        Debug.println("Phase 1: Descending to maximum depth (pool bottom)");
        measure(MAX_TARGET, 3); 
        delay(500); 
        Debug.println("Phase 2: Ascending to surface");
        measure(FLOAT_LENGTH, 3); 
        
        stepper.disableOutputs(); 
        
        profile_count++;
        Debug.printf("Profile %d completed\n", profile_count);
        send_message("INFO:PROFILE_COMPLETE", 0);
        // }
        
        status = 0; // Return to idle
        auto_committed = 0; // Reset auto_committed after profile execution
      }
      break;
      
    case 2: // SEND_DATA - Send stored sensor data to Control Station
      {
        send_message("ACK:SEND_DATA_RECEIVED", 0);
        char line[OUTPUT_LEN];
        sensor_data data;
        uint16_t packet_count = 0;
        
        Debug.println("Sending stored data to Control Station");
        
        while (eeprom_read_ptr + sizeof(sensor_data) <= eeprom_write_ptr) {
          // Read struct from EEPROM
          EEPROM.get(eeprom_read_ptr, data);
          eeprom_read_ptr += sizeof(sensor_data);
          
          // Format the JSON string
          snprintf(line, OUTPUT_LEN,
                   "{\"company_number\":\"EX16\",\"pressure\":\"%.2f\",\"depth\":\"%.2f\",\"temperature\":\"%.2f\",\"mseconds\":\"%d\"}",
                   data.pressure, f_depth(data.pressure), data.temperature, packet_count*WRITE_PERIOD);
          
          delay(50); // Slow down sending rate
          send_message(line, 0); // Send data line to CS
          packet_count++;
        }
        
        // strcpy(line, "STOP_DATA"); // Use send_message directly
        send_message("INFO:STOP_DATA_TRANSMISSION", 0); // Signal end of data transmission
        
        Debug.println("Data transmission completed");
        status = 0; // Return to idle
      }
      break;
      
    case 3: // BALANCE - Move syringes top then bottom usefult to fill them up with water
      {
        send_message("ACK:BALANCE_RECEIVED",0);
        // uint8_t result = send_message(CMD3_ACK, 1000);
        // if (result) {
        Debug.println("Executing balance command");
        safeMoveTo(MAX_STEPS - ENDSTOP_MARGIN);
        delay(5000);
        safeMoveTo(ENDSTOP_MARGIN);

        // }
        status = 0;
      }
      break;
      
    case 4: // CLEAR_EEPROM - Clear EEPROM data (renamed from CLEAR_SD)
      {
        send_message("ACK:CLEAR_EEPROM_RECEIVED", 0);
        // uint8_t result = send_message(CMD4_ACK, 1000);
        // if (result) {
        Debug.println("Clearing EEPROM data");
        eeprom_write_ptr = 0;
        eeprom_read_ptr = 0;
        // Clear first few bytes to mark as empty
        for (int i = 0; i < 100; i++) {
          EEPROM.write(i, 0);
        }
        EEPROM.commit();
        // }
        status = 0;
      }
      break;
      
    case 5: // SWITCH_AUTO_MODE - Toggle auto mode
      {
        send_message("ACK:SWITCH_AUTO_MODE_RECEIVED", 0);
        // uint8_t result = send_message(CMD5_ACK, 1000);
        // if (result) {
        auto_mode_active = !auto_mode_active;
        Debug.printf("Auto mode: %s\n", auto_mode_active ? "ENABLED" : "DISABLED");
        char auto_msg[50];
        sprintf(auto_msg, "INFO:AUTO_MODE_%s", auto_mode_active ? "ENABLED" : "DISABLED");
        send_message(auto_msg, 0);
        if (auto_mode_active)
            setLEDState(LED_AUTO_MODE);
        else
            setLEDState(LED_IDLE); // Or current appropriate state
        // }
        status = 0;
      }
      break;
      
    case 6: // SEND_PACKAGE - Send single test packet
      {
        send_message("ACK:SEND_PACKAGE_RECEIVED", 0);
        char package[OUTPUT_LEN];
        
        Debug.println("Sending test package");
        sensor.read(); // Update sensor readings
        
        // Format test package with current sensor data
        snprintf(package, OUTPUT_LEN,
                "{\"company_number\":\"EX16\",\"pressure\":\"%.2f\",\"depth\":\"%.2f\",\"temperature\":\"%.2f\",\"mseconds\":\"%d\"}",
                sensor.pressure(MS5837::Pa), f_depth(), sensor.temperature(), millis());
        
        send_message(package, 0); // Send test package (acknowledgment is the package itself)
        
        Debug.println("Test package sent");
        status = 0;
      }
      break;
      
    case 7: // OTA_UPDATE - Start OTA server (renamed from TRY_UPLOAD)
      {
        send_message("ACK:OTA_UPDATE_RECEIVED", 0);
        // uint8_t result = send_message(CMD7_ACK, 1000);
        // if (result) {
        setLEDState(LED_OTA_MODE);
        Debug.println("Starting OTA server...");
        Debug.printf("OTA Server: http://%s/update\n", WiFi.softAPIP().toString().c_str());

        bool original_debug_mode_state = debug_mode_active;
        if (original_debug_mode_state) {
            Debug.println("Temporarily disabling remote debug for OTA.");
            debug_mode_active = false; 
        }

        // ESP-NOW de-init and WiFi mode changes for ESP-NOW are removed.
        // The AP is already running. ElegantOTA will use it.
        
        // Start ElegantOTA
        ElegantOTA.begin(&http_server); // Pass WebServer instance
        http_server.begin();
            
        unsigned long ota_start_time = millis();
        Debug.println("OTA server running for 5 minutes...");
        send_message("INFO:OTA_SERVER_RUNNING_5_MIN",0);

        while (millis() - ota_start_time < 300000) { // 5 minutes
            http_server.handleClient(); // Handle HTTP requests for OTA
            ElegantOTA.loop();      // Process ElegantOTA
            updateLED();            // Keep LED status updated
            delay(10);
        }
          
        Debug.println("OTA period finished");
        send_message("INFO:OTA_PERIOD_FINISHED",0);
        http_server.stop();
        // WiFi AP remains active for the TCP client. No need to disconnect/reconnect AP.

        // ESP-NOW re-initialization is removed.
        if (original_debug_mode_state) {
            debug_mode_active = true; 
            Debug.println("Remote debug re-enabled after OTA process.");
        }
        setLEDState(LED_IDLE); // Or appropriate state
        // }
        status = 0;
      }
      break;
      
    case 8: // UPDATE_PID - Update PID parameters
      {
        send_message("ACK:PARAMS_RECEIVED", 0);
        // uint8_t result = send_message(CMD8_ACK, 1000);
        // if (result) {
        // Update PID parameters from received command_params
        Kp = command_params.params[0];
        Ki = command_params.params[1]; 
        Kd = command_params.params[2];
          
        Debug.printf("PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
        char pid_msg[100];
        sprintf(pid_msg, "INFO:PID_UPDATED Kp=%.2f Ki=%.2f Kd=%.2f", Kp, Ki, Kd);
        send_message(pid_msg, 0);
        // }
        status = 0;
      }
      break;
      
    case 9: // TEST_FREQ - Set test frequency
      {
        send_message("ACK:TEST_FREQ_RECEIVED", 0);
        // uint8_t result = send_message(CMD9_ACK, 1000);
        // if (result) {
        // Limit frequency to safe range
        if (command_params.freq > 1200) command_params.freq = 1200;
        if (command_params.freq < 10) command_params.freq = 10;
        test_speed = command_params.freq;
        Debug.printf("Test frequency set to: %u Hz\n", test_speed); // %u for uint32_t
        char freq_msg[50];
        sprintf(freq_msg, "INFO:TEST_FREQ_SET %u", test_speed);
        send_message(freq_msg, 0);
        // }
        status = 0;
      }
      break;
      
    case 10: // TEST_STEPS - Execute test movement
      {
        send_message("ACK:TEST_STEPS_RECEIVED", 0);
        // uint8_t result = send_message(CMD10_ACK, 1000);
        // if (result) {
        Debug.printf("Executing test movement: %ld steps\n", command_params.steps);
          
        // Set motor speed based on test frequency
        stepper.setMaxSpeed(test_speed);
        
        stepper.enableOutputs();
        stepper.setAcceleration(test_speed / 2);
        stepper.move(command_params.steps);
        
        Debug.printf("Moving to %ld\n", command_params.steps);
        // Run until target reached or emergency stop
        while (stepper.distanceToGo() != 0) {
          stepper.run();
          //processEndstops(false); // Use normal mode (not homing)
          updateLED();
          yield();
        }

        stepper.disableOutputs();
        
        // Restore normal motor speed
        stepper.setMaxSpeed(MAX_SPEED);
        // }
        status = 0;
      }
      break;
      
    case 11: // DEBUG_MODE - Toggle debug mode (send serial output over ESP-NOW)
      {
        send_message("ACK:DEBUG_MODE_RECEIVED", 0);
        // uint8_t result = send_message(CMD11_ACK, 1000);
        // if (result) {
        debug_mode_active = !debug_mode_active;
        if (debug_mode_active) {
            Debug.println("DEBUG MODE ACTIVATED - Serial output will be forwarded over TCP");
            send_message("INFO:DEBUG_MODE_ACTIVATED",0);
        } else {
            // Debug.println("DEBUG MODE DEACTIVATED - Serial output local only"); // This would be sent via TCP if still active
            send_message("INFO:DEBUG_MODE_DEACTIVATED",0); // Send this before disabling
            Serial.println("DEBUG MODE DEACTIVATED - Serial output local only"); // Local confirmation
        }
        // }
        status = 0;
      }
      break;
      
    case 12: // HOME - Manual homing command
      {
        send_message("ACK:HOME_RECEIVED", 0);
        Debug.println("Manual homing requested");
        
        if (homeMotor()) {
          Debug.println("Manual homing completed successfully");
          send_message("INFO:HOMING_SUCCESS", 0);
        } else {
          Debug.println("Manual homing failed");
          send_message("ERROR:HOMING_FAILED", 0);
        }
        
        status = 0;
      }
      break;
      
    default:
      Debug.printf("Unknown command: %d\n", status);
      status = 0;
      break;
  }
  
  delay(10);
}
