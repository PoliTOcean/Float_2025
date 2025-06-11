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
#include <esp_now.h>           // Esp_now library for WiFi connections establishing and management
#include <WiFi.h>              // Layer for proper initialization and setting of ESP32 WiFi module
#include <Wire.h>              // Library for I2C connection protocol interface
#include <EEPROM.h>            // Library for EEPROM read/write operations
#include <ElegantOTA.h>        // OTA firmware updating (new library)
#include <WebServer.h>         // Web server for OTA
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
const uint32_t MAX_SPEED         = 200;        // Maximum motor speed in steps/sec
const uint32_t HOMING_SPEED      = 300;        // Homing speed in steps/sec
const uint16_t ENDSTOP_MARGIN    = 30;         // Safety margin from endstops in steps
const uint32_t HOMING_TIMEOUT    = 5000;      // Homing timeout in milliseconds
const uint16_t ENDSTOP_ACTIVE_STEPS = 150;    // Only check endstops when motor is within this many steps of them

/** TIMING CONSTANTS **/
const uint16_t MEAS_PERIOD       = 100;        // Period between measurements in ms
const uint16_t WRITE_PERIOD      = 5000;       // Period between EEPROM writes in ms
const uint16_t CONN_CHECK_PERIOD = 500;        // Period between acknowledgements in ms

/** PID CONTROL CONSTANTS **/
float    Kp                = 10.0;       // Proportional gain (increased for underwater response)
float    Ki                = 0.0;        // Integral gain (for steady-state error)
float    Kd                = 300.0;       // Derivative gain (for stability)
const float    PID_OUTPUT_LIMIT  = 80.0;      // Maximum PID output in steps
const float    PID_INTEGRAL_LIMIT = 5.0;     // Anti-windup limit for integral term

/** FLOAT SPECIFIC CONSTANTS **/
const uint8_t  MAX_PROFILES      = 2;          // Number of profiles for maximum points
const int8_t   MAX_TARGET        = -1;         // Encodes the pool bottom as target when given as parameter to the measure() function
const float    FLOAT_LENGTH      = 0.51;       // Length of the FLOAT, measured form the very bottom to the pressure sensor top, expressed in m
const float    MAX_ERROR         = 0.45;        // Error span in which the FLOAT can be considered at target depth during a profile, expressed in m
const float    EPSILON           = 0.01;       // Error span in which two consecutive measures are considered equal, expressed in m
const float    TARGET_DEPTH      = 2.50;        // Target depth to be met and mantained when sinking, expressed in m
const float    STAT_TIME         = 45;          // Time period in which the FLOAT has to maintain TARGET_DEPTH, expressed in s 
const float    TIMEOUT_PID_TIME  = 180;        // Maximum time for PID to reach target in seconds
const float    TIMEOUT_TIME      = 30;        // Maximum time for float to reach the target

/** NETWORK CONSTANTS **/
const char*    SSID              = "PIPO";     // OTA WiFi network name
const char*    PASSWORD          = "politocean"; // OTA WiFi password

/** GLOBAL OBJECTS **/
INA_Class           INA;           // Current/voltage monitor
MS5837              sensor;        // Pressure sensor
esp_now_peer_info_t peerInfo;      // ESP-NOW peer info
WebServer           http_server(80);    // Web server for OTA
AccelStepper        stepper(AccelStepper::DRIVER, STEP, DIR); // Stepper motor controller
RGBLed              led(R_PIN, G_PIN, B_PIN, RGBLed::COMMON_CATHODE); // RGB LED controller

/** MAC ADDRESSES **/
uint8_t espA_mac[6] = {0x5C, 0x01, 0x3B, 0x2C, 0xE0, 0x68}; // This ESP32 MAC
uint8_t broadcastAddress[] = {0xEC, 0xE3, 0x34, 0x66, 0xE1, 0x20}; // ESPB MAC

/** GLOBAL VARIABLES **/
uint8_t  profile_count    = 0;     // Number of completed profiles
uint8_t  auto_mode_active = 0;     // Auto mode status
uint8_t  idle             = 0;  // 1 if in idle phase, 0 otherwise
uint8_t  auto_committed   = 0;     // Auto mode commit flag
uint8_t  deviceNumber     = -1;    // INA220 device number
int8_t   send_result      = -1;    // Message sending result
int8_t   status           = 0;     // Main state machine status
uint16_t eeprom_read_ptr  = 0;     // EEPROM read pointer
uint16_t eeprom_write_ptr = 0;     // EEPROM write pointer
uint16_t current_step     = 0;     // Current motor position
uint32_t test_speed       = MAX_SPEED;    // Default test speed in steps/sec
float    atm_pressure     = 0;     // Atmospheric pressure reference
float    depth            = 0;     // Current depth measurement

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
FloatLEDState current_led_state = LED_OFF;
unsigned long led_last_update = 0;

/** DEBUG MODE VARIABLES **/
bool debug_mode_active = false;

/** COMMUNICATION STRUCTURES **/
input_message  status_to_send;    // Data to send to espB (charge and messages)  
output_message command_received;  // Commands received from espB

/** FUNCTION DECLARATIONS **/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void setLEDState(FloatLEDState state);
void updateLED();
bool homeMotor();
bool safeMoveTo(long targetPosition);
bool safeMoveSteps(long steps);
float calculatePID(float targetDepth, float currentDepth);
void handleEndstopHit();
uint8_t send_message(const char* message, uint32_t timeout);
float f_depth();
void measure(float targetDepth, float time, float timeout);
void processEndstops(bool is_homing);

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
      led.setColor(RGBLed::WHITE); // White solid
      break;
    case LED_OTA_MODE:
      led.flash(RGBLed::ORANGE, 500); // Orange blink
      break;
  }
}

void updateLED() {
  // Handle battery status override
  if (status_to_send.charge < BATT_THRESH && current_led_state != LED_LOW_BATTERY) {
    setLEDState(LED_LOW_BATTERY);
    return;
  }
  
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
  stepper.setAcceleration(HOMING_SPEED);
  
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
  stepper.setAcceleration(MAX_SPEED);
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
    stepper.disableOutputs();
    return false; // Movement failed
  }
  
  // If loop exited because distanceToGo is 0 (successful move)
  current_step = stepper.currentPosition();
  stepper.disableOutputs();
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

    stepper.setAcceleration(MAX_SPEED); 
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

    stepper.setAcceleration(HOMING_SPEED); // Use homing accel for consistency
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
  stepper.disableOutputs(); // Disable motor after backing off
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  send_result = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&command_received, incomingData, sizeof(command_received));
}

uint8_t send_message(const char* message, uint32_t timeout) {
  strcpy(status_to_send.message, message);
  
  send_result = -1;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &status_to_send, sizeof(status_to_send));
  
  if (result != ESP_OK) {
    Debug.println("Error sending message");
    return 0;
  }
  
  // Wait for send confirmation
  unsigned long startTime = millis();
  while (send_result == -1 && (millis() - startTime) < timeout) {
    delay(10);
    updateLED();
  }
  
  return (send_result == 1) ? 1 : 0;
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

/** MAIN MEASUREMENT AND CONTROL FUNCTION **/
void measure(float targetDepth, float time, float timeout) {
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

    safeMoveTo(500); // Move a little before starting PID control, this will accellerate the descent
  }
  
  uint64_t start_time = millis();
  uint64_t last_measurement = 0;
  uint64_t last_write = 0;
  bool motor_stopped = false;
  float last_depth_check = 0;
  int stable_count = 0;
  
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
        if (targetDepth != MAX_TARGET && targetDepth != FLOAT_LENGTH){  //doing PID control
          data.temperature = 100; // Mark PID control points
          if (abs(depth - targetDepth) < MAX_ERROR) {
          stable_count++;
          if (stable_count >= ((time * 1000 / WRITE_PERIOD) + 1)) {
            Debug.println("Profile at target depth complete");
            break;
          }
        }
        }
        
        // Write to EEPROM (implementation depends on your data format)
        EEPROM.put(eeprom_write_ptr, data);
        eeprom_write_ptr += sizeof(sensor_data);
        if (eeprom_write_ptr >= EEPROM_SIZE - sizeof(sensor_data)) {
          eeprom_write_ptr = 0; // Wrap around
        }
        EEPROM.commit();
        last_write = millis();
      }

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

      last_depth_check = depth;
    }
    
    // Allow other tasks to run
    yield();

    if (millis() - start_time > timeout * 1000) {
      Debug.println("Time out");
      break; // Exit if PID takes too long
    }
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
  stepper.setAcceleration(MAX_SPEED);
  stepper.setEnablePin(EN);
  stepper.setPinsInverted(true, false, true); // Enable pin is active low
  stepper.disableOutputs();

  status_to_send.charge = BATT_THRESH +1; // Initialize with a value above threshold
  command_received.command = 0; // Initialize command to 0 (idle)
  
  // Initialize EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
    setLEDState(LED_ERROR);
    while(1){
      updateLED();
      yield();
    }
  }
  eeprom_write_ptr = 0;
  eeprom_read_ptr = 0;
  
  // Initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    setLEDState(LED_ERROR);
    while(1){
      updateLED();
      yield();
    }
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    setLEDState(LED_ERROR);
    while(1){
      updateLED();
      yield();
    }
  }

  // Initialize DebugSerial library
  Debug.begin(&debug_mode_active, send_message);
  Debug.println("Debug serial initialized - ready for remote debugging");
  
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
    while(1){
      updateLED();
      yield();
    }
  }

  setLEDState(LED_INIT);   // First GREEN 0.5sec blink to indicate INA220 found
  delay(500);
  setLEDState(LED_OFF);
  delay(500);
  
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
    yield();
  }
  
  sensor.setFluidDensity(997); // Freshwater density
  sensor.read();
  atm_pressure = sensor.pressure(MS5837::Pa);
  Debug.printf("Atmospheric pressure: %.2f Pa\n", atm_pressure);

  setLEDState(LED_INIT);   // Second GREEN 0.5sec blink to indicate BAR02 found
  delay(500);
  setLEDState(LED_OFF);
  delay(500);
  
  // Perform motor homing
  Debug.println("Starting motor homing...");
  if (!homeMotor()) {
    Debug.println("CRITICAL: Motor homing failed!");
    setLEDState(LED_ERROR);
    while(1) {
      updateLED();
      yield();
    }
  }
  
  Debug.println("Initialization complete - Float ready!");
}

/** MAIN LOOP **/
void loop() {
  updateLED();

  // Process endstops, especially if motor is idle, to catch unexpected hits.
  // Movement functions also call this, but this is a safety net.
  if (stepper.distanceToGo() == 0 && !emergency_stop) { // If motor is supposed to be idle
      processEndstops(false); // Normal mode, not homing
      if (emergency_stop) { // An endstop was hit while idle
          Debug.println("Endstop hit while motor was idle!");
          handleEndstopHit(); // Back off
          stepper.disableOutputs(); // Disable motor after handling idle hit
          setLEDState(LED_ERROR); 
      }
  }
  
  switch (status) {
    case 0: // Idle
      {
        uint8_t result;
        auto_committed = 0;
        
        // Read battery charge
        INA.waitForConversion(deviceNumber);
        status_to_send.charge = INA.getBusMilliVolts(deviceNumber);
        
        if (!idle) command_received.command = 0;
        
        // Send appropriate acknowledgment
        if (eeprom_write_ptr > eeprom_read_ptr) {
          setLEDState(LED_IDLE_DATA);
          result = send_message(IDLE_W_DATA_ACK, 5000);
        } else {
          setLEDState(LED_IDLE);
          result = send_message(IDLE_ACK, 5000);
        }
        
        if (result) {
          idle = 1;
          uint64_t prec_time = millis();
          
          // Wait for command or timeout
          while ((millis() - prec_time < CONN_CHECK_PERIOD) && command_received.command == 0) {
            updateLED();
            delay(10);
          }
          
          status = command_received.command;
          if (status != 0) {
            idle = 0;
            setLEDState(LED_COMMUNICATION);
          }
        } else if (profile_count < MAX_PROFILES && auto_mode_active) {
          // Auto mode activation
          setLEDState(LED_AUTO_MODE);
          Serial.println("Communication failed - activating auto mode");
          status = 1; // Go to profile execution
          auto_committed = 1;
        }
      }
      break;
      
    case 1: // GO - Execute profile
      {
        uint8_t result;
        
        if (auto_committed) {
          setLEDState(LED_AUTO_MODE);
          result = 1; // Auto mode doesn't need acknowledgment
        } else {
          result = send_message(CMD1_ACK, 1000);
        }
        
        if (result) {
          Debug.println("Starting profile execution");

          eeprom_read_ptr = 0; // Ensures only data from this profile are saved and sent
          eeprom_write_ptr = 0; // Reset write pointer for new profile data
          
          // Execute three-phase depth profile
          Debug.println("Phase 0: Descending to target depth");
          measure(TARGET_DEPTH, STAT_TIME, TIMEOUT_PID_TIME); 
          // delay(500); // Wait between phases
          // Debug.println("Phase 1: Descending to maximum depth (pool bottom)");
          // measure(MAX_TARGET, 3); 
          delay(500); // Wait between phases
          Debug.println("Phase 2: Ascending to surface");
          measure(FLOAT_LENGTH, 3, TIMEOUT_TIME); 
          
          stepper.disableOutputs(); // Disable motor after profile completion
          
          profile_count++;
          Debug.printf("Profile %d completed\n", profile_count);
        }
        
        status = 0; // Return to idle
      }
      break;
      
    case 2: // SEND_DATA - Send stored sensor data to Control Station
      {
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
                   "{\"company_number\":\"EX10\",\"pressure\":\"%.2f\",\"depth\":\"%.2f\",\"temperature\":\"%.2f\",\"mseconds\":\"%d\"}",
                   data.pressure, f_depth(data.pressure), data.temperature, packet_count*WRITE_PERIOD);
          
          delay(50); // Slow down sending rate
          send_message(line, 100); // Send data line to CS
          packet_count++;
        }
        
        strcpy(line, "STOP_DATA");
        send_message(line, 100); // Signal end of data transmission
        
        Debug.println("Data transmission completed");
        status = 0; // Return to idle
      }
      break;
      
    case 3: // BALANCE - Move syringes top then bottom usefult to fill them up with water
      {
        uint8_t result = send_message(CMD3_ACK, 1000);
        if (result) {
          Debug.println("Executing balance command");
          safeMoveTo(MAX_STEPS - ENDSTOP_MARGIN);
          delay(5000);
          safeMoveTo(ENDSTOP_MARGIN);

        }
        status = 0;
      }
      break;
      
    case 4: // CLEAR_SD - Clear EEPROM data
      {
        uint8_t result = send_message(CMD4_ACK, 1000);
        if (result) {
          Debug.println("Clearing EEPROM data");
          eeprom_write_ptr = 0;
          eeprom_read_ptr = 0;
          // Clear first few bytes to mark as empty
          for (int i = 0; i < 100; i++) {
            EEPROM.write(i, 0);
          }
          EEPROM.commit();
        }
        status = 0;
      }
      break;
      
    case 5: // SWITCH_AUTO_MODE - Toggle auto mode
      {
        uint8_t result = send_message(CMD5_ACK, 1000);
        if (result) {
          auto_mode_active = !auto_mode_active;
          Debug.printf("Auto mode: %s\n", auto_mode_active ? "ENABLED" : "DISABLED");
          if (auto_mode_active)
            setLEDState(LED_AUTO_MODE);
          else 
            setLEDState(LED_IDLE);
        }
        status = 0;
      }
      break;
      
    case 6: // SEND_PACKAGE - Send single test packet
      {
        char package[OUTPUT_LEN];
        
        Debug.println("Sending test package");
        sensor.read(); // Update sensor readings
        
        // Format test package with current sensor data
        snprintf(package, OUTPUT_LEN,
                "{\"company_number\":\"EX10\",\"pressure\":\"%.2f\",\"depth\":\"%.2f\",\"temperature\":\"%.2f\",\"mseconds\":\"%d\"}",
                sensor.pressure(MS5837::Pa), f_depth(), sensor.temperature(), millis());
        
        send_message(package, 1000); // Send test package (acknowledgment is the package itself)
        
        Debug.println("Test package sent");
        status = 0;
      }
      break;
      
    case 7: // TRY_UPLOAD - Start OTA server
      {
        uint8_t result = send_message(CMD7_ACK, 1000);
        if (result) {
          setLEDState(LED_OTA_MODE);
          Debug.println("Starting OTA server...");
          Debug.println("Should be at: http://192.168.4.1/update");

          bool original_debug_mode_state = debug_mode_active;
          if (original_debug_mode_state) {
            Debug.println("Temporarily disabling remote debug for OTA setup.");
            debug_mode_active = false; // Disable remote debug to prevent recursion
          }

          // De-initialize ESP-NOW
          Serial.println("De-initializing ESP-NOW for OTA...");
          esp_err_t deinit_status = esp_now_deinit();
          if (deinit_status == ESP_OK) {
            Serial.println("ESP-NOW de-initialized successfully (local serial only).");
          } else {
            Serial.printf("ESP-NOW de-initialization failed or was not initialized: %s (local serial only)\n", esp_err_to_name(deinit_status));
          }
          delay(100); // Allow time for de-initialization to complete

          // Stop WiFi if it was in STA mode for ESP-NOW
          WiFi.disconnect(true); // Disconnect from any network
          WiFi.mode(WIFI_OFF);   // Turn off WiFi completely
          delay(100);            // Allow WiFi to turn off
          
          // Configure and start Access Point
          Serial.printf("Setting up AP: %s\n", SSID); // Use Serial directly
          WiFi.mode(WIFI_AP);
          if (WiFi.softAP(SSID, PASSWORD)) {
            Serial.println("Soft AP created successfully");
          } else {
            Serial.println("Soft AP creation failed!");
            // Handle AP creation failure (e.g., return to idle, set error LED)
            status = 0;
            setLEDState(LED_ERROR);
            
            // Attempt to re-initialize ESP-NOW before breaking
            Serial.println("Attempting to re-initialize ESP-NOW after AP failure");
            WiFi.mode(WIFI_STA); // ESP-NOW requires STA mode
            delay(100);
            if (esp_now_init() == ESP_OK) {
              esp_now_register_send_cb(OnDataSent);
              esp_now_register_recv_cb(OnDataRecv);
              esp_now_add_peer(&peerInfo);
              Serial.println("ESP-NOW re-initialized after AP failure");
            } else {
              Serial.println("Failed to re-initialize ESP-NOW after AP failure");
            }
            if (original_debug_mode_state) {
              debug_mode_active = true; // Restore remote debug
              Debug.println("Remote debug re-enabled after AP failure handling.");
            }
            break; // Exit case 7
          }
          
          IPAddress apIP = WiFi.softAPIP();
          Serial.printf("AP IP address: %s)\n", apIP.toString().c_str());
          Serial.printf("OTA Server: http://%s/update\n", apIP.toString().c_str());
            
          // Start ElegantOTA
          ElegantOTA.begin(&http_server); // Pass WebServer instance
          http_server.begin();
            
          // Keep OTA server running for 5 minutes
          unsigned long ota_start_time = millis();
          Serial.println("OTA server running for 5 minutes...");
          while (millis() - ota_start_time < 300000) { // 5 minutes
            http_server.handleClient();
            ElegantOTA.loop(); // Process ElegantOTA
            updateLED(); // updateLED does not use Debug
            delay(10);
          }
          
          Serial.println("OTA period finished");
          // Stop server and AP
          http_server.stop();
          WiFi.softAPdisconnect(true);
          WiFi.mode(WIFI_OFF); // Turn off WiFi before re-initializing ESP-NOW
          delay(100);

          // Re-initialize ESP-NOW
          Serial.println("Re-initializing ESP-NOW (local serial only)...");
          WiFi.mode(WIFI_STA); // ESP-NOW requires STA mode
          delay(100); // Give WiFi time to switch to STA mode
          if (esp_now_init() != ESP_OK) {
            Serial.println("Error re-initializing ESP-NOW (local serial only)");
            setLEDState(LED_ERROR);
            // Potentially loop forever or restart
          } else {
            esp_now_register_send_cb(OnDataSent);
            esp_now_register_recv_cb(OnDataRecv);
            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
              Serial.println("Failed to re-add peer (local serial only)");
              setLEDState(LED_ERROR);
            } else {
              Serial.println("ESP-NOW re-initialized successfully (local serial only).");
            }
          }
          if (original_debug_mode_state) {
            debug_mode_active = true; // Restore remote debug
            Debug.println("Remote debug re-enabled after OTA process.");
          }
        }
        status = 0;
      }
      break;
      
    case 8: // UPDATE_PID - Update PID parameters
      {
        uint8_t result = send_message(CMD8_ACK, 1000);
        if (result) {
          // Update PID parameters from received command
          Kp = command_received.params[0];
          Ki = command_received.params[1]; 
          Kd = command_received.params[2];
          
          Debug.printf("PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
        }
        status = 0;
      }
      break;
      
    case 9: // TEST_FREQ - Set test frequency
      {
        uint8_t result = send_message(CMD9_ACK, 1000);
        if (result) {
          // Limit frequency to safe range
          if (command_received.freq > 1200) command_received.freq = 1200;
          if (command_received.freq < 10) command_received.freq = 10;
          test_speed = command_received.freq;
          Debug.printf("Test frequency set to: %u Hz\n", test_speed);
        }
        status = 0;
      }
      break;
      
    case 10: // TEST_STEPS - Execute test movement
      {
        uint8_t result = send_message(CMD10_ACK, 1000);
        if (result) {
          Debug.printf("Executing test movement: %ld steps\n", command_received.steps);
          
          // Set motor speed based on test frequency
          stepper.setMaxSpeed(test_speed);
          stepper.setAcceleration(test_speed); // Set acceleration for test speed
          stepper.enableOutputs();
          stepper.move(command_received.steps);

          while (stepper.distanceToGo() != 0) {
            stepper.run();
            updateLED();
            yield();
          }
          
          Debug.printf("Test movement completed. Current position: %d\n", current_step);
          
          // Restore normal motor speed and acceleration
          stepper.setMaxSpeed(MAX_SPEED);
          stepper.setAcceleration(MAX_SPEED); // Restore default acceleration
          stepper.disableOutputs(); // Ensure motor is disabled after test steps
        }
        status = 0;
      }
      break;
      
    case 11: // DEBUG_MODE - Toggle debug mode (send serial output over ESP-NOW)
      {
        uint8_t result = send_message(CMD11_ACK, 1000);
        if (result) {
          debug_mode_active = !debug_mode_active;
          if (debug_mode_active) {
            Debug.println("DEBUG MODE ACTIVATED - Serial output will be forwarded over ESP-NOW");
            Debug.println("DEBUG: Remote serial output active");
          } else {
            Debug.println("DEBUG MODE DEACTIVATED - Serial output local only");
          }
        }
        status = 0;
      }
      break;
      
    case 12: // HOME - Perform Homing as requested by remote
      {
        uint8_t result = send_message(CMD12_ACK, 1000); 
        if (result) {
          Debug.println("Executing Homing command as per remote request.");
          if (!homeMotor()) {
            Debug.println("CRITICAL: Homing command (requested remotely) failed!");
            setLEDState(LED_ERROR); // Indicate error
          } else {
            Debug.println("Homing command (requested remotely) completed successfully.");
          }
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
