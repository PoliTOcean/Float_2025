/*
*********************************************************************************************************
*                            Code of the ESP32 on the FLOAT board (ESPA)
*
* Filename: EspA.ino
* Version: 8.1.1
* Developers: Fachechi Gino Marco, Gullotta Salvatore
* Company: Team PoliTOcean @ Politecnico di Torino
* Arduino board package: esp32 by Espressif Systems, v2.0.17
*
* Description:
* The code has a switch in the main loop driven by a state 
* called status. The status is mainly modified by the packages
* sended by the ESPB over the MAC layer.
* The zero state is the idle one, in which
* the FLOAT communicates to CS that it is listening and
* waits for the next command to execute. The idle acknowledgement
* is sended every CONN_CHECK_PERIOD milliseconds.
* In case the microSD has some data to send, the acknowledgement
* informs the CS of that, too.
* If the ack fails to be delivered, the FLOAT can enter in Auto Mode
* (hereinafter shortened to as AM), that commits automatically a profile. 
* The other states correspond to specific command routines, called when
* their relative command arrives.
* After a command completion, the FLOAT returns to idle phase.
* The FLOAT should guarantee to send a command acknowledgement only when
* the command success is sure. In the same way, if the acknowledgement sending
* fails, the command is ignored to keep consistency with the ESPB.
********************************************************************************************************* 
*/

/** INCLUDES **/
//                                                          Description                                             | Version
//                                                                                                                  |
#include <Arduino.h>           // Library needed if you are compiling on VSC PlatformIO plugin                      |    /
#include <MS5837.h>            // Library for pressure sensor (Bar02) control over I2C                              | v1.1.1
#include <esp_now.h>           // Esp_now library for WiFi connections establishing and management                  |    /
#include <WiFi.h>              // Layer for proper initialization and setting of ESP32 WiFi module                  |    /
#include <Wire.h>              // Library for I2C connection protocol interface, used by MS5837.h and RTClib.h      |    /
#include <SPI.h>               // Library for SPI connection protocol interface, used by SD.h                       |    /
#include <SD.h>                // An SDcard control interface                                                       | v1.2.4
#include <AsyncTCP.h>          // Library for TCP connection protocol interface, used by ESPAsyncWebServer.h        | v1.1.4
#include <ESPAsyncWebServer.h> // Library for creation of a web server running on ESP32 , used by AsyncElegantOTA.h | v2.10.8
#include <AsyncElegantOTA.h>   // Library for creation of the OTA web page for new firmware uploading               | v2.2.8

/** DEFINES **/
#ifndef ESPA_INO
  #define ESPA_INO
#endif

#define OUTPUT_LEN      250               // Length of the output on the MAC layer 
#define FILE_NAME       "/FLOAT_data.txt" // Name of the file containing FLOAT measurements
// List of messages for the ESPA acknoledgements: CS has to be aware of these 
#define IDLE_ACK        "FLOAT_IDLE"        
#define IDLE_W_DATA_ACK "FLOAT_IDLE_W_DATA"
#define CMD1_ACK        "GO_RECVD"
#define CMD3_ACK        "CMD3_RECVD"
#define CMD4_ACK        "CMD4_RECVD"
#define CMD5_ACK        "SWITCH_AM_RECVD"
#define CMD7_ACK        "TRY_UPLOAD_RECVD"
#define CMD8_ACK        "CHNG_PARMS_RECVD"

/** I/O STRUCTS 
  Structs containing message and command for communication over MAC. 
  Must match the receiver (ESPB) structures.
  Necessary for esp_now library compliance.
**/
typedef struct output_message {
  char message[OUTPUT_LEN];
  float charge;
} output_message;

typedef struct input_message {
  uint8_t command = 0;
  float params[3];
} input_message;

output_message output;
input_message  input;

/** LIBRARY VARIABLES **/
MS5837              sensor;   // Object for Bar02 interfacing
File                myFile;   // File object pointing to the microSD file currently modified
esp_now_peer_info_t peerInfo; // Object containing info about the MAC peer we want to connect with
AsyncWebServer      server(80);
// Esp_now constant for peer MAC address value. Replace with the MAC address of your receiver (ESPB) 
uint8_t             broadcastAddress[] = {0xE4, 0x65, 0xB8, 0xA7E, 0x27, 0xAC};

/** PROGRAM GLOBAL CONSTANTS **/
const char*    SSID              = "Mi10";     // Name of the net the ESPA has to connect to for OTA firmware upload
const char*    PASSWORD          = "ciao1234"; // Password for the net the ESPA has to connect to for OTA firmware upload
const uint8_t  DIR               = 12;         // Pin for motor direction driving
const uint8_t  STEP              = 14;         // Pin for motor tension driving (used as PWM)
const uint8_t  EN                = 27;         // Active-low pin for motor enabling
const uint8_t  MAX_PROFILES      = 2;          // Number of profiles to complete in order to gain maximum points
const uint8_t  R_PIN             = 19          // Pin of the RED LED
const uint8_t  G_PIN             = 18          // Pin of the GREEN LED
const uint8_t  B_PIN             = 5           // Pin of the BLUE LED
const uint16_t MEAS_PERIOD       = 100;        // Period between two consecutive measurements during immersion phase, expressed in ms
const uint16_t WRITE_PERIOD      = 5000;       // Period between two consecutive writes to microSD during immersion phase, expressed in ms
const uint16_t CONN_CHECK_PERIOD = 500;        // Period between two consecutive acknoledgements during idle phase, expressed in ms
const uint32_t MAX_STEPS         = 2000        // Number of motor steps necessary to fully empty/fill the syringes, to be found empirically
const int8_t   MAX_TARGET        = -1          // Encodes the pool bottom as target when given as parameter to the measure() function
const float    FLOAT_LENGTH      = 0.51;       // Length of the FLOAT, measured form the very bottom to the pressure sensor top, expressed in m
const float    MAX_ERROR         = 0.5;        // Error span in which the FLOAT can be considered at target depth during a profile, expressed in m
const float    EPSILON           = 0.01;       // Error span in which two consecutive measures are considered equal, expressed in m
const float    TARGET_DEPTH      = 2.5;        // Target depth to be met and mantained when sinking, expressed in m
const float    STAT_TIME         = 50;         // Time period in which the FLOAT has to maintain TARGET_DEPTH, expressed in s 

/** PROGRAM GLOBAL VARIABLES **/
uint8_t  meas_cnt         = 0;  // Number of measurements occurred since the last write to microSD, must be reset before each profile
uint8_t  profile_count    = 0;  // Number of profiles committed since startup
uint8_t  auto_mode_active = 0;  // 1 if AM is active, 0 otherwise
uint8_t  idle             = 0;  // 1 if in idle phase, 0 otherwise
uint8_t  auto_committed   = 0;  // During immersion phase, 1 if immersion has been triggered by AM, 0 otherwise
uint8_t  r_v,g_v,b_v      = 0;  // Intensity (duty cycle) values for each RGB LED 
uint8_t  led_state        = 0;  // Tells about the RGB LED being on or off
int8_t   send_result      = -1; // Flag for sending-over-MAC logic, needed to handle sending failure
int8_t   status           = 0;  // Status of the FLOAT, drives the main switch and keeps track of the current phase
uint16_t new_data         = 0;  // During idle phase, 1 if ready-to-send data is in microSD, 0 otherwise
uint16_t file_read_ptr    = 0;  // Pointer to the location of the last read byte in the microSD data file
uint16_t file_write_ptr   = 0;  // Pointer to the location of the last written byte in the microSD data file
uint16_t led_blink        = 0;  // Blinking period for the RGB LED, expressed in ms
uint32_t current_step     = 0;  // Number of steps done by the motor in the same direction wrt its initial position
uint64_t blink_ref        = 0;  // Time reference for the blinking period of the RGB LED
uint64_t time_ref         = 0;  // Time reference for writing on the microSD during immersion phase, updated at each profile start
float    atm_pressure     = 0;  // Stores the initial atmosphere pressure, needed for correct depth calculation
float    depth            = 0;  // Depth measured during immersion phase, writed in microSD and used to sense FLOAT stationarity, expressed in Pa
float    Kp               = 1;  // PID parameters, modifiable with a specific command
float    Kd               = 0;
float    Ki               = 0;  
float    depth_integral   = 0;  // Integral of the depth values measured in a profile. Used for PID calculations.
//                                 Must be reset before each profile

/** PROGRAM LOCAL FUNCTIONS **/
/*
*********************************************************************************************************
*                                          OnDataRecv()
*
* Description : Callback for data arrival on the MAC layer: receives a command during idle phase.
*               If the command in the input data struct is 0, the callback updates it with the 
*               incoming command. The value in the struct will be reset to 0 after
*               the completion of the routine relative to the received command.
*               Meanwhile no other incoming command will be accepted.
*
* Argument(s) : mac          : MAC address of the peer from which the data was received.
*
*               incomingData : Struct containing incoming data. Data maximum length is 250 bytes.
*
*               len          : Length of the received data.
*
* Return(s)   : none
*********************************************************************************************************
*/
void OnDataRecv (const uint8_t * mac, const uint8_t * incomingData, int len) {

  if (input.command == 0) memcpy(&input, incomingData, sizeof(input));  
}

/*
*********************************************************************************************************
*                                          OnDataSent()
*
* Description : Callback for data sending on the MAC layer: it updates send_result flag to 
*               inform other logics whether the sending of the last package went well or not. 
*               The flag has to be cleared to -1 before each sending.
*
* Argument(s) : mac    : MAC address of the peer to which the data was sended.
*
*               status : Status of the sending. It evaluates to ESP_NOW_SEND_SUCCESS if it succeeded.
*
* Return(s)   : none
*********************************************************************************************************
*/
void OnDataSent (const uint8_t * mac, esp_now_send_status_t status) {
  
  if (status == ESP_NOW_SEND_SUCCESS) send_result = 1; // If sending succeeds sets the flag to 1
  else send_result = 0;                                // Otherwise sets it to 0
  
}

/*
*********************************************************************************************************
*                                          send_message()
*
* Description : Function for message sending over the MAC layer: target peer is set during setup and 
*               should be the CS (ESPB). In case of sending failure, signalled by the send_result flag, 
*               the message is continously sended until it succeeds or 
*               the max_conn_time milliseconds elapse.
*
* Argument(s) : message_str   : Char array containing the message string.
*
*               max_conn_time : Time that has to elapse before stopping to try the sending. 
*                               Expressed in milliseconds. 
*
* Return(s)   : 1 if sending succeeded, 0 if given time elapsed.
*********************************************************************************************************
*/
uint8_t send_message (char * message_str, uint16_t max_conn_time) {
  memcpy(&output.message, message_str, sizeof(output.message));          // Message is copied in the output data struct
  uint64_t prec_time = millis();                                         // A time reference (ms elapsed from startup) is stored in prec_time
  while (true) {                                                         // [* sending sequence start]
    send_result = -1;                                                    // send_result flat is cleared
    esp_now_send(broadcastAddress, (uint8_t *) &output, sizeof(output)); // Tries to send the message over MAC layer
    while (send_result == -1);                                           // Waits for OnDataSent callback to set send_result flag
    if(send_result) return 1;                                            // If sending succeeds, function returns 1
    else if (millis() - prec_time > max_conn_time) return 0;             // If sending failed and max_conn_time milliseconds elapsed from 
  }//                                                                       prec_time set, the function returns 0. Otherwise it starts again 
//                                                                          the sending sequence from [* sending sequence start]
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

/*
*********************************************************************************************************
*                                           write_SD()
*
* Description : Writes to the microSD mounted on the board. In particular it writes a single line
*               containing all the data needed by the CS, wrapped in a JSON format and terminated 
*               by a new line. The write starts after the last written byte in the file. The 
*               function finally updates the file_write_ptr pointer with the position
*               following the last written byte in the file (the new line). The data is mostly 
*               provided by the sensor object which has to be updated before calling this
*               function, by calling sensor.read function. 
*
* Argument(s) : none
*
* Return(s)   : none
*********************************************************************************************************
*/
void write_SD () {
  if (myFile) {                                // Checks if the file is open and the File pointer is valid
    myFile.seek(file_write_ptr);               // Puts the file cursor on the position stored in file_write_ptr.
//                                                The following writes will start from that position
    // Writes company number,
    myFile.print("{\"company_number\":\"EX16\",\"pressure\":\"");
    // pressure as measured from the bottom of the FLOAT: the multiplication is for m to Pa conversion in water,
    myFile.print(sensor.pressure(MS5837::Pa) + (FLOAT_LENGTH*10000));
    myFile.print("\",\"depth\":\"");
    // depth value as calculated from f_depth function, in m,
    myFile.print(depth);
    myFile.print("\",\"temperature\":\"");
    // temperature in Celsius,
    myFile.print(sensor.temperature());
    myFile.print("\",\"mseconds\":\"");
    // milliseconds elapsed from the profile start
    myFile.print((millis() - time_ref));
    myFile.println("\"}");
    myFile.flush();                            // Commits the writes
    file_write_ptr = myFile.position();        // Stores file cursor for next writes
    new_data = file_write_ptr - file_read_ptr; // Updates the new_data variable with the delta in bytes between 
//                                                already-read data and the still-to-read one
  } //else Serial.println("write_SD(): error in opening file!");
}

/*
*********************************************************************************************************
*                                           measure()
*
* Description : The function is called every MEAS_PERIOD ms during immersion phase. It's in charge
*               of stationarity detection and microSD writing. The write is performed by calling 
*               the write_SD function every n times measure itself is called, where n is 
*               the ratio between the period of the writes, WRITE_PERIOD, and the period with
*               which measure is called, MEAS_PERIOD, that is usually smaller. This works better
*               if the first is a multiple of the latter. The FLOAT is signalled as stationary
*               when two depth measurements in a row are equal within an EPSILON of error.
*
* Argument(s) : targetDepth, time
*
* Return(s)   : none
*********************************************************************************************************
*/
void measure (float targetDepth, float time) {
  uint8_t elapsed_stat = 0;
  uint8_t motor_stopped = 1;
  const uint16_t REV_FREQ = 300;
  uint64_t prec_time_meas = millis();                 // Stores time reference for measurement period timer
  uint64_t start_time = millis();
  float prevDepth = 0;
  float rev_time = 0;   
  integral = 0;

  while(true) {                                       // Waits for the FLOAT to stop
    if (millis() - prec_time_meas > MEAS_PERIOD) {    // Every MEAS_PERIOD ms reads from the sensor and tries to detect if FLOAT
//                                                       stopped after updating the motor PWM using the PID output. 
//                                                       It also writes to microSD every WRITE_PERIOD ms
      prec_time_meas = millis();                      // Resets the MEAS_PERIOD timer for next measurement
      
      LED_check_for_blink();                          // Makes the RGB LED blink at set period

      sensor.read();                                  // Reads measurements of the Bar02 pressure sensor, updating the sensor object
      f_depth();                                      // Calculates depth value
      meas_cnt++;                                  
      if (meas_cnt >= (WRITE_PERIOD / MEAS_PERIOD)) { // Checks if enough measurements occurred since the last write on microSD,
        write_SD();                                   // before writing again on it
        meas_cnt = 0;                                 
      }

      if (targetDepth == 0 || targetDepth == MAX_TARGET) {
        if (motor_stopped) {
          if (targetDepth == MAX_TARGET) {
            rev_time = ((MAX_STEPS-current_step) / (float) REV_FREQ) * 1000; 
            digitalWrite(DIR, 0);
            current_step = MAX_STEPS;
          }
          else {
            rev_time = (current_step/ (float) REV_FREQ) * 1000; // time of motor revolution in ms
            digitalWrite(DIR, 1);
            current_step = 0;
          }
          analogWriteFrequency(REV_FREQ);
          motor_stopped = 0;
        } else {
          if (millis() - start_time > rev_time) {
            analogWriteFrequency(0);
            if (prevDepth < depth + EPSILON && prevDepth > depth - EPSILON) {
              elapsed_stat++;
              if (elapsed_stat >= (time*1000)/MEAS_PERIOD) return;
            }
          }
        }
      } else {
        // PID output calculations ---
        float error = targetDepth - depth;
        float velocity = (prevDepth - depth) / (MEAS_PERIOD / 1000);
        integral += error * (MEAS_PERIOD / 1000); 
        float PID_res = error * Kp + velocity * Kd + integral * Ki;
        // ---------------------------

        if (PID_res >= 0) {
          //if (PID_res > 1000) PID_res = 1000;          // Cap the PID output to the maximum of the frequency. This should not be necessary with good params  
          digitalWrite(DIR, 0);                        // Sets DIR pin with the needed rotation direction. In this case the FLOAT sinks                        
          analogWriteFrequency(PID_res);
        } else {
          //if (-PID_res > 1000) PID_res = -1000;
          digitalWrite(DIR, 1);                        // The FLOAT goes towards the surface
          analogWriteFrequency(-PID_res);
        }
        current_step += (uint32_t) (PID_res * (MEAS_PERIOD/ (float) 1000));

        if (target < depth + MAX_ERROR && target > depth - MAX_ERROR) {
          elapsed_stat++;
          if (elapsed_stat >= (time*1000)/MEAS_PERIOD) return;
        }
      }

      Serial.print("The just read depth is: ");
      Serial.println(depth);
      
      prevDepth = depth;                             // Updates previous depth measurement with the last one
    }
  }
}

void setLED (uint8_t R, uint8_t G, uint8_t B, uint16_t period) {
  r_v = R; g_v = G; b_v = B; led_blink = period;
  LED_on();
} 

void toggleLED () {
  if (led_state) LED_off;
  else LED_on;
}

void LED_on () {
  analogWrite(R_PIN, r_v);
  analogWrite(G_PIN, g_v);
  analogWrite(B_PIN, b_v);
  led_state = 1;
}

void LED_off () {
  analogWrite(R_PIN, 0);
  analogWrite(G_PIN, 0);
  analogWrite(B_PIN, 0);
  led_state = 0;
}

void LED_check_for_blink () {
  if (millis() - blink_ref > led_blink) {
    if (LED_blink == 0) LED_on();
    else toggleLED();  
    blink_ref = millis();
  } 
}

/** SETUP **/
void setup () {
  Serial.begin(115200);                                                // Inits Serial Monitor

  pinMode(STEP, OUTPUT);                                               // Pin assignment
  pinMode(DIR, OUTPUT); 
  pinMode(R_PIN, OUTPUT);                                                 // R LED
  pinMode(G_PIN, OUTPUT);                                                 // G LED
  pinMode(B_PIN, OUTPUT);                                                 // B LED
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);                                              // Disables the motor until needed

  while (!SD.begin()) Serial.println("Failed to initialize SD card!"); // Inits microSD
  if (SD.exists(FILE_NAME)) SD.remove(FILE_NAME);                      // Clears data file
 
  WiFi.mode(WIFI_STA);                                                 // Sets device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) {                                      // Inits esp_now
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);                                // Registers esp_now arrival callback
  esp_now_register_send_cb(OnDataSent);                                // Registers esp_now sending callback
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);                     // Registers peer by passing peer object pointer
  peerInfo.channel = 0;                                                // Channel selection
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){                          // Adds peer
    Serial.println("Failed to add peer");
    return;
  }

  sensor.setModel(1);                                                  // Inits Bar02 by specifing model,
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar02: White=SDA, Green=SCL");
    delay(5000);
  }
  sensor.setFluidDensity(997);                                         // and operative density in Kg/m^3 (freshwater, 1029 for seawater)
  sensor.read();
  atm_pressure = sensor.pressure(MS5837::Pa);                          // Stores pressure value at water level for depth calculation, in Pa
}

/** MAIN SWITCH **/
void loop () {
  switch (status) {   
    case 0: // Idle
      {
        uint8_t result;

        auto_committed = 0;  

        if (charge < BATT_THRESH)                                                   // If battery charge is low, yellow blinking LED 
          setLED(255, 255, 0, 350);  
        else setLED(0, 255, 0, 750);                                                // Else, green blinking LED                                                   

        if (!idle) input.command = 0;                                               // If not already waiting for a command (1st idle loop), resets the command.
//                                                                                     This check is necessary as the new command can arrive 
//                                                                                     right after the CONN_CHECK_PERIOD timer triggers. Reset
//                                                                                     is necessary for the new command detection
        if (new_data != 0) result = send_message(IDLE_W_DATA_ACK, 5000);            // If there are data to be sent to CS, sends an acknowledgement
//                                                                                     with that information to the ESPB
        else result = send_message(IDLE_ACK, 5000);                                 // Otherwise signals only that is waiting for the next command

        if (result) {                                                               // Checks for acknowledgement success
          idle = 1;                                                                 // Sets the idle flag: while is set the FLOAT is considered
//                                                                                     waiting for a new command by all the logic. Like for commands
//                                                                                     routines, the FLOAT is set in idle only if the ack arrives
          uint64_t prec_time = millis();                                            // Stores a time reference for the CONN_CHECK_PERIOD timer

          while ((millis()-prec_time < CONN_CHECK_PERIOD) && input.command == 0) {  // Waits for a new command to arrive or for CONN_CHECK_PERIOD 
//                                                                                     milliseconds to elapse. In the meantime, the RGB LED is made blink
            LED_check_for_blink();
          }

          status = input.command;                                                   // Copies the input struct command into the status driving the 
//                                                                                     main switch. If the time elapsed, the command may be 0   
        } else if (profile_count < MAX_PROFILES && auto_mode_active) {              // If the acknowledgement failed, AM is active and the FLOAT
//                                                                                     committed less then MAX_PROFILES profiles, immersion is
//                                                                                     committed automatically
          auto_committed = 1;                                                       // Signals to other logics that the next profile has been
//                                                                                     committed automatically. Flag has to be cleared after 
//                                                                                     each command completion  
          status = 1;                                                               // Sets immersion status, committing a profile
        }
        if (status != 0) {
          idle = 0;                                                                 // If a command has been committed, signals to other logics
//                                                                                     that the FLOAT exited idle phase. 
          setLED(255, 127, 0, 0);                                                   // Orange fixed LED for the command execution time (can be overwritten)
        }
      }//                                                                              Otherwise, the case 0 repeats remaining in idle                                      
      break;
    case 1: // Profile commit and measuring
      { 
        myFile = SD.open(FILE_NAME, FILE_WRITE);                     // Opens file in write mode, or creates it if non existing. 
//                                                                      File pointer is now at 0
        if (!myFile) /*Serial.println("idle: error in opening file!")*/;
        else {
          uint8_t result;

          result = send_message(CMD1_ACK, 1000);                     // If file succeeds to open, tries to send an acknowledgement to CS
          if (result || auto_committed) {                            // If ack arrived or AM committed the profile, starts immersion phase
            
            if (auto_committed)
              setLED(255, 0, 0, 1000);                               // If AM committed, red blinking LED
            else setLED(0, 0, 255, 1000);                            // Else, blue blinking LED

            meas_cnt = 0;
            time_ref = millis();                                     // Stores time reference for the entire profile. Used in microSD writing
            profile_count++;                                         // Increases the global counter of committed profiles
            file_read_ptr = file_write_ptr;                          // If old profile data is written on microSD, discards it
            analogWriteFrequency(0);                                 // Sets analog PWM generator frequency to 0                                      
            analogWrite(STEP, 127);                                  // Sets the duty cycle of the generated PWM to 50%
            digitalWrite(EN, LOW);                                   // Enables motor

            for (int i=0; i<3; i++) {                                // Logic repeats three times: two for descending phase and the last for ascending phase
              switch (i) {
                case 0: measure(TARGET_DEPTH, STAT_TIME); break;     // measure function will drive the FLOAT and                    
                case 1: measure(MAX_TARGET, 0); break;               // returns when detects stationarity for a requested time period
                case 2: measure(0, 0); break;
                default: break;
              }
            }

            analogWrite(STEP, 0);                                    // stops the motor (duty cycle at 0%),
            analogWriteFrequency(1000);                              // resets the PWM generator frequency,
            digitalWrite(EN, HIGH);                                  // and disables the motor

            myFile.println("STOP_DATA");               // Adds a profile-data terminator to the file
            myFile.flush();                            // Commits the write
            file_write_ptr = myFile.position();        // Stores file cursor for next writes
            new_data = file_write_ptr - file_read_ptr; // Updates the new_data variable with the delta in bytes between 
//                                                        already-read data and the still-to-read one
            myFile.close();                            // Closes file
          }
        }
        status = 0;                                    // Returns to idle
      }
      break;
    case 2: // Data sending                                           // The command doesn't need an acknowledgement as 
      {//                                                                the sended data already works as ack                      
        myFile = SD.open(FILE_NAME, FILE_READ);                       // Opens file in read mode, or creates it if non existing
        if (!myFile) /*Serial.println("sending data: error in opening file!")*/;
        else {
          uint8_t result = 1;
          int line_len = 0;

          myFile.seek(file_read_ptr);                                 // Starts to read after the last read byte
          while (myFile.available()) {                                // Tries to read until it reaches end of file
            delay(50);                                                // Slowing down sending rate to match CS reading speed 
//                                                                       (USB buffer is small in ESPB-CS connection)
            char line[OUTPUT_LEN];
            
            line_len = myFile.readBytesUntil('\n', line, OUTPUT_LEN); // Reads from microSD until encounters a new line, while copying
//                                                                       into char array line. New line character is not copied
            for(int i=line_len; i<OUTPUT_LEN; i++) line[i]='\0';      // Fills the remaining part of the array with string terminators

            send_message(line, 100);                                  // Tries to send read data line to CS: if a sending fails, relative
//                                                                       data package is lost
            file_read_ptr = myFile.position();                        // Update last read byte position and hence the new_data variable:
//                                                                       this effectively marks just read data as old, so that it will not be 
            new_data = file_write_ptr - file_read_ptr;                // read again at next command call. This is true also if data fails to be
//                                                                       sended to CS (you want to plot only data relative to last profile)
          }
          myFile.close();                                             // Closes file
        }
        status = 0;                                                   // Returns to idle
      }
      break;
    case 3: // Motor balance
      {
        uint8_t result;

        result = send_message(CMD3_ACK, 1000);
        if (result) { // Commands execute only if their ack sending succeeds
          digitalWrite(DIR, 1);
          analogWriteFrequency(STEP_FREQ);
          analogWrite(STEP, 127);
          digitalWrite(EN, LOW);
          delay(ROT_TIME);
          analogWrite(STEP, 0);
          analogWriteFrequency(1000);
          digitalWrite(EN, HIGH);
        }
        status = 0;
      }
      break;
    case 4: // File clear
      {
        if (SD.exists(FILE_NAME)) {         // Checks for the file to exists in microSD file system
          uint8_t result;

          result = send_message(CMD4_ACK, 1000);
          if (result) SD.remove(FILE_NAME); // Deletes the file: will be recreate as empty at file opening
        }
        status = 0;
      }
      break;
    case 5: // AM toggle
      {
        uint8_t result;

        result = send_message(CMD5_ACK, 1000);
        if (result) auto_mode_active = !auto_mode_active; // Toggles AM: if is not active, next time connection drops
//                                                           the FLOAT will not enter in AM 
        status = 0;
      }
      break;
    case 6: // Single package sending. The command doesn't need an acknowledgement as the sended data already works as ack       
      {
        char package[OUTPUT_LEN];

        sensor.read(); // Updates sensor object
        sprintf(package, "{\"company_number\":\"EX16\",\"pressure\":\"%0.2f\",\"depth\":\"%0.2f\",\"temperature\":\"%0.2f\",\"mseconds\":\"0\"}", 
          sensor.pressure(MS5837::Pa) + (FLOAT_LENGTH*10000), f_depth(), sensor.temperature());
        send_message(package, 1000);
        status = 0;
      }
      break;
    case 7: // Firmware upload
      { 
        uint8_t result;

        result = send_message(CMD7_ACK, 1000); 
        if (result) {
          int timer = 0;

          WiFi.begin(SSID, PASSWORD);     // Sets WiFi module to connect to an external router, giving its SSID and PASSWORD 
//                                           When server is on, the router can be used to access the ElegantOTA page at the
//                                           IP assigned to the ESPA
          while (WiFi.status() != WL_CONNECTED && timer<=20) {
            timer++;
            delay(500);
            Serial.print(".");
          }
          Serial.println("");
          Serial.print("Connected to ");
          Serial.println(SSID);
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());

          server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            request->send(200, "text/plain", "Hi! I am ESP32.");
          });
          AsyncElegantOTA.begin(&server); // Starts the ElegantOTA server on the ESPA
          server.begin();
          Serial.println("HTTP server started");
        }
        status = 0;
      }
      break;
    case 8: // PID Params change
      {
        uint8_t result;
    
        result = send_message(CMD8_ACK, 1000); 
        if (result) {
          Kp = input.params[0];
          Kd = input.params[1]; 
          Ki = input.params[2];
        }
        status = 0; // Returns to idle
      }
      break;
    //case 9: // New command routine
    //  {
    //    uint8_t result;
    //
    //    result = send_message(CMD9_ACK, 1000); 
    //    if (result) {
    //      // Insert here your code
    //    }
    //    status = 0; // Returns to idle
    //  }
    //  break;
    default: // Default case
      Serial.println("No command recognized, returning to idle...");
      status = 0;
      break;
  }
}