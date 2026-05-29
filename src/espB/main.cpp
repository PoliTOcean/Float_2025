/*
*********************************************************************************************************
*                            Code of the ESP32 attached to CS (ESPB)
*
* Filename: main.cpp (formerly espB.ino)
* Version: 10.0.0
* Maintainers: Colabella Davide, Benevenga Filippo
* Past contributors: Fachechi Gino Marco, Gullotta Salvatore
* Company: Team PoliTOcean @ Politecnico di Torino
* Arduino board package: esp32 by Espressif Systems, v2.0.17
*
* Description:
* Enhanced communication bridge between Control Station and Float.
* Upgraded with RGBLed library for better status indication and improved protocol handling.
* The ESPB main loop is a broker driven by two buffers, USB and WiFi one, filled by two callbacks.
* When a command arrives from the CS, the code sends it to the FLOAT or 
* feedbacks the CS with the system state (that may be stale). 
* While doing the latter, the ESPB tries to send a dummy package:
* if sending fails, it also reports the loss of connection to the CS. Part of the feedback 
* to the CS is also the current activation state of the Auto Mode (AM), as it is on the FLOAT.
* When data arrives from the FLOAT, the ESPB updates the system state accordingly
* and forwards the data to the CS, as a real time feedback.
********************************************************************************************************* 
*/

/** INCLUDES **/
#include <Arduino.h>
#include <float_common.h>  // Shared definitions with espA
#include <config.h>
#include <espb_bridge_core.h>
#include <esp_now.h>       // ESP-NOW library for WiFi communication
#include <esp_wifi.h>
#include <WiFi.h>          // Layer for proper initialization and setting of ESP32 WiFi module

/** HARDWARE PIN DEFINITIONS **/
const uint8_t BUILTIN_LED_PIN = 2;  // Built-in LED pin on ESP32

/** PROGRAM GLOBAL CONSTANTS **/
#define BUFFER_SIZE 192     // Serial command buffer size
const uint16_t MAX_CONN_TIME = 100; // Time in ms that has to elapse before send_message function stops to try a sending

/** GLOBAL OBJECTS **/
esp_now_peer_info_t peerInfo; // Object containing info about the MAC peer we want to connect with

/** PROGRAM GLOBAL VARIABLES **/
uint8_t serial_rdy       = 0;     // Flag for signaling that the Serial software buffer serialInput is full and ready to be consumed
uint8_t message_rdy      = 0;     // Flag for signaling that a new message arrived on the MAC layer and is ready to be read 
int8_t  send_result      = -1;    // Flag for sending-over-MAC logic, needed to handle sending failure
char    serialInput[BUFFER_SIZE]; // Serial software buffer used to empty the hardware one as soon as a new command arrives
EspbBridgeState bridgeState;      // Cached FLOAT status exposed to the GUI

/** LED STATE MANAGEMENT **/
LEDState current_led_state = LEDState::OFF;
unsigned long led_last_update = 0;


/** RSSI STRUCTURES **/
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

/** COMMUNICATION STRUCTURES **/
output_message output; // Message to send to espA
input_message  input;  // Message received from espA

/** FUNCTION DECLARATIONS **/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void setLEDState(LEDState state);
void updateLED();
uint8_t send_command(const output_message& message, uint16_t max_conn_time);
void serial_handler();
void setEspNowChannel();

/** LED CONTROL FUNCTIONS **/
void setLEDState(LEDState state) {
  if (current_led_state != state) {
    current_led_state = state;
    led_last_update = millis();
  }
}

void updateLED() {
  unsigned long currentTime = millis();
  
  switch (current_led_state) {
    case LEDState::OFF:
      digitalWrite(BUILTIN_LED_PIN, LOW);
      break;
      
    case LEDState::IDLE:
      // Solid on when idle and connected
      digitalWrite(BUILTIN_LED_PIN, HIGH);
      break;
      
    case LEDState::ERROR:
      // Very fast blink for errors (150ms on/off)
      if ((currentTime - led_last_update) % 300 < 150) {
        digitalWrite(BUILTIN_LED_PIN, HIGH);
      } else {
        digitalWrite(BUILTIN_LED_PIN, LOW);
      }
      break;
      
    default:
      digitalWrite(BUILTIN_LED_PIN, LOW);
      break;
  }
}

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  bridgeState.lastRssi = ppkt->rx_ctrl.rssi;
}

void setEspNowChannel() {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

/*
*********************************************************************************************************
*                                          OnDataRecv()
*
* Description : Callback for data arrival on the MAC layer: receives and manages the messages.
*               If the message_rdy flag is 0, the callback updates the message in the input data struct 
*               with the incoming message and sets the flag. The flag will be reset to 0 after 
*               the message has been consumed. Meanwhile no other incoming message will be accepted.
*
* Argument(s) : mac          : MAC address of the peer from which the data was received.
*               incomingData : Struct containing incoming data. Data maximum length is 250 bytes.
*               len          : Length of the received data.
*
* Return(s)   : none
*********************************************************************************************************
*/
void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len) {
  if (!message_rdy && len == sizeof(input_message)) {
    message_rdy = 1;
    memcpy(&input.charge, incomingData, sizeof(input.charge));
    memcpy(&input.message, incomingData + sizeof(input.charge), sizeof(input.message));
  }
}

/*
*********************************************************************************************************
*                                          OnDataSent()
*
* Description : Callback for data sending on the MAC layer: it updates send_result flag to 
*               inform other logics whether the sending of the last package went well or not. 
*               The flag has to be cleared to -1 before each sending.
*
* Argument(s) : mac    : MAC address of the peer to which the data was sent.
*               status : Status of the sending. It evaluates to ESP_NOW_SEND_SUCCESS if it succeeded.
*
* Return(s)   : none
*********************************************************************************************************
*/
void OnDataSent(const uint8_t * mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    send_result = 1; // If sending succeeds sets the flag to 1
  } else {
    send_result = 0; // Otherwise sets it to 0
    setLEDState(LEDState::ERROR); // Indicate communication error
  }
}

/*
*********************************************************************************************************
*                                          send_command()
*
* Description : Function for command sending over the MAC layer: target peer is set during setup and 
*               should be the FLOAT (ESPA). In case of sending failure, signalled by the send_result flag, 
*               the message is continuously sent until it succeeds or 
*               the max_conn_time milliseconds elapse.
*
* Argument(s) : cmd_code      : Command code to send.
*               max_conn_time : Time that has to elapse before stopping to try the sending. 
*                               Expressed in milliseconds. 
*
* Return(s)   : 1 if sending succeeded, 0 if given time elapsed.
*********************************************************************************************************
*/
uint8_t send_command(const output_message& message, uint16_t max_conn_time) {
  output = message;
  uint64_t prec_time = millis();                                         // A time reference (ms elapsed from startup) is stored in prec_time
  
  while (true) {                                                         // [* sending sequence start]
    send_result = -1;                                                    // send_result flag is cleared
    esp_err_t err = esp_now_send(MAC_ESPA, (uint8_t *) &output, sizeof(output)); // Tries to send the code over MAC layer
    if (err != ESP_OK) {
      send_result = 0;
    }
    
    while(send_result == -1) {                                           // Waits for OnDataSent callback to set send_result flag
      delay(1); // Small delay to prevent watchdog issues
    }
    
    if (send_result) {
      setLEDState(LEDState::IDLE); // Communication successful
      return 1;                                                          // If sending succeeds, function returns 1
    } else if (millis() - prec_time > max_conn_time) {
      setLEDState(LEDState::ERROR); // Communication failed
      return 0;                                                          // If sending failed and max_conn_time milliseconds elapsed
    }
  }
} 

/*
*********************************************************************************************************
*                                          serial_handler()
*
* Description : Callback function for Serial buffer management. If serialInput has already 
*               been consumed, when new data arrives on the Serial channel, the function
*               takes bytes from the hardware buffer, stopping at a line ending
*               character, copies them into the serialInput char array and fills the rest with 
*               string terminators. It also signals that the serialInput buffer is now ready to be 
*               read by setting the serial_rdy flag, that will be cleared only after serialInput
*               consumption. Meanwhile all new data arrivals on Serial channel will be ignored.
*
* Argument(s) : none 
* Return(s)   : none
*********************************************************************************************************
*/
void serial_handler() {
  if (!serial_rdy) {                                                   // Checks if the flag is clear, hence serialInput is ready to be filled
    serial_rdy = 1;                                                    // Sets the serial_rdy flag, creating a lock on the serialInput array

    uint8_t buffer_index = 0;                                          
    char c = Serial.read();

    while (c != '\n' && c != '\r' && buffer_index < BUFFER_SIZE - 1) {
      serialInput[buffer_index++] = c;                                 // Empties the hardware buffer into the serialInput buffer
      if (!Serial.available()) {
        break;
      }
      c = Serial.read();
    }
    for(int i = buffer_index; i < BUFFER_SIZE; i++) {
      serialInput[i] = '\0';                                           // Fill the rest of the array with string terminators
    }
  }
}

/** SETUP **/ 
void setup() {
  Serial.begin(115200);                                                // Inits Serial Monitor
  Serial.onReceive(serial_handler);                                    // Sets callback for Serial channel data arrival
  
  // Initialize built-in LED
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  
  WiFi.mode(WIFI_STA);                                                 // Sets device as a Wi-Fi Station
  setEspNowChannel();

  if (esp_now_init() != ESP_OK) {                                      // Inits esp_now
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.printf("ESPB MAC Address: %s\n", WiFi.macAddress().c_str());
  Serial.printf("ESPB ESP-NOW channel: %u\n", ESPNOW_CHANNEL);
  
  esp_now_register_recv_cb(OnDataRecv);                                // Registers esp_now arrival callback
  esp_now_register_send_cb(OnDataSent);                                // Registers esp_now sending callback

  esp_wifi_set_promiscuous(true);                                      // Enables reading RSSI values from incoming packets
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  
  memcpy(peerInfo.peer_addr, MAC_ESPA, 6);                             // Registers peer by passing peer object pointer
  peerInfo.channel = ESPNOW_CHANNEL;                                   // Channel selection
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){                          // Adds peer
    Serial.println("Failed to add peer");
    return;
  }
  
  // Initialization complete
  setLEDState(LEDState::IDLE);
  Serial.println("ESPB initialized successfully");
}

/** BROKER **/
void loop() {
  // Update LED status
  updateLED();
  
  if (message_rdy) {                                                                         // Checks if there's any message ready to be read
    if (espbApplyIncomingMessage(bridgeState, input)) {
      Serial.println(input.message);
    }
    
    message_rdy = 0;                                                                       // Releases the lock on the message container
  }

  if (serial_rdy) {                            // Checks if there's any command ready to be consumed 

    EspbParsedCommand parsed = espbParseSerialCommand(serialInput);
    if (parsed.type == EspbParsedCommandType::ForwardToEspA) {
      send_command(parsed.message, MAX_CONN_TIME);
    } else if (parsed.type == EspbParsedCommandType::Status) {
      output_message dummy = makeOutputMessage(CMD_IDLE);

      const bool connectionOk = send_command(dummy, MAX_CONN_TIME);
      char statusLine[128];
      espbFormatStatus(statusLine, sizeof(statusLine), bridgeState, connectionOk);
      Serial.println(statusLine);
    }

    serial_rdy = 0;                            // Releases the lock on the command container
  }
}
