#include "comms.h"
#include "../config.h"
#include "../led/led.h"
#include <DebugSerial.h>

/*
 *******************************************************************************
 * comms.cpp
 *******************************************************************************
 */

// debug_mode_active is defined in main.cpp and shared across modules
extern bool debug_mode_active;

CommsManager* CommsManager::_instance = nullptr;
CommsManager  comms;

// ---------------------------------------------------------------------------
CommsManager::CommsManager() {
    _instance = this;
    memset(&status_to_send, 0, sizeof(status_to_send));
    memset(&_received,      0, sizeof(_received));
}

// ---------------------------------------------------------------------------
void CommsManager::begin() {
    WiFi.mode(WIFI_STA);
    Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
    _initEspNow();
}

// ---------------------------------------------------------------------------
void CommsManager::_initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("CRITICAL: ESP-NOW init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    esp_now_register_send_cb(_onDataSent);
    esp_now_register_recv_cb(_onDataRecv);

    memcpy(_peerInfo.peer_addr, MAC_ESPB, 6);
    _peerInfo.channel = 0;
    _peerInfo.encrypt = false;

    if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
        Serial.println("CRITICAL: ESP-NOW add peer failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    Debug.println("ESP-NOW initialised");
}

// ---------------------------------------------------------------------------
void CommsManager::_deInitEspNow() {
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
}

// ---------------------------------------------------------------------------
void CommsManager::_reInitEspNow() {
    WiFi.mode(WIFI_STA);
    delay(100);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW re-init failed");
        ledController.setState(LEDState::ERROR);
        return;
    }
    esp_now_register_send_cb(_onDataSent);
    esp_now_register_recv_cb(_onDataRecv);
    esp_now_add_peer(&_peerInfo);
    Serial.println("ESP-NOW re-initialised");
}

// ---------------------------------------------------------------------------
bool CommsManager::sendMessage(const char* message, uint32_t timeoutMs) {
    strncpy(status_to_send.message, message, sizeof(status_to_send.message) - 1);

    _sendResult = -1;
    esp_err_t err = esp_now_send(
        const_cast<uint8_t*>(MAC_ESPB),
        reinterpret_cast<const uint8_t*>(&status_to_send),
        sizeof(status_to_send)
    );

    if (err != ESP_OK) {
        Debug.println("ESP-NOW: send call failed");
        return false;
    }

    unsigned long t0 = millis();
    while (_sendResult == -1 && (millis() - t0) < timeoutMs) {
        delay(10);
        ledController.update();
    }

    return _sendResult == 1;
}

// ---------------------------------------------------------------------------
void CommsManager::runOTASession() {
    const bool debugWasActive = debug_mode_active;

    if (debugWasActive) {
        debug_mode_active = false; // Disable remote debug to avoid recursion
        Serial.println("Remote debug paused for OTA");
    }

    _deInitEspNow();

    // Start Access Point
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("ERROR: Failed to create Access Point for OTA");
        _reInitEspNow();
        if (debugWasActive) debug_mode_active = true;
        return;
    }

    Serial.printf("OTA AP ready — connect to \"%s\" then go to http://%s/update\n",
                  WIFI_SSID, WiFi.softAPIP().toString().c_str());

    WebServer httpServer(80);
    ElegantOTA.begin(&httpServer);
    httpServer.begin();
    ledController.setState(LEDState::OTA_MODE);

    const unsigned long OTA_WINDOW_MS = 5UL * 60UL * 1000UL; // 5 minutes
    unsigned long t0 = millis();
    Serial.println("OTA server running for 5 minutes...");
    while (millis() - t0 < OTA_WINDOW_MS) {
        httpServer.handleClient();
        ElegantOTA.loop();
        ledController.update();
        delay(10);
    }

    httpServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    _reInitEspNow();

    if (debugWasActive) {
        debug_mode_active = true;
        Debug.println("Remote debug restored after OTA");
    }
}

// ---------------------------------------------------------------------------
// Static ESP-NOW callbacks
// ---------------------------------------------------------------------------
void CommsManager::_onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (_instance) _instance->_sendResult = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
}

void CommsManager::_onDataRecv(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (_instance && len == sizeof(output_message)) {
        memcpy(&_instance->_received, data, sizeof(output_message));
    }
}