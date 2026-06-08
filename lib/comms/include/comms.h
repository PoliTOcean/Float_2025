#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "float_common.h"

/*
 *******************************************************************************
 * comms.h
 * ESP-NOW communication and OTA firmware update management.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

class CommsManager {
public:
    CommsManager();

    // Call once in setup() — initialises WiFi in STA mode and ESP-NOW
    void begin();

    // Send a message string to ESPB.
    // Blocks for up to timeoutMs waiting for the send callback.
    // Returns true if the peer acknowledged delivery.
    bool sendMessage(const char* message, uint32_t timeoutMs = 1000);

    // Access the last received command (set by the ESP-NOW receive callback).
    // Returns a snapshot by value: the receive callback runs in the WiFi task,
    // so the read/write must be guarded to avoid a torn struct.
    output_message lastCommand() const;
    void clearCommand();

    // Outgoing packet — callers fill status_to_send.charge before calling sendMessage()
    input_message status_to_send;

    // Start an OTA access point, serve updates for 5 minutes, then restore ESP-NOW.
    // Blocks for the entire OTA window.
    void runOTASession();

private:
    esp_now_peer_info_t _peerInfo;
    output_message      _received;
    volatile int8_t     _sendResult = -1; // -1=pending, 0=fail, 1=success

    // Guards _received against concurrent access between the WiFi-task receive
    // callback (writer) and the main loop (reader).
    mutable portMUX_TYPE _recvMux = portMUX_INITIALIZER_UNLOCKED;

    void _initEspNow();
    void _deInitEspNow();
    void _reInitEspNow();

    // ESP-NOW callbacks (must be static to match C function pointer signature)
    static void _onDataSent(const uint8_t* mac, esp_now_send_status_t status);
    static void _onDataRecv(const uint8_t* mac, const uint8_t* data, int len);

    // Pointer back to the singleton so static callbacks can reach it
    static CommsManager* _instance;
};

// Singleton
extern CommsManager comms;
