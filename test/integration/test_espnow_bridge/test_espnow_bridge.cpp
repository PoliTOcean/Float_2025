/*
 *******************************************************************************
 * ESPB <-> ESPA ESP-NOW bridge integration test
 *
 * Requires ESPA powered on, flashed with the real espA firmware, and initialized
 * to idle. This test only uses the connection dummy command and AUTO_MODE toggle;
 * it does not move the motor or start a profile.
 *******************************************************************************
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <unity.h>

#include "config.h"
#include "espb_bridge_core.h"

namespace {
esp_now_peer_info_t peerInfo;
esp_now_peer_info_t broadcastPeerInfo;
volatile int8_t sendResult = -1;
volatile bool receivedReady = false;
input_message receivedMessage;
constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setEspNowChannel() {
    esp_wifi_set_promiscuous(true);
    const esp_err_t err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (err != ESP_OK) {
        Serial.printf("ESPB test set channel failed: %d\n", static_cast<int>(err));
    }
}

void onDataSent(const uint8_t*, esp_now_send_status_t status) {
    sendResult = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;
}

void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
    if (len == sizeof(input_message)) {
        memcpy(&receivedMessage, data, sizeof(receivedMessage));
        receivedReady = true;
    }
}

bool sendCommandTo(const uint8_t* peerMac, uint8_t command, uint32_t timeoutMs);

bool sendCommand(uint8_t command, uint32_t timeoutMs) {
    return sendCommandTo(MAC_ESPA, command, timeoutMs);
}

bool sendBroadcastCommand(uint8_t command, uint32_t timeoutMs) {
    return sendCommandTo(BROADCAST_MAC, command, timeoutMs);
}

bool sendCommandTo(const uint8_t* peerMac, uint8_t command, uint32_t timeoutMs) {
    output_message message;
    memset(&message, 0, sizeof(message));
    message.command = command;

    sendResult = -1;
    const esp_err_t err = esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&message), sizeof(message));
    if (err != ESP_OK) {
        Serial.printf("ESPB test esp_now_send returned error: %d\n", static_cast<int>(err));
        return false;
    }

    const unsigned long startMs = millis();
    while (sendResult == -1 && millis() - startMs < timeoutMs) {
        delay(5);
        yield();
    }

    return sendResult == 1;
}

void formatRadioDebug(char* buffer, size_t size, const char* prefix) {
    uint8_t primaryChannel = 0;
    wifi_second_chan_t secondChannel = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primaryChannel, &secondChannel);

    snprintf(buffer, size,
             "%s | ESPB test MAC=%s configChannel=%u actualChannel=%u peer ESPA=%02X:%02X:%02X:%02X:%02X:%02X sendResult=%d",
             prefix,
             WiFi.macAddress().c_str(),
             ESPNOW_CHANNEL,
             primaryChannel,
             MAC_ESPA[0], MAC_ESPA[1], MAC_ESPA[2],
             MAC_ESPA[3], MAC_ESPA[4], MAC_ESPA[5],
             static_cast<int>(sendResult));
}

bool waitForMessage(const char* expected, uint32_t timeoutMs) {
    const unsigned long startMs = millis();
    while (millis() - startMs < timeoutMs) {
        if (receivedReady) {
            noInterrupts();
            input_message localCopy = receivedMessage;
            receivedReady = false;
            interrupts();

            if (strcmp(localCopy.message, expected) == 0) {
                return true;
            }
        }
        delay(10);
        yield();
    }
    return false;
}

bool waitForIdle(uint32_t timeoutMs) {
    const unsigned long startMs = millis();
    while (millis() - startMs < timeoutMs) {
        if (receivedReady) {
            noInterrupts();
            input_message localCopy = receivedMessage;
            receivedReady = false;
            interrupts();

            if (strcmp(localCopy.message, IDLE_ACK) == 0 ||
                strcmp(localCopy.message, IDLE_W_DATA_ACK) == 0) {
                return true;
            }
        }
        delay(10);
        yield();
    }
    return false;
}
}

void setUp() {}

void tearDown() {}

void test_espnow_dummy_command_reaches_real_espa() {
    const bool sent = sendCommand(0, 3000);
    char failure[192];
    formatRadioDebug(failure, sizeof(failure), "Dummy command did not reach ESPA");
    TEST_ASSERT_TRUE_MESSAGE(sent, failure);
}

void test_real_espa_idle_heartbeat_is_received() {
    TEST_ASSERT_TRUE_MESSAGE(waitForIdle(7000),
                             "No FLOAT_IDLE/FLOAT_IDLE_W_DATA heartbeat received from ESPA");
}

void test_auto_mode_toggle_roundtrip_and_restore() {
    TEST_ASSERT_TRUE_MESSAGE(sendCommand(5, 1000), "SWITCH_AUTO_MODE send failed");
    TEST_ASSERT_TRUE_MESSAGE(waitForMessage(CMD5_ACK, 3000), "Missing SWITCH_AM_RECVD after first toggle");

    TEST_ASSERT_TRUE_MESSAGE(sendCommand(5, 1000), "SWITCH_AUTO_MODE restore send failed");
    TEST_ASSERT_TRUE_MESSAGE(waitForMessage(CMD5_ACK, 3000), "Missing SWITCH_AM_RECVD after restore toggle");
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    setEspNowChannel();

    UNITY_BEGIN();

    if (esp_now_init() != ESP_OK) {
        TEST_FAIL_MESSAGE("ESP-NOW init failed");
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, MAC_ESPA, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        TEST_FAIL_MESSAGE("ESP-NOW add peer failed");
    }

    RUN_TEST(test_espnow_dummy_command_reaches_real_espa);
    RUN_TEST(test_real_espa_idle_heartbeat_is_received);
    RUN_TEST(test_auto_mode_toggle_roundtrip_and_restore);

    UNITY_END();
}

void loop() {}
