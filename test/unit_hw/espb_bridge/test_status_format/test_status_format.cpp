/*
 *******************************************************************************
 * ESPB bridge status formatting unit test
 *******************************************************************************
 */

#include <Arduino.h>
#include <unity.h>

#include "espb_bridge_core.h"

void setUp() {}

void tearDown() {}

void test_initial_status_is_unknown_and_connection_lost() {
    EspbBridgeState state;
    char line[128];

    espbFormatStatus(line, sizeof(line), state, false);
    TEST_ASSERT_EQUAL_STRING("UNKNOWN | AUTO_MODE_NO | CONN_LOST | BATTERY: 0 | RSSI: 0", line);
}

void test_status_formatter_keeps_five_gui_fields() {
    EspbBridgeState state;
    state.status = ESPB_STATUS_CONNECTED_W_DATA;
    state.autoModeActive = true;
    state.batteryCharge = 12450;
    state.lastRssi = -63;

    char line[128];
    espbFormatStatus(line, sizeof(line), state, true);

    TEST_ASSERT_EQUAL_STRING("CONNECTED_W_DATA | AUTO_MODE_YES | CONN_OK | BATTERY: 12450 | RSSI: -63", line);

    uint8_t separators = 0;
    for (const char* p = line; *p != '\0'; ++p) {
        if (p[0] == ' ' && p[1] == '|' && p[2] == ' ') {
            separators++;
        }
    }
    TEST_ASSERT_EQUAL_UINT8(4, separators);
}

void test_incoming_messages_update_cached_state() {
    EspbBridgeState state;
    input_message message;
    memset(&message, 0, sizeof(message));

    message.charge = 12050;
    strncpy(message.message, IDLE_ACK, sizeof(message.message) - 1);
    TEST_ASSERT_FALSE(espbApplyIncomingMessage(state, message));
    TEST_ASSERT_EQUAL_INT8(ESPB_STATUS_CONNECTED, state.status);
    TEST_ASSERT_EQUAL_UINT16(12050, state.batteryCharge);

    memset(&message, 0, sizeof(message));
    message.charge = 12100;
    strncpy(message.message, IDLE_W_DATA_ACK, sizeof(message.message) - 1);
    TEST_ASSERT_FALSE(espbApplyIncomingMessage(state, message));
    TEST_ASSERT_EQUAL_INT8(ESPB_STATUS_CONNECTED_W_DATA, state.status);

    memset(&message, 0, sizeof(message));
    message.charge = 12200;
    strncpy(message.message, CMD5_ACK, sizeof(message.message) - 1);
    TEST_ASSERT_TRUE(espbApplyIncomingMessage(state, message));
    TEST_ASSERT_EQUAL_INT8(ESPB_STATUS_EXECUTING_CMD, state.status);
    TEST_ASSERT_TRUE(state.autoModeActive);
}

void test_invalid_status_code_is_status_error() {
    TEST_ASSERT_EQUAL_STRING("STATUS_ERROR", espbStatusString(99));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_initial_status_is_unknown_and_connection_lost);
    RUN_TEST(test_status_formatter_keeps_five_gui_fields);
    RUN_TEST(test_incoming_messages_update_cached_state);
    RUN_TEST(test_invalid_status_code_is_status_error);
    UNITY_END();
}

void loop() {}
