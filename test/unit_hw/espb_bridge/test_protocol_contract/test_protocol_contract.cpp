/*
 *******************************************************************************
 * GUI / ESPB / ESPA protocol contract unit test
 *******************************************************************************
 */

#include <Arduino.h>
#include <unity.h>

#include "espb_bridge_core.h"

namespace {
uint8_t commandType(EspbParsedCommandType type) {
    return static_cast<uint8_t>(type);
}
}

void setUp() {}

void tearDown() {}

void test_protocol_contract_table_matches_parser() {
    size_t count = 0;
    const EspbProtocolCommand* commands = espbProtocolCommands(count);
    TEST_ASSERT_EQUAL_UINT8(12, count);

    for (size_t i = 0; i < count; ++i) {
        char commandLine[64];
        if (commands[i].commandCode == 8) {
            snprintf(commandLine, sizeof(commandLine), "%s 1 2 3", commands[i].commandText);
        } else if (commands[i].commandCode == 9) {
            snprintf(commandLine, sizeof(commandLine), "%s 300", commands[i].commandText);
        } else if (commands[i].commandCode == 10) {
            snprintf(commandLine, sizeof(commandLine), "%s -100", commands[i].commandText);
        } else {
            snprintf(commandLine, sizeof(commandLine), "%s", commands[i].commandText);
        }

        EspbParsedCommand parsed = espbParseSerialCommand(commandLine);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(commandType(EspbParsedCommandType::ForwardToEspA),
                                        commandType(parsed.type),
                                        commands[i].commandText);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(commands[i].commandCode,
                                        parsed.message.command,
                                        commands[i].commandText);
        TEST_ASSERT_NOT_NULL_MESSAGE(commands[i].expectedAck, commands[i].commandText);
        TEST_ASSERT_NOT_EQUAL_MESSAGE('\0', commands[i].expectedAck[0], commands[i].commandText);
    }
}

void test_ack_constants_match_gui_contract() {
    TEST_ASSERT_EQUAL_STRING("FLOAT_IDLE", IDLE_ACK);
    TEST_ASSERT_EQUAL_STRING("FLOAT_IDLE_W_DATA", IDLE_W_DATA_ACK);
    TEST_ASSERT_EQUAL_STRING("GO_RECVD", CMD1_ACK);
    TEST_ASSERT_EQUAL_STRING("CMD3_RECVD", CMD3_ACK);
    TEST_ASSERT_EQUAL_STRING("CMD4_RECVD", CMD4_ACK);
    TEST_ASSERT_EQUAL_STRING("SWITCH_AM_RECVD", CMD5_ACK);
    TEST_ASSERT_EQUAL_STRING("TRY_UPLOAD_RECVD", CMD7_ACK);
    TEST_ASSERT_EQUAL_STRING("CHNG_PARMS_RECVD", CMD8_ACK);
    TEST_ASSERT_EQUAL_STRING("TEST_FREQ_RECVD", CMD9_ACK);
    TEST_ASSERT_EQUAL_STRING("TEST_STEPS_RECVD", CMD10_ACK);
    TEST_ASSERT_EQUAL_STRING("DEBUG_MODE_RECVD", CMD11_ACK);
    TEST_ASSERT_EQUAL_STRING("HOME_RECVD", CMD12_ACK);
}

void test_status_tokens_are_gui_parseable() {
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", espbStatusString(ESPB_STATUS_UNKNOWN));
    TEST_ASSERT_EQUAL_STRING("CONNECTED", espbStatusString(ESPB_STATUS_CONNECTED));
    TEST_ASSERT_EQUAL_STRING("CONNECTED_W_DATA", espbStatusString(ESPB_STATUS_CONNECTED_W_DATA));
    TEST_ASSERT_EQUAL_STRING("EXECUTING_CMD", espbStatusString(ESPB_STATUS_EXECUTING_CMD));
    TEST_ASSERT_EQUAL_STRING("STATUS_ERROR", espbStatusString(42));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_protocol_contract_table_matches_parser);
    RUN_TEST(test_ack_constants_match_gui_contract);
    RUN_TEST(test_status_tokens_are_gui_parseable);
    UNITY_END();
}

void loop() {}
