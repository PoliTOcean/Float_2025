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
    TEST_ASSERT_EQUAL_UINT8(23, count);

    for (size_t i = 0; i < count; ++i) {
        char commandLine[64];
        switch (commands[i].commandCode) {
            case CMD_PID_CONFIG_SET:
                snprintf(commandLine, sizeof(commandLine), "%s 1 2 3 50 0.25 5 0.001 0.011", commands[i].commandText);
                break;
            case CMD_BALANCE_CONFIG_SET:
                snprintf(commandLine, sizeof(commandLine), "%s 5000 5 3 50", commands[i].commandText);
                break;
            case CMD_MOTOR_CONFIG_SET:
                snprintf(commandLine, sizeof(commandLine), "%s 1800 1800 1200 300", commands[i].commandText);
                break;
            case CMD_TEST_STEPS:
                snprintf(commandLine, sizeof(commandLine), "%s -100", commands[i].commandText);
                break;
            case CMD_SYRINGE_SET:
                snprintf(commandLine, sizeof(commandLine), "%s 0.5 5", commands[i].commandText);
                break;
            case CMD_PID_HOLD:
                snprintf(commandLine, sizeof(commandLine), "%s 1.5 30", commands[i].commandText);
                break;
            case CMD_PID_STEP:
                snprintf(commandLine, sizeof(commandLine), "%s 1.5", commands[i].commandText);
                break;
            case CMD_SET_SURFACE_OFFSET:
                snprintf(commandLine, sizeof(commandLine), "%s 0.10", commands[i].commandText);
                break;
            case CMD_PROFILE_SET:
                snprintf(commandLine, sizeof(commandLine), "%s 2 2.5 0.4 0.33 30 180 120 0.10", commands[i].commandText);
                break;
            default:
                snprintf(commandLine, sizeof(commandLine), "%s", commands[i].commandText);
                break;
        }

        EspbParsedCommand parsed = espbParseSerialCommand(commandLine);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(commandType(EspbParsedCommandType::ForwardToEspA),
                                        commandType(parsed.type),
                                        commands[i].commandText);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(static_cast<uint8_t>(commands[i].commandCode),
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
    TEST_ASSERT_EQUAL_STRING("PID_CONFIG_SET_RECVD", CMD8_ACK);
    TEST_ASSERT_EQUAL_STRING("PID_CONFIG_SET_ERR", CMD8_ERR);
    TEST_ASSERT_EQUAL_STRING("TEST_STEPS_RECVD", CMD10_ACK);
    TEST_ASSERT_EQUAL_STRING("DEBUG_MODE_RECVD", CMD11_ACK);
    TEST_ASSERT_EQUAL_STRING("HOME_RECVD", CMD12_ACK);
    TEST_ASSERT_EQUAL_STRING("STOP_RECVD", CMD13_ACK);
    TEST_ASSERT_EQUAL_STRING("SYRINGE_SET_RECVD", CMD15_ACK);
    TEST_ASSERT_EQUAL_STRING("PID_HOLD_RECVD", CMD16_ACK);
    TEST_ASSERT_EQUAL_STRING("PID_STEP_RECVD", CMD17_ACK);
    TEST_ASSERT_EQUAL_STRING("SURFACE_OFF_RECVD", CMD18_ACK);
    TEST_ASSERT_EQUAL_STRING("PROFILE_SET_RECVD", CMD19_ACK);
    TEST_ASSERT_EQUAL_STRING("PROFILE_SET_ERR", CMD19_ERR);
    TEST_ASSERT_EQUAL_STRING("BALANCE_CONFIG_SET_RECVD", CMD21_ACK);
    TEST_ASSERT_EQUAL_STRING("BALANCE_CONFIG_SET_ERR", CMD21_ERR);
    TEST_ASSERT_EQUAL_STRING("MOTOR_CONFIG_SET_RECVD", CMD23_ACK);
    TEST_ASSERT_EQUAL_STRING("MOTOR_CONFIG_SET_ERR", CMD23_ERR);
}

void test_reserved_command_is_not_parseable() {
    EspbParsedCommand parsed = espbParseSerialCommand("CMD_RESERVED_9");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(parsed.type));
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
    RUN_TEST(test_reserved_command_is_not_parseable);
    RUN_TEST(test_status_tokens_are_gui_parseable);
    UNITY_END();
}

void loop() {}
