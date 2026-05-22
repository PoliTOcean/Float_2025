/*
 *******************************************************************************
 * ESPB bridge parser unit test
 *
 * Verifies GUI/Serial commands are translated to ESPA command packets without
 * requiring ESPA or ESP-NOW connectivity.
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

void test_simple_commands_map_to_espa_codes() {
    struct Case {
        const char* text;
        uint8_t code;
    };

    const Case cases[] = {
        {"GO", 1},
        {"LISTENING", 2},
        {"BALANCE", 3},
        {"CLEAR_SD", 4},
        {"SWITCH_AUTO_MODE", 5},
        {"SEND_PACKAGE", 6},
        {"TRY_UPLOAD", 7},
        {"DEBUG", 11},
        {"HOME_MOTOR", 12},
    };

    for (const Case& testCase : cases) {
        EspbParsedCommand parsed = espbParseSerialCommand(testCase.text);
        TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(parsed.type));
        TEST_ASSERT_EQUAL_UINT8(testCase.code, parsed.message.command);
    }
}

void test_parameterized_commands_fill_payload() {
    EspbParsedCommand params = espbParseSerialCommand("PARAMS 1.2 0.3 0.01");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(params.type));
    TEST_ASSERT_EQUAL_UINT8(8, params.message.command);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.2f, params.message.params[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, params.message.params[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, params.message.params[2]);

    EspbParsedCommand freq = espbParseSerialCommand("TEST_FREQ 300");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(freq.type));
    TEST_ASSERT_EQUAL_UINT8(9, freq.message.command);
    TEST_ASSERT_EQUAL_UINT16(300, freq.message.freq);

    EspbParsedCommand steps = espbParseSerialCommand("TEST_STEPS -100");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(steps.type));
    TEST_ASSERT_EQUAL_UINT8(10, steps.message.command);
    TEST_ASSERT_EQUAL_INT32(-100, steps.message.steps);
}

void test_status_is_local_command() {
    EspbParsedCommand parsed = espbParseSerialCommand("STATUS");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Status), commandType(parsed.type));
    TEST_ASSERT_EQUAL_UINT8(0, parsed.message.command);
}

void test_invalid_commands_are_rejected() {
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("UNKNOWN").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("GO extra").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PARAMS 1 2").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("TEST_FREQ -1").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("TEST_STEPS nope").type));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_simple_commands_map_to_espa_codes);
    RUN_TEST(test_parameterized_commands_fill_payload);
    RUN_TEST(test_status_is_local_command);
    RUN_TEST(test_invalid_commands_are_rejected);
    UNITY_END();
}

void loop() {}
