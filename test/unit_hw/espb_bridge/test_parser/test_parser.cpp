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
        {"GO", CMD_GO},
        {"LISTENING", CMD_SEND_DATA},
        {"BALANCE", CMD_BALANCE},
        {"CLEAR_SD", CMD_CLEAR_EEPROM},
        {"SWITCH_AUTO_MODE", CMD_AUTO_MODE},
        {"SEND_PACKAGE", CMD_SEND_PACKAGE},
        {"TRY_UPLOAD", CMD_OTA},
        {"DEBUG", CMD_DEBUG_MODE},
        {"HOME_MOTOR", CMD_HOME},
        {"STOP", CMD_STOP},
        {"PID_CONFIG_GET", CMD_PID_CONFIG_GET},
        {"PROFILE_GET", CMD_PROFILE_GET},
        {"BALANCE_CONFIG_GET", CMD_BALANCE_CONFIG_GET},
        {"MOTOR_CONFIG_GET", CMD_MOTOR_CONFIG_GET},
    };

    for (const Case& testCase : cases) {
        EspbParsedCommand parsed = espbParseSerialCommand(testCase.text);
        TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(parsed.type));
        TEST_ASSERT_EQUAL_UINT8(testCase.code, parsed.message.command);
    }
}

void test_parameterized_commands_fill_payload() {
    EspbParsedCommand params = espbParseSerialCommand("PID_CONFIG_SET 1.2 0.3 0.01 50 0.25 5 0.001 0.011");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(params.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_PID_CONFIG_SET, params.message.command);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.2f, params.message.payload.pidConfig.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, params.message.payload.pidConfig.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, params.message.payload.pidConfig.kd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, params.message.payload.pidConfig.periodMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, params.message.payload.pidConfig.alphaD);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, params.message.payload.pidConfig.integralLimit);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.001f, params.message.payload.pidConfig.minRetargetFrac);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.011f, params.message.payload.pidConfig.uNeutral);

    EspbParsedCommand balance = espbParseSerialCommand("BALANCE_CONFIG_SET 5000 5.0 3 50");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(balance.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_BALANCE_CONFIG_SET, balance.message.command);
    TEST_ASSERT_EQUAL_UINT32(5000, balance.message.payload.balanceConfig.holdMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, balance.message.payload.balanceConfig.stopPressureDeltaKpa);
    TEST_ASSERT_EQUAL_UINT8(3, balance.message.payload.balanceConfig.stopPressureSamples);
    TEST_ASSERT_EQUAL_UINT16(50, balance.message.payload.balanceConfig.samplePeriodMs);

    EspbParsedCommand motorConfig = espbParseSerialCommand("MOTOR_CONFIG_SET 1800 1800 1200 300");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(motorConfig.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_MOTOR_CONFIG_SET, motorConfig.message.command);
    TEST_ASSERT_EQUAL_UINT32(1800, motorConfig.message.payload.motorConfig.maxSpeed);
    TEST_ASSERT_EQUAL_UINT32(1800, motorConfig.message.payload.motorConfig.maxAcceleration);
    TEST_ASSERT_EQUAL_UINT32(1200, motorConfig.message.payload.motorConfig.homingSpeed);
    TEST_ASSERT_EQUAL_UINT32(300, motorConfig.message.payload.motorConfig.testSpeed);

    EspbParsedCommand steps = espbParseSerialCommand("TEST_STEPS -100");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(steps.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_TEST_STEPS, steps.message.command);
    TEST_ASSERT_EQUAL_INT32(-100, steps.message.payload.testSteps.steps);

    EspbParsedCommand profile = espbParseSerialCommand("PROFILE_SET 2 2.5 0.4 0.33 30 180 120 0.10");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::ForwardToEspA), commandType(profile.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_PROFILE_SET, profile.message.command);
    TEST_ASSERT_EQUAL_UINT8(2, profile.message.payload.profileSet.profileCount);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, profile.message.payload.profileSet.descentTargetM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, profile.message.payload.profileSet.ascentTargetM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.33f, profile.message.payload.profileSet.depthToleranceM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, profile.message.payload.profileSet.holdTimeS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 180.0f, profile.message.payload.profileSet.descentTimeoutS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 120.0f, profile.message.payload.profileSet.ascentTimeoutS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.10f, profile.message.payload.profileSet.surfaceRestOffsetM);
}

void test_output_message_protocol_shape() {
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(250, sizeof(output_message));

    output_message message = makeOutputMessage(CMD_GO);
    TEST_ASSERT_EQUAL_UINT8(CMD_GO, message.command);
    TEST_ASSERT_EQUAL_UINT8(0, message.payload.empty.reserved);
}

void test_status_is_local_command() {
    EspbParsedCommand parsed = espbParseSerialCommand("STATUS");
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Status), commandType(parsed.type));
    TEST_ASSERT_EQUAL_UINT8(CMD_IDLE, parsed.message.command);
}

void test_invalid_commands_are_rejected() {
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("UNKNOWN").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("GO extra").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PARAMS 1 2 3").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PARAMS_EXT 50 0.25").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("TEST_FREQ 300").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PID_CONFIG_SET 1 2 3").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("BALANCE_CONFIG_SET 5000 5 3").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("MOTOR_CONFIG_SET 1800 1800 1200").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("TEST_STEPS nope").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PROFILE_GET extra").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PROFILE_SET 0 2.5 0.4 0.33 30 180 120 0.10").type));
    TEST_ASSERT_EQUAL_UINT8(commandType(EspbParsedCommandType::Invalid), commandType(espbParseSerialCommand("PROFILE_SET 2 2.5 0.4 0.33 30 180 120").type));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_simple_commands_map_to_espa_codes);
    RUN_TEST(test_parameterized_commands_fill_payload);
    RUN_TEST(test_output_message_protocol_shape);
    RUN_TEST(test_status_is_local_command);
    RUN_TEST(test_invalid_commands_are_rejected);
    UNITY_END();
}

void loop() {}
