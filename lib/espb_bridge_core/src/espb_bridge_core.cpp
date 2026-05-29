#include "espb_bridge_core.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

/*
 *******************************************************************************
 * espb_bridge_core.cpp
 * Pure-logic implementation of the ESPB bridge: GUI command string parsing
 * (table-driven), parameter extraction, and STATUS line formatting. No
 * hardware or framework dependencies, to remain unit-testable.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

namespace {
constexpr size_t COMMAND_BUFFER_SIZE = 192;

constexpr EspbProtocolCommand PROTOCOL_COMMANDS[] = {
    {"GO", CMD_GO, CMD1_ACK},
    {"LISTENING", CMD_SEND_DATA, "DATA_OR_STOP_DATA"},
    {"BALANCE", CMD_BALANCE, CMD3_ACK},
    {"CLEAR_SD", CMD_CLEAR_EEPROM, CMD4_ACK},
    {"SWITCH_AUTO_MODE", CMD_AUTO_MODE, CMD5_ACK},
    {"SEND_PACKAGE", CMD_SEND_PACKAGE, "JSON_LIVE_PACKET"},
    {"TRY_UPLOAD", CMD_OTA, CMD7_ACK},
    {"PID_CONFIG_SET", CMD_PID_CONFIG_SET, CMD8_ACK},
    {"TEST_STEPS", CMD_TEST_STEPS, CMD10_ACK},
    {"DEBUG", CMD_DEBUG_MODE, CMD11_ACK},
    {"HOME_MOTOR", CMD_HOME, CMD12_ACK},
    {"STOP", CMD_STOP, CMD13_ACK},
    {"PID_CONFIG_GET", CMD_PID_CONFIG_GET, "PID_CONFIG_JSON"},
    {"SYRINGE_SET", CMD_SYRINGE_SET, CMD15_ACK},
    {"PID_HOLD", CMD_PID_HOLD, CMD16_ACK},
    {"PID_STEP", CMD_PID_STEP, CMD17_ACK},
    {"SURFACE_OFFSET", CMD_SET_SURFACE_OFFSET, CMD18_ACK},
    {"PROFILE_SET", CMD_PROFILE_SET, CMD19_ACK},
    {"PROFILE_GET", CMD_PROFILE_GET, "PROFILE_JSON"},
    {"BALANCE_CONFIG_SET", CMD_BALANCE_CONFIG_SET, CMD21_ACK},
    {"BALANCE_CONFIG_GET", CMD_BALANCE_CONFIG_GET, "BALANCE_CONFIG_JSON"},
    {"MOTOR_CONFIG_SET", CMD_MOTOR_CONFIG_SET, CMD23_ACK},
    {"MOTOR_CONFIG_GET", CMD_MOTOR_CONFIG_GET, "MOTOR_CONFIG_JSON"},
};

void copyTrimmedCommand(const char* input, char* output, size_t outputSize) {
    if (outputSize == 0) {
        return;
    }

    size_t start = 0;
    while (input[start] != '\0' && isspace(static_cast<unsigned char>(input[start]))) {
        start++;
    }

    size_t end = strlen(input + start);
    while (end > 0 && isspace(static_cast<unsigned char>(input[start + end - 1]))) {
        end--;
    }

    if (end >= outputSize) {
        end = outputSize - 1;
    }

    memcpy(output, input + start, end);
    output[end] = '\0';
}

bool parseFloatToken(const char* token, float& value) {
    if (token == nullptr || *token == '\0') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    value = strtof(token, &end);
    return errno == 0 && end != token && *end == '\0';
}

bool parseLongToken(const char* token, long& value) {
    if (token == nullptr || *token == '\0') {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    value = strtol(token, &end, 10);
    return errno == 0 && end != token && *end == '\0';
}

bool hasNoExtraToken() {
    return strtok(nullptr, " ") == nullptr;
}

EspbParsedCommand makeForwardCommand(FloatCommand commandCode) {
    EspbParsedCommand parsed;
    parsed.type = EspbParsedCommandType::ForwardToEspA;
    parsed.message = makeOutputMessage(commandCode);
    return parsed;
}
}

EspbParsedCommand espbParseSerialCommand(const char* line) {
    EspbParsedCommand parsed;
    parsed.message = makeOutputMessage(CMD_IDLE);

    if (line == nullptr) {
        return parsed;
    }

    char commandLine[COMMAND_BUFFER_SIZE];
    copyTrimmedCommand(line, commandLine, sizeof(commandLine));
    if (commandLine[0] == '\0') {
        return parsed;
    }

    char* token = strtok(commandLine, " ");
    if (token == nullptr) {
        return parsed;
    }

    if (strcmp(token, "STATUS") == 0) {
        if (!hasNoExtraToken()) {
            return parsed;
        }
        parsed.type = EspbParsedCommandType::Status;
        return parsed;
    }

    if (strcmp(token, "SYRINGE_SET") == 0) {
        // SYRINGE_SET <u_norm> <duration_s>
        float u = 0.0f;
        float dur = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), u) ||
            !parseFloatToken(strtok(nullptr, " "), dur) ||
            !hasNoExtraToken()) {
            return parsed;
        }
        parsed = makeForwardCommand(CMD_SYRINGE_SET);
        parsed.message.payload.syringeSet.uNorm = u;
        parsed.message.payload.syringeSet.durationS = dur;
        return parsed;
    }

    if (strcmp(token, "PID_HOLD") == 0) {
        // PID_HOLD <depth_m> <duration_s>
        float depth = 0.0f;
        float dur = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), depth) ||
            !parseFloatToken(strtok(nullptr, " "), dur) ||
            !hasNoExtraToken()) {
            return parsed;
        }
        parsed = makeForwardCommand(CMD_PID_HOLD);
        parsed.message.payload.pidHold.depthM = depth;
        parsed.message.payload.pidHold.durationS = dur;
        return parsed;
    }

    if (strcmp(token, "PID_STEP") == 0) {
        // PID_STEP <depth_m>
        float depth = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), depth) ||
            !hasNoExtraToken()) {
            return parsed;
        }
        parsed = makeForwardCommand(CMD_PID_STEP);
        parsed.message.payload.pidStep.depthM = depth;
        return parsed;
    }

    if (strcmp(token, "SURFACE_OFFSET") == 0) {
        // SURFACE_OFFSET <metres>
        float offset = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), offset) ||
            !hasNoExtraToken()) {
            return parsed;
        }
        parsed = makeForwardCommand(CMD_SET_SURFACE_OFFSET);
        parsed.message.payload.surfaceOffset.meters = offset;
        return parsed;
    }

    if (strcmp(token, "PROFILE_SET") == 0) {
        // PROFILE_SET <count> <deep> <shallow_top> <tol> <hold> <pid_timeout> <ascent_timeout> <surface_offset>
        long count = 0;
        float values[7] = {};
        if (!parseLongToken(strtok(nullptr, " "), count) ||
            count < 1 || count > 10) {
            return parsed;
        }

        for (float& value : values) {
            if (!parseFloatToken(strtok(nullptr, " "), value)) {
                return parsed;
            }
        }

        if (!hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_PROFILE_SET);
        parsed.message.payload.profileSet.profileCount = static_cast<uint8_t>(count);
        parsed.message.payload.profileSet.deepTargetM = values[0];
        parsed.message.payload.profileSet.shallowTopTargetM = values[1];
        parsed.message.payload.profileSet.depthToleranceM = values[2];
        parsed.message.payload.profileSet.holdTimeS = values[3];
        parsed.message.payload.profileSet.pidTimeoutS = values[4];
        parsed.message.payload.profileSet.ascentTimeoutS = values[5];
        parsed.message.payload.profileSet.surfaceOffsetM = values[6];
        return parsed;
    }

    if (strcmp(token, "PID_CONFIG_SET") == 0) {
        float values[8] = {};
        for (float& value : values) {
            if (!parseFloatToken(strtok(nullptr, " "), value)) {
                return parsed;
            }
        }

        if (!hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_PID_CONFIG_SET);
        parsed.message.payload.pidConfig.kp = values[0];
        parsed.message.payload.pidConfig.ki = values[1];
        parsed.message.payload.pidConfig.kd = values[2];
        parsed.message.payload.pidConfig.periodMs = values[3];
        parsed.message.payload.pidConfig.alphaD = values[4];
        parsed.message.payload.pidConfig.integralLimit = values[5];
        parsed.message.payload.pidConfig.minRetargetFrac = values[6];
        parsed.message.payload.pidConfig.uNeutral = values[7];
        return parsed;
    }

    if (strcmp(token, "BALANCE_CONFIG_SET") == 0) {
        long holdMs = 0;
        float stopDeltaKpa = 0.0f;
        long stopSamples = 0;
        long samplePeriodMs = 0;
        if (!parseLongToken(strtok(nullptr, " "), holdMs) ||
            !parseFloatToken(strtok(nullptr, " "), stopDeltaKpa) ||
            !parseLongToken(strtok(nullptr, " "), stopSamples) ||
            !parseLongToken(strtok(nullptr, " "), samplePeriodMs) ||
            holdMs < 0 ||
            stopSamples < 0 || stopSamples > UINT8_MAX ||
            samplePeriodMs < 0 || samplePeriodMs > UINT16_MAX ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_BALANCE_CONFIG_SET);
        parsed.message.payload.balanceConfig.holdMs = static_cast<uint32_t>(holdMs);
        parsed.message.payload.balanceConfig.stopPressureDeltaKpa = stopDeltaKpa;
        parsed.message.payload.balanceConfig.stopPressureSamples = static_cast<uint8_t>(stopSamples);
        parsed.message.payload.balanceConfig.samplePeriodMs = static_cast<uint16_t>(samplePeriodMs);
        return parsed;
    }

    if (strcmp(token, "MOTOR_CONFIG_SET") == 0) {
        long maxSpeed = 0;
        long maxAcceleration = 0;
        long homingSpeed = 0;
        long testSpeed = 0;
        if (!parseLongToken(strtok(nullptr, " "), maxSpeed) ||
            !parseLongToken(strtok(nullptr, " "), maxAcceleration) ||
            !parseLongToken(strtok(nullptr, " "), homingSpeed) ||
            !parseLongToken(strtok(nullptr, " "), testSpeed) ||
            maxSpeed < 0 ||
            maxAcceleration < 0 ||
            homingSpeed < 0 ||
            testSpeed < 0 ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_MOTOR_CONFIG_SET);
        parsed.message.payload.motorConfig.maxSpeed = static_cast<uint32_t>(maxSpeed);
        parsed.message.payload.motorConfig.maxAcceleration = static_cast<uint32_t>(maxAcceleration);
        parsed.message.payload.motorConfig.homingSpeed = static_cast<uint32_t>(homingSpeed);
        parsed.message.payload.motorConfig.testSpeed = static_cast<uint32_t>(testSpeed);
        return parsed;
    }

    if (strcmp(token, "TEST_STEPS") == 0) {
        long steps = 0;
        if (!parseLongToken(strtok(nullptr, " "), steps) ||
            steps < INT32_MIN || steps > INT32_MAX ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_TEST_STEPS);
        parsed.message.payload.testSteps.steps = static_cast<int32_t>(steps);
        return parsed;
    }

    for (const EspbProtocolCommand& command : PROTOCOL_COMMANDS) {
        if (strcmp(token, command.commandText) == 0) {
            if (command.commandCode == CMD_PID_CONFIG_SET ||
                command.commandCode == CMD_BALANCE_CONFIG_SET ||
                command.commandCode == CMD_MOTOR_CONFIG_SET ||
                command.commandCode == CMD_TEST_STEPS ||
                command.commandCode == CMD_SYRINGE_SET ||
                command.commandCode == CMD_PID_HOLD ||
                command.commandCode == CMD_PID_STEP ||
                command.commandCode == CMD_SET_SURFACE_OFFSET ||
                command.commandCode == CMD_PROFILE_SET ||
                !hasNoExtraToken()) {
                return parsed;
            }
            return makeForwardCommand(command.commandCode);
        }
    }

    return parsed;
}

bool espbApplyIncomingMessage(EspbBridgeState& state, const input_message& message) {
    state.batteryCharge = message.charge;

    if (strcmp(message.message, IDLE_ACK) == 0) {
        state.status = ESPB_STATUS_CONNECTED;
        return false;
    }

    if (strcmp(message.message, IDLE_W_DATA_ACK) == 0) {
        state.status = ESPB_STATUS_CONNECTED_W_DATA;
        return false;
    }

    state.status = ESPB_STATUS_EXECUTING_CMD;
    if (strcmp(message.message, CMD5_ACK) == 0) {
        state.autoModeActive = !state.autoModeActive;
    }

    return true;
}

const char* espbStatusString(int8_t status) {
    switch (status) {
        case ESPB_STATUS_UNKNOWN:
            return "UNKNOWN";
        case ESPB_STATUS_CONNECTED:
            return "CONNECTED";
        case ESPB_STATUS_CONNECTED_W_DATA:
            return "CONNECTED_W_DATA";
        case ESPB_STATUS_EXECUTING_CMD:
            return "EXECUTING_CMD";
        default:
            return "STATUS_ERROR";
    }
}

const EspbProtocolCommand* espbProtocolCommands(size_t& count) {
    count = sizeof(PROTOCOL_COMMANDS) / sizeof(PROTOCOL_COMMANDS[0]);
    return PROTOCOL_COMMANDS;
}

void espbFormatStatus(char* buffer,
                      size_t bufferSize,
                      const EspbBridgeState& state,
                      bool connectionOk) {
    if (buffer == nullptr || bufferSize == 0) {
        return;
    }

    snprintf(buffer,
             bufferSize,
             "%s | %s | %s | BATTERY: %u | RSSI: %d",
             espbStatusString(state.status),
             state.autoModeActive ? "AUTO_MODE_YES" : "AUTO_MODE_NO",
             connectionOk ? "CONN_OK" : "CONN_LOST",
             state.batteryCharge,
             state.lastRssi);
}
