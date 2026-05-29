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
constexpr size_t COMMAND_BUFFER_SIZE = 96;

constexpr EspbProtocolCommand PROTOCOL_COMMANDS[] = {
    {"GO", CMD_GO, CMD1_ACK},
    {"LISTENING", CMD_SEND_DATA, "DATA_OR_STOP_DATA"},
    {"BALANCE", CMD_BALANCE, CMD3_ACK},
    {"CLEAR_SD", CMD_CLEAR_EEPROM, CMD4_ACK},
    {"SWITCH_AUTO_MODE", CMD_AUTO_MODE, CMD5_ACK},
    {"SEND_PACKAGE", CMD_SEND_PACKAGE, "JSON_LIVE_PACKET"},
    {"TRY_UPLOAD", CMD_OTA, CMD7_ACK},
    {"PARAMS", CMD_UPDATE_PID, CMD8_ACK},
    {"TEST_FREQ", CMD_SET_SPEED, CMD9_ACK},
    {"TEST_STEPS", CMD_TEST_STEPS, CMD10_ACK},
    {"DEBUG", CMD_DEBUG_MODE, CMD11_ACK},
    {"HOME_MOTOR", CMD_HOME, CMD12_ACK},
    {"STOP", CMD_STOP, CMD13_ACK},
    {"PARAMS_EXT", CMD_UPDATE_PID_EXT, CMD14_ACK},
    {"SYRINGE_SET", CMD_SYRINGE_SET, CMD15_ACK},
    {"PID_HOLD", CMD_PID_HOLD, CMD16_ACK},
    {"PID_STEP", CMD_PID_STEP, CMD17_ACK},
    {"SURFACE_OFFSET", CMD_SET_SURFACE_OFFSET, CMD18_ACK},
};

void zeroMessage(output_message& message) {
    memset(&message, 0, sizeof(message));
}

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

EspbParsedCommand makeForwardCommand(uint8_t commandCode) {
    EspbParsedCommand parsed;
    parsed.type = EspbParsedCommandType::ForwardToEspA;
    zeroMessage(parsed.message);
    parsed.message.command = commandCode;
    return parsed;
}
}

EspbParsedCommand espbParseSerialCommand(const char* line) {
    EspbParsedCommand parsed;
    zeroMessage(parsed.message);

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

    if (strcmp(token, "PARAMS") == 0) {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), kp) ||
            !parseFloatToken(strtok(nullptr, " "), ki) ||
            !parseFloatToken(strtok(nullptr, " "), kd) ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_UPDATE_PID);
        parsed.message.params[0] = kp;
        parsed.message.params[1] = ki;
        parsed.message.params[2] = kd;
        return parsed;
    }

    if (strcmp(token, "PARAMS_EXT") == 0) {
        // PARAMS_EXT period_ms alpha_d   (third param reserved, always 0)
        float periodMs = 0.0f;
        float alphaD   = 0.0f;
        if (!parseFloatToken(strtok(nullptr, " "), periodMs) ||
            !parseFloatToken(strtok(nullptr, " "), alphaD) ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_UPDATE_PID_EXT);
        parsed.message.params[0] = periodMs;
        parsed.message.params[1] = alphaD;
        parsed.message.params[2] = 0.0f;
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
        parsed.message.params[0] = u;
        parsed.message.params[1] = dur;
        parsed.message.params[2] = 0.0f;
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
        parsed.message.params[0] = depth;
        parsed.message.params[1] = dur;
        parsed.message.params[2] = 0.0f;
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
        parsed.message.params[0] = depth;
        parsed.message.params[1] = 0.0f;
        parsed.message.params[2] = 0.0f;
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
        parsed.message.params[0] = offset;
        parsed.message.params[1] = 0.0f;
        parsed.message.params[2] = 0.0f;
        return parsed;
    }

    if (strcmp(token, "TEST_FREQ") == 0) {
        long freq = 0;
        if (!parseLongToken(strtok(nullptr, " "), freq) ||
            freq < 0 || freq > UINT16_MAX ||
            !hasNoExtraToken()) {
            return parsed;
        }

        parsed = makeForwardCommand(CMD_SET_SPEED);
        parsed.message.freq = static_cast<uint16_t>(freq);
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
        parsed.message.steps = static_cast<int32_t>(steps);
        return parsed;
    }

    for (const EspbProtocolCommand& command : PROTOCOL_COMMANDS) {
        if (strcmp(token, command.commandText) == 0) {
            if (command.commandCode == CMD_UPDATE_PID ||
                command.commandCode == CMD_UPDATE_PID_EXT ||
                command.commandCode == CMD_SET_SPEED ||
                command.commandCode == CMD_TEST_STEPS ||
                command.commandCode == CMD_SYRINGE_SET ||
                command.commandCode == CMD_PID_HOLD ||
                command.commandCode == CMD_PID_STEP ||
                command.commandCode == CMD_SET_SURFACE_OFFSET ||
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
