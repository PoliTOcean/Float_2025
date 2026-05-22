#pragma once

#include <Arduino.h>
#include <float_common.h>

constexpr int8_t ESPB_STATUS_UNKNOWN = -1;
constexpr int8_t ESPB_STATUS_CONNECTED = 0;
constexpr int8_t ESPB_STATUS_CONNECTED_W_DATA = 1;
constexpr int8_t ESPB_STATUS_EXECUTING_CMD = 2;

enum class EspbParsedCommandType : uint8_t {
    Invalid,
    Status,
    ForwardToEspA,
};

struct EspbParsedCommand {
    EspbParsedCommandType type = EspbParsedCommandType::Invalid;
    output_message message;
};

struct EspbBridgeState {
    int8_t status = ESPB_STATUS_UNKNOWN;
    bool autoModeActive = false;
    uint16_t batteryCharge = 0;
    int lastRssi = 0;
};

struct EspbProtocolCommand {
    const char* commandText;
    uint8_t commandCode;
    const char* expectedAck;
};

EspbParsedCommand espbParseSerialCommand(const char* line);
bool espbApplyIncomingMessage(EspbBridgeState& state, const input_message& message);
const char* espbStatusString(int8_t status);
const EspbProtocolCommand* espbProtocolCommands(size_t& count);
void espbFormatStatus(char* buffer,
                      size_t bufferSize,
                      const EspbBridgeState& state,
                      bool connectionOk);
