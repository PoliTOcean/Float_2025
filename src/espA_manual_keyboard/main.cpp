#include <Arduino.h>

#include "config.h"
#include "motor.h"

MotorController motor;

constexpr float JOG_MM_VALUES[] = {1.0f, 5.0f, 10.0f, 25.0f};
constexpr uint8_t JOG_MM_COUNT = sizeof(JOG_MM_VALUES) / sizeof(JOG_MM_VALUES[0]);
constexpr uint8_t DEFAULT_JOG_INDEX = 1;

uint8_t jogIndex = DEFAULT_JOG_INDEX;
uint8_t escapeState = 0;
bool outputsEnabledForMove = false;

long jogSteps() {
    return static_cast<long>(JOG_MM_VALUES[jogIndex] * MOTOR_STEPS_PER_MM + 0.5f);
}

void printStatus() {
    Serial.printf("pos=%ld step (%.1f mm logical), jog=%.1f mm (%ld step)\n",
                  motor.position(),
                  static_cast<float>(motor.position()) / MOTOR_STEPS_PER_MM,
                  JOG_MM_VALUES[jogIndex],
                  jogSteps());
}

void printHelp() {
    Serial.println();
    Serial.println("Manual motor keyboard control");
    Serial.println("WARNING: no homing is performed. Start away from mechanical endstops.");
    Serial.println("up/w    = move toward home/up");
    Serial.println("down/s  = move toward extension/down");
    Serial.println("left/-  = smaller jog");
    Serial.println("right/+ = larger jog");
    Serial.println("space/x = stop and disable outputs");
    Serial.println("p       = print position");
    Serial.println("h/?     = help");
    printStatus();
    Serial.println();
}

void startJog(long deltaSteps) {
    const long target = motor.position() + deltaSteps;
    motor.enableOutputs();
    outputsEnabledForMove = true;
    motor.startMoveSteps(deltaSteps);

    Serial.printf("target=%ld step, delta=%ld step (%.1f mm)\n",
                  target,
                  deltaSteps,
                  static_cast<float>(deltaSteps) / MOTOR_STEPS_PER_MM);
}

void smallerJog() {
    if (jogIndex > 0) {
        jogIndex--;
    }
    printStatus();
}

void largerJog() {
    if (jogIndex + 1 < JOG_MM_COUNT) {
        jogIndex++;
    }
    printStatus();
}

void stopMotorNow() {
    motor.stop();
    motor.disableOutputs();
    outputsEnabledForMove = false;
    Serial.println("stopped, outputs disabled");
    printStatus();
}

void handleCommand(char command) {
    switch (command) {
        case 'A':
        case 'w':
        case 'W':
            startJog(-jogSteps());
            break;

        case 'B':
        case 's':
        case 'S':
            startJog(jogSteps());
            break;

        case 'D':
        case '-':
            smallerJog();
            break;

        case 'C':
        case '+':
        case '=':
            largerJog();
            break;

        case ' ':
        case 'x':
        case 'X':
            stopMotorNow();
            break;

        case 'p':
        case 'P':
            printStatus();
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        default:
            break;
    }
}

void handleSerialByte(char byteValue) {
    if (escapeState == 0) {
        if (byteValue == 0x1B) {
            escapeState = 1;
            return;
        }

        handleCommand(byteValue);
        return;
    }

    if (escapeState == 1) {
        escapeState = (byteValue == '[') ? 2 : 0;
        return;
    }

    escapeState = 0;
    handleCommand(byteValue);
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    motor.begin();
    motor.setCurrentPosition(static_cast<long>(MOTOR_MAX_STEPS / 2));

    printHelp();
}

void loop() {
    while (Serial.available() > 0) {
        handleSerialByte(static_cast<char>(Serial.read()));
    }

    motor.run();

    if (outputsEnabledForMove && motor.distanceToGo() == 0) {
        motor.disableOutputs();
        outputsEnabledForMove = false;
        Serial.println("move complete");
        printStatus();
    }

    yield();
}
