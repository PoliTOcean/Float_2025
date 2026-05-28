#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "motor.h"
#include "tof.h"

MotorController motor;
TofSensor tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);

constexpr float MANUAL_MOTOR_SPEED = MOTOR_MAX_SPEED;
constexpr float MANUAL_MOTOR_ACCELERATION = MOTOR_MAX_ACCELERATION;
constexpr uint16_t MANUAL_HOLD_TIMEOUT_MS = 250;
constexpr uint16_t TOF_PRINT_PERIOD_MS = 100;

uint8_t escapeState = 0;
bool outputsEnabledForMove = false;
bool tofReady = false;
int8_t activeDirection = 0;
unsigned long lastMoveCommandMs = 0;
unsigned long lastTofPrintMs = 0;

void printStatus() {
    Serial.printf("pos=%ld step (%.1f mm logical), speed=%.1f mm/s\n",
                  motor.position(),
                  static_cast<float>(motor.position()) / MOTOR_STEPS_PER_MM,
                  MANUAL_MOTOR_SPEED / MOTOR_STEPS_PER_MM);
}

void printHelp() {
    Serial.println();
    Serial.println("Manual motor keyboard control");
    Serial.println("WARNING: no homing is performed. Start away from mechanical endstops.");
    Serial.println("up/w    = move continuously toward home/up");
    Serial.println("down/s  = move continuously toward extension/down");
    Serial.println("space/x = stop and disable outputs");
    Serial.println("p       = print position");
    Serial.println("t       = print one TOF reading");
    Serial.println("h/?     = help");
    printStatus();
    Serial.println();
}

void startHoldMove(int8_t direction) {
    const long target = (direction < 0)
        ? static_cast<long>(MOTOR_ENDSTOP_MARGIN)
        : static_cast<long>(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
    const long deltaSteps = target - motor.position();
    const float estimatedSeconds =
        fabs(static_cast<float>(deltaSteps)) / MANUAL_MOTOR_SPEED;

    lastMoveCommandMs = millis();

    if (outputsEnabledForMove && activeDirection == direction && motor.distanceToGo() != 0) {
        return;
    }

    if (outputsEnabledForMove && activeDirection != direction) {
        motor.stop();
    }

    motor.enableOutputs();
    motor.startMoveTo(target);
    outputsEnabledForMove = true;
    activeDirection = direction;

    Serial.printf("hold target=%ld step, delta=%ld step (%.1f mm), estimated-to-limit=%.1f s\n",
                  target,
                  deltaSteps,
                  static_cast<float>(deltaSteps) / MOTOR_STEPS_PER_MM,
                  estimatedSeconds);
}

void moveTowardHome() {
    startHoldMove(-1);
}

void moveTowardExtension() {
    startHoldMove(1);
}

void stopMotorNow() {
    motor.stop();
    motor.disableOutputs();
    outputsEnabledForMove = false;
    activeDirection = 0;
    Serial.println("stopped, outputs disabled");
    printStatus();
}

void printTofReading(const char* prefix) {
    if (!tofReady) {
        Serial.println("tof not ready");
        return;
    }

    TofMeasurement measurement;
    if (!tofSensor.readMeasurement(measurement)) {
        Serial.printf("%s TOF: no new sample\n", prefix);
        return;
    }

    Serial.printf(
        "%s TOF: valid=%u raw=%.1f mm distance=%.1f mm zones=%u status=%u pos=%ld step (%.1f mm)\n",
        prefix,
        measurement.valid ? 1 : 0,
        measurement.rawDistanceMm,
        measurement.distanceMm,
        measurement.validZoneCount,
        measurement.rangeStatus,
        motor.position(),
        static_cast<float>(motor.position()) / MOTOR_STEPS_PER_MM
    );

    if (outputsEnabledForMove &&
        activeDirection > 0 &&
        TOF_MAX_STOP_DISTANCE_MM > 0.0f &&
        measurement.valid &&
        measurement.distanceMm >= TOF_MAX_STOP_DISTANCE_MM) {
        Serial.printf("TOF max extension stop reached: %.1f >= %.1f mm\n",
                      measurement.distanceMm,
                      TOF_MAX_STOP_DISTANCE_MM);
        stopMotorNow();
    }
}

void handleCommand(char command) {
    switch (command) {
        case 'A':
        case 'w':
        case 'W':
            moveTowardHome();
            break;

        case 'B':
        case 's':
        case 'S':
            moveTowardExtension();
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

        case 't':
        case 'T':
            printTofReading("manual");
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

    Wire.begin();

    motor.begin();
    motor.setMaxSpeed(MANUAL_MOTOR_SPEED);
    motor.setAcceleration(MANUAL_MOTOR_ACCELERATION);
    motor.setCurrentPosition(static_cast<long>(MOTOR_MAX_STEPS / 2));

    Serial.println("Initializing TOF sensor...");
    tofReady = tofSensor.begin();
    Serial.println(tofReady ? "TOF sensor ready" : "TOF sensor init failed");

    printHelp();
}

void loop() {
    while (Serial.available() > 0) {
        handleSerialByte(static_cast<char>(Serial.read()));
    }

    motor.run();

    if (tofReady && outputsEnabledForMove && millis() - lastTofPrintMs >= TOF_PRINT_PERIOD_MS) {
        lastTofPrintMs = millis();
        printTofReading("move");
    }

    if (outputsEnabledForMove &&
        activeDirection != 0 &&
        millis() - lastMoveCommandMs > MANUAL_HOLD_TIMEOUT_MS) {
        stopMotorNow();
    }

    if (outputsEnabledForMove && motor.distanceToGo() == 0) {
        motor.disableOutputs();
        outputsEnabledForMove = false;
        activeDirection = 0;
        Serial.println("move complete");
        printStatus();
    }

    yield();
}
