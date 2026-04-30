#include <Arduino.h>
#include <unity.h>
#include "motor.h"
#include "led.h"
#include "config.h"

LEDController ledController(0, 0, 0);
MotorController motor;

constexpr long TEST_HOME_POSITION = 0;
constexpr long TEST_TARGET_POSITION = 10000;

void setUp() {
    motor.begin(/*initTof=*/false);
    motor.assumeHomedAt(TEST_HOME_POSITION);
}

void tearDown() {
    motor.stepper.disableOutputs();
}

void test_motor_moveTo_without_sensors() {
    Serial.println("Moving with MotorController::moveTo()");

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(TEST_TARGET_POSITION),
        "moveTo(TEST_TARGET_POSITION) failed"
    );

    TEST_ASSERT_EQUAL_INT32(TEST_TARGET_POSITION, motor.position());
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("Motor moveTo test: TOF skipped, fake homing enabled");

    UNITY_BEGIN();
    RUN_TEST(test_motor_moveTo_without_sensors);
    UNITY_END();
}

void loop() {}
