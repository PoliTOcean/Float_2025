#include <Arduino.h>
#include <unity.h>
#include "motor.h"
#include "led.h"
#include "config.h"

LEDController ledController(0, 0, 0);

MotorController motor;

void setUp() {
    motor.begin();
}

void tearDown() {
    motor.home();   // riporta a home dopo ogni test
}

void test_motor_moveToMax() {
    TEST_ASSERT_TRUE_MESSAGE(motor.home(), "Homing fallito");
    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN),
        "moveTo(MAX) fallito"
    );
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_motor_moveToMax);
    UNITY_END();
}

void loop() {}