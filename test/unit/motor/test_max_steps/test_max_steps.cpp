/*
 *******************************************************************************
 * Motor maximum-position unit test
 *
 * This test verifies that the motor controller can command the stepper to the
 * configured maximum safe extension position once a known reference position has
 * been assigned.
 *
 * Procedure:
 *   1. Initialise the motor controller.
 *   2. Mark the current position as 0.
 *   3. Command a move to MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN.
 *   4. Check that the reported motor position equals that target.
 *
 * Pass criteria:
 *   - moveTo() must return true.
 *   - The final position must equal MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN.
 *******************************************************************************
 */

#include <Arduino.h>
#include <unity.h>
#include "motor.h"
#include "config.h"

MotorController motor;

void setUp() {
    motor.begin();
    motor.setCurrentPosition(0);
}

void tearDown() {
    motor.moveTo(MOTOR_ENDSTOP_MARGIN);
    motor.disableOutputs();
}

void test_motor_moveToMax() {
    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN),
        "moveTo(MAX) failed"
    );
    TEST_ASSERT_EQUAL(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN, motor.position());
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_motor_moveToMax);
    UNITY_END();
}

void loop() {}
