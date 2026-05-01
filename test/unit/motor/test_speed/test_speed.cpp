/*
 *******************************************************************************
 * Motor speed movement unit test
 *
 * This test verifies that the motor controller can complete a long movement with
 * a configured maximum speed and acceleration once the motor position is known.
 *
 * Procedure:
 *   1. Initialise the motor controller.
 *   2. Assign a safe starting position.
 *   3. Configure the requested speed and test acceleration.
 *   4. Move TEST_MOVE_STEPS forward.
 *   5. Check that the reported motor position equals the requested target.
 *
 * Pass criteria:
 *   - moveTo() must return true.
 *   - The final position must equal start_position + TEST_MOVE_STEPS.
 *******************************************************************************
 */

#include <Arduino.h>
#include <unity.h>
#include "motor.h"
#include "config.h"

MotorController motor;

constexpr long TEST_MOVE_STEPS = 100000;
constexpr long TEST_START_POSITION = MOTOR_ENDSTOP_MARGIN + 1000;
constexpr float TEST_ACCELERATION = 300;

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

void run_speed_test(float speed) {
    const long currentPosition = motor.position();
    const long targetPosition = currentPosition + TEST_MOVE_STEPS;

    motor.setMaxSpeed(speed);
    motor.setAcceleration(TEST_ACCELERATION);

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(targetPosition),
        "moveTo() failed during speed test"
    );
    TEST_ASSERT_EQUAL(targetPosition, motor.position());
}

void test_all_speeds() {
    run_speed_test(1500);
}

void setup() {
    delay(2000);
    motor.begin();
    motor.setCurrentPosition(TEST_START_POSITION);

    UNITY_BEGIN();
    RUN_TEST(test_all_speeds);
    UNITY_END();
}

void loop() {}
