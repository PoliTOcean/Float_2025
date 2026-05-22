/*
 *******************************************************************************
 * Motor speed movement unit test
 *
 * This test verifies that the motor controller can complete alternating 4 cm
 * movements from the homing reference while speed and acceleration increase
 * together up to the configured maximum test speed.
 *
 * Procedure:
 *   1. Initialise the motor controller.
 *   2. Assume the motor starts from the homing position.
 *   3. Configure the requested speed and matching acceleration.
 *   4. Move TEST_MOVE_DISTANCE_MM, alternating direction on each step.
 *   5. Repeat with speed and acceleration increased up to TEST_MAX_SPEED.
 *
 * Pass criteria:
 *   - Each move_distance() call must return true.
 *******************************************************************************
 */

#include <Arduino.h>
#include <math.h>
#include <unity.h>
#include "motor.h"
#include "config.h"

MotorController motor;

constexpr float TEST_MOVE_DISTANCE_MM = 40.0f;
constexpr float TEST_START_SPEED = 1500.0f;
constexpr float TEST_MAX_SPEED = 2300.0f;
constexpr uint8_t TEST_MOVE_COUNT = 6;
constexpr float TEST_SPEED_STEP =
    (TEST_MAX_SPEED - TEST_START_SPEED) / (TEST_MOVE_COUNT - 1);
constexpr long TEST_MOVE_DISTANCE_STEPS =
    static_cast<long>(TEST_MOVE_DISTANCE_MM * MOTOR_STEPS_PER_MM + 0.5f);

static_assert(TEST_MOVE_DISTANCE_STEPS <= MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN,
              "Speed test movement must fit inside the configured safe range");

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

static bool move_distance(float distanceMm) {
    const long steps = lroundf(distanceMm * MOTOR_STEPS_PER_MM);
    return motor.moveSteps(steps);
}

void test_all_speeds() {
    float direction = 1.0f;
    for (uint8_t i = 0; i < TEST_MOVE_COUNT; i++) {
        const float speed = TEST_START_SPEED + TEST_SPEED_STEP * i;
        motor.setMaxSpeed(speed);
        motor.setAcceleration(speed);

        TEST_ASSERT_TRUE_MESSAGE(
            move_distance(TEST_MOVE_DISTANCE_MM * direction),
            "move_distance() failed during speed test"
        );
        direction = -direction;
    }
}

void setup() {
    delay(2000);
    motor.begin();
    motor.setCurrentPosition(0);

    UNITY_BEGIN();
    RUN_TEST(test_all_speeds);
    UNITY_END();
}

void loop() {}
