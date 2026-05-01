/*
 *******************************************************************************
 * Homing and full-travel integration test
 *
 * This test verifies the basic real-hardware motion chain from TOF-based homing
 * to a full extension command.
 *
 * Procedure:
 *   1. Initialise the TOF sensor and motor controller.
 *   2. Run the TOF-based homing routine.
 *   3. Command the motor to the maximum safe extension position.
 *   4. Check that the final motor position matches the requested safe limit.
 *
 * Pass criteria:
 *   - The TOF sensor must initialise successfully.
 *   - Homing must complete successfully.
 *   - The motor must reach MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN.
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>
#include "config.h"
#include "led.h"
#include "motor.h"
#include "tof.h"
#include "motion_control.h"

LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor tofSensor(Wire, TOF_LPN_PIN, TOF_I2C_RST_PIN);
MotionController motionController(motor, tofSensor);

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

void test_homing_then_move_to_max() {
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");

    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN),
        "moveTo(MAX) failed after homing"
    );

    TEST_ASSERT_EQUAL(
        MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN,
        motor.position()
    );
}

void setup() {
    delay(2000);
    Serial.begin(115200);

    Wire.begin();
    motor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_homing_then_move_to_max);
    UNITY_END();
}

void loop() {}
