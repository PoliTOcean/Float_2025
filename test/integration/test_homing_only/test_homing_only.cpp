/*
 *******************************************************************************
 * TOF-only homing integration test
 *
 * This test isolates the homing routine from any follow-up travel command.
 *
 * Procedure:
 *   1. Initialise the TOF sensor and motor controller.
 *   2. Run TOF-based homing only.
 *   3. Check that the logical motor position is reset to 0.
 *   4. Read the TOF distance at home for failure diagnostics.
 *
 * Pass criteria:
 *   - The TOF sensor must initialise successfully.
 *   - Homing must complete successfully.
 *   - The motor reference position must be 0 after homing.
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "config.h"
#include "led.h"
#include "motor.h"
#include "motion_control.h"
#include "sensors.h"
#include "tof.h"

bool debug_mode_active = false;
LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);
MotionController motionController(motor, tofSensor);
SensorManager sensors;

constexpr uint32_t TOF_HOME_SAMPLE_TIMEOUT_MS = 1500;
constexpr uint16_t TOF_HOME_SAMPLE_PERIOD_MS = 40;

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

static bool readHomeTofMinDistanceMm(float& distanceMm) {
    const unsigned long startMs = millis();

    while (millis() - startMs < TOF_HOME_SAMPLE_TIMEOUT_MS) {
        if (tofSensor.readDistanceMm(distanceMm)) {
            return true;
        }

        delay(TOF_HOME_SAMPLE_PERIOD_MS);
        yield();
    }

    return false;
}

void test_homing_only() {
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");

    const unsigned long startMs = millis();
    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );
    const unsigned long durationMs = millis() - startMs;

    float homeDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readHomeTofMinDistanceMm(homeDistanceMm),
        "TOF did not provide a valid home distance after homing"
    );

    const long finalPosition = motor.position();
    char message[160];
    snprintf(message, sizeof(message),
             "Motor position must be 0 after homing; pos=%ld steps, duration=%lu ms, home_tof_min=%.1f mm",
             finalPosition,
             durationMs,
             homeDistanceMm);
    TEST_ASSERT_EQUAL_MESSAGE(0, finalPosition, message);
}

void setup() {
    delay(2000);
    Serial.begin(115200);

    Wire.begin();
    motor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_homing_only);
    UNITY_END();
}

void loop() {}
