/*
 *******************************************************************************
 * TOF-only homing integration test
 *
 * This test isolates the homing routine from any follow-up travel command.
 *
 * Procedure:
 *   1. Initialise the TOF sensor and motor controller.
 *   2. Configure the active TOF zones used for homing.
 *   3. Run TOF-based homing only.
 *   4. Check that the logical motor position is reset to 0.
 *   5. Read the TOF distance at home for diagnostic output.
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
#include "tof.h"

LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor tofSensor(Wire, TOF_LPN_PIN, TOF_I2C_RST_PIN);
MotionController motionController(motor, tofSensor);

constexpr uint32_t TOF_HOME_SAMPLE_TIMEOUT_MS = 1500;
constexpr uint16_t TOF_HOME_SAMPLE_PERIOD_MS = 40;

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

static bool readHomeTofMinDistanceMm(float& distanceMm) {
    const unsigned long startMs = millis();

    while (millis() - startMs < TOF_HOME_SAMPLE_TIMEOUT_MS) {
        if (tofSensor.readActiveMinDistanceMm(distanceMm)) {
            return true;
        }

        delay(TOF_HOME_SAMPLE_PERIOD_MS);
        yield();
    }

    return false;
}

void test_homing_only() {
    Serial.println();
    Serial.println("Homing-only test: initializing TOF");
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");
    TEST_ASSERT_TRUE_MESSAGE(
        tofSensor.setActiveZones(TOF_ACTIVE_ZONES, TOF_ACTIVE_ZONE_COUNT),
        "TOF active zone configuration failed"
    );

    Serial.printf("Homing-only test: threshold=%.1f mm, speed=%u steps/s\n",
                  TOF_HOMING_THRESHOLD,
                  MOTOR_HOMING_SPEED);

    const unsigned long startMs = millis();
    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );
    const unsigned long durationMs = millis() - startMs;

    Serial.printf("Homing-only test: homing duration=%lu ms, position=%ld steps\n",
                  durationMs,
                  motor.position());

    float homeDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readHomeTofMinDistanceMm(homeDistanceMm),
        "TOF did not provide a valid home distance after homing"
    );

    Serial.printf("Homing-only test: home TOF min distance=%.1f mm\n", homeDistanceMm);

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
