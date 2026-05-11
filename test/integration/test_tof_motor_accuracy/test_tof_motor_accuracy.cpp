/*
 *******************************************************************************
 * TOF / motor accuracy integration test
 *
 * This test verifies that the VL53L7CX distance reading tracks the stepper motor
 * position after a real TOF-based homing sequence.
 *
 * Procedure:
 *   1. Initialise the TOF sensor, motor, and configured active TOF zones.
 *   2. Run homing and assert that the motor reference position becomes 0.
 *   3. Read a stable TOF baseline at home and check it is close to the homing
 *      threshold.
 *   4. Move the syringe to several known travel distances.
 *   5. At each point, compare the measured TOF distance against:
 *        home_distance + motor_position / MOTOR_STEPS_PER_MM
 *
 * Pass criteria:
 *   - The TOF must provide enough valid samples at each point.
 *   - The home reading must be within TOF_HOME_TOLERANCE_MM.
 *   - Each travel reading must be within TOF_DISTANCE_TOLERANCE_MM.
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
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

constexpr float TOF_HOME_TOLERANCE_MM = 5.0f;
constexpr float TOF_DISTANCE_TOLERANCE_MM = 5.0f;
constexpr uint8_t TOF_MIN_VALID_SAMPLES = 3;
constexpr uint8_t TOF_MAX_SAMPLES = 7;
constexpr uint32_t TOF_SAMPLE_TIMEOUT_MS = 1500;
constexpr uint16_t TOF_SAMPLE_PERIOD_MS = 40;
constexpr uint16_t TOF_SETTLE_MS = 250;

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

static void sortSamples(float* samples, uint8_t count) {
    for (uint8_t i = 1; i < count; i++) {
        const float value = samples[i];
        uint8_t j = i;
        while (j > 0 && samples[j - 1] > value) {
            samples[j] = samples[j - 1];
            j--;
        }
        samples[j] = value;
    }
}

static bool readStableTofDistanceMm(float& distanceMm) {
    float samples[TOF_MAX_SAMPLES];
    uint8_t count = 0;
    const unsigned long startMs = millis();

    while (millis() - startMs < TOF_SAMPLE_TIMEOUT_MS && count < TOF_MAX_SAMPLES) {
        float sampleMm = 0.0f;
        if (tofSensor.readActiveDistanceMm(sampleMm)) {
            samples[count++] = sampleMm;
        }
        delay(TOF_SAMPLE_PERIOD_MS);
        yield();
    }

    if (count < TOF_MIN_VALID_SAMPLES) {
        return false;
    }

    sortSamples(samples, count);
    if ((count % 2) == 1) {
        distanceMm = samples[count / 2];
    } else {
        distanceMm = (samples[(count / 2) - 1] + samples[count / 2]) * 0.5f;
    }

    return true;
}

static void assertTofMatchesMotorTravel(float homeDistanceMm, float travelMm) {
    const long targetSteps = lroundf(travelMm * MOTOR_STEPS_PER_MM);

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(targetSteps),
        "moveTo() failed during TOF accuracy test"
    );

    delay(TOF_SETTLE_MS);

    float measuredDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(measuredDistanceMm),
        "TOF did not provide enough valid samples"
    );

    const float actualTravelMm = static_cast<float>(motor.position()) / MOTOR_STEPS_PER_MM;
    const float expectedDistanceMm = homeDistanceMm + actualTravelMm;

    char message[128];
    snprintf(message, sizeof(message),
             "TOF out of tolerance: travel=%.1fmm expected=%.1fmm measured=%.1fmm",
             actualTravelMm, expectedDistanceMm, measuredDistanceMm);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        TOF_DISTANCE_TOLERANCE_MM,
        expectedDistanceMm,
        measuredDistanceMm,
        message
    );
}

void test_tof_distance_matches_motor_position_after_homing() {
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");
    TEST_ASSERT_TRUE_MESSAGE(
        tofSensor.setActiveZones(TOF_ACTIVE_ZONES, TOF_ACTIVE_ZONE_COUNT),
        "TOF active zone configuration failed"
    );

    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );

    TEST_ASSERT_EQUAL_MESSAGE(0, motor.position(), "Motor position must be 0 after homing");

    delay(TOF_SETTLE_MS);

    float homeDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(homeDistanceMm),
        "TOF did not read a stable home distance"
    );

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        TOF_HOME_TOLERANCE_MM,
        TOF_HOMING_THRESHOLD,
        homeDistanceMm,
        "At home, TOF should read close to the homing threshold"
    );

    const float testTravelMm[] = {
        5.0f,   // About 1.5 cm absolute distance if home is near 1 cm.
        10.0f,  // About 2 cm.
        20.0f,  // About 3 cm.
        30.0f,  // About 4 cm.
        40.0f,  // About 5 cm.
        60.0f,  // About 7 cm.
        75.0f   // About 8.5 cm, close to the 80 mm travel limit.
    };

    for (float travelMm : testTravelMm) {
        assertTofMatchesMotorTravel(homeDistanceMm, travelMm);
    }
}

void setup() {
    delay(2000);
    Serial.begin(115200);

    Wire.begin();
    motor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_tof_distance_matches_motor_position_after_homing);
    UNITY_END();
}

void loop() {}
