/*
 *******************************************************************************
 * Motor direction integration test
 *
 * This test verifies the motor direction convention used by MotorController.
 * By default it uses the TOF sensor to establish a safe reference and to confirm
 * that positive motor travel physically increases the measured distance.
 *
 * Procedure with TEST_DIRECTION_USE_TOF = 1:
 *   1. Initialise TOF, motor, and motion controller.
 *   2. Run TOF-based homing.
 *   3. Move to a known safe internal start position.
 *   4. Move a positive travel distance and check that TOF distance increases.
 *   5. Move the same distance back and check that logical position decreases.
 *
 * Procedure with TEST_DIRECTION_USE_TOF = 0:
 *   1. Initialise the motor controller only.
 *   2. Assign the safe internal start position logically.
 *   3. Move positive and negative steps and check the reported position.
 *
 * Pass criteria:
 *   - Positive travel must increase the logical motor position.
 *   - Negative travel must decrease the logical motor position.
 *   - When TOF is enabled, positive travel must increase TOF distance.
 *
 * Note:
 *   TEST_DIRECTION_USE_TOF = 0 is a manual free-motion mode. Use it only when
 *   the motor is physically away from endstops and can safely move both ways.
 *******************************************************************************
 */

#include <Arduino.h>
#include <math.h>
#include <unity.h>

#include "config.h"
#include "motor.h"

#ifndef TEST_DIRECTION_USE_TOF
#define TEST_DIRECTION_USE_TOF 1
#endif

MotorController motor;

#if TEST_DIRECTION_USE_TOF
#include <Wire.h>
#include "led.h"
#include "motion_control.h"
#include "tof.h"

LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
TofSensor tofSensor(Wire, TOF_LPN_PIN, TOF_I2C_RST_PIN);
MotionController motionController(motor, tofSensor);
#endif

constexpr float TEST_START_TRAVEL_MM = 10.0f;
constexpr float TEST_DIRECTION_TRAVEL_MM = 5.0f;
constexpr float TEST_TOF_MIN_DELTA_MM = 2.0f;
constexpr uint8_t TEST_TOF_MIN_VALID_SAMPLES = 3;
constexpr uint8_t TEST_TOF_MAX_SAMPLES = 7;
constexpr uint16_t TEST_TOF_SAMPLE_PERIOD_MS = 40;
constexpr uint16_t TEST_TOF_SETTLE_MS = 250;
constexpr uint32_t TEST_TOF_SAMPLE_TIMEOUT_MS = 1500;

constexpr long TEST_START_STEPS =
    static_cast<long>(TEST_START_TRAVEL_MM * MOTOR_STEPS_PER_MM + 0.5f);
constexpr long TEST_DIRECTION_STEPS =
    static_cast<long>(TEST_DIRECTION_TRAVEL_MM * MOTOR_STEPS_PER_MM + 0.5f);
constexpr long TEST_END_STEPS = TEST_START_STEPS + TEST_DIRECTION_STEPS;

static_assert(TEST_START_STEPS >= MOTOR_ENDSTOP_MARGIN,
              "Direction test start is below the safe motor range");
static_assert(TEST_END_STEPS <= MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN,
              "Direction test end is above the safe motor range");

void setUp() {}

void tearDown() {
    motor.disableOutputs();
}

#if TEST_DIRECTION_USE_TOF
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
    float samples[TEST_TOF_MAX_SAMPLES];
    uint8_t count = 0;
    const unsigned long startMs = millis();

    while (millis() - startMs < TEST_TOF_SAMPLE_TIMEOUT_MS &&
           count < TEST_TOF_MAX_SAMPLES) {
        float sampleMm = 0.0f;
        if (tofSensor.readActiveDistanceMm(sampleMm)) {
            samples[count++] = sampleMm;
        }
        delay(TEST_TOF_SAMPLE_PERIOD_MS);
        yield();
    }

    if (count < TEST_TOF_MIN_VALID_SAMPLES) {
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
#endif

void test_positive_and_negative_motor_direction() {
#if TEST_DIRECTION_USE_TOF
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");
    TEST_ASSERT_TRUE_MESSAGE(
        tofSensor.setActiveZones(TOF_ACTIVE_ZONES, TOF_ACTIVE_ZONE_COUNT),
        "TOF active zone configuration failed"
    );
    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );
#else
    motor.setCurrentPosition(TEST_START_STEPS);
#endif

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(TEST_START_STEPS),
        "Move to direction-test start failed"
    );

#if TEST_DIRECTION_USE_TOF
    delay(TEST_TOF_SETTLE_MS);
    float startDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(startDistanceMm),
        "TOF did not provide enough valid start samples"
    );
#endif

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveSteps(TEST_DIRECTION_STEPS),
        "Positive moveSteps() failed"
    );
    TEST_ASSERT_EQUAL(TEST_END_STEPS, motor.position());

#if TEST_DIRECTION_USE_TOF
    delay(TEST_TOF_SETTLE_MS);
    float endDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(endDistanceMm),
        "TOF did not provide enough valid end samples"
    );

    const float deltaMm = endDistanceMm - startDistanceMm;
    char message[128];
    snprintf(message, sizeof(message),
             "Physical direction may be inverted: start=%.1fmm end=%.1fmm delta=%.1fmm",
             startDistanceMm, endDistanceMm, deltaMm);
    TEST_ASSERT_TRUE_MESSAGE(deltaMm > TEST_TOF_MIN_DELTA_MM, message);
#endif

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveSteps(-TEST_DIRECTION_STEPS),
        "Negative moveSteps() failed"
    );
    TEST_ASSERT_EQUAL(TEST_START_STEPS, motor.position());
}

void setup() {
    delay(2000);
    Serial.begin(115200);

#if TEST_DIRECTION_USE_TOF
    Wire.begin();
#endif
    motor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_positive_and_negative_motor_direction);
    UNITY_END();
}

void loop() {}
