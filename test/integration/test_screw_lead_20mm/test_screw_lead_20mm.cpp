/*
 *******************************************************************************
 * Screw lead 20 mm integration test
 *
 * This test starts from a real TOF-based homing position, moves the motor by
 * 20 configured millimeters, and checks with the TOF sensor that the physical
 * travel is about 20 mm. Its purpose is to validate MOTOR_SCREW_PITCH_MM,
 * MOTOR_SCREW_STARTS, and the derived MOTOR_SCREW_LEAD_MM.
 *
 * Procedure:
 *   1. Initialize TOF and motor.
 *   2. Run TOF-based homing.
 *   3. Read a stable TOF distance at home.
 *   4. Move to +20 mm from home.
 *   5. Read a stable TOF distance again.
 *   6. Assert that TOF delta is within tolerance of 20 mm.
 *
 * Pass criteria:
 *   - Homing must complete successfully.
 *   - The motor must reach the configured +20 mm target.
 *   - The TOF delta must match 20 mm within TEST_TOF_DELTA_TOLERANCE_MM.
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <unity.h>

#include "config.h"
#include "led.h"
#include "motion_control.h"
#include "motor.h"
#include "tof.h"

LEDController ledController(PIN_LED_R, PIN_LED_G, PIN_LED_B);
MotorController motor;
TofSensor tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);
MotionController motionController(motor, tofSensor);

constexpr float TEST_TRAVEL_MM = 20.0f;
constexpr float TEST_TOF_DELTA_TOLERANCE_MM = 3.0f;
constexpr uint8_t TEST_TOF_MIN_VALID_SAMPLES = 3;
constexpr uint8_t TEST_TOF_MAX_SAMPLES = 7;
constexpr uint16_t TEST_TOF_SAMPLE_PERIOD_MS = 40;
constexpr uint16_t TEST_TOF_SETTLE_MS = 250;
constexpr uint32_t TEST_TOF_SAMPLE_TIMEOUT_MS = 1500;
constexpr long TEST_TRAVEL_STEPS =
    static_cast<long>(TEST_TRAVEL_MM * MOTOR_STEPS_PER_MM + 0.5f);

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
    float samples[TEST_TOF_MAX_SAMPLES];
    uint8_t count = 0;
    const unsigned long startMs = millis();

    while (millis() - startMs < TEST_TOF_SAMPLE_TIMEOUT_MS &&
           count < TEST_TOF_MAX_SAMPLES) {
        float sampleMm = 0.0f;
        if (tofSensor.readDistanceMm(sampleMm)) {
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

void test_screw_lead_moves_20mm_after_homing() {
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");
    TEST_ASSERT_TRUE_MESSAGE(
        motionController.homeWithTof(),
        "TOF-based homing failed"
    );
    TEST_ASSERT_EQUAL_MESSAGE(0, motor.position(), "Motor position must be 0 after homing");

    delay(TEST_TOF_SETTLE_MS);

    float startDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(startDistanceMm),
        "TOF did not provide enough valid start samples"
    );

    TEST_ASSERT_TRUE_MESSAGE(
        motor.moveTo(TEST_TRAVEL_STEPS),
        "Move to +20 mm target failed"
    );
    TEST_ASSERT_EQUAL(TEST_TRAVEL_STEPS, motor.position());

    delay(TEST_TOF_SETTLE_MS);

    float endDistanceMm = 0.0f;
    TEST_ASSERT_TRUE_MESSAGE(
        readStableTofDistanceMm(endDistanceMm),
        "TOF did not provide enough valid end samples"
    );

    const float tofDeltaMm = endDistanceMm - startDistanceMm;
    char message[160];
    snprintf(message, sizeof(message),
             "TOF delta does not match motor travel: expected=%.1fmm measured=%.1fmm start=%.1fmm end=%.1fmm",
             TEST_TRAVEL_MM,
             tofDeltaMm,
             startDistanceMm,
             endDistanceMm);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        TEST_TOF_DELTA_TOLERANCE_MM,
        TEST_TRAVEL_MM,
        tofDeltaMm,
        message
    );
}

void setup() {
    delay(2000);
    Serial.begin(115200);

    Wire.begin();
    motor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_screw_lead_moves_20mm_after_homing);
    UNITY_END();
}

void loop() {}
