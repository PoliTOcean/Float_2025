/*
 *******************************************************************************
 * TOF reading integration test
 *
 * This test verifies that the VL53L4CD TOF sensor can initialize and provide
 * distance samples for about 30 seconds without moving the motor.
 *
 * Procedure:
 *   1. Initialize I2C and the TOF sensor.
 *   2. Poll the sensor for about 30 seconds.
 *   3. Pass if at least one positive distance reading is received.
 *
 * Pass criteria:
 *   - The TOF sensor must initialize successfully.
 *   - At least one valid distance sample in millimeters must be read.
 *******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include "config.h"
#include "tof.h"

TofSensor tofSensor(Wire, TOF_XSHUT_PIN, TOF_GPIO1_PIN);

constexpr uint32_t TOF_READ_DURATION_MS = 30000;
constexpr uint16_t TOF_READ_PERIOD_MS = 40;

void setUp() {}

void tearDown() {}

void test_tof_reads_distance_samples_for_30_seconds() {
    TEST_ASSERT_TRUE_MESSAGE(tofSensor.begin(), "TOF initialization failed");

    uint32_t validSampleCount = 0;
    uint32_t missedSampleCount = 0;
    const unsigned long startMs = millis();

    while (millis() - startMs < TOF_READ_DURATION_MS) {
        TofMeasurement measurement;
        if (tofSensor.readMeasurement(measurement)) {
            if (measurement.valid) {
                validSampleCount++;
            } else {
                missedSampleCount++;
            }
        } else {
            missedSampleCount++;
        }

        delay(TOF_READ_PERIOD_MS);
        yield();
    }

    char message[128];
    snprintf(message, sizeof(message),
             "TOF did not provide valid samples during %lu ms; missed=%lu",
             static_cast<unsigned long>(TOF_READ_DURATION_MS),
             static_cast<unsigned long>(missedSampleCount));
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(0, validSampleCount, message);
}

void setup() {
    delay(2000);
    Serial.begin(115200);

    Wire.begin();

    UNITY_BEGIN();
    RUN_TEST(test_tof_reads_distance_samples_for_30_seconds);
    UNITY_END();
}

void loop() {}
