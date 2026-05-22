/*
 *******************************************************************************
 * Flash storage hardware unit test
 *
 * This test verifies FlashStorageManager against ESP32 internal LittleFS.
 *
 * Pass criteria:
 *   - LittleFS initialisation succeeds.
 *   - The log can be cleared and recreated with its CSV header.
 *   - Records can be appended and the file grows.
 *   - The log can be streamed and removed.
 *******************************************************************************
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <unity.h>
#include "flash_storage.h"
#include "config.h"

namespace {
constexpr char EXPECTED_CSV[] =
    "company_number,profile_id,time_s,pressure_kpa,depth_m,phase,sensor_depth_m\n"
    "EX10,1,0.00,101.30,0.00,start,-0.51\n"
    "EX10,1,5.00,126.00,2.50,hold_2_5m,1.99\n"
    "EX10,1,10.00,126.20,2.52,hold_2_5m,2.01\n";
}

void setUp() {}

void tearDown() {}

void test_flash_config_values() {
    TEST_ASSERT_EQUAL_STRING("/mission/current_profile.csv", FLASH_LOG_PATH);
    TEST_ASSERT_EQUAL_STRING("EX10", COMPANY_NUMBER);
    TEST_ASSERT_EQUAL_UINT16(1000, PROFILE_LOG_PERIOD_MS);
    TEST_ASSERT_EQUAL_UINT16(5000, DATA_PACKET_PERIOD_MS);
}

void test_flash_csv_logging() {
    TEST_ASSERT_TRUE_MESSAGE(flashStorage.begin(), "LittleFS begin failed");
    TEST_ASSERT_TRUE(flashStorage.isAvailable());

    TEST_ASSERT_TRUE_MESSAGE(flashStorage.clearLog(), "Failed to clear/create CSV log");
    const size_t headerSize = flashStorage.logSize();
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(0, headerSize, "CSV header was not written");

    TEST_ASSERT_TRUE(flashStorage.appendRecord(COMPANY_NUMBER, 1, 0.0f, 101.3f, 0.00f, "start", -0.51f));
    TEST_ASSERT_TRUE(flashStorage.appendRecord(COMPANY_NUMBER, 1, 5.0f, 126.0f, 2.50f, "hold_2_5m", 1.99f));
    TEST_ASSERT_TRUE(flashStorage.appendRecord(COMPANY_NUMBER, 1, 10.0f, 126.2f, 2.52f, "hold_2_5m", 2.01f));
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(headerSize, flashStorage.logSize(), "CSV log did not grow");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(strlen(EXPECTED_CSV), flashStorage.logSize(), "CSV log size mismatch");

    File file = LittleFS.open(FLASH_LOG_PATH, FILE_READ);
    TEST_ASSERT_TRUE_MESSAGE(file, "Failed to open CSV log for content check");

    char actual[sizeof(EXPECTED_CSV)];
    const size_t bytesRead = file.readBytes(actual, sizeof(actual) - 1);
    actual[bytesRead] = '\0';
    file.close();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(strlen(EXPECTED_CSV), bytesRead, "CSV bytes read mismatch");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(EXPECTED_CSV, actual, "CSV content mismatch");

    TEST_ASSERT_TRUE_MESSAGE(flashStorage.printLogTo(Serial), "Failed to print CSV log");
    TEST_ASSERT_TRUE_MESSAGE(flashStorage.removeLog(), "Failed to remove CSV log");
    TEST_ASSERT_EQUAL_UINT32(0, flashStorage.logSize());
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_flash_config_values);
    RUN_TEST(test_flash_csv_logging);
    UNITY_END();
}

void loop() {}
