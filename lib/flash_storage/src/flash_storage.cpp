#include "flash_storage.h"
#include "config.h"
#include "float_common.h"
#include "DebugSerial.h"
#include <LittleFS.h>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

/*
 *******************************************************************************
 * flash_storage.cpp
 *******************************************************************************
 */

namespace {
constexpr char CSV_HEADER[] =
    "company_number,profile_id,time_s,pressure_kpa,depth_m,phase,sensor_depth_m";

constexpr char LITTLEFS_BASE_PATH[] = "/littlefs";

bool logFileExists() {
    char path[sizeof(LITTLEFS_BASE_PATH) + sizeof(FLASH_LOG_PATH) - 1];
    snprintf(path, sizeof(path), "%s%s", LITTLEFS_BASE_PATH, FLASH_LOG_PATH);

    struct stat info;
    return stat(path, &info) == 0;
}
}

FlashStorageManager flashStorage;

// ---------------------------------------------------------------------------
FlashStorageManager::FlashStorageManager() {}

// ---------------------------------------------------------------------------
bool FlashStorageManager::begin() {
    if (!LittleFS.begin(true)) {
        _available = false;
        Debug.println("Flash: LittleFS mount failed");
        return false;
    }

    _available = true;
    Debug.println("Flash: LittleFS mounted");
    return ensureLogFile();
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::ensureLogFile() {
    if (!_available) return false;

    if (logFileExists()) return true;

    if (!_ensureParentDir()) return false;

    return _writeHeader();
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::appendRecord(const char* companyNumber,
                                       uint8_t profileId,
                                       float timeS,
                                       float pressureKpa,
                                       float depthM,
                                       const char* phase,
                                       float sensorDepthM) {
    if (!ensureLogFile()) return false;

    File file = LittleFS.open(FLASH_LOG_PATH, FILE_APPEND);
    if (!file) {
        Debug.println("Flash: failed to open log for append");
        return false;
    }

    _writeCsvField(file, companyNumber);
    file.print(',');
    file.print(profileId);
    file.print(',');
    file.print(timeS, 2);
    file.print(',');
    file.print(pressureKpa, 2);
    file.print(',');
    file.print(depthM, 2);
    file.print(',');
    _writeCsvField(file, phase);
    file.print(',');
    file.print(sensorDepthM, 2);
    file.println();

    const bool ok = file.getWriteError() == 0;
    file.close();

    if (!ok) Debug.println("Flash: write error while appending record");
    return ok;
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::clearLog() {
    if (!_available) return false;

    if (logFileExists() && !LittleFS.remove(FLASH_LOG_PATH)) {
        Debug.println("Flash: failed to remove log during clear");
        return false;
    }

    if (!_ensureParentDir()) return false;

    return _writeHeader();
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::transmitDataPackets(PacketSender sender, uint32_t timeoutMs) {
    if (!_available || sender == nullptr || !logFileExists()) return false;

    File file = LittleFS.open(FLASH_LOG_PATH, FILE_READ);
    if (!file) {
        Debug.println("Flash: failed to open log for transmission");
        return false;
    }

    char line[160];
    char packet[OUTPUT_LEN];
    uint16_t packetCount = 0;
    long lastSentSlot = -1;
    bool headerSkipped = false;

    while (file.available()) {
        const size_t len = file.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';
        if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

        if (!headerSkipped) {
            headerSkipped = true;
            continue;
        }
        if (line[0] == '\0') continue;

        char* save = nullptr;
        char* companyNumber = strtok_r(line, ",", &save);
        char* profileId     = strtok_r(nullptr, ",", &save);
        char* timeS         = strtok_r(nullptr, ",", &save);
        char* pressureKpa   = strtok_r(nullptr, ",", &save);
        char* depthM        = strtok_r(nullptr, ",", &save);
        char* phase         = strtok_r(nullptr, ",", &save);
        char* sensorDepthM  = strtok_r(nullptr, ",", &save);

        if (companyNumber == nullptr || profileId == nullptr || timeS == nullptr ||
            pressureKpa == nullptr || depthM == nullptr || phase == nullptr) {
            continue;
        }

        const unsigned long timeMs =
            static_cast<unsigned long>(lroundf(static_cast<float>(atof(timeS) * 1000.0)));
        const unsigned long roundedSecondMs = ((timeMs + 500UL) / 1000UL) * 1000UL;
        if (roundedSecondMs % DATA_PACKET_PERIOD_MS != 0) continue;

        const long slot = static_cast<long>(roundedSecondMs / DATA_PACKET_PERIOD_MS);
        if (slot == lastSentSlot) continue;
        lastSentSlot = slot;

        snprintf(packet, OUTPUT_LEN,
                 "{\"company_number\":\"%s\","
                 "\"profile_id\":%u,"
                 "\"time_s\":%.2f,"
                 "\"pressure_kpa\":%.2f,"
                 "\"depth_m\":%.2f,"
                 "\"phase\":\"%s\","
                 "\"sensor_depth_m\":%.2f}",
                 companyNumber,
                 static_cast<unsigned>(atoi(profileId)),
                 atof(timeS),
                 atof(pressureKpa),
                 atof(depthM),
                 phase,
                 sensorDepthM == nullptr ? 0.0 : atof(sensorDepthM));

        sender(packet, timeoutMs);
        packetCount++;
        delay(50);
    }

    file.close();
    Debug.printf("Flash: transmitted %u data packets\n", packetCount);
    return true;
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::printLogTo(Stream& out) {
    if (!_available || !logFileExists()) return false;

    File file = LittleFS.open(FLASH_LOG_PATH, FILE_READ);
    if (!file) {
        Debug.println("Flash: failed to open log for read");
        return false;
    }

    while (file.available()) {
        out.write(file.read());
    }

    file.close();
    return true;
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::removeLog() {
    if (!_available) return false;
    if (!logFileExists()) return true;

    const bool ok = LittleFS.remove(FLASH_LOG_PATH);
    if (!ok) Debug.println("Flash: failed to remove log");
    return ok;
}

// ---------------------------------------------------------------------------
size_t FlashStorageManager::logSize() {
    if (!_available || !logFileExists()) return 0;

    File file = LittleFS.open(FLASH_LOG_PATH, FILE_READ);
    if (!file) return 0;

    const size_t size = file.size();
    file.close();
    return size;
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::_writeHeader() {
    File file = LittleFS.open(FLASH_LOG_PATH, FILE_WRITE);
    if (!file) {
        Debug.println("Flash: failed to create log");
        return false;
    }

    file.println(CSV_HEADER);
    const bool ok = file.getWriteError() == 0;
    file.close();

    if (!ok) Debug.println("Flash: write error while creating header");
    return ok;
}

// ---------------------------------------------------------------------------
bool FlashStorageManager::_ensureParentDir() {
    const char* lastSlash = strrchr(FLASH_LOG_PATH, '/');
    if (lastSlash == nullptr || lastSlash == FLASH_LOG_PATH) return true;

    char dir[64];
    const size_t len = static_cast<size_t>(lastSlash - FLASH_LOG_PATH);
    if (len >= sizeof(dir)) {
        Debug.println("Flash: log directory path too long");
        return false;
    }

    memcpy(dir, FLASH_LOG_PATH, len);
    dir[len] = '\0';

    if (LittleFS.exists(dir)) return true;
    if (LittleFS.mkdir(dir)) return true;

    Debug.println("Flash: failed to create log directory");
    return false;
}

// ---------------------------------------------------------------------------
void FlashStorageManager::_writeCsvField(File& file, const char* value) {
    if (value == nullptr) return;

    bool needsQuotes = false;
    for (const char* p = value; *p != '\0'; p++) {
        if (*p == ',' || *p == '"' || *p == '\n' || *p == '\r') {
            needsQuotes = true;
            break;
        }
    }

    if (!needsQuotes) {
        file.print(value);
        return;
    }

    file.print('"');
    for (const char* p = value; *p != '\0'; p++) {
        if (*p == '"') file.print('"');
        file.print(*p);
    }
    file.print('"');
}
