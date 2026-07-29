#pragma once

#include <Arduino.h>
#include <FS.h>

/*
 *******************************************************************************
 * flash_storage.h
 * ESP32 internal flash CSV logging helper backed by LittleFS.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

class FlashStorageManager {
public:
    using PacketSender = bool (*)(const char* message, uint32_t timeoutMs);

    FlashStorageManager();

    // Mount LittleFS and create the configured log file when missing.
    // Returns false on failure and never blocks the firmware indefinitely.
    bool begin();

    bool isAvailable() const { return _available; }

    // Create the configured log file with a CSV header if it does not exist.
    bool ensureLogFile();

    // Append one profile record to the CSV log.
    bool appendRecord(const char* companyNumber,
                      uint8_t profileId,
                      float timeS,
                      float pressureKpa,
                      float depthM,
                      const char* phase,
                      float sensorDepthM,
                      float syringeU);

    // Transmit CSV records whose time matches DATA_PACKET_PERIOD_MS.
    bool transmitDataPackets(PacketSender sender, uint32_t timeoutMs);

    // Delete and recreate the CSV log with only the header.
    bool clearLog();

    // Print the complete CSV log to any Arduino Stream.
    bool printLogTo(Stream& out);

    // Delete the CSV log without recreating it.
    bool removeLog();

    // Return current CSV log size in bytes, or 0 if unavailable/missing.
    size_t logSize();

private:
    bool _available = false;

    bool _writeHeader();
    bool _ensureParentDir();
    void _writeCsvField(File& file, const char* value);
};

// Singleton
extern FlashStorageManager flashStorage;
