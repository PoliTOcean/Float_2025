#include "imu.h"
#include <Wire.h>
#include <DebugSerial.h>

constexpr uint8_t WT61_ADDR = 0x50;

// Registri di esempio WT61 (verificate sul datasheet WIT-Motion)
constexpr uint8_t REG_ACCX = 0x34;
constexpr uint8_t REG_ROLL = 0x3D;

IMUController imu;

IMUController::IMUController() : roll(0), pitch(0), yaw(0), temperature(0), _isInitialized(false) {}

bool IMUController::begin() {
    // La Wire.begin() è già chiamata in sensors.begin() nel vostro FLOAT
    
    Wire.beginTransmission(WT61_ADDR);
    if (Wire.endTransmission() == 0) {
        _isInitialized = true;
        Debug.println("IMU WT61 rilevata su bus I2C (0x50)");
    } else {
        Debug.println("ERRORE: IMU WT61 non trovata. Controllare i cavi (SDA/SCL)!");
        return false;
    }
    return true;
}

void IMUController::read() {
    if (!_isInitialized) return;

    // ESEMPIO DI LETTURA ANGOLI (Roll, Pitch, Yaw)
    Wire.beginTransmission(WT61_ADDR);
    Wire.write(REG_ROLL); // Registro di partenza per l'angolo
    Wire.endTransmission(false);
    
    // Leggiamo 6 byte (2 byte Roll, 2 byte Pitch, 2 byte Yaw)
    Wire.requestFrom(WT61_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
        int16_t raw_roll  = (Wire.read() | (Wire.read() << 8));
        int16_t raw_pitch = (Wire.read() | (Wire.read() << 8));
        int16_t raw_yaw   = (Wire.read() | (Wire.read() << 8));

        // Conversione basata sul datasheet della WT61 (solitamente / 32768 * 180)
        double roll_new  = (float)raw_roll / 32768.0f * 180.0f;
        double pitch_new = (float)raw_pitch / 32768.0f * 180.0f;
        double yaw_new   = (float)raw_yaw / 32768.0f * 180.0f;

        // Scambio degli assi mantenendo la logica usata in Oceanix
        if (roll_new != 0)  pitch = roll_new;  
        if (yaw_new != 0)   yaw = yaw_new * -1;
        if (pitch_new != 0) roll = pitch_new;
    }
}