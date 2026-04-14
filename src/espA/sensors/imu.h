#pragma once
#include <Arduino.h>

class IMUController {
public:
    IMUController();

    // Inizializza la comunicazione con la WT61
    bool begin();

    // Aggiorna le misurazioni lette dal sensore
    void read();

    // Variabili pubbliche per i dati di navigazione
    float roll;
    float pitch;
    float yaw;
    float acc[3];
    float gyro[3];
    float temperature;
    
private:
    bool _isInitialized;
};

extern IMUController imu;