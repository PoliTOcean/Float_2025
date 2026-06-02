#pragma once

#include <Arduino.h>
#include "motor.h"
#include "tof.h"

/*
 *******************************************************************************
 * motion_control.h
 * Firmware-level motion control that coordinates motor, TOF, LED and debug.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

class MotionController {
public:
    MotionController(MotorController& motor, TofSensor& tof);

    bool homeWithTof(float stopPressureKpa = 0.0f, bool* pressureStop = nullptr, uint8_t* pressureStopSamples = nullptr);
    bool waitForMotor(uint32_t timeoutMs);
    bool moveToMax(uint32_t timeoutMs = 0, float stopPressureKpa = 0.0f, bool* pressureStop = nullptr, uint8_t* pressureStopSamples = nullptr);
    bool moveToWithTimeout(long targetPosition, uint32_t timeoutMs, bool keepOutputsEnabled = false);
    bool manualStepTest(long steps, uint32_t speed);
    bool balance();

    bool motionAllowed();
    bool emergencyStopActive() const { return _emergencyStop; }
    void clearEmergencyStop();
    void emergencyStop(const char* reason);
    void serviceEmergencyStop();
    bool remoteStopRequested();

    // Diagnostica dell'ultimo emergency stop, per loggarla nel flash CSV: il
    // reason è stampato solo su Serial in tempo reale, ma in piscina la USB è
    // scollegata, quindi va salvato. _lastStopTofMm vale -1 se lo stop non è
    // stato causato dal TOF (timeout, remote stop, ...).
    const char* lastStopReason() const { return _lastStopReason; }
    float lastStopTofMm() const { return _lastStopTofMm; }

private:
    MotorController& _motor;
    TofSensor& _tof;
    bool _emergencyStop = false;
    const char* _lastStopReason = "";
    float _lastStopTofMm = -1.0f;
    // Letture TOF consecutive fuori range: un emergency stop scatta solo dopo
    // TOF_SAFETY_STOP_SAMPLES conferme, per ignorare glitch singoli in acqua.
    uint8_t _tofOutOfRangeCount = 0;

    float readPressureKpa();
    bool tofMaxExtensionStopReached(unsigned long nowMs,
                                    unsigned long& lastTofSampleMs,
                                    const char* context);
    bool pressureStopReached(float stopPressureKpa,
                             uint8_t requiredSamples,
                             uint8_t* pressureStopSamples = nullptr);
    bool waitWithPressureStop(uint32_t waitMs,
                              float stopPressureKpa,
                              uint8_t requiredSamples,
                              uint16_t samplePeriodMs,
                              uint8_t* pressureStopSamples = nullptr);

    // Esegue un singolo stroke del balance (extend o retract) come move assoluto
    // verso targetPos. Ritorna true se va a buon fine, false in caso di
    // remoteStop/pressureStop/timeout (l'esito esatto è loggato e propagato
    // tramite pressureStopHit). label è usata solo per i log.
    bool _balanceStrokeTo(long targetPos,
                          const char* label,
                          float stopPressureKpa,
                          uint8_t* pressureStopSamples,
                          uint32_t timeoutMs,
                          bool& pressureStopHit,
                          bool& remoteStopHit);
};

// Singleton defined by the main firmware.
extern MotionController motionController;
