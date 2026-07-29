#include "motion_control.h"
#include "config.h"
#include "led.h"
#include "sensors.h"
#include "comms.h"
#include "DebugSerial.h"
#include "runtime_config.h"
#include "flash_storage.h"

/*
 *******************************************************************************
 * motion_control.cpp
 * High-level motion routines coordinating motor, TOF, LED and emergency stop:
 * two-phase TOF homing, safe full extension, balance/purge cycles with
 * pressure-based stop, and remote stop / safety-range supervision.
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

MotionController::MotionController(MotorController& motor, TofSensor& tof)
    : _motor(motor),
      _tof(tof) {}

void MotionController::clearEmergencyStop() {
    _emergencyStop = false;
}

void MotionController::emergencyStop(const char* reason) {
    // Guard di idempotenza: emergencyStop() può essere richiamato durante un
    // emergency stop già attivo. Logghiamo (e fermiamo) una sola volta.
    const bool firstTrigger = !_emergencyStop;

    _emergencyStop = true;
    _lastStopReason = reason;
    _motor.stop();
    _motor.disableOutputs();
    Debug.printf("Motor emergency stop: %s\n", reason);
    ledController.setState(LEDState::ERROR);

    if (!firstTrigger) {
        return;
    }

    // Registra l'evento sul flash NELL'ISTANTE in cui scatta, non a posteriori:
    // il vecchio approccio (blocco "aborted" in loop()) mancava lo stop se
    // measure() usciva per timeout di fase invece che per emergency stop, o se
    // l'auto-recovery azzerava lo stop prima del check. Qui è impossibile
    // mancarlo. Scrittura singola (firstTrigger) → nessun rischio di append
    // ripetuti. reason e tof esistono solo qui: in piscina la USB è scollegata.
    sensors.read();
    char phase[64];
    snprintf(phase, sizeof(phase), "emergency_stop:%s tof=%.1fmm",
             reason, _lastStopTofMm);
    flashStorage.appendRecord(COMPANY_NUMBER, 0,
                              static_cast<float>(millis()) / 1000.0f,
                              sensors.pressure() / 1000.0f,
                              sensors.depth(), phase,
                              sensors.sensorDepth(),
                              motorPosToU(_motor.position()));
}

void MotionController::logHomingEvent(const char* detail) {
    // Stesso meccanismo di emergencyStop(): scriviamo subito sul flash perché
    // in piscina la USB è scollegata e vogliamo poter ricostruire a posteriori
    // dove l'homing ha sbagliato (errore vs sforamento distanza massima).
    sensors.read();
    char phase[96];
    snprintf(phase, sizeof(phase), "homing:%s", detail);
    flashStorage.appendRecord(COMPANY_NUMBER, 0,
                              static_cast<float>(millis()) / 1000.0f,
                              sensors.pressure() / 1000.0f,
                              sensors.depth(), phase,
                              sensors.sensorDepth(),
                              motorPosToU(_motor.position()));
}

void MotionController::serviceEmergencyStop() {
    if (!_emergencyStop) {
        return;
    }

    _motor.stop();
    _motor.disableOutputs();
    ledController.setState(LEDState::ERROR);
}

bool MotionController::remoteStopRequested() {
    if (comms.lastCommand().command != CMD_STOP) {
        return false;
    }

    comms.clearCommand();
    emergencyStop("remote stop");
    comms.sendMessage(CMD13_ACK, 1000);
    return true;
}

bool MotionController::motionAllowed() {
    if (!_emergencyStop) {
        return true;
    }

    Debug.println("Motor command blocked: emergency stop active");
    serviceEmergencyStop();
    return false;
}

bool MotionController::waitForMotor(uint32_t timeoutMs) {
    const unsigned long startMs = millis();
    unsigned long lastTofSampleMs = 0;

    while (_motor.distanceToGo() != 0) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (timeoutMs > 0 && millis() - startMs > timeoutMs) {
            emergencyStop("movement timeout");
            return false;
        }

        _motor.run();
        switch (tofGuard(nowMs, lastTofSampleMs, "movement")) {
            case TofGuard::Emergency:
                return false;
            case TofGuard::ExtendLimit:
                // Fondo corsa esteso: stop pulito, il movimento è "completato"
                // al limite fisico senza emergency (protezione tappo).
                _motor.stop();
                return true;
            case TofGuard::Ok:
                break;
        }

        ledController.update();
        yield();
    }

    return true;
}

float MotionController::readPressureKpa() {
    sensors.read();
    return sensors.pressure() / 1000.0f;
}

TofGuard MotionController::tofGuard(unsigned long nowMs,
                                   unsigned long& lastTofSampleMs,
                                   const char* context) {
    if (!_tof.isInitialized()) {
        return TofGuard::Ok;
    }

    if (nowMs - lastTofSampleMs < MOTOR_HOMING_TOF_PERIOD_MS) {
        return TofGuard::Ok;
    }
    lastTofSampleMs = nowMs;

    float distanceMm = 0.0f;
    if (!_tof.readDistanceMm(distanceMm)) {
        return TofGuard::Ok;
    }

    const bool tooClose = distanceMm < TOF_SAFE_RANGE_MIN_MM;
    const bool tooFar   = distanceMm > TOF_SAFE_RANGE_MAX_MM;

    if (!tooClose && !tooFar) {
        // Lettura valida: azzera la conferma in corso (un glitch isolato non
        // deve accumularsi nel tempo).
        _tofOutOfRangeCount = 0;
        return TofGuard::Ok;
    }

    // Lettura fuori range: conferma prima di agire, per ignorare glitch singoli
    // (bolle, riflessi, torbidità) tipici del TOF in acqua.
    if (++_tofOutOfRangeCount < TOF_SAFETY_STOP_SAMPLES) {
        Debug.printf("%s: TOF out of range (%.1f mm), sample %u/%u\n",
                     context, distanceMm,
                     _tofOutOfRangeCount, TOF_SAFETY_STOP_SAMPLES);
        return TofGuard::Ok;
    }

    _tofOutOfRangeCount = 0;
    _lastStopTofMm = distanceMm;
    if (tooClose) {
        // Limite inferiore = siringa a fondo estensione, prima del tappo. STOP
        // PULITO: il chiamante ferma il pistone senza abortire la missione.
        Debug.printf("%s: TOF extension limit, too close (%.1f < %.1f mm)\n",
                     context, distanceMm, TOF_SAFE_RANGE_MIN_MM);
        return TofGuard::ExtendLimit;
    }

    // Limite superiore = anomalia (passi persi, verso sbagliato): emergency stop.
    Debug.printf("%s: TOF safety stop, too far (%.1f > %.1f mm)\n",
                 context, distanceMm, TOF_SAFE_RANGE_MAX_MM);
    emergencyStop("TOF above safe range");
    return TofGuard::Emergency;
}

bool MotionController::pressureStopReached(float stopPressureKpa,
                                           uint8_t requiredSamples,
                                           uint8_t* pressureStopSamples) {
    if (stopPressureKpa <= 0.0f) {
        return false;
    }

    const float pressureKpa = readPressureKpa();
    if (pressureKpa > stopPressureKpa) {
        if (pressureStopSamples != nullptr) {
            (*pressureStopSamples)++;
            if (*pressureStopSamples < requiredSamples) {
                return false;
            }
        }

        _motor.stop();
        _motor.disableOutputs();
        Debug.printf("Balance: pressure stop %.2f kPa > %.2f kPa\n",
                     pressureKpa,
                     stopPressureKpa);
        return true;
    }

    if (pressureStopSamples != nullptr) {
        *pressureStopSamples = 0;
    }

    return false;
}

bool MotionController::waitWithPressureStop(uint32_t waitMs,
                                            float stopPressureKpa,
                                            uint8_t requiredSamples,
                                            uint16_t samplePeriodMs,
                                            uint8_t* pressureStopSamples) {
    const unsigned long startMs = millis();

    while (millis() - startMs < waitMs) {
        if (remoteStopRequested()) {
            return true;
        }

        if (pressureStopReached(stopPressureKpa, requiredSamples, pressureStopSamples)) {
            return true;
        }

        ledController.update();
        delay(samplePeriodMs);
    }

    return false;
}

bool MotionController::homeWithTof(float stopPressureKpa, bool* pressureStop, uint8_t* pressureStopSamples) {
    if (pressureStop != nullptr) {
        *pressureStop = false;
    }

    Debug.println("Motor homing: starting with TOF");
    clearEmergencyStop();
    ledController.setState(LEDState::HOMING);

    {
        float startTof = -1.0f;
        _tof.readDistanceMm(startTof);
        char detail[80];
        snprintf(detail, sizeof(detail),
                 "start startTof=%.1fmm approachThr=%.1f homeThr=%.1f maxSteps=%ld",
                 startTof, (float)TOF_HOMING_APPROACH_MM, (float)TOF_HOMING_THRESHOLD,
                 (long)MOTOR_MAX_STEPS);
        logHomingEvent(detail);
    }

    _motor.clearPosition();
    _motor.enableOutputs();
    const RuntimeMotorConfig& motorConfig = runtimeConfig.motor();
    const RuntimeBalanceConfig& balanceConfig = runtimeConfig.balance();
    _motor.setMaxSpeed(motorConfig.homingSpeed);
    _motor.setAcceleration(motorConfig.homingSpeed);

    const unsigned long startMs = millis();
    unsigned long lastTofSampleMs = 0;
    unsigned long lastPressureSampleMs = 0;

    // Phase 1: approach — move TOWARD the TOF (negative direction, siringa che si estende)
    // finché il TOF legge sotto TOF_HOMING_APPROACH_MM. Garantisce un punto di partenza
    // riproducibile indipendentemente dalla posizione iniziale.
    Debug.println("Motor homing: phase 1 (approach toward TOF)");
    _motor.startMoveSteps(-static_cast<long>(MOTOR_MAX_STEPS) * 2);

    bool approachDone = false;
    uint8_t approachSamples = 0;
    while (_motor.distanceToGo() != 0 && !approachDone) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor homing: timed out during approach");
            char detail[80];
            snprintf(detail, sizeof(detail),
                     "phase1_timeout pos=%ld toGo=%ld elapsed=%lums",
                     (long)_motor.position(), (long)_motor.distanceToGo(),
                     millis() - startMs);
            logHomingEvent(detail);
            emergencyStop("homing timeout");
            return false;
        }

        _motor.run();

        if (nowMs - lastTofSampleMs >= MOTOR_HOMING_TOF_PERIOD_MS) {
            lastTofSampleMs = nowMs;

            float distanceMm = 0.0f;
            if (_tof.readDistanceMm(distanceMm)) {
                if (distanceMm < TOF_HOMING_APPROACH_MM) {
                    if (++approachSamples >= TOF_HOMING_CONFIRM_SAMPLES) {
                        Debug.printf("Motor homing: approach reached (%.1f < %.1f mm)\n",
                                     distanceMm, TOF_HOMING_APPROACH_MM);
                        approachDone = true;
                        _motor.stop();
                    }
                } else {
                    approachSamples = 0;
                }
            }
        }

        ledController.update();
        yield();
    }

    if (!approachDone) {
        Debug.println("Motor homing: approach phase failed");
        // distanceToGo()==0 senza approachDone significa che il motore ha
        // esaurito la corsa massima (2*MOTOR_MAX_STEPS) senza che il TOF
        // scendesse sotto la soglia di approccio → "sfora la distanza massima".
        float distanceMm = -1.0f;
        _tof.readDistanceMm(distanceMm);
        char detail[80];
        snprintf(detail, sizeof(detail),
                 "phase1_no_approach pos=%ld lastTof=%.1fmm thr=%.1f",
                 (long)_motor.position(), distanceMm, (float)TOF_HOMING_APPROACH_MM);
        logHomingEvent(detail);
        emergencyStop("homing approach failed");
        return false;
    }

    {
        const unsigned long settleStart = millis();
        while (_motor.distanceToGo() != 0) {
            if (remoteStopRequested()) {
                return false;
            }
            if (millis() - settleStart > 2000) {
                Debug.println("Motor homing: timeout settling approach");
                char detail[64];
                snprintf(detail, sizeof(detail),
                         "phase1_settle_timeout pos=%ld toGo=%ld",
                         (long)_motor.position(), (long)_motor.distanceToGo());
                logHomingEvent(detail);
                emergencyStop("homing approach settle timeout");
                return false;
            }
            _motor.run();
            ledController.update();
            yield();
        }
    }

    // Phase 2: homing — invert direction (positive, siringa che si retrae) finché TOF legge
    // sopra TOF_HOMING_THRESHOLD.
    Debug.println("Motor homing: phase 2 (retract away from TOF)");
    {
        char detail[48];
        snprintf(detail, sizeof(detail), "phase2_start pos=%ld",
                 (long)_motor.position());
        logHomingEvent(detail);
    }
    _motor.startMoveSteps(static_cast<long>(MOTOR_MAX_STEPS) * 2);
    bool homeDetected = false;
    uint8_t homeSamples = 0;

    while (_motor.distanceToGo() != 0 && !homeDetected) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastPressureSampleMs >= balanceConfig.samplePeriodMs) {
            lastPressureSampleMs = nowMs;
            if (pressureStopReached(stopPressureKpa, balanceConfig.stopPressureSamples, pressureStopSamples)) {
                if (pressureStop != nullptr) {
                    *pressureStop = true;
                }
                return true;
            }
        }

        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor homing: timed out");
            char detail[80];
            snprintf(detail, sizeof(detail),
                     "phase2_timeout pos=%ld toGo=%ld elapsed=%lums",
                     (long)_motor.position(), (long)_motor.distanceToGo(),
                     millis() - startMs);
            logHomingEvent(detail);
            emergencyStop("homing timeout");
            return false;
        }

        _motor.run();

        if (nowMs - lastTofSampleMs >= MOTOR_HOMING_TOF_PERIOD_MS) {
            lastTofSampleMs = nowMs;

            float distanceMm = 0.0f;
            if (_tof.readDistanceMm(distanceMm)) {
                if (distanceMm > TOF_HOMING_THRESHOLD) {
                    if (++homeSamples >= TOF_HOMING_CONFIRM_SAMPLES) {
                        Debug.printf("Motor homing: threshold reached (%.1f > %.1f mm)\n",
                                     distanceMm, TOF_HOMING_THRESHOLD);
                        homeDetected = true;
                        _motor.stop();
                    }
                } else {
                    homeSamples = 0;
                }
            }
        }

        ledController.update();
        yield();
    }

    if (!homeDetected) {
        Debug.println("Motor homing: no TOF detection");
        // distanceToGo()==0 senza homeDetected: il motore ha consumato tutta la
        // corsa (2*MOTOR_MAX_STEPS) in retrazione senza che il TOF superasse la
        // soglia di home → è il caso "sfora la distanza massima".
        float distanceMm = -1.0f;
        _tof.readDistanceMm(distanceMm);
        char detail[80];
        snprintf(detail, sizeof(detail),
                 "phase2_no_detect pos=%ld lastTof=%.1fmm thr=%.1f",
                 (long)_motor.position(), distanceMm, (float)TOF_HOMING_THRESHOLD);
        logHomingEvent(detail);
        emergencyStop("homing finished without TOF detection");
        return false;
    }

    delay(100);

    _motor.startMoveSteps(static_cast<long>(MOTOR_ENDSTOP_MARGIN));
    if (!waitForMotor(2000)) {
        Debug.println("Motor homing: timeout during backoff");
        char detail[48];
        snprintf(detail, sizeof(detail), "backoff_timeout pos=%ld",
                 (long)_motor.position());
        logHomingEvent(detail);
        return false;
    }

    _motor.setCurrentPosition(0);
    runtimeConfig.applyMotorConfig();
    _motor.disableOutputs();
    clearEmergencyStop();

    Debug.println("Motor homing: complete, position set to 0");
    logHomingEvent("complete pos=0");
    return true;
}

bool MotionController::moveToWithTimeout(long targetPosition,
                                         uint32_t timeoutMs,
                                         bool keepOutputsEnabled) {
    if (!motionAllowed()) {
        return false;
    }

    if (!_motor.isPositionKnown()) {
        emergencyStop("move requested before motor position is known");
        return false;
    }

    _motor.enableOutputs();
    _motor.startMoveTo(targetPosition);

    const bool success = waitForMotor(timeoutMs);
    if (success && !keepOutputsEnabled) {
        _motor.disableOutputs();
    }

    return success;
}

bool MotionController::moveToMax(uint32_t timeoutMs,
                                 float stopPressureKpa,
                                 bool* pressureStop,
                                 uint8_t* pressureStopSamples) {
    if (pressureStop != nullptr) {
        *pressureStop = false;
    }

    if (!motionAllowed()) {
        return false;
    }

    if (!_motor.isPositionKnown()) {
        emergencyStop("moveToMax requested before motor position is known");
        return false;
    }

    // Home (pos=0) = pistone tutto inserito, siringa vuota → galleggia.
    // u=1 → siringa piena → affonda. uToMotorPos() rispetta MOTOR_INVERT_LOGICAL.
    const long targetPosition = uToMotorPos(1.0f);
    _motor.enableOutputs();
    _motor.startMoveTo(targetPosition);

    const unsigned long startMs = millis();
    unsigned long lastTofSampleMs = 0;
    unsigned long lastPressureSampleMs = 0;
    const RuntimeBalanceConfig& balanceConfig = runtimeConfig.balance();

    while (_motor.distanceToGo() != 0) {
        if (remoteStopRequested()) {
            return false;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastPressureSampleMs >= balanceConfig.samplePeriodMs) {
            lastPressureSampleMs = nowMs;
            if (pressureStopReached(stopPressureKpa, balanceConfig.stopPressureSamples, pressureStopSamples)) {
                if (pressureStop != nullptr) {
                    *pressureStop = true;
                }
                return true;
            }
        }

        if (timeoutMs > 0 && millis() - startMs > timeoutMs) {
            emergencyStop("moveToMax timeout");
            return false;
        }

        _motor.run();

        // Limite TOF inferiore = siringa completamente estesa: stop PULITO (non
        // emergency). Permette al chiamante (es. balance) di fare l'hold a fine
        // corsa invece di considerarlo un errore. Limite superiore = anomalia →
        // emergency stop (già scattato dentro tofGuard).
        switch (tofGuard(nowMs, lastTofSampleMs, "moveToMax")) {
            case TofGuard::Emergency:
                return false;
            case TofGuard::ExtendLimit:
                _motor.stop();
                _motor.disableOutputs();
                return true;
            case TofGuard::Ok:
                break;
        }

        ledController.update();
        yield();
    }

    _motor.disableOutputs();
    return true;
}

bool MotionController::manualStepTest(long steps, uint32_t speed) {
    if (!motionAllowed()) {
        return false;
    }

    Debug.printf("Test: moving %ld steps at %u steps/s\n", steps, speed);

    _motor.setMaxSpeed(speed);
    _motor.setAcceleration(runtimeConfig.motor().maxAcceleration);
    _motor.enableOutputs();
    _motor.startMoveSteps(steps);

    const bool success = waitForMotor(0);

    runtimeConfig.applyMotorConfig();
    if (success) {
        _motor.disableOutputs();
        Debug.printf("Test complete — pos %ld\n", _motor.position());
    }

    return success;
}

bool MotionController::_balanceStrokeTo(long targetPos,
                                        const char* label,
                                        float stopPressureKpa,
                                        uint8_t* pressureStopSamples,
                                        uint32_t timeoutMs,
                                        bool& pressureStopHit,
                                        bool& remoteStopHit) {
    pressureStopHit = false;
    remoteStopHit   = false;

    Debug.printf("Balance: %s to pos=%ld\n", label, targetPos);
    _motor.enableOutputs();
    _motor.startMoveTo(targetPos);

    const unsigned long moveStart = millis();
    unsigned long lastTofSampleMs = 0;
    char tofCtx[32];
    snprintf(tofCtx, sizeof(tofCtx), "balance %s", label);
    while (_motor.distanceToGo() != 0) {
        if (remoteStopRequested()) {
            remoteStopHit = true;
            return false;
        }
        if (pressureStopReached(stopPressureKpa,
                                runtimeConfig.balance().stopPressureSamples,
                                pressureStopSamples)) {
            pressureStopHit = true;
            return false;
        }
        if (timeoutMs > 0 && millis() - moveStart > timeoutMs) {
            char reason[48];
            snprintf(reason, sizeof(reason), "balance %s timeout", label);
            emergencyStop(reason);
            return false;
        }
        _motor.run();
        // Supervisione TOF: fondo corsa esteso (tappo) = stop pulito a fine
        // stroke; oltre il limite superiore = emergency (già scattato).
        switch (tofGuard(millis(), lastTofSampleMs, tofCtx)) {
            case TofGuard::Emergency:
                return false;
            case TofGuard::ExtendLimit:
                _motor.stop();
                _motor.disableOutputs();
                return true;
            case TofGuard::Ok:
                break;
        }
        ledController.update();
        yield();
    }

    _motor.disableOutputs();
    return true;
}

bool MotionController::balance() {
    // Reset di sicurezza: la balance è una routine di spurgo manuale, parte
    // sempre pulita anche se un emergency stop precedente non è stato cancellato.
    clearEmergencyStop();
    Debug.println("Balance: starting");

    if (!_motor.isPositionKnown()) {
        // Senza homing non conosciamo l'orientamento della corsa: estendere
        // alla cieca rischia di sbattere meccanicamente. Si richiede CMD_HOME.
        Debug.println("Balance: homing required (motor position unknown)");
        return false;
    }

    const RuntimeBalanceConfig& balanceConfig = runtimeConfig.balance();
    const uint32_t holdMs = balanceConfig.holdMs;
    const float baselinePressureKpa = readPressureKpa();
    const float stopPressureKpa = baselinePressureKpa + balanceConfig.stopPressureDeltaKpa;
    uint8_t pressureStopSamples = 0;

    Debug.printf("Balance: baseline=%.2f kPa stop=%.2f kPa delta=%.2f kPa\n",
                 baselinePressureKpa,
                 stopPressureKpa,
                 balanceConfig.stopPressureDeltaKpa);

    // u=1 → prende acqua (verso il TOF, direzione negativa) = extend.
    // u=0 → spinge acqua fuori (home, pos=0) = retract.
    const long extendedPos = uToMotorPos(1.0f);
    const long retractedPos = uToMotorPos(0.0f);

    while (motionAllowed()) {
        if (remoteStopRequested()) return false;
        if (pressureStopReached(stopPressureKpa, balanceConfig.stopPressureSamples, &pressureStopSamples)) return true;

        bool pressureHit = false, remoteHit = false;
        if (!_balanceStrokeTo(extendedPos, "extend", stopPressureKpa,
                              &pressureStopSamples, MOTOR_HOMING_TIMEOUT,
                              pressureHit, remoteHit)) {
            return pressureHit; // true se fermato da pressione, false altrimenti
        }

        Debug.printf("Balance: hold extended (%lu ms)\n", (unsigned long)holdMs);
        if (waitWithPressureStop(holdMs,
                                 stopPressureKpa,
                                 balanceConfig.stopPressureSamples,
                                 balanceConfig.samplePeriodMs,
                                 &pressureStopSamples)) {
            return true;
        }

        if (!_balanceStrokeTo(retractedPos, "retract", stopPressureKpa,
                              &pressureStopSamples, MOTOR_HOMING_TIMEOUT,
                              pressureHit, remoteHit)) {
            return pressureHit;
        }

        Debug.printf("Balance: hold retracted (%lu ms)\n", (unsigned long)holdMs);
        if (waitWithPressureStop(holdMs,
                                 stopPressureKpa,
                                 balanceConfig.stopPressureSamples,
                                 balanceConfig.samplePeriodMs,
                                 &pressureStopSamples)) {
            return true;
        }
    }

    return false;
}
