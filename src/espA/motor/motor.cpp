#include "motor.h"
#include "../config.h"
#include "../led/led.h"
#include <DebugSerial.h>

/*
 *******************************************************************************
 * motor.cpp
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
// TOF Sensor (file-scope global)
// ---------------------------------------------------------------------------
static VL53L7CX g_tofSensor(&Wire, TOF_LPN_PIN, TOF_I2C_RST_PIN);

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------
MotorController motor;

// ---------------------------------------------------------------------------
MotorController::MotorController()
    : stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR) {}

// ---------------------------------------------------------------------------
void MotorController::begin() {
    pinMode(PIN_EN,        OUTPUT);
    pinMode(PIN_DRV_SLEEP, OUTPUT);
    pinMode(PIN_DRV_RST,   OUTPUT);

    // Driver awake e reset non asserted PRIMA di tutto
    digitalWrite(PIN_DRV_SLEEP, HIGH);
    digitalWrite(PIN_DRV_RST,   HIGH);
    digitalWrite(PIN_EN,        HIGH); // Disabilitato a mano (HIGH = disabilitato su DRV8825)

    // ORDINE CRITICO: prima setPinsInverted, poi setEnablePin
    stepper.setPinsInverted(/*dir*/true, /*step*/false, /*enable*/true);
    stepper.setEnablePin(PIN_EN);

    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);
    _disableOutputs();

    // --- Initialize TOF Sensor ---
    Debug.println("Motor: initializing TOF sensor");
    
    // Hardware reset
    pinMode(TOF_LPN_PIN,       OUTPUT);
    pinMode(TOF_I2C_RST_PIN,   OUTPUT);
    digitalWrite(TOF_LPN_PIN,      LOW);
    digitalWrite(TOF_I2C_RST_PIN,  LOW);
    delay(100);
    digitalWrite(TOF_LPN_PIN,      HIGH);
    digitalWrite(TOF_I2C_RST_PIN,  HIGH);
    delay(100);

    // I2C init (already done in main setup, but ensure clock is set)
    Wire.setClock(1000000);

    // Load firmware (5-10 sec)
    Debug.println("Motor: Loading TOF firmware (wait 5-10 sec)...");
    if (g_tofSensor.begin() != VL53L7CX_STATUS_OK) {
        Debug.println("ERROR: TOF begin() failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    if (g_tofSensor.init_sensor() != VL53L7CX_STATUS_OK) {
        Debug.println("ERROR: TOF init_sensor failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // Set resolution to 4x4
    uint8_t status = g_tofSensor.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
    if (status != VL53L7CX_STATUS_OK) {
        Debug.println("ERROR: TOF set_resolution failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // Increase ranging frequency for faster homing detection
    status = g_tofSensor.vl53l7cx_set_ranging_frequency_hz(30);
    if (status != VL53L7CX_STATUS_OK) {
        Debug.println("WARNING: TOF set_ranging_frequency_hz failed, continuing with default");
    } else {
        Debug.println("Motor: TOF ranging frequency set to 30 Hz");
    }

    // Start ranging
    status = g_tofSensor.vl53l7cx_start_ranging();
    if (status != VL53L7CX_STATUS_OK) {
        Debug.println("ERROR: TOF start_ranging failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    Debug.println("Motor: TOF sensor initialized successfully");
}

// ---------------------------------------------------------------------------
bool MotorController::home() {
    Debug.println("Motor: starting homing sequence with TOF");
    ledController.setState(LEDState::HOMING);

    _homed         = false;
    _upperHit      = false;
    _lowerHit      = false;
    _emergencyStop = false;

    _enableOutputs();
    stepper.setMaxSpeed(MOTOR_HOMING_SPEED);
    stepper.setAcceleration(MOTOR_HOMING_SPEED);
    stepper.move(-static_cast<long>(MOTOR_MAX_STEPS) * 2); // Overshoot to guarantee contact

    unsigned long startMs = millis();
    bool tof_detected = false;

    while (!tof_detected && !_emergencyStop) {
        if (millis() - startMs > MOTOR_HOMING_TIMEOUT) {
            Debug.println("Motor: homing timed out");
            stepper.stop();
            _disableOutputs();
            return false;
        }

        // Check TOF sensor
        uint8_t ready = 0;
        g_tofSensor.vl53l7cx_check_data_ready(&ready);
        
        if (ready) {
            VL53L7CX_ResultsData data;
            g_tofSensor.vl53l7cx_get_ranging_data(&data);
            float distance_mm = data.distance_mm[5]; // Center pixel of 4x4 grid
            
            Debug.printf("Motor: homing — TOF distance = %.1f mm\n", distance_mm);
            
            if (distance_mm < TOF_HOMING_THRESHOLD) {
                Debug.printf("Motor: TOF threshold reached (%.1f < %.1f mm)\n", 
                            distance_mm, TOF_HOMING_THRESHOLD);
                tof_detected = true;
                _lowerHit = true;
            }
        }

        stepper.run();
        ledController.update();
        yield();
    }

    delay(100); // Let the motor settle

    if (!tof_detected) {
        Debug.println("Motor: homing aborted — no TOF detection");
        _disableOutputs();
        return false;
    }

    // Back off and define home position
    _handleEndstopHit();                  // Backs off ENDSTOP_MARGIN steps
    stepper.setCurrentPosition(0);        // Zero = home
    _emergencyStop = false;

    // Restore normal operating speeds
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);

    _homed = true;
    _disableOutputs();
    Debug.println("Motor: homing complete — position set to 0");
    return true;
}

// ---------------------------------------------------------------------------
bool MotorController::moveTo(long targetPosition) {
    if (!_homed) {
        Debug.println("Motor: moveTo() called before homing — ignoring");
        return false;
    }

    // Clamp to safe range
    targetPosition = constrain(targetPosition,
                                MOTOR_ENDSTOP_MARGIN,
                                MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);

    _upperHit      = false;
    _lowerHit      = false;
    _emergencyStop = false;

    _enableOutputs();
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_MAX_SPEED);
    stepper.moveTo(targetPosition);

    Debug.printf("Motor: moving to %ld\n", targetPosition);

    while (stepper.distanceToGo() != 0 && !_emergencyStop) {
        stepper.run();
        processEndstops(/*isHoming=*/false);
        ledController.update();
        yield();
    }

    if (_emergencyStop) {
        Debug.println("Motor: emergency stop during moveTo");
        _handleEndstopHit();
        _disableOutputs();
        return false;
    }

    _disableOutputs();
    Debug.printf("Motor: reached position %ld\n", stepper.currentPosition());
    return true;
}

// ---------------------------------------------------------------------------
bool MotorController::moveSteps(long steps) {
    return moveTo(stepper.currentPosition() + steps);
}

// ---------------------------------------------------------------------------
long MotorController::position() {
    return stepper.currentPosition();
}

// ---------------------------------------------------------------------------
void MotorController::processEndstops(bool isHoming) {
    // With TOF homing, processEndstops is no longer needed during normal operation.
    // Homing now detects TOF directly in home().
    // This function is kept for compatibility but performs no action.
    (void)isHoming; // Suppress unused parameter warning
}

// ---------------------------------------------------------------------------
void MotorController::_handleEndstopHit() {
    _enableOutputs();
    constexpr unsigned long BACKOFF_TIMEOUT_MS = 2000;

    if (_upperHit) {
        stepper.setAcceleration(MOTOR_MAX_SPEED);
        stepper.move(-MOTOR_ENDSTOP_MARGIN);

        unsigned long t0 = millis();
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            ledController.update();
            yield();
            if (millis() - t0 > BACKOFF_TIMEOUT_MS) {
                Debug.println("Motor: timeout backing off upper endstop");
                stepper.stop();
                break;
            }
        }
        stepper.setCurrentPosition(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
        Debug.printf("Motor: backed off upper endstop → pos %ld\n", stepper.currentPosition());
        _upperHit = false;
    }

    if (_lowerHit) {
        stepper.setAcceleration(MOTOR_HOMING_SPEED);
        stepper.move(MOTOR_ENDSTOP_MARGIN);

        unsigned long t0 = millis();
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            ledController.update();
            yield();
            if (millis() - t0 > BACKOFF_TIMEOUT_MS) {
                Debug.println("Motor: timeout backing off lower endstop");
                stepper.stop();
                break;
            }
        }
        stepper.setCurrentPosition(MOTOR_ENDSTOP_MARGIN);
        Debug.printf("Motor: backed off lower endstop → pos %ld\n", stepper.currentPosition());
        _lowerHit = false;
    }

    _disableOutputs();
}

// ---------------------------------------------------------------------------
void MotorController::_enableOutputs()  { stepper.enableOutputs(); }
void MotorController::_disableOutputs() { stepper.disableOutputs(); }