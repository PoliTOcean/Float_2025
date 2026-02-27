/*
 *******************************************************************************
 *                      FLOAT board — ESP32-A (ESPA)  v11.0
 *
 * main.cpp
 * Entry point: setup() + loop() with the main state machine.
 * All subsystem logic lives in the dedicated modules under src/.
 *
 * Module map:
 *   config.h              — pin definitions, tuning constants
 *   led/led.h             — RGB LED state machine
 *   motor/motor.h         — AccelStepper, homing, safe moves, endstops
 *   pid/pid.h             — depth PID controller
 *   sensors/sensors.h     — Bar02 pressure sensor + INA219 power monitor
 *   comms/comms.h         — ESP-NOW messaging + OTA
 *   profile/profile.h     — depth profile execution + EEPROM logging
 *
 * Maintainers: Colabella Davide
 * Past contributors: Fachechi Gino Marco, Gullotta Salvatore
 * Company   : Team PoliTOcean @ Politecnico di Torino
 * Board pkg : esp32 by Espressif Systems v2.0.17
 *******************************************************************************
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <float_common.h>
#include <DebugSerial.h>

#include "config.h"
#include "led/led.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/sensors.h"
#include "comms/comms.h"
#include "profile/profile.h"

// ---------------------------------------------------------------------------
// State machine status codes
// ---------------------------------------------------------------------------
enum Command : uint8_t {
    CMD_IDLE         = 0,
    CMD_GO           = 1,
    CMD_SEND_DATA    = 2,
    CMD_BALANCE      = 3,
    CMD_CLEAR_EEPROM = 4,
    CMD_AUTO_MODE    = 5,
    CMD_SEND_PACKAGE = 6,
    CMD_OTA          = 7,
    CMD_UPDATE_PID   = 8,
    CMD_SET_SPEED    = 9,
    CMD_TEST_STEPS   = 10,
    CMD_DEBUG_MODE   = 11,
    CMD_HOME         = 12,
};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static uint8_t  g_status          = CMD_IDLE;
static uint8_t  g_profileCount    = 0;
static bool     g_autoModeActive  = false;
static bool     g_autoCommitted   = false;
static bool     g_idle            = false;
static bool     g_debugModeActive = false;
static uint32_t g_testSpeed       = MOTOR_MAX_SPEED;

// Make debug_mode_active reachable by DebugSerial / comms (extern linkage)
bool debug_mode_active = false;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    delay(100);
    Serial.begin(115200);
    Serial.println("=== Float ESPA v10.0 — starting ===");

    // --- LED (first, so we can signal errors immediately) ---
    ledController.setState(LEDState::INIT);

    // --- EEPROM ---
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("CRITICAL: EEPROM init failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // --- Communication (ESP-NOW) ---
    comms.begin();

    // --- DebugSerial ---
    // Wraps Serial so Debug.println() can optionally forward over ESP-NOW
    Debug.begin(&debug_mode_active,
                [](const char* msg, uint32_t timeout) -> uint8_t {
                    return comms.sendMessage(msg, timeout) ? 1 : 0;
                });
    Debug.println("DebugSerial ready");

    // --- Sensors (Bar02 + INA219) ---
    // Both _initPressureSensor() and _initPowerMonitor() will block and
    // show LED_ERROR if the hardware is not found.
    sensors.begin();

    // Two short green blinks to confirm both sensors are alive
    ledController.setState(LEDState::INIT);
    delay(500);
    ledController.setState(LEDState::OFF);
    delay(500);
    ledController.setState(LEDState::INIT);
    delay(500);
    ledController.setState(LEDState::OFF);
    delay(500);

    // --- Motor + homing ---
    motor.begin();
    //motor_selftest();
    Debug.println("Starting motor homing...");
    if (!motor.home()) {
        Debug.println("CRITICAL: motor homing failed");
        ledController.setState(LEDState::ERROR);
        while (true) { ledController.update(); yield(); }
    }

    // Initialise outgoing packet battery field above threshold
    comms.status_to_send.charge = BATT_THRESH + 1;

    Debug.println("=== Initialisation complete — float ready ===");
    ledController.setState(LEDState::IDLE);
}

// ---------------------------------------------------------------------------
// LOOP — main state machine
// ---------------------------------------------------------------------------
void loop() {
    ledController.update();

    // Safety net: catch accidental endstop hits while the motor is idle
    if (motor.stepper.distanceToGo() == 0 && !motor.emergencyStop()) {
        motor.processEndstops(/*isHoming=*/false);
        if (motor.emergencyStop()) {
            Debug.println("WARNING: endstop triggered while motor was idle");
            // handleEndstopHit is private; re-home to recover to a known state
            ledController.setState(LEDState::ERROR);
        }
    }

    // -----------------------------------------------------------------------
    switch (g_status) {

    // -----------------------------------------------------------------------
    case CMD_IDLE:
    {
        g_autoCommitted = false;

        // Update battery reading in the outgoing packet
        comms.status_to_send.charge = sensors.batteryMilliVolts();

        // Low battery overrides LED
        if (comms.status_to_send.charge < BATT_THRESH) {
            ledController.setState(LEDState::LOW_BATTERY);
        }

        if (!g_idle) comms.clearCommand();

        // Send heartbeat / acknowledgement
        bool sent;
        if (profileManager.writePtr() > profileManager.readPtr()) {
            ledController.setState(LEDState::IDLE_WITH_DATA);
            sent = comms.sendMessage(IDLE_W_DATA_ACK, 5000);
        } else {
            ledController.setState(LEDState::IDLE);
            sent = comms.sendMessage(IDLE_ACK, 5000);
        }

        if (sent) {
            g_idle = true;
            unsigned long t0 = millis();
            while (millis() - t0 < PERIOD_CONN_CHECK &&
                   comms.lastCommand().command == 0) {
                ledController.update();
                delay(10);
            }

            g_status = comms.lastCommand().command;
            if (g_status != CMD_IDLE) {
                g_idle = false;
                ledController.setState(LEDState::COMMUNICATION);
            }
        } else if (g_profileCount < PROFILE_MAX_COUNT && g_autoModeActive) {
            // No comms — activate autonomous mode
            Serial.println("No comms — entering auto mode");
            ledController.setState(LEDState::AUTO_MODE);
            g_status       = CMD_GO;
            g_autoCommitted = true;
        }
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_GO: // Execute a full depth profile
    {
        bool ack = g_autoCommitted ? true : comms.sendMessage(CMD1_ACK, 1000);

        if (ack) {
            Debug.println("Profile: starting");
            profileManager.resetEEPROM();

            Debug.println("Phase 1: PID descent to target depth");
            profileManager.measure(TARGET_DEPTH, STAT_TIME, TIMEOUT_PID_TIME);

            delay(500);

            Debug.println("Phase 2: Ascent to surface");
            profileManager.measure(TARGET_SURFACE, 3.0f, TIMEOUT_ASCENT);

            motor.stepper.disableOutputs();
            g_profileCount++;
            Debug.printf("Profile %d complete\n", g_profileCount);
        }

        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SEND_DATA: // Stream buffered EEPROM data to control station
    {
        Debug.println("Sending stored sensor data");
        profileManager.sendStoredData();
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_BALANCE: // Drive syringe to full extension then retraction
    {
        if (comms.sendMessage(CMD3_ACK, 1000)) {
            Debug.println("Balance: extending");
            motor.moveTo(MOTOR_MAX_STEPS - MOTOR_ENDSTOP_MARGIN);
            delay(5000);
            Debug.println("Balance: retracting");
            motor.moveTo(MOTOR_ENDSTOP_MARGIN);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_CLEAR_EEPROM:
    {
        if (comms.sendMessage(CMD4_ACK, 1000)) {
            profileManager.clearEEPROM();
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_AUTO_MODE: // Toggle autonomous profile execution
    {
        if (comms.sendMessage(CMD5_ACK, 1000)) {
            g_autoModeActive = !g_autoModeActive;
            Debug.printf("Auto mode: %s\n", g_autoModeActive ? "ON" : "OFF");
            ledController.setState(g_autoModeActive ? LEDState::AUTO_MODE : LEDState::IDLE);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SEND_PACKAGE: // Send a single live sensor snapshot
    {
        sensors.read();
        char packet[OUTPUT_LEN];
        snprintf(packet, OUTPUT_LEN,
                 "{\"company_number\":\"EX10\","
                 "\"pressure\":\"%.2f\","
                 "\"depth\":\"%.2f\","
                 "\"temperature\":\"%.2f\","
                 "\"mseconds\":\"%lu\"}",
                 sensors.pressure(),
                 sensors.depth(),
                 sensors.temperature(),
                 millis());

        comms.sendMessage(packet, 1000);
        Debug.println("Live snapshot sent");
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_OTA: // Start OTA update server (blocks for 5 min)
    {
        if (comms.sendMessage(CMD7_ACK, 1000)) {
            ledController.setState(LEDState::OTA_MODE);
            comms.runOTASession(); // Blocking — handles its own ESP-NOW teardown/restore
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_UPDATE_PID: // Update PID gains at runtime
    {
        if (comms.sendMessage(CMD8_ACK, 1000)) {
            pidController.Kp = comms.lastCommand().params[0];
            pidController.Ki = comms.lastCommand().params[1];
            pidController.Kd = comms.lastCommand().params[2];
            Debug.printf("PID updated: Kp=%.2f Ki=%.2f Kd=%.2f\n",
                         pidController.Kp, pidController.Ki, pidController.Kd);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_SET_SPEED: // Set test movement speed
    {
        if (comms.sendMessage(CMD9_ACK, 1000)) {
            uint32_t freq = comms.lastCommand().freq;
            g_testSpeed   = constrain(freq, 10u, 1200u);
            Debug.printf("Test speed set to %u steps/s\n", g_testSpeed);
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_TEST_STEPS: // Manual stepper test
    {
        if (comms.sendMessage(CMD10_ACK, 1000)) {
            long steps = comms.lastCommand().steps;
            Debug.printf("Test: moving %ld steps at %u steps/s\n", steps, g_testSpeed);

            motor.stepper.setMaxSpeed(g_testSpeed);
            motor.stepper.setAcceleration(g_testSpeed);
            motor.stepper.enableOutputs();
            motor.stepper.move(steps);

            while (motor.stepper.distanceToGo() != 0) {
                motor.stepper.run();
                ledController.update();
                yield();
            }

            // Restore normal operating speed
            motor.stepper.setMaxSpeed(MOTOR_MAX_SPEED);
            motor.stepper.setAcceleration(MOTOR_MAX_SPEED);
            motor.stepper.disableOutputs();
            Debug.printf("Test complete — pos %ld\n", motor.position());
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_DEBUG_MODE: // Toggle remote serial forwarding
    {
        if (comms.sendMessage(CMD11_ACK, 1000)) {
            debug_mode_active    = !debug_mode_active;
            g_debugModeActive    = debug_mode_active;
            Debug.printf("Debug mode: %s\n", debug_mode_active ? "ON" : "OFF");
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    case CMD_HOME: // Remote-triggered homing
    {
        if (comms.sendMessage(CMD12_ACK, 1000)) {
            Debug.println("Remote homing requested");
            if (!motor.home()) {
                Debug.println("ERROR: remote homing failed");
                ledController.setState(LEDState::ERROR);
            } else {
                Debug.println("Remote homing complete");
            }
        }
        g_status = CMD_IDLE;
        break;
    }

    // -----------------------------------------------------------------------
    default:
        Debug.printf("Unknown command: %d\n", g_status);
        g_status = CMD_IDLE;
        break;
    }

    delay(10);
}