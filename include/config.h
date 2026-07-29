#pragma once

#include <Arduino.h>

/*
 *******************************************************************************
 * config.h
 * Centralized configuration: pin definitions, tuning constants, network params.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo
 * Team PoliTOcean @ Politecnico di Torino
 *******************************************************************************
 */

// ---------------------------------------------------------------------------
// HARDWARE PIN DEFINITIONS
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_DIR           = 32;   // IO32: DIR  (dalla PCB)
constexpr uint8_t PIN_STEP          = 33;   // IO33: STEP (dalla PCB)
constexpr uint8_t PIN_EN            = 27;   // IO27: ENABLE ✓
constexpr uint8_t PIN_DRV_SLEEP     = 25;   // IO25: SLEEP (dalla PCB)
constexpr uint8_t PIN_DRV_RST       = 26;   // IO26: RESET (dalla PCB)

// Note: PIN_ENDSTOP_DOWN and PIN_ENDSTOP_UP removed — replaced with TOF sensor
constexpr uint8_t PIN_LED_R         = 19;   // Red LED channel
constexpr uint8_t PIN_LED_G         = 18;   // Green LED channel
constexpr uint8_t PIN_LED_B         = 5;    // Blue LED channel

// ---------------------------------------------------------------------------
// MOTOR CONSTANTS
// ---------------------------------------------------------------------------
constexpr uint16_t MOTOR_STEPS_PER_REV   = 200;   // Motor steps per revolution (motor specific 360/1.8)
constexpr uint8_t  MOTOR_MICROSTEP       = 1;     // Microstepping (4 = quarter step)
constexpr float    MOTOR_GEAR_RATIO      = 26.85124f; // Gearbox ratio (26 + 103/121)
constexpr float    MOTOR_SCREW_PITCH_MM  = 2.0f;  // Distance between adjacent thread crests (mm)
constexpr uint8_t  MOTOR_SCREW_STARTS    = 4;     // Number of thread starts/principles

constexpr float MOTOR_SCREW_LEAD_MM =
    MOTOR_SCREW_PITCH_MM * MOTOR_SCREW_STARTS;
constexpr float MOTOR_REVS_PER_MM =
    MOTOR_GEAR_RATIO / MOTOR_SCREW_LEAD_MM;
constexpr float MOTOR_STEPS_PER_MM =
    MOTOR_STEPS_PER_REV * MOTOR_MICROSTEP * MOTOR_REVS_PER_MM;

constexpr float    MOTOR_TRAVEL_MM       = 45.0f; // Normal commanded syringe travel (mm)
constexpr uint32_t MOTOR_MAX_STEPS       = static_cast<uint32_t>(MOTOR_TRAVEL_MM *
																 MOTOR_STEPS_PER_MM + 0.5f);
constexpr uint32_t MOTOR_MAX_SPEED       = 1400;  // Normal operating speed (steps/s)
constexpr uint32_t MOTOR_MAX_ACCELERATION = 1400; // Normal acceleration/deceleration (steps/s^2)
constexpr uint32_t MOTOR_HOMING_SPEED    = 1400;   // Homing speed (steps/s)
constexpr uint16_t MOTOR_ENDSTOP_MARGIN  = 10;    // Safety margin from endstops (steps)

// Geometria reale (verificata col balance, coerente con l'homing):
//   home (motor_pos=0)      = piastra lontana dal TOF, acqua spinta fuori → galleggia.
//   motor_pos negativo      = piastra verso il TOF, prende acqua → affonda.
// La direzione "verso il TOF / prende acqua" è NEGATIVA (vedi homeWithTof fase 1).
// Convenzione "logica" del PID/profile: u=0 → galleggia, u=1 → affonda.
// Quindi u cresce muovendosi in direzione negativa:
//   uToMotorPos(0.0f) = 0 (home), uToMotorPos(1.0f) = -(MAX - 2*margin).
inline long uToMotorPos(float u) {
    const long travel = (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
    return -(long)(u * (float)travel);
}
inline float motorPosToU(long position) {
    const long travel = (long)MOTOR_MAX_STEPS - 2L * (long)MOTOR_ENDSTOP_MARGIN;
    if (travel <= 0) return 0.0f;

    const float u = -(float)position / (float)travel;
    return constrain(u, 0.0f, 1.0f);
}
constexpr uint32_t MOTOR_HOMING_TIMEOUT  = 30000;  // Homing timeout (ms)
constexpr uint16_t MOTOR_HOMING_TOF_PERIOD_MS = 50; // TOF polling period during homing (ms)

// TOF (Time-of-Flight) sensor - VL53L7CX
constexpr uint8_t  TOF_XSHUT_PIN         = 16;    // LPn (sensor enable) pin
constexpr uint8_t  TOF_GPIO1_PIN         = 15;    // Optional INT pin, unused in polling mode
// VL53L7CX 4x4 zone mask: bit 0..15 maps directly to driver zone index
// results.distance_mm[i] / results.target_status[i]. 1 = enabled, 0 = ignored.
constexpr uint8_t  TOF_MATRIX_ZONE_COUNT = 16;
constexpr uint16_t TOF_ZONE_ENABLE_MASK  = 0x0660; // Central zones: 5, 6, 9, 10
constexpr float    TOF_DISTANCE_RAW_OFFSET_MM = 6.0f; // Raw distance is this much higher than real distance
constexpr float    TOF_HOMING_THRESHOLD     = 70.0f; // Homing stop distance: stop when TOF reads ABOVE this (siringa retratta = lontana dal TOF) (mm). Lo stop reale cade qualche mm sopra (polling 50ms + conferma + risoluzione TOF grezza): a 70 lo stop effettivo ~73-76 mm, con margine sotto TOF_SAFE_RANGE_MAX_MM=82.
constexpr float    TOF_HOMING_APPROACH_MM   = 50.0f; // Approach phase: move toward TOF until reading BELOW this, then start homing (mm)
// Letture TOF consecutive oltre soglia richieste prima di accettare il trigger
// di homing in ciascuna fase. Un singolo campione (frame recuperato, zona valida
// ma rumorosa, riflesso) non deve fermare la fase: serve conferma. Stesso pattern
// di TOF_SAFETY_STOP_SAMPLES.
constexpr uint8_t  TOF_HOMING_CONFIRM_SAMPLES = 2;
// Soglie tarate sulla finestra TOF reale misurata in piscina (distanza CORRETTA,
// cioè raw - TOF_DISTANCE_RAW_OFFSET_MM): pistone esteso ≈ 29 mm, retratto ≈ 79 mm
// (raw 85). Pendenza ≈ 1.1 mm TOF per mm motore.
// MIN: sotto il fondo corsa esteso si APRE IL TAPPO ed entra acqua. 32 mm lascia
// ~3 mm di margine sopra il 29 fisico: al raggiungimento si fa uno STOP PULITO
// (clamp, niente emergency) per fermare il pistone PRIMA del tappo senza abortire
// la missione (vedi MotionController::tofGuard / TofGuard::ExtendLimit).
constexpr float    TOF_SAFE_RANGE_MIN_MM    = 32.0f; // Safety range lower bound: siringa estesa, vicina al TOF — oltre = tappo aperto (mm)
// MAX: massimo gestibile raw 85 → 79 mm corretta (offset 6 mm). 82 mm = ~3 mm
// sopra il 79 fisico, copre il rumore post-homing. Verso il retratto il motore
// può sforare ancora parecchio: oltre 82 è un'ANOMALIA (passi persi, verso
// sbagliato) → emergency stop (TofGuard::Emergency).
constexpr float    TOF_SAFE_RANGE_MAX_MM    = 82.0f; // Safety range upper bound: siringa retratta, lontana dal TOF — oltre = anomalia (mm)
// Soglia (numero di letture TOF consecutive fuori range) prima di scatenare un
// emergency stop durante un movimento. Un singolo campione fuori soglia in
// acqua (bolle, riflessi, torbidità) non deve fermare la missione: serve una
// conferma. Stesso pattern del pressure-stop del balance.
constexpr uint8_t  TOF_SAFETY_STOP_SAMPLES  = 3;
// Range PID utile: limitiamo l'output u del PID a [MIN, MAX] (anziché [0,1])
// così la siringa non raggiunge mai gli estremi meccanici che coincidono con
// le soglie TOF di sicurezza (32/82 mm), lasciando margine contro passi persi
// e rumore. Con lo zero a TOF≈75 mm e pendenza ~1.1, a u=0.92 il TOF ≈ 40 mm,
// sopra MIN=32 con margine; a u=1.0 ≈ 34 mm (ecco perché MAX resta < 1.0).
// PID_U_MIN=0 così il PID può svuotare completamente la siringa per risalire
// (un MIN>0 lasciava il float troppo galleggiante e nascondeva la dinamica
// reale agli u bassi). Il MAX resta sotto 1.0 per margine verso la soglia TOF
// in piena estensione.
// PID_U_MIN tarato in MIL (model/) a 0.20: NON svuotare del tutto la siringa in
// risalita limita la spinta verso l'alto e riduce l'overshoot che rompeva la
// superficie. Era 0.0 (svuotamento completo → risalita troppo veloce, sfondava).
constexpr float    PID_U_MIN                = 0.20f;
constexpr float    PID_U_MAX                = 0.92f;

// ---------------------------------------------------------------------------
// BALANCE / PURGE CONTROL
// ---------------------------------------------------------------------------
constexpr float    BALANCE_STOP_PRESSURE_DELTA_KPA = 5.0f; // Stop balance when Bar02 rises above baseline by this amount
constexpr uint8_t  BALANCE_STOP_PRESSURE_SAMPLES = 3; // Consecutive above-threshold samples required
constexpr uint16_t BALANCE_PRESSURE_SAMPLE_PERIOD_MS = 50; // Bar02 polling period during balance

// ---------------------------------------------------------------------------
// TIMING CONSTANTS  (ms unless noted)
// ---------------------------------------------------------------------------
constexpr uint16_t PERIOD_MEASUREMENT   = 100;   // Between depth readings
constexpr uint16_t PERIOD_CONN_CHECK    = 500;   // Between idle acknowledgements

constexpr uint16_t PERIOD_EEPROM_WRITE   = 5000; // Between EEPROM writes / hold checks
constexpr uint16_t PROFILE_LOG_PERIOD_MS = 1000; // Between flash profile writes
constexpr uint16_t DATA_PACKET_PERIOD_MS = 5000; // Packet cadence shown to judges

// ---------------------------------------------------------------------------
// PID TUNING (output normalizzato in [0,1] = frazione di corsa siringa)
// ---------------------------------------------------------------------------
// Defaults runtime per PID_CONFIG_SET/GET. Espressi in "frazione di corsa per
// metro di errore", portabili tra siringhe — se cambia MOTOR_MAX_STEPS, i
// guadagni restano validi.
constexpr uint16_t PID_PERIOD_DEFAULT_MS  = 50;    // Default tick PID (ms)
// Tuning iterato sui test in piscina:
//  - Kp=0.17 (originale): troppo debole, il float non si muoveva (u≈0.03).
//  - Kp=2.0 Kd=0.13: il float si muoveva ma OSCILLAVA (±15cm, pompaggio) —
//    Kp troppo alto e Kd insufficiente per un sistema lento come il float.
//  - Kp=1.0 Kd=0.5: smorzato ma si "sedeva" in superficie (ripresa debole).
//  - Kp=1.7 Ki=0.1 Kd=0.3: converge sul target con oscillazione finale ±1cm.
//  - Kp=1.0 Kd=2.0 (dump 2026-06-19): in discesa il float sfonda il target di
//    ~0.7 m (picco 2.22 m con target 1.5). Dal log ad alta risoluzione il
//    problema NON è u saturo a fondo corsa (u sale gradualmente, max ~0.93, poi
//    scende da solo): è INERZIA idrodinamica — u è già a 0 ma il float continua
//    ad affondare per il momento accumulato. Serve frenare PRIMA, quindi più Kd.
//  - Kp=0.5 Kd=3.0 (attuale): meno spinta proporzionale in discesa + freno
//    derivativo più deciso, per anticipare l'arresto e contenere l'overshoot.
//    Affinare ancora a runtime con PID_CONFIG_SET se serve.
constexpr float    PID_KP_DEFAULT         = 0.5f;  // frazione_corsa / m
constexpr float    PID_KI_DEFAULT         = 0.1f;  // frazione_corsa / (m·s)
constexpr float    PID_KD_DEFAULT         = 3.0f;  // frazione_corsa / (m/s)
constexpr float    PID_INTEGRAL_LIMIT     = 5.0f;  // m·s (bound conservativo)
constexpr float    PID_ALPHA_D_DEFAULT    = 0.25f; // LPF IIR coeff per derivata
constexpr float    PID_U_NEUTRAL          = 0.011f;// kick-start offset (~500/47100)
constexpr float    PID_MIN_RETARGET_FRAC  = 0.001f;// dead-band ri-comando (frazione corsa)
// Pre-posizionamento siringa all'inizio della discesa PID (kick-start): u alto
// per avviare l'affondamento. Era 0.979 (siringa quasi piena) ma faceva tirare
// il float dritto fino al fondo prima che il PID frenasse (overshoot ~26cm in
// vasca). Ridotto a 0.30, poi a 0.15: con 0.30 + spinta del PID (Kp*error) la
// discesa partiva ancora troppo veloce e si sfondava il target di oltre 1 m.
constexpr float    PID_DESCENT_KICK_U     = 0.15f;

// ---------------------------------------------------------------------------
// FLOAT PHYSICAL / MISSION CONSTANTS
// ---------------------------------------------------------------------------
constexpr float    FLOAT_LENGTH         = 0.49f;  // Bottom-to-sensor height (m): corpo 48 cm + barometro 1 cm sopra il tappo
constexpr float    SENSOR_TO_BOTTOM_M   = FLOAT_LENGTH; // Pressure sensor to bottom reference
// Vertical offset of the physical top of the float relative to the barometer,
// signed so that topDepth = sensorDepth - SENSOR_TO_TOP_M (see sensors.cpp).
// Positive = top is BELOW the sensor; negative = top is ABOVE the sensor.
// Misurato in hardware: il Bar02 sporge 1 cm SOPRA la cima del float, quindi la
// cima è 1 cm più in profondità del sensore → offset NEGATIVO (-0.01 m).
constexpr float    FLOAT_TOP_TO_SENSOR_M = -0.01f;
constexpr float    SENSOR_TO_TOP_M       = FLOAT_TOP_TO_SENSOR_M;
// Operational target: how deep the *top* of the float should sit below the
// water surface when the float is "floating"/resting. Runtime-tunable via
// CMD_SET_SURFACE_OFFSET / USB SURFACE_OFFSET command — this is the default.
// 0.15 m tiene l'antenna (~12 cm sopra il tappo) sommersa di qualche cm a riposo,
// così il float non rompe la superficie (penalità -5 punti) in attesa del recupero.
constexpr float    SURFACE_TARGET_OFFSET_M = 0.15f;
constexpr float    DEPTH_EPSILON       = 0.01f;  // "Stationary" tolerance (m)

constexpr uint8_t  PROFILE_MAX_COUNT   = 2;      // Profiles before auto-stop
constexpr float    DEPTH_MAX_ERROR     = 0.33f;  // MATE depth tolerance (m)
constexpr float    TARGET_DEPTH        = 2.50f;  // Deep hold: bottom reference (m)
// Tarato in MIL (model/) a 0.65 m: tenere la cima piu' in basso (dentro la
// tolleranza MATE 0.40 +/- 0.33 = [0.07, 0.73]) lascia margine all'overshoot di
// risalita cosi' il float non rompe la superficie (penalita' -5). Era 0.40.
constexpr float    TARGET_SHALLOW_TOP_DEPTH = 0.65f; // Shallow hold: top reference (m)
constexpr float    STAT_TIME           = 30.0f;  // MATE hold time at target (s)
constexpr float    TIMEOUT_PID_TIME    = 180.0f; // Max PID phase time (s)
constexpr float    TIMEOUT_ASCENT      = 120.0f; // Max ascent + shallow hold time (s)
// Sosta finale sotto il pelo (top a SURFACE_TARGET_OFFSET_M) dopo i due profili:
// tiene attivo il PID finché il float non viene recuperato (o scade la finestra).
constexpr float    REST_WINDOW_S       = 120.0f; // Active surface-rest hold window (s)

constexpr float    TARGET_SHALLOW_BOTTOM_DEPTH =
    TARGET_SHALLOW_TOP_DEPTH + SENSOR_TO_BOTTOM_M + SENSOR_TO_TOP_M;

// Sentinel values passed to measure() as targetDepth
constexpr float    TARGET_SURFACE      = FLOAT_LENGTH; // Legacy surface sentinel
constexpr int8_t   TARGET_BOTTOM       = -1;           // Descend to pool floor

// ---------------------------------------------------------------------------
// SENSOR CONSTANTS
// ---------------------------------------------------------------------------
constexpr float    WATER_DENSITY_FRESH = 997.0f;   // kg/m³
constexpr float    GRAVITY             = 9.80665f;

// ---------------------------------------------------------------------------
// FLOAT SIMULATOR (HIL da banco: motore reale, barometro SIMULATO)
// ---------------------------------------------------------------------------
// Con SIM attivo le letture Bar02 sono sostituite da un modello fisico 2° ordine:
// la siringa (u = motorPosToU(motor.position())) genera spinta netta, la quota
// del SENSORE viene integrata con inerzia idrodinamica e drag quadratico:
//   a = SIM_ACCEL_GAIN*(u - SIM_U_NEUTRAL) - SIM_DRAG_QUAD*v*|v|
//   v += a*dt ;  z += v*dt
// Convenzione coerente col resto del firmware: u > neutral => affonda (z cresce).
// Tutti i parametri sono ritarabili a runtime con SIM_CONFIG senza riflashare.
// Default tarati con la simulazione offline del loop chiuso (PID + motore lento
// ~2 mm/s + modello): a uNeutral=0.35 la discesa converge ~2.5 m con overshoot
// ~0.28 m (realistico, sink-biased come il float vero) e l'effetto di Kd è ben
// visibile. RITARALI sul TUO float con SIM_CONFIG: uNeutral = u a cui la siringa
// regge la quota senza muoversi (lo vedi col valore di u a regime in PID_HOLD).
constexpr float    SIM_U_NEUTRAL   = 0.35f; // u di galleggiamento neutro (net buoyancy = 0)
constexpr float    SIM_ACCEL_GAIN  = 0.05f; // accelerazione [m/s^2] per unità di (u - neutral)
constexpr float    SIM_DRAG_QUAD   = 1.50f; // coeff. drag quadratico [1/m]
constexpr float    SIM_POOL_DEPTH  = 3.00f; // profondità vasca simulata [m] (vasca NRC ~3 m)
constexpr float    SIM_MAX_DT_S    = 0.20f; // clamp dt integrazione per stabilità [s]

// ---------------------------------------------------------------------------
// NETWORK / OTA
// ---------------------------------------------------------------------------
constexpr char     WIFI_SSID[]         = "PIPO";
constexpr char     WIFI_PASSWORD[]     = "politocean";

// constexpr uint8_t  MAC_ESPB[6]        = {0xEC, 0xE3, 0x34, 0xCE, 0x59, 0x1C};
constexpr uint8_t MAC_ESPB[6] = {0x88, 0x57, 0x21, 0x84, 0x8C, 0xE8};
constexpr uint8_t MAC_ESPA[6] = {0x88, 0x57, 0x21, 0x84, 0x7E, 0xCC};
constexpr uint8_t ESPNOW_CHANNEL = 1;

// ---------------------------------------------------------------------------
// EEPROM / DATA
// ---------------------------------------------------------------------------
constexpr char     COMPANY_NUMBER[]     = "EX12";
constexpr char     FLASH_LOG_PATH[]     = "/mission/current_profile.csv";

// EEPROM_SIZE and sensor_data struct come from float_common.h
