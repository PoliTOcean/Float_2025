#ifndef FLOAT_CORE_TYPES_H
#define FLOAT_CORE_TYPES_H

/*
 *******************************************************************************
 * float_core_types.h
 * CONTRATTO D'INTERFACCIA del control core (Model-Based Software Design).
 *
 * Questo header e' l'unica "single source of truth" del confine modello <-> HAL:
 *   - definisce i segnali di ingresso/uscita del core (CoreInputs/CoreOutputs)
 *   - e' C PURO, zero dipendenze da Arduino/ESP-IDF: compila identico nel
 *     firmware, nell'harness nativo (MIL/SIL su PC) e nel codice generato da
 *     Embedded Coder.
 *
 * Il core NON tocca mai hardware: niente I2C, ESP-NOW, LittleFS, millis(),
 * delay(), Serial. Riceve i sensori gia' letti + dt, restituisce comandi ed
 * EVENTI discreti (ack/log) che l'HAL traduce in azioni reali.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Eventi/comandi cui il core reagisce (sottoinsieme mission-critical).      */
/* La decodifica GUI/ESP-NOW/seriale resta nell'HAL: qui arriva solo l'evento */
/* latched, tipicamente NONE.                                                 */
/* ------------------------------------------------------------------------- */
typedef enum {
    CORE_EVT_NONE = 0,
    CORE_EVT_GO,     /* avvia la missione (profili + sosta finale) */
    CORE_EVT_HOME,   /* esegue l'homing TOF */
    CORE_EVT_STOP    /* emergency stop remoto */
} CoreEvent;

/* Stato di supervisione del core. */
typedef enum {
    CORE_MODE_INIT = 0,
    CORE_MODE_HOMING,
    CORE_MODE_IDLE,
    CORE_MODE_MISSION,
    CORE_MODE_SAFE_STOP
} CoreMode;

/* Sotto-fase della missione (tag per telemetria/log + scelta del setpoint). */
typedef enum {
    CORE_PHASE_NONE = 0,
    CORE_PHASE_PRE_DESCENT,   /* pacchetto di deployment pre-discesa */
    CORE_PHASE_DESCEND_HOLD,  /* PID verso il target di fondo + hold */
    CORE_PHASE_ASCEND_HOLD,   /* PID verso il target di risalita + hold */
    CORE_PHASE_SURFACE_REST,  /* sosta finale sotto pelo (non esce all'hold) */
    CORE_PHASE_DONE
} CorePhase;

/* Sotto-fase dell'homing TOF (approach -> retract -> backoff). */
typedef enum {
    CORE_HOME_APPROACH = 0,
    CORE_HOME_RETRACT,
    CORE_HOME_BACKOFF,
    CORE_HOME_DONE,
    CORE_HOME_FAILED
} CoreHomePhase;

/* Richiesta LED (mirror logico di LEDState, tenuto indipendente/portabile). */
typedef enum {
    CORE_LED_OFF = 0,
    CORE_LED_INIT,
    CORE_LED_IDLE,
    CORE_LED_IDLE_WITH_DATA,
    CORE_LED_ERROR,
    CORE_LED_PROFILE,
    CORE_LED_HOMING,
    CORE_LED_PID_CONTROL,
    CORE_LED_COMMUNICATION
} CoreLed;

/* Modo del comando motore emesso dal core. */
typedef enum {
    CORE_MOTOR_HOLD = 0, /* mantieni posizione (l'HAL puo' disabilitare uscite) */
    CORE_MOTOR_GOTO,     /* vai a motor_target (assoluto, in step) */
    CORE_MOTOR_JOG,      /* homing: moto continuo nel verso di sign(motor_target) */
    CORE_MOTOR_STOP      /* ferma ora + disabilita uscite */
} CoreMotorMode;

/* Evento discreto verso la CS (comms). L'HAL sceglie la stringa ACK reale. */
typedef enum {
    CORE_ACK_NONE = 0,
    CORE_ACK_GO_RECVD,
    CORE_ACK_HOME_RECVD,
    CORE_ACK_STOP_RECVD,
    CORE_ACK_IDLE,        /* heartbeat: FLOAT_IDLE */
    CORE_ACK_IDLE_W_DATA  /* heartbeat: FLOAT_IDLE_W_DATA */
} CoreAckEvent;

/* Tipo di record che il core chiede all'HAL di scrivere sul flash log. */
typedef enum {
    CORE_LOG_NONE = 0,
    CORE_LOG_DEPLOY,       /* pacchetto pre-discesa */
    CORE_LOG_PROFILE,      /* campione periodico di profilo */
    CORE_LOG_PHASE_START,  /* inizio fase con parametri effettivi */
    CORE_LOG_EXIT_HOLD_OK, /* hold completato con successo */
    CORE_LOG_EXIT_TIMEOUT, /* fase troncata dal timeout */
    CORE_LOG_EXIT_REMOTE,  /* uscita per remote stop */
    CORE_LOG_EMERGENCY,    /* emergency stop (TOF/anomalia) */
    CORE_LOG_HOMING        /* evento di homing */
} CoreLogKind;

/* ------------------------------------------------------------------------- */
/* Configurazione: parametri di missione, PID, geometria e safety TOF.        */
/* Popolata dall'HAL da config.h/NVS. Il core la tratta come read-only.       */
/* I target sono gia' convertiti in riferimento FONDO (come il sensore).      */
/* ------------------------------------------------------------------------- */
typedef struct {
    /* --- missione --- */
    uint8_t profile_count;
    float   descent_target_m;        /* target discesa (rif. fondo) */
    float   ascent_target_bottom_m;  /* target risalita (rif. fondo, gia' convertito) */
    float   rest_target_bottom_m;    /* target sosta finale (rif. fondo) */
    float   depth_tolerance_m;
    float   hold_time_s;
    float   descent_timeout_s;
    float   ascent_timeout_s;
    float   rest_window_s;

    /* --- PID (output normalizzato u in [0,1], guadagni per metro d'errore) --- */
    float   kp, ki, kd;
    float   alpha_d;
    float   integral_limit;
    float   u_neutral;         /* offset kick-start sommato all'output */
    float   min_retarget_frac; /* dead-band ri-comando (frazione di corsa) */
    float   pid_period_s;      /* passo fisso del modello (tempo di campionamento) */
    float   descent_kick_u;    /* pre-posizionamento a inizio discesa */
    float   u_min, u_max;      /* clamp dell'output PID (margine dai fine corsa) */

    /* --- geometria motore --- */
    long    usable_steps;         /* MOTOR_MAX_STEPS - 2*margine */
    long    endstop_margin_steps;
    long    max_steps;

    /* --- safety / homing TOF --- */
    float   tof_safe_min_mm;
    float   tof_safe_max_mm;
    uint8_t tof_safety_stop_samples;
    float   tof_homing_approach_mm;
    float   tof_homing_threshold_mm;
    uint8_t tof_homing_confirm_samples;
    float   homing_timeout_s;
} CoreConfig;

/* ------------------------------------------------------------------------- */
/* Ingressi: HAL -> core, campionati a ogni tick a passo fisso pid_period_s.  */
/* ------------------------------------------------------------------------- */
typedef struct {
    float    dt_s;                 /* passo dall'ultimo step (== pid_period_s) */
    float    depth_m;              /* profondita' fusa (rif. fondo), reale o SIM */
    float    pressure_kpa;
    float    tof_mm;
    bool     tof_valid;            /* true se tof_mm e' un campione fresco valido */
    long     motor_pos;            /* posizione corrente (step) */
    bool     motor_position_known; /* homing eseguito almeno una volta */
    bool     motor_busy;           /* distanceToGo() != 0 */
    uint16_t battery_mv;
    bool     has_stored_data;      /* flash log ha dati non ancora inviati */
    CoreEvent event;               /* comando latched (NONE nella maggioranza dei tick) */
    const CoreConfig* cfg;
} CoreInputs;

/* ------------------------------------------------------------------------- */
/* Uscite: core -> HAL. Comandi motore/LED + eventi discreti da attuare.      */
/* ------------------------------------------------------------------------- */
typedef struct {
    CoreMotorMode motor_mode;
    long          motor_target;   /* GOTO: posizione assoluta; JOG: verso = segno */
    bool          motor_enable;
    bool          motor_zero_here;/* HAL: azzera la posizione qui (fine homing) */

    CoreLed       led;
    CorePhase     phase;          /* fase corrente (per log/telemetria) */

    CoreAckEvent  ack_event;      /* !=NONE se c'e' un ack/heartbeat da inviare */
    bool          log_now;        /* true se l'HAL deve scrivere un record */
    CoreLogKind   log_kind;

    bool          mission_active;
    uint8_t       completed_profiles;
} CoreOutputs;

/* ------------------------------------------------------------------------- */
/* Stato persistente del core tra un tick e l'altro (il "memory" del modello).*/
/* Nessun timestamp assoluto: i tempi sono accumulati con dt (portabile).     */
/* ------------------------------------------------------------------------- */
typedef struct {
    CoreMode      mode;
    CorePhase     phase;
    CoreHomePhase home_phase;
    uint8_t       completed_profiles;

    /* timer accumulati (s) */
    float phase_elapsed_s;
    float stable_s;        /* tempo continuo entro tolleranza */
    float home_elapsed_s;
    float log_elapsed_s;   /* cadenza scrittura record di profilo */

    /* stato PID */
    float pid_integral;
    float pid_dfilt;
    float pid_last_depth;
    bool  pid_has_last;

    /* memoria comando motore */
    long  last_commanded_target;
    bool  at_extension_limit;

    /* contatori di conferma TOF */
    uint8_t tof_oor_count;      /* out-of-range consecutivi (safety) */
    uint8_t home_confirm_count; /* conferme soglia homing */

    /* flag di controllo */
    bool  emergency;
    bool  home_after_stop;   /* SAFE_STOP -> auto-recovery via HOMING */
    bool  kick_done;         /* pre-posizionamento discesa gia' comandato */
    bool  entered;           /* helper: entry-action della fase corrente eseguita */
} CoreState;

#ifdef __cplusplus
}
#endif

#endif /* FLOAT_CORE_TYPES_H */
