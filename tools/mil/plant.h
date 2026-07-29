#ifndef FLOAT_MIL_PLANT_H
#define FLOAT_MIL_PLANT_H

/*
 *******************************************************************************
 * plant.h  (host-only, MIL)
 * Modello del PLANT del float: attuatore motore (rate-limited ~2 mm/s),
 * dinamica di profondita' del 2o ordine (spinta siringa + drag quadratico),
 * e modelli sensore TOF e pressione. E' la controparte "fisica" del control
 * core: in Simulink diventera' float_plant.slx.
 *
 * Convenzione coerente col firmware: u>uNeutral => affonda (z cresce).
 *******************************************************************************
 */

#include "float_core_types.h"

typedef struct {
    /* --- attuatore --- */
    long   motor_pos;       /* step correnti */
    long   motor_target;    /* target del GOTO / verso del JOG */
    int    motor_mode;      /* CoreMotorMode dell'ultimo comando */
    int    motor_jog_dir;   /* -1/0/+1 durante il JOG */
    bool   position_known;
    long   max_step_per_tick;

    /* --- dinamica di profondita' (2o ordine) --- */
    double z;               /* quota (rif. fondo), m */
    double v;               /* velocita' verticale, m/s (+ = affonda) */
    bool   dynamics_active; /* integra solo durante la missione */

    /* --- parametri fisici (come i SIM_* di config.h) --- */
    double u_neutral;
    double accel_gain;
    double drag_quad;
    double pool_depth;

    /* --- geometria per i modelli sensore --- */
    long   usable_steps;
    double tof_home_mm;     /* TOF a pos=0 (home) */
    double tof_ext_mm;      /* TOF a u=1 (piena estensione) */
} Plant;

/* Inizializza il plant (posizione iniziale non-homed per esercitare l'homing). */
void plant_init(Plant* p, const CoreConfig* cfg, long initial_pos);

/* Applica un comando del core all'attuatore e avanza la dinamica di dt secondi. */
void plant_apply_and_step(Plant* p, const CoreOutputs* out, double dt);

/* Modelli sensore. */
double plant_tof_mm(const Plant* p);
double plant_pressure_kpa(const Plant* p);
bool   plant_motor_busy(const Plant* p);

#endif /* FLOAT_MIL_PLANT_H */
