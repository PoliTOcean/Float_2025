/*
 *******************************************************************************
 * plant.c  (host-only, MIL)  — vedi plant.h
 *******************************************************************************
 */

#include "plant.h"
#include "float_core.h"

static long clamp_l(long v, long lo, long hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void plant_init(Plant* p, const CoreConfig* cfg, long initial_pos) {
    p->motor_pos     = initial_pos;
    p->motor_target  = initial_pos;
    p->motor_mode    = CORE_MOTOR_HOLD;
    p->motor_jog_dir = 0;
    p->position_known = false;
    /* ~2 mm/s: MOTOR_MAX_SPEED (1400 step/s) * dt */
    p->max_step_per_tick = 70; /* 1400 * 0.05 */

    p->z = 0.0;
    p->v = 0.0;
    p->dynamics_active = false;

    p->u_neutral  = 0.35;  /* SIM_U_NEUTRAL */
    p->accel_gain = 0.05;  /* SIM_ACCEL_GAIN */
    p->drag_quad  = 1.50;  /* SIM_DRAG_QUAD */
    p->pool_depth = 3.00;  /* SIM_POOL_DEPTH */

    p->usable_steps = cfg->usable_steps;
    p->tof_home_mm  = 75.0; /* pos=0 */
    p->tof_ext_mm   = 34.0; /* u=1 */
}

double plant_tof_mm(const Plant* p) {
    /* frazione di estensione: 0 = home, 1 = piena estensione (pos = -usable) */
    const double frac = -(double)p->motor_pos / (double)p->usable_steps;
    return p->tof_home_mm - frac * (p->tof_home_mm - p->tof_ext_mm);
}

double plant_pressure_kpa(const Plant* p) {
    /* pressione idrostatica di gauge: rho*g*z */
    return 997.0 * 9.80665 * (p->z) / 1000.0;
}

bool plant_motor_busy(const Plant* p) {
    if (p->motor_mode == CORE_MOTOR_JOG) return true;
    /* GOTO/HOLD: "busy" finche' non ha raggiunto il target agganciato */
    if (p->motor_mode == CORE_MOTOR_STOP) return false;
    return p->motor_pos != p->motor_target;
}

/* Contratto HAL dell'attuatore (come FastAccelStepper.startMoveTo):
 *   GOTO -> AGGANCIA un nuovo target; l'attuatore lo insegue.
 *   HOLD -> nessun nuovo comando: continua a inseguire il target agganciato.
 *   STOP -> ferma qui (target = posizione attuale).
 *   JOG  -> moto continuo nel verso indicato. */
static void move_toward_target(Plant* p) {
    long d = p->motor_target - p->motor_pos;
    if (d >  p->max_step_per_tick) d =  p->max_step_per_tick;
    if (d < -p->max_step_per_tick) d = -p->max_step_per_tick;
    p->motor_pos += d;
}

static void actuator_step(Plant* p, const CoreOutputs* out) {
    p->motor_mode = out->motor_mode;

    if (out->motor_zero_here) {
        p->motor_pos = 0;
        p->motor_target = 0;
        p->position_known = true;
    }

    switch (out->motor_mode) {
    case CORE_MOTOR_GOTO:
        p->motor_target = out->motor_target; /* aggancia il nuovo target */
        move_toward_target(p);
        break;
    case CORE_MOTOR_HOLD:
        move_toward_target(p);               /* continua verso il target agganciato */
        break;
    case CORE_MOTOR_JOG:
        p->motor_jog_dir = (out->motor_target < 0) ? -1 : +1;
        p->motor_pos += p->motor_jog_dir * p->max_step_per_tick;
        p->motor_pos = clamp_l(p->motor_pos, -2 * p->usable_steps, p->usable_steps);
        p->motor_target = p->motor_pos;      /* fermo qui se poi arriva HOLD */
        break;
    case CORE_MOTOR_STOP:
    default:
        p->motor_target = p->motor_pos;      /* ferma qui */
        break;
    }
}

static void dynamics_step(Plant* p, double dt) {
    if (!p->dynamics_active) return;

    /* u dalla posizione reale del motore (come motorPosToU nel firmware) */
    double u = -(double)p->motor_pos / (double)p->usable_steps;
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    const double a = p->accel_gain * (u - p->u_neutral) - p->drag_quad * p->v * (p->v < 0 ? -p->v : p->v);
    p->v += a * dt;
    p->z += p->v * dt;

    if (p->z < 0.0) { p->z = 0.0; if (p->v < 0.0) p->v = 0.0; }
    if (p->z > p->pool_depth) { p->z = p->pool_depth; if (p->v > 0.0) p->v = 0.0; }
}

void plant_apply_and_step(Plant* p, const CoreOutputs* out, double dt) {
    actuator_step(p, out);
    dynamics_step(p, dt);
}
