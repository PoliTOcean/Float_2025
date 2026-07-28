/*
 *******************************************************************************
 * mil_main.c  (host-only, MIL)
 * Anello chiuso NATIVO (pre-Simulink): control core (float_core.c) + plant
 * (plant.c) su PC, senza hardware ne' MATLAB. Esegue:
 *   boot -> homing -> IDLE -> GO -> 2 profili (discesa/hold/risalita/hold)
 *   -> sosta finale -> IDLE.
 * Stampa il CSV della missione e verifica i requisiti MATE.
 *
 * Scopo: feedback immediato sul comportamento del core e "specifica eseguibile"
 * che il modello Simulink/Stateflow dovra' riprodurre (equivalenza MIL).
 *
 * Build:  make -C tools/mil     (oppure vedi Makefile)
 *******************************************************************************
 */

#include "float_core.h"
#include "plant.h"
#include <stdio.h>
#include <math.h>

/* --- Costanti geometriche/di missione (mirror di include/config.h) --------- */
#define STEPS_PER_MM   (200.0 * 1.0 * (26.85124 / 8.0))     /* 671.28 */
#define TRAVEL_MM      45.0
#define ENDSTOP_MARGIN 10L

/* Riferimenti FONDO (come li calcola ProfileManager):
 *   SENSOR_TO_BOTTOM_M = 0.49 ; SENSOR_TO_TOP_M = -0.01
 *   ascent_bottom = ascent_top(0.40) + 0.49 + (-0.01) = 0.88
 *   rest_bottom   = rest_top(0.15)  + 0.49 + (-0.01) = 0.63 */
#define ASCENT_TOP     0.40
#define REST_TOP       0.15
#define SENSOR_TO_BOTTOM 0.49
#define SENSOR_TO_TOP  (-0.01)

static CoreConfig make_config(void) {
    CoreConfig c;
    const long max_steps = (long)(TRAVEL_MM * STEPS_PER_MM + 0.5);

    c.profile_count          = 2;
    c.descent_target_m       = 2.50;
    c.ascent_target_bottom_m = ASCENT_TOP + SENSOR_TO_BOTTOM + SENSOR_TO_TOP; /* 0.88 */
    c.rest_target_bottom_m   = REST_TOP  + SENSOR_TO_BOTTOM + SENSOR_TO_TOP;  /* 0.63 */
    c.depth_tolerance_m      = 0.33;
    c.hold_time_s            = 30.0;
    c.descent_timeout_s      = 180.0;
    c.ascent_timeout_s       = 120.0;
    c.rest_window_s          = 120.0;

    c.kp = 0.5f;  c.ki = 0.1f;  c.kd = 3.0f;
    c.alpha_d = 0.25f;
    c.integral_limit = 5.0f;
    c.u_neutral = 0.011f;
    c.min_retarget_frac = 0.001f;
    c.pid_period_s = 0.05f;
    c.descent_kick_u = 0.15f;
    c.u_min = 0.0f;  c.u_max = 0.92f;

    c.usable_steps        = max_steps - 2L * ENDSTOP_MARGIN;
    c.endstop_margin_steps = ENDSTOP_MARGIN;
    c.max_steps           = max_steps;

    c.tof_safe_min_mm            = 32.0f;
    c.tof_safe_max_mm            = 82.0f;
    c.tof_safety_stop_samples    = 3;
    c.tof_homing_approach_mm     = 50.0f;
    c.tof_homing_threshold_mm    = 70.0f;
    c.tof_homing_confirm_samples = 2;
    c.homing_timeout_s           = 30.0f;
    return c;
}

static const char* phase_name(CorePhase p) {
    switch (p) {
    case CORE_PHASE_PRE_DESCENT:  return "pre_descent";
    case CORE_PHASE_DESCEND_HOLD: return "descend_hold";
    case CORE_PHASE_ASCEND_HOLD:  return "ascend_hold";
    case CORE_PHASE_SURFACE_REST: return "surface_rest";
    case CORE_PHASE_DONE:         return "done";
    default:                      return "-";
    }
}

static const char* mode_name(CoreMode m) {
    switch (m) {
    case CORE_MODE_INIT:      return "INIT";
    case CORE_MODE_HOMING:    return "HOMING";
    case CORE_MODE_IDLE:      return "IDLE";
    case CORE_MODE_MISSION:   return "MISSION";
    case CORE_MODE_SAFE_STOP: return "SAFE_STOP";
    default:                  return "?";
    }
}

static double target_for_phase(const CoreConfig* c, CorePhase p) {
    switch (p) {
    case CORE_PHASE_DESCEND_HOLD: return c->descent_target_m;
    case CORE_PHASE_ASCEND_HOLD:  return c->ascent_target_bottom_m;
    case CORE_PHASE_SURFACE_REST: return c->rest_target_bottom_m;
    default:                      return 0.0;
    }
}

int main(void) {
    const CoreConfig cfg = make_config();
    const double dt = 0.05;

    CoreState  st;
    CoreOutputs out;
    Plant p;

    float_core_init(&st);
    plant_init(&p, &cfg, /*initial_pos=*/3000); /* non-homed: forza l'homing */

    int homed = 0, go_sent = 0, mission_ran = 0;

    /* metriche per la verifica MATE (per fase) */
    double deep_closest = 1e9, shallow_closest = 1e9;
    double max_depth = 0.0, min_top_depth = 1e9;

    printf("t_s,mode,phase,depth_m,target_m,u,motor_pos,tof_mm,stable_s\n");

    const long max_ticks = (long)(1000.0 / dt); /* limite di sicurezza: 1000 s */
    for (long k = 0; k < max_ticks; k++) {
        const double t = k * dt;

        CoreInputs in;
        in.dt_s = (float)dt;
        in.depth_m = (float)p.z;
        in.pressure_kpa = (float)plant_pressure_kpa(&p);
        in.tof_mm = (float)plant_tof_mm(&p);
        in.tof_valid = true;
        in.motor_pos = p.motor_pos;
        in.motor_position_known = p.position_known;
        in.motor_busy = plant_motor_busy(&p);
        in.battery_mv = 12500;
        in.has_stored_data = (st.completed_profiles > 0);
        in.cfg = &cfg;

        /* iniezione comando: appena si raggiunge IDLE dopo l'homing, invia GO */
        in.event = CORE_EVT_NONE;
        if (st.mode == CORE_MODE_IDLE && !homed) {
            homed = 1;
        }
        if (st.mode == CORE_MODE_IDLE && homed && !go_sent) {
            in.event = CORE_EVT_GO;
            go_sent = 1;
            p.z = 0.0; p.v = 0.0; p.dynamics_active = true; /* missione parte dal pelo */
        }

        float_core_step(&in, &st, &out);
        plant_apply_and_step(&p, &out, dt);

        /* --- raccolta metriche durante le fasi di missione --- */
        if (out.mission_active) {
            mission_ran = 1;
            const double tgt = target_for_phase(&cfg, out.phase);
            const double top_depth = p.z - SENSOR_TO_BOTTOM - SENSOR_TO_TOP; /* z - 0.48 */
            if (p.z > max_depth) max_depth = p.z;

            if (out.phase == CORE_PHASE_DESCEND_HOLD) {
                double e = fabs(p.z - tgt);
                if (e < deep_closest) deep_closest = e;
            } else if (out.phase == CORE_PHASE_ASCEND_HOLD) {
                double e = fabs(p.z - tgt);
                if (e < shallow_closest) shallow_closest = e;
                if (top_depth < min_top_depth) min_top_depth = top_depth;
            }
        }

        /* --- CSV: una riga al secondo, piu' i cambi di modo/fase --- */
        static CoreMode  last_mode = CORE_MODE_INIT;
        static CorePhase last_phase = CORE_PHASE_NONE;
        if ((k % 20) == 0 || st.mode != last_mode || out.phase != last_phase) {
            printf("%.2f,%s,%s,%.3f,%.3f,%.4f,%ld,%.1f,%.1f\n",
                   t, mode_name(st.mode), phase_name(out.phase),
                   p.z, target_for_phase(&cfg, out.phase),
                   float_core_motor_pos_to_u(&cfg, p.motor_pos),
                   p.motor_pos, plant_tof_mm(&p), st.stable_s);
            last_mode = st.mode;
            last_phase = out.phase;
        }

        /* fine: missione conclusa e tornati a IDLE */
        if (mission_ran && go_sent && st.mode == CORE_MODE_IDLE) {
            printf("# mission complete at t=%.1fs\n", t);
            break;
        }
        if (st.mode == CORE_MODE_SAFE_STOP) {
            printf("# SAFE_STOP reached at t=%.1fs (unexpected in nominal MIL)\n", t);
        }
    }

    /* ------------------------------------------------------------------ */
    /* VERIFICA REQUISITI MATE                                             */
    /* ------------------------------------------------------------------ */
    printf("\n===== MIL SUMMARY =====\n");
    printf("homing completed          : %s\n", homed ? "YES" : "NO");
    printf("mission executed          : %s\n", mission_ran ? "YES" : "NO");
    printf("profiles completed        : %u / %u\n", st.completed_profiles, cfg.profile_count);
    printf("deep  closest to 2.50 m   : %.3f m (tol %.2f)\n", deep_closest, cfg.depth_tolerance_m);
    printf("shallow closest to 0.88 m : %.3f m (tol %.2f)\n", shallow_closest, cfg.depth_tolerance_m);
    printf("max depth (overshoot)     : %.3f m\n", max_depth);
    printf("min top-depth (surface)   : %.3f m  (>=0 = non rompe la superficie)\n", min_top_depth);

    int pass = homed && mission_ran &&
               (st.completed_profiles == cfg.profile_count) &&
               (deep_closest < cfg.depth_tolerance_m) &&
               (shallow_closest < cfg.depth_tolerance_m) &&
               (min_top_depth >= 0.0);
    printf("RESULT: %s\n", pass ? "PASS" : "CHECK (vedi metriche/tuning)");
    return pass ? 0 : 1;
}
