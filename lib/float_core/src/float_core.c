/*
 *******************************************************************************
 * float_core.c
 * Implementazione di RIFERIMENTO del control core, tick-based e senza I/O.
 * Riproduce fedelmente la logica del firmware attuale (runVerticalProfiles +
 * ProfileManager::measure + PIDController + MotionController::tofGuard/homing)
 * ma come funzione pura float_core_step(): niente millis()/delay()/Serial,
 * niente while bloccanti. I tempi si accumulano con dt.
 *
 * Questa e' la specifica eseguibile che il modello Simulink/Stateflow dovra'
 * riprodurre in MIL; in seguito il codice generato da Embedded Coder rimpiazza
 * questo file mantenendo la stessa firma.
 *
 * Maintainers: Colabella Davide, Benevenga Filippo — Team PoliTOcean
 *******************************************************************************
 */

#include "float_core.h"

/* ------------------------------------------------------------------------- */
/* Helper numerici                                                            */
/* ------------------------------------------------------------------------- */
static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static long labs_l(long v) { return v < 0 ? -v : v; }

static float fabsf_l(float v) { return v < 0.0f ? -v : v; }

long float_core_u_to_motor_pos(const CoreConfig* cfg, float u) {
    /* u=0 -> home (0); u=1 -> -usable. u cresce verso posizioni negative. */
    return -(long)(u * (float)cfg->usable_steps);
}

float float_core_motor_pos_to_u(const CoreConfig* cfg, long position) {
    if (cfg->usable_steps <= 0) return 0.0f;
    float u = -(float)position / (float)cfg->usable_steps;
    return clampf(u, 0.0f, 1.0f);
}

/* ------------------------------------------------------------------------- */
/* PID di profondita' — identico a pid.cpp ma con dt esplicito.               */
/* ------------------------------------------------------------------------- */
static void pid_reset(CoreState* st) {
    st->pid_integral  = 0.0f;
    st->pid_dfilt     = 0.0f;
    st->pid_last_depth = 0.0f;
    st->pid_has_last  = false;
}

static float pid_compute(CoreState* st, const CoreConfig* cfg,
                         float target, float depth, float dt) {
    if (dt <= 0.0f || dt > 1.0f) dt = cfg->pid_period_s;

    const float error = target - depth;
    const float P = cfg->kp * error;

    const float dRaw = st->pid_has_last ? (depth - st->pid_last_depth) / dt : 0.0f;
    st->pid_dfilt = cfg->alpha_d * dRaw + (1.0f - cfg->alpha_d) * st->pid_dfilt;
    const float D = -cfg->kd * st->pid_dfilt;

    const float I = cfg->ki * st->pid_integral;

    const float uRaw = cfg->u_neutral + P + I + D;
    const float uSat = clampf(uRaw, 0.0f, 1.0f);

    const bool satHigh = (uRaw > 1.0f);
    const bool satLow  = (uRaw < 0.0f);
    if (!((satHigh && error > 0.0f) || (satLow && error < 0.0f))) {
        st->pid_integral += error * dt;
        st->pid_integral = clampf(st->pid_integral, -cfg->integral_limit, cfg->integral_limit);
    }

    st->pid_last_depth = depth;
    st->pid_has_last   = true;
    return uSat;
}

/* ------------------------------------------------------------------------- */
/* Utility di transizione                                                     */
/* ------------------------------------------------------------------------- */
static void enter_phase(CoreState* st, CorePhase phase) {
    st->phase   = phase;
    st->entered = false;
}

static void enter_safe_stop(CoreState* st, bool auto_recover) {
    st->mode            = CORE_MODE_SAFE_STOP;
    st->emergency       = true;
    st->home_after_stop = auto_recover;
    st->entered         = false;
}

static void enter_homing(CoreState* st) {
    st->mode              = CORE_MODE_HOMING;
    st->home_phase        = CORE_HOME_APPROACH;
    st->home_elapsed_s    = 0.0f;
    st->home_confirm_count = 0;
    st->entered           = false;
}

static void enter_mission(CoreState* st) {
    st->mode               = CORE_MODE_MISSION;
    st->completed_profiles = 0;
    st->emergency          = false;
    enter_phase(st, CORE_PHASE_PRE_DESCENT);
}

/* ------------------------------------------------------------------------- */
/* init                                                                       */
/* ------------------------------------------------------------------------- */
void float_core_init(CoreState* st) {
    /* azzeramento completo dello stato */
    st->mode = CORE_MODE_INIT;
    st->phase = CORE_PHASE_NONE;
    st->home_phase = CORE_HOME_APPROACH;
    st->completed_profiles = 0;
    st->phase_elapsed_s = 0.0f;
    st->stable_s = 0.0f;
    st->home_elapsed_s = 0.0f;
    st->log_elapsed_s = 0.0f;
    pid_reset(st);
    st->last_commanded_target = 0;
    st->at_extension_limit = false;
    st->tof_oor_count = 0;
    st->home_confirm_count = 0;
    st->emergency = false;
    st->home_after_stop = false;
    st->kick_done = false;
    st->entered = false;
}

/* ------------------------------------------------------------------------- */
/* Supervisore di safety TOF (attivo solo in MISSION). Ritorna true se ha     */
/* scatenato un emergency stop (il chiamante deve uscire subito).             */
/* ------------------------------------------------------------------------- */
static bool safety_supervisor(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    const CoreConfig* cfg = in->cfg;
    if (!in->tof_valid) return false;

    const bool tooClose = in->tof_mm < cfg->tof_safe_min_mm;
    const bool tooFar   = in->tof_mm > cfg->tof_safe_max_mm;

    if (!tooClose && !tooFar) {
        st->tof_oor_count = 0;
        return false;
    }

    /* conferma prima di agire: ignora glitch singoli (bolle, riflessi) */
    if (++st->tof_oor_count < cfg->tof_safety_stop_samples) {
        return false;
    }
    st->tof_oor_count = 0;

    if (tooClose) {
        /* fondo corsa esteso (tappo): STOP PULITO, inibisci ulteriore
         * estensione ma NON abortire la missione */
        st->at_extension_limit = true;
        st->last_commanded_target = in->motor_pos;
        out->motor_mode = CORE_MOTOR_STOP;
        return false;
    }

    /* siringa troppo lontana dal TOF = anomalia (passi persi, verso sbagliato):
     * emergency stop con auto-recovery */
    enter_safe_stop(st, true);
    out->motor_mode = CORE_MOTOR_STOP;
    out->motor_enable = false;
    out->led = CORE_LED_ERROR;
    out->log_now = true;
    out->log_kind = CORE_LOG_EMERGENCY;
    return true;
}

/* ------------------------------------------------------------------------- */
/* Una fase PID di profilo: guida la siringa verso target, accumula il tempo  */
/* di stabilita' entro tolleranza e logga periodicamente. Ritorna:            */
/*   0 = in corso, 1 = hold completato, 2 = timeout.                          */
/* La sosta finale usa la stessa logica ma ignora l'esito 1 (vedi chiamante). */
/* ------------------------------------------------------------------------- */
static int pid_phase_run(const CoreInputs* in, CoreState* st, CoreOutputs* out,
                         float target, float hold_s, float timeout_s,
                         bool exit_on_hold) {
    const CoreConfig* cfg = in->cfg;

    out->led = CORE_LED_PID_CONTROL;
    out->motor_enable = true;

    if (safety_supervisor(in, st, out)) return 0; /* emergency: mode cambiato */

    const float u = clampf(pid_compute(st, cfg, target, in->depth_m, in->dt_s),
                           cfg->u_min, cfg->u_max);
    const long posTarget = float_core_u_to_motor_pos(cfg, u);
    const long deadband  = (long)(cfg->min_retarget_frac * (float)cfg->usable_steps);
    const bool retreating = posTarget > in->motor_pos;

    if ((!st->at_extension_limit || retreating) &&
        labs_l(posTarget - st->last_commanded_target) >= deadband) {
        out->motor_mode   = CORE_MOTOR_GOTO;
        out->motor_target = posTarget;
        st->last_commanded_target = posTarget;
        st->at_extension_limit = false;
    }

    /* accumulo tempo entro tolleranza (equivalente all'hold check del firmware,
     * ma continuo invece che quantizzato a 5 s) */
    if (fabsf_l(in->depth_m - target) < cfg->depth_tolerance_m) {
        st->stable_s += in->dt_s;
    } else {
        st->stable_s = 0.0f;
    }

    /* log periodico di profilo (~1 Hz) */
    st->log_elapsed_s += in->dt_s;
    if (st->log_elapsed_s >= 1.0f) {
        st->log_elapsed_s = 0.0f;
        out->log_now = true;
        out->log_kind = CORE_LOG_PROFILE;
    }

    st->phase_elapsed_s += in->dt_s;

    if (exit_on_hold && st->stable_s >= hold_s) return 1;
    if (st->phase_elapsed_s >= timeout_s)       return 2;
    return 0;
}

/* Entry-action comune di una fase PID: reset PID/timer, kick opzionale. */
static void pid_phase_enter(const CoreInputs* in, CoreState* st, CoreOutputs* out,
                            bool with_kick) {
    const CoreConfig* cfg = in->cfg;
    pid_reset(st);
    st->phase_elapsed_s = 0.0f;
    st->stable_s = 0.0f;
    st->log_elapsed_s = 0.0f;
    st->at_extension_limit = false;
    st->last_commanded_target = in->motor_pos;
    out->led = CORE_LED_PID_CONTROL;
    out->motor_enable = true;
    out->log_now = true;
    out->log_kind = CORE_LOG_PHASE_START;

    if (with_kick) {
        const long kick = float_core_u_to_motor_pos(cfg, cfg->descent_kick_u);
        out->motor_mode = CORE_MOTOR_GOTO;
        out->motor_target = kick;
        st->last_commanded_target = kick;
    }
    st->entered = true;
}

/* ------------------------------------------------------------------------- */
/* HOMING                                                                     */
/* ------------------------------------------------------------------------- */
static void step_homing(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    const CoreConfig* cfg = in->cfg;
    out->led = CORE_LED_HOMING;
    out->motor_enable = true;
    out->phase = CORE_PHASE_NONE;

    st->home_elapsed_s += in->dt_s;
    if (st->home_elapsed_s > cfg->homing_timeout_s &&
        st->home_phase != CORE_HOME_DONE) {
        out->motor_mode = CORE_MOTOR_STOP;
        out->log_now = true;
        out->log_kind = CORE_LOG_HOMING;
        enter_safe_stop(st, false); /* homing fallito = errore fatale, no auto-recovery */
        return;
    }

    switch (st->home_phase) {
    case CORE_HOME_APPROACH:
        /* muovi VERSO il TOF (negativo) finche' tof < approach_mm */
        out->motor_mode = CORE_MOTOR_JOG;
        out->motor_target = -1; /* segno = verso */
        if (in->tof_valid && in->tof_mm < cfg->tof_homing_approach_mm) {
            if (++st->home_confirm_count >= cfg->tof_homing_confirm_samples) {
                st->home_confirm_count = 0;
                out->motor_mode = CORE_MOTOR_STOP;
                st->home_phase = CORE_HOME_RETRACT;
            }
        } else if (in->tof_valid) {
            st->home_confirm_count = 0;
        }
        break;

    case CORE_HOME_RETRACT:
        /* inverti (positivo, retrae) finche' tof > threshold */
        out->motor_mode = CORE_MOTOR_JOG;
        out->motor_target = +1;
        if (in->tof_valid && in->tof_mm > cfg->tof_homing_threshold_mm) {
            if (++st->home_confirm_count >= cfg->tof_homing_confirm_samples) {
                st->home_confirm_count = 0;
                out->motor_mode = CORE_MOTOR_STOP;
                /* backoff target = posizione attuale + margine fine corsa */
                st->last_commanded_target = in->motor_pos + cfg->endstop_margin_steps;
                st->home_phase = CORE_HOME_BACKOFF;
            }
        } else if (in->tof_valid) {
            st->home_confirm_count = 0;
        }
        break;

    case CORE_HOME_BACKOFF:
        out->motor_mode = CORE_MOTOR_GOTO;
        out->motor_target = st->last_commanded_target;
        if (!in->motor_busy) {
            out->motor_zero_here = true; /* HAL: setCurrentPosition(0) */
            out->log_now = true;
            out->log_kind = CORE_LOG_HOMING;
            st->home_phase = CORE_HOME_DONE;
        }
        break;

    case CORE_HOME_DONE:
    default:
        st->emergency = false;
        st->mode = CORE_MODE_IDLE;
        st->phase = CORE_PHASE_NONE;
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* IDLE                                                                       */
/* ------------------------------------------------------------------------- */
static void step_idle(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    st->emergency = false;
    out->motor_mode = CORE_MOTOR_HOLD;
    out->motor_enable = false;
    out->phase = CORE_PHASE_NONE;
    out->led = in->has_stored_data ? CORE_LED_IDLE_WITH_DATA : CORE_LED_IDLE;

    /* heartbeat ~2 Hz (PERIOD_CONN_CHECK = 500 ms) */
    st->phase_elapsed_s += in->dt_s;
    if (st->phase_elapsed_s >= 0.5f) {
        st->phase_elapsed_s = 0.0f;
        out->ack_event = in->has_stored_data ? CORE_ACK_IDLE_W_DATA : CORE_ACK_IDLE;
    }

    if (in->event == CORE_EVT_GO) {
        out->ack_event = CORE_ACK_GO_RECVD;
        out->led = CORE_LED_COMMUNICATION;
        enter_mission(st);
    } else if (in->event == CORE_EVT_HOME) {
        out->ack_event = CORE_ACK_HOME_RECVD;
        out->led = CORE_LED_COMMUNICATION;
        enter_homing(st);
    }
}

/* ------------------------------------------------------------------------- */
/* MISSION                                                                    */
/* ------------------------------------------------------------------------- */
static void step_mission(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    const CoreConfig* cfg = in->cfg;
    out->mission_active = true;
    out->phase = st->phase;

    switch (st->phase) {

    case CORE_PHASE_PRE_DESCENT:
        /* pacchetto di deployment pre-discesa, poi discesa profilo 1 */
        out->log_now = true;
        out->log_kind = CORE_LOG_DEPLOY;
        out->led = CORE_LED_PID_CONTROL;
        enter_phase(st, CORE_PHASE_DESCEND_HOLD);
        break;

    case CORE_PHASE_DESCEND_HOLD:
        if (!st->entered) { pid_phase_enter(in, st, out, /*with_kick=*/true); break; }
        {
            const int r = pid_phase_run(in, st, out, cfg->descent_target_m,
                                        cfg->hold_time_s, cfg->descent_timeout_s,
                                        /*exit_on_hold=*/true);
            if (r != 0) {
                out->log_now = true;
                out->log_kind = (r == 1) ? CORE_LOG_EXIT_HOLD_OK : CORE_LOG_EXIT_TIMEOUT;
                enter_phase(st, CORE_PHASE_ASCEND_HOLD);
            }
        }
        break;

    case CORE_PHASE_ASCEND_HOLD:
        if (!st->entered) { pid_phase_enter(in, st, out, /*with_kick=*/false); break; }
        {
            const int r = pid_phase_run(in, st, out, cfg->ascent_target_bottom_m,
                                        cfg->hold_time_s, cfg->ascent_timeout_s,
                                        /*exit_on_hold=*/true);
            if (r != 0) {
                out->log_now = true;
                out->log_kind = (r == 1) ? CORE_LOG_EXIT_HOLD_OK : CORE_LOG_EXIT_TIMEOUT;
                st->completed_profiles++;
                out->completed_profiles = st->completed_profiles;
                if (st->completed_profiles < cfg->profile_count) {
                    enter_phase(st, CORE_PHASE_DESCEND_HOLD); /* profilo successivo */
                } else {
                    enter_phase(st, CORE_PHASE_SURFACE_REST);
                }
            }
        }
        break;

    case CORE_PHASE_SURFACE_REST:
        if (!st->entered) { pid_phase_enter(in, st, out, /*with_kick=*/false); break; }
        {
            /* come una fase PID, ma NON esce all'hold: resta attiva finche' non
             * scade la finestra (o arriva un remote stop, gestito globalmente) */
            const int r = pid_phase_run(in, st, out, cfg->rest_target_bottom_m,
                                        cfg->hold_time_s, cfg->rest_window_s,
                                        /*exit_on_hold=*/false);
            if (r == 2) { /* solo timeout della finestra di sosta */
                out->log_now = true;
                out->log_kind = CORE_LOG_EXIT_TIMEOUT;
                enter_phase(st, CORE_PHASE_DONE);
            }
        }
        break;

    case CORE_PHASE_DONE:
    default:
        out->motor_mode = CORE_MOTOR_STOP;
        out->motor_enable = false;
        st->mode = CORE_MODE_IDLE;
        st->phase = CORE_PHASE_NONE;
        st->phase_elapsed_s = 0.0f;
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* SAFE_STOP                                                                  */
/* ------------------------------------------------------------------------- */
static void step_safe_stop(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    out->led = CORE_LED_ERROR;
    out->motor_mode = CORE_MOTOR_STOP;
    out->motor_enable = false;
    out->phase = CORE_PHASE_NONE;

    if (st->home_after_stop) {
        /* auto-recovery: l'homing cancella l'emergency e riporta a galleggiamento */
        st->home_after_stop = false;
        enter_homing(st);
    } else if (in->event == CORE_EVT_HOME) {
        enter_homing(st);
    }
    /* altrimenti resta in SAFE_STOP: ogni GO e' ignorato finche' non si fa HOME */
}

/* ------------------------------------------------------------------------- */
/* STEP principale                                                            */
/* ------------------------------------------------------------------------- */
void float_core_step(const CoreInputs* in, CoreState* st, CoreOutputs* out) {
    /* --- default delle uscite (ogni tick riparte da qui) --- */
    out->motor_mode      = CORE_MOTOR_HOLD;
    out->motor_target    = in->motor_pos;
    out->motor_enable    = false;
    out->motor_zero_here = false;
    out->led             = CORE_LED_OFF;
    out->phase           = st->phase;
    out->ack_event       = CORE_ACK_NONE;
    out->log_now         = false;
    out->log_kind        = CORE_LOG_NONE;
    out->mission_active  = (st->mode == CORE_MODE_MISSION);
    out->completed_profiles = st->completed_profiles;

    /* --- remote STOP: priorita' massima (tranne se gia' in SAFE_STOP) --- */
    if (in->event == CORE_EVT_STOP && st->mode != CORE_MODE_SAFE_STOP) {
        enter_safe_stop(st, /*auto_recover=*/false);
        out->ack_event   = CORE_ACK_STOP_RECVD;
        out->motor_mode  = CORE_MOTOR_STOP;
        out->led         = CORE_LED_ERROR;
        out->log_now     = true;
        out->log_kind    = CORE_LOG_EMERGENCY;
        return;
    }

    switch (st->mode) {
    case CORE_MODE_INIT:
        out->led = CORE_LED_INIT;
        enter_homing(st); /* il boot esegue l'homing prima di IDLE */
        break;
    case CORE_MODE_HOMING:    step_homing(in, st, out);    break;
    case CORE_MODE_IDLE:      step_idle(in, st, out);      break;
    case CORE_MODE_MISSION:   step_mission(in, st, out);   break;
    case CORE_MODE_SAFE_STOP: step_safe_stop(in, st, out); break;
    default:
        st->mode = CORE_MODE_IDLE;
        break;
    }
}
