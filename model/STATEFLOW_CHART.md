# Chart Stateflow `float_core` — specifica di costruzione

Traduzione 1:1 di `lib/float_core/src/float_core.c` in Stateflow. Costruiscilo
seguendo questo documento; la logica è già validata (`tools/mil`). Linguaggio di
azione: **MATLAB** (non C).

## 1. Proprietà del chart
- **Update method**: `Discrete`, **Sample Time** = `P.pid_period_s` (0.05 s).
- **Action Language**: MATLAB.
- **Enable C-bit operations**: off. **Execute at initialization**: on.
- Superstate radice implicita = il chart.

## 2. Dati del chart (Model Explorer → chart)
| Nome | Scope | Tipo | Note |
|---|---|---|---|
| `In`  | Input     | Bus: `CoreInputsBus`  | sensori + evento |
| `Out` | Output    | Bus: `CoreOutputsBus` | comandi + eventi |
| `P`   | Parameter | Bus/struct (da `core_params.m`) | CoreConfig |
| `st_completed` | Local | uint8 | profili completati |
| `st_phase_t`, `st_stable_t`, `st_home_t`, `st_log_t` | Local | double | timer (s) |
| `pid_integral`, `pid_dfilt`, `pid_last_depth` | Local | double | stato PID |
| `pid_has_last` | Local | boolean | |
| `last_cmd_target` | Local | double | ultimo target motore comandato |
| `at_ext_limit` | Local | boolean | inibizione estensione (safety) |
| `tof_oor`, `home_confirm` | Local | uint8 | contatori di conferma |
| `req_safe`, `home_after` | Local | boolean | richiesta SAFE_STOP / auto-recovery |
| `phase_r` | Local | double | esito fase PID (0/1/2) |

## 3. Codici enum (== float_core_types.h)
```
motor_mode : HOLD=0  GOTO=1  JOG=2  STOP=3
event      : NONE=0  GO=1    HOME=2 STOP=3
led        : OFF=0 INIT=1 IDLE=2 IDLE_WD=3 ERROR=4 PROFILE=5 HOMING=6 PID=7 COMM=8
phase      : NONE=0 PRE_DESCENT=1 DESCEND_HOLD=2 ASCEND_HOLD=3 SURFACE_REST=4 DONE=5
ack        : NONE=0 GO_RECVD=1 HOME_RECVD=2 STOP_RECVD=3 IDLE=4 IDLE_W_DATA=5
log        : NONE=0 DEPLOY=1 PROFILE=2 PHASE_START=3 EXIT_HOLD_OK=4 EXIT_TIMEOUT=5 EXIT_REMOTE=6 EMERGENCY=7 HOMING=8
```

## 4. Funzioni grafiche (Chart → Add → Function, linguaggio MATLAB)

```matlab
function reset_outputs()
    % Default di ogni tick: chiamala per PRIMA in ogni entry: e during:.
    Out.motor_mode = int32(0);              % HOLD
    Out.motor_target = In.motor_pos;        % HOLD mantiene il target agganciato
    Out.motor_enable = false;
    Out.motor_zero_here = false;
    Out.led = int32(0);
    Out.phase = int32(0);
    Out.ack_event = int32(0);
    Out.log_now = false;
    Out.log_kind = int32(0);
    Out.mission_active = false;
    Out.completed_profiles = st_completed;
end
```
```matlab
function pos = u_to_pos(u)
    pos = int32(-round(u * double(P.usable_steps)));
end
```
```matlab
function pid_reset()
    pid_integral = 0; pid_dfilt = 0; pid_last_depth = 0; pid_has_last = false;
end
```
```matlab
function u = pid_compute(target, depth)
    dt = double(In.dt_s);
    if dt <= 0 || dt > 1, dt = double(P.pid_period_s); end
    e = target - depth;
    P_ = double(P.kp) * e;
    if pid_has_last, dRaw = (depth - pid_last_depth)/dt; else, dRaw = 0; end
    pid_dfilt = double(P.alpha_d)*dRaw + (1-double(P.alpha_d))*pid_dfilt;
    D_ = -double(P.kd)*pid_dfilt;
    I_ = double(P.ki)*pid_integral;
    uRaw = double(P.u_neutral) + P_ + I_ + D_;
    u = min(1, max(0, uRaw));
    satHigh = uRaw > 1; satLow = uRaw < 0;
    if ~((satHigh && e > 0) || (satLow && e < 0))
        pid_integral = pid_integral + e*dt;
        pid_integral = min(double(P.integral_limit), max(-double(P.integral_limit), pid_integral));
    end
    pid_last_depth = depth; pid_has_last = true;
end
```
```matlab
function em = safety()
    % Supervisore TOF (attivo solo in MISSION). Ritorna true su emergency.
    em = false;
    if ~In.tof_valid, return; end
    tooClose = In.tof_mm < P.tof_safe_min_mm;
    tooFar   = In.tof_mm > P.tof_safe_max_mm;
    if ~tooClose && ~tooFar, tof_oor = uint8(0); return; end
    tof_oor = tof_oor + uint8(1);
    if tof_oor < P.tof_safety_stop_samples, return; end
    tof_oor = uint8(0);
    if tooClose
        at_ext_limit = true; last_cmd_target = double(In.motor_pos);
        Out.motor_mode = int32(3); return;      % STOP pulito, inibisci estensione
    end
    req_safe = true; home_after = true;          % anomalia -> SAFE_STOP + auto-recovery
    Out.motor_mode = int32(3); Out.motor_enable = false;
    Out.led = int32(4); Out.log_now = true; Out.log_kind = int32(7);
    em = true;
end
```
```matlab
function pid_phase_enter(with_kick)
    pid_reset();
    phase_r = 0;   % IMPORTANTE in Stateflow: azzera l'esito fase all'ingresso,
                   % altrimenti la transizione uscente [phase_r~=0] scatta subito
                   % (ereditando l'1 della fase precedente) e la during non gira.
    st_phase_t = 0; st_stable_t = 0; st_log_t = 0;
    at_ext_limit = false; last_cmd_target = double(In.motor_pos);
    Out.led = int32(7); Out.motor_enable = true;
    if with_kick
        Out.motor_mode = int32(1);                       % GOTO
        Out.motor_target = u_to_pos(double(P.descent_kick_u));
        last_cmd_target = double(Out.motor_target);
    end
end
```
```matlab
function r = pid_phase(target, hold_s, timeout_s, exit_on_hold)
    r = 0;
    Out.led = int32(7); Out.motor_enable = true;
    if safety(), return; end                              % emergency: req_safe settato
    u = min(double(P.u_max), max(double(P.u_min), pid_compute(target, double(In.depth_m))));
    posTarget = double(u_to_pos(u));
    deadband  = double(P.min_retarget_frac) * double(P.usable_steps);
    retreating = posTarget > double(In.motor_pos);
    if (~at_ext_limit || retreating) && abs(posTarget - last_cmd_target) >= deadband
        Out.motor_mode = int32(1); Out.motor_target = int32(posTarget);
        last_cmd_target = posTarget; at_ext_limit = false;
    end
    if abs(double(In.depth_m) - target) < double(P.depth_tolerance_m)
        st_stable_t = st_stable_t + double(In.dt_s);
    else
        st_stable_t = 0;
    end
    st_log_t = st_log_t + double(In.dt_s);
    if st_log_t >= 1.0
        st_log_t = 0; Out.log_now = true; Out.log_kind = int32(2);  % PROFILE
    end
    st_phase_t = st_phase_t + double(In.dt_s);
    if exit_on_hold && st_stable_t >= hold_s, r = 1; return; end
    if st_phase_t >= timeout_s, r = 2; end
end
```

## 5. Gerarchia degli stati

Struttura: un superstato **RUN** contiene INIT/HOMING/IDLE/MISSION; **SAFE_STOP**
è fuori. Due transizioni dal bordo di RUN gestiscono lo stop globale.

```
float_core
├─ RUN
│   ├─ INIT            (default di RUN)
│   ├─ HOMING ─ H_APPROACH(def) · H_RETRACT · H_BACKOFF
│   ├─ IDLE
│   └─ MISSION ─ M_PRE_DESCEND(def) · M_DESCEND · M_ASCEND · M_REST
└─ SAFE_STOP
```

### Transizioni globali dal bordo di RUN (massima priorità)
```
RUN --[In.event==3]--> SAFE_STOP
     { Out.ack_event=int32(3); Out.motor_mode=int32(3); Out.led=int32(4);
       Out.log_now=true; Out.log_kind=int32(7); home_after=false; }
RUN --[req_safe]--> SAFE_STOP { req_safe=false; }
```

### INIT
```
during: reset_outputs(); Out.led=int32(1);
INIT --> HOMING { st_home_t=0; home_confirm=uint8(0); }   % transizione incondizionata
```

### HOMING (comune a tutte le foglie)
Metti nel **during del superstato HOMING**:
```
during: reset_outputs(); Out.led=int32(6); Out.motor_enable=true;
        st_home_t = st_home_t + double(In.dt_s);
        if st_home_t > P.homing_timeout_s
            req_safe=true; home_after=false;
            Out.motor_mode=int32(3); Out.log_now=true; Out.log_kind=int32(8);
        end
```
Foglie:
```
H_APPROACH during:
    Out.motor_mode=int32(2); Out.motor_target=int32(-1);   % JOG verso il TOF
    if In.tof_valid
        if In.tof_mm < P.tof_homing_approach_mm, home_confirm=home_confirm+uint8(1);
        else, home_confirm=uint8(0); end
    end
H_APPROACH --[home_confirm >= P.tof_homing_confirm_samples]--> H_RETRACT
           { home_confirm=uint8(0); Out.motor_mode=int32(3); }

H_RETRACT during:
    Out.motor_mode=int32(2); Out.motor_target=int32(1);    % JOG via dal TOF
    if In.tof_valid
        if In.tof_mm > P.tof_homing_threshold_mm, home_confirm=home_confirm+uint8(1);
        else, home_confirm=uint8(0); end
    end
H_RETRACT --[home_confirm >= P.tof_homing_confirm_samples]--> H_BACKOFF
          { home_confirm=uint8(0);
            last_cmd_target=double(In.motor_pos)+double(P.endstop_margin_steps);
            Out.motor_mode=int32(3); }

H_BACKOFF during:
    Out.motor_mode=int32(1); Out.motor_target=int32(last_cmd_target);  % GOTO backoff
H_BACKOFF --[~In.motor_busy]--> IDLE
          { Out.motor_zero_here=true; Out.log_now=true; Out.log_kind=int32(8); }
```

### IDLE
```
entry:  req_safe=false;
during: reset_outputs();
        if In.has_stored_data, Out.led=int32(3); else, Out.led=int32(2); end
        st_phase_t = st_phase_t + double(In.dt_s);
        if st_phase_t >= 0.5
            st_phase_t = 0;
            if In.has_stored_data, Out.ack_event=int32(5); else, Out.ack_event=int32(4); end
        end
IDLE --[In.event==1]--> MISSION
     { Out.ack_event=int32(1); Out.led=int32(8); st_completed=uint8(0); }
IDLE --[In.event==2]--> HOMING
     { Out.ack_event=int32(2); Out.led=int32(8); st_home_t=0; home_confirm=uint8(0); }
```

### MISSION
```
M_PRE_DESCEND during:
    reset_outputs(); Out.mission_active=true; Out.phase=int32(1);
    Out.led=int32(7); Out.log_now=true; Out.log_kind=int32(1);   % DEPLOY
M_PRE_DESCEND --[after(1,tick)]--> M_DESCEND

M_DESCEND entry:  pid_phase_enter(true);
M_DESCEND during:
    reset_outputs(); Out.mission_active=true; Out.phase=int32(2);
    phase_r = pid_phase(double(P.descent_target_m), double(P.hold_time_s), ...
                        double(P.descent_timeout_s), true);
M_DESCEND --[phase_r~=0]--> M_ASCEND

M_ASCEND entry:   pid_phase_enter(false);
M_ASCEND during:
    reset_outputs(); Out.mission_active=true; Out.phase=int32(3);
    phase_r = pid_phase(double(P.ascent_target_bottom_m), double(P.hold_time_s), ...
                        double(P.ascent_timeout_s), true);
M_ASCEND --[phase_r~=0 && (st_completed+1) < P.profile_count]--> M_DESCEND
         { st_completed = st_completed + uint8(1); }
M_ASCEND --[phase_r~=0]--> M_REST
         { st_completed = st_completed + uint8(1); }

M_REST entry:     pid_phase_enter(false);
M_REST during:
    reset_outputs(); Out.mission_active=true; Out.phase=int32(4);
    phase_r = pid_phase(double(P.rest_target_bottom_m), double(P.hold_time_s), ...
                        double(P.rest_window_s), false);
M_REST --[phase_r==2]--> IDLE
```

### SAFE_STOP
```
during: reset_outputs(); Out.led=int32(4); Out.motor_mode=int32(3); Out.motor_enable=false;
SAFE_STOP --[home_after || In.event==2]--> HOMING
          { home_after=false; st_home_t=0; home_confirm=uint8(0); }
```

## 6. Divergenze volute dal riferimento C (accettabili in MIL)
- **Granularità del flash-log**: il modello emette solo `DEPLOY` (pre-discesa) e
  `PROFILE` (~1 Hz). I record diagnostici `PHASE_START`/`EXIT_*` del C sono
  omessi (non influenzano il controllo, solo il contenuto del log). Se li vuoi,
  aggiungi stati-ponte da 1 tick sulle transizioni di fase.
- **`cfg`**: nel C è un puntatore dentro `CoreInputs`; qui è il parametro `P`.
  In SIL l'interfaccia del codice generato avrà `P` come argomento separato: si
  adatta l'harness back-to-back di conseguenza.
- **`after(1,tick)`** in `M_PRE_DESCEND`: fa dimorare la fase 1 tick per emettere
  il pacchetto di deployment una sola volta (equivalente all'entry-once del C).
```
