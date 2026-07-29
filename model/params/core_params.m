%% core_params.m
%  Single source of truth dei parametri del modello MIL, allineato a
%  include/config.h e a tools/mil/mil_main.c (gia' validato).
%  Crea nel base workspace:
%    P  - config del control core (= CoreConfig del C)
%    PL - config del plant (attuatore + fisica + sensori)
%    CoreInputsBus / CoreOutputsBus - bus objects per gli I/O del chart
%
%  Esegui questo script PRIMA di aprire/eseguire float_mil.slx.
%  Maintainers: Colabella Davide, Benevenga Filippo - Team PoliTOcean

%% ---- Geometria motore (mirror config.h) --------------------------------
STEPS_PER_MM = 200 * 1 * (26.85124 / 8);      % 671.28 step/mm
TRAVEL_MM    = 45;
MAX_STEPS    = round(TRAVEL_MM * STEPS_PER_MM);% 30208
ENDSTOP_MARG = 10;

SENSOR_TO_BOTTOM = 0.49;
SENSOR_TO_TOP    = -0.01;

%% ---- P : CoreConfig ----------------------------------------------------
P = struct();
% missione
P.profile_count          = uint8(2);
P.descent_target_m       = single(2.50);
P.ascent_target_bottom_m = single(0.65 + SENSOR_TO_BOTTOM + SENSOR_TO_TOP);
P.rest_target_bottom_m   = single(0.15 + SENSOR_TO_BOTTOM + SENSOR_TO_TOP); % 0.63
P.depth_tolerance_m      = single(0.33);
P.hold_time_s            = single(30);
P.descent_timeout_s      = single(180);
P.ascent_timeout_s       = single(120);
P.rest_window_s          = single(120);
% PID
P.kp = single(0.5);  P.ki = single(0.1);  P.kd = single(3.0);
P.alpha_d          = single(0.25);
P.integral_limit   = single(5.0);
P.u_neutral        = single(0.011);
P.min_retarget_frac = single(0.001);
P.pid_period_s     = single(0.05);
P.descent_kick_u   = single(0.15);
P.u_min = single(0.20);  P.u_max = single(0.92);
% geometria motore
P.usable_steps         = int32(MAX_STEPS - 2*ENDSTOP_MARG);
P.endstop_margin_steps = int32(ENDSTOP_MARG);
P.max_steps            = int32(MAX_STEPS);
% safety / homing TOF
P.tof_safe_min_mm            = single(32);
P.tof_safe_max_mm            = single(82);
P.tof_safety_stop_samples    = uint8(3);
P.tof_homing_approach_mm     = single(50);
P.tof_homing_threshold_mm    = single(70);
P.tof_homing_confirm_samples = uint8(2);
P.homing_timeout_s           = single(30);

%% ---- PL : plant (mirror tools/mil/plant.c) -----------------------------
PL = struct();
PL.max_step_per_tick = int32(70);   % MOTOR_MAX_SPEED(1400) * 0.05
PL.u_neutral  = single(0.35);       % SIM_U_NEUTRAL
PL.accel_gain = single(0.05);       % SIM_ACCEL_GAIN
PL.drag_quad  = single(1.50);       % SIM_DRAG_QUAD
PL.pool_depth = single(3.00);       % SIM_POOL_DEPTH
PL.usable_steps = P.usable_steps;
PL.tof_home_mm  = single(75);       % TOF a pos=0
PL.tof_ext_mm   = single(34);       % TOF a u=1
PL.initial_pos  = int32(3000);      % non-homed: forza l'homing
% geometria per il calcolo top-depth nelle verifiche
PL.sensor_to_bottom = single(SENSOR_TO_BOTTOM);
PL.sensor_to_top    = single(SENSOR_TO_TOP);

%% ---- Bus objects per gli I/O del chart --------------------------------
% Ordine dei campi = ordine delle struct C (per codegen coerente in SIL).
% NB: 'cfg' NON e' nel bus di ingresso: e' il parametro P.
elems = { ...
  'dt_s','single'; 'depth_m','single'; 'pressure_kpa','single'; ...
  'tof_mm','single'; 'tof_valid','boolean'; 'motor_pos','int32'; ...
  'motor_position_known','boolean'; 'motor_busy','boolean'; ...
  'battery_mv','uint16'; 'has_stored_data','boolean'; 'event','int32' };
CoreInputsBus = local_make_bus(elems);

elems = { ...
  'motor_mode','int32'; 'motor_target','int32'; 'motor_enable','boolean'; ...
  'motor_zero_here','boolean'; 'led','int32'; 'phase','int32'; ...
  'ack_event','int32'; 'log_now','boolean'; 'log_kind','int32'; ...
  'mission_active','boolean'; 'completed_profiles','uint8' };
CoreOutputsBus = local_make_bus(elems);

% Bus per il PARAMETRO P (= CoreConfig). Ordine/tipi == struct P sopra.
% Serve per dichiarare P nel chart come Parameter di tipo Bus: CoreConfigBus.
elems = { ...
  'profile_count','uint8'; 'descent_target_m','single'; ...
  'ascent_target_bottom_m','single'; 'rest_target_bottom_m','single'; ...
  'depth_tolerance_m','single'; 'hold_time_s','single'; ...
  'descent_timeout_s','single'; 'ascent_timeout_s','single'; ...
  'rest_window_s','single'; 'kp','single'; 'ki','single'; 'kd','single'; ...
  'alpha_d','single'; 'integral_limit','single'; 'u_neutral','single'; ...
  'min_retarget_frac','single'; 'pid_period_s','single'; ...
  'descent_kick_u','single'; 'u_min','single'; 'u_max','single'; ...
  'usable_steps','int32'; 'endstop_margin_steps','int32'; 'max_steps','int32'; ...
  'tof_safe_min_mm','single'; 'tof_safe_max_mm','single'; ...
  'tof_safety_stop_samples','uint8'; 'tof_homing_approach_mm','single'; ...
  'tof_homing_threshold_mm','single'; 'tof_homing_confirm_samples','uint8'; ...
  'homing_timeout_s','single' };
CoreConfigBus = local_make_bus(elems);

assignin('base','P',P);
assignin('base','PL',PL);
assignin('base','CoreInputsBus',CoreInputsBus);
assignin('base','CoreOutputsBus',CoreOutputsBus);
assignin('base','CoreConfigBus',CoreConfigBus);
fprintf('core_params: P, PL, CoreInputsBus/OutputsBus/ConfigBus creati.\n');

%% ---- helper ------------------------------------------------------------
function bus = local_make_bus(elems)
  n = size(elems,1);
  be(n) = Simulink.BusElement;
  for i = 1:n
    be(i) = Simulink.BusElement;
    be(i).Name = elems{i,1};
    be(i).DataType = elems{i,2};
    be(i).Dimensions = 1;
  end
  bus = Simulink.Bus;
  bus.Elements = be;
end
