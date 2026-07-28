function In = float_plant(Out, event_in, PL)
%#codegen
% float_plant - PLANT del float per il MIL. Mirror di tools/mil/plant.c.
%   Attuatore rate-limited (~2 mm/s, contratto latch), dinamica di profondita'
%   del 2o ordine (spinta siringa + drag quadratico), modelli sensore TOF e
%   pressione.
%
% USO come blocco MATLAB Function in float_mil.slx:
%   Ingressi : Out (bus CoreOutputsBus, dal chart, RITARDATO di 1 tick con
%              Unit Delay per rompere l'algebraic loop), event_in (int32, dal
%              trigger GO/HOME/STOP), PL (parametro, da core_params.m).
%   Uscita   : In (bus CoreInputsBus, verso il chart).
%
% Codici enum (== float_core_types.h): motor_mode HOLD=0 GOTO=1 JOG=2 STOP=3.

    % Stato persistente del plant
    persistent motor_pos motor_target z v dyn_active pos_known prev_mission
    if isempty(motor_pos)
        motor_pos    = double(PL.initial_pos);
        motor_target = double(PL.initial_pos);
        z = 0; v = 0;
        dyn_active = false;
        pos_known  = false;
        prev_mission = false;
    end

    dt      = double(P_pid_period());   % passo fisso 0.05 s
    usable  = double(PL.usable_steps);
    maxstep = double(PL.max_step_per_tick);

    % --- attivazione dinamica sul fronte di salita di mission_active ---
    if Out.mission_active && ~prev_mission
        z = 0; v = 0; dyn_active = true;   % la missione parte dal pelo
    end
    prev_mission = logical(Out.mission_active);

    % --- attuatore (contratto latch) ---
    if Out.motor_zero_here
        motor_pos = 0; motor_target = 0; pos_known = true;
    end
    switch double(Out.motor_mode)
        case 1  % GOTO: aggancia il target e inseguilo
            motor_target = double(Out.motor_target);
            motor_pos = motor_pos + max(-maxstep, min(maxstep, motor_target - motor_pos));
        case 2  % JOG: moto continuo nel verso del segno di motor_target
            dir = -1; if double(Out.motor_target) >= 0, dir = 1; end
            motor_pos = motor_pos + dir*maxstep;
            motor_pos = max(-2*usable, min(usable, motor_pos));
            motor_target = motor_pos;
        case 3  % STOP: ferma qui
            motor_target = motor_pos;
        otherwise % HOLD(0): continua verso il target agganciato
            motor_pos = motor_pos + max(-maxstep, min(maxstep, motor_target - motor_pos));
    end

    % --- dinamica di profondita' (2o ordine) ---
    if dyn_active
        u = -motor_pos / usable;
        u = max(0, min(1, u));
        a = double(PL.accel_gain)*(u - double(PL.u_neutral)) ...
            - double(PL.drag_quad)*v*abs(v);
        v = v + a*dt;
        z = z + v*dt;
        if z < 0, z = 0; if v < 0, v = 0; end, end
        zmax = double(PL.pool_depth) - double(PL.sensor_to_bottom);
        if zmax > 0 && z > zmax, z = zmax; if v > 0, v = 0; end, end
    end

    % --- modelli sensore ---
    frac  = -motor_pos / usable;
    tofmm = double(PL.tof_home_mm) - frac*(double(PL.tof_home_mm) - double(PL.tof_ext_mm));
    press = 997.0*9.80665*z/1000.0;

    busy = false;
    m = double(Out.motor_mode);
    if m == 2
        busy = true;
    elseif m ~= 3
        busy = (motor_pos ~= motor_target);
    end

    % --- assembla il bus di ingresso per il chart ---
    In.dt_s                 = single(dt);
    In.depth_m              = single(z);
    In.pressure_kpa         = single(press);
    In.tof_mm               = single(tofmm);
    In.tof_valid            = true;
    In.motor_pos            = int32(round(motor_pos));
    In.motor_position_known = pos_known;
    In.motor_busy           = busy;
    In.battery_mv           = uint16(12500);
    In.has_stored_data      = (Out.completed_profiles > 0);
    In.event                = int32(event_in);
end

function ts = P_pid_period()
    ts = 0.05;  % deve coincidere con P.pid_period_s
end
