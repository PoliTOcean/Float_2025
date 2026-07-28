%% run_mil.m
%  Lancia la simulazione MIL di float_mil.slx e verifica i requisiti MATE.
%  Replica la verifica di tools/mil/mil_main.c ma sul modello Simulink/Stateflow.
%
%  Prerequisiti nel modello float_mil.slx:
%   - Signal Logging attivo su tre segnali, rinominati esattamente cosi':
%       'depth_m'            = In.depth_m
%       'phase'              = Out.phase   (codici CorePhase, vedi sotto)
%       'completed_profiles' = Out.completed_profiles
%   - Trigger GO one-shot (vedi go_trigger sotto o un Pulse a t=30 s).
%
%  Codici CorePhase: NONE=0 PRE_DESCENT=1 DESCEND_HOLD=2 ASCEND_HOLD=3
%                    SURFACE_REST=4 DONE=5.

run(fullfile(fileparts(mfilename('fullpath')),'..','params','core_params.m'));

mdl = 'float_mil';
if ~bdIsLoaded(mdl), load_system(mdl); end

% StopTime 600: la missione (con l'oscillazione in risalita) dura oltre 450 s;
% 600 s cattura i 2 profili completi + la sosta finale.
simOut = sim(mdl, 'StopTime', '600', 'SignalLogging','on', ...
             'SignalLoggingName','logsout');

% Controllo esplicito: se i segnali non sono marcati per il logging, logsout
% non esiste. Serve "Log Selected Signals" sui fili depth_m/phase/completed_profiles.
if ~ismember('logsout', simOut.who)
    error(['Nessun segnale loggato. Nel modello, click destro sui fili ' ...
           '"depth_m", "phase" e "completed_profiles" -> "Log Selected Signals", ' ...
           'e imposta il Logging name uguale al nome. Poi ri-lancia run_mil.']);
end
lg    = simOut.logsout;

% I segnali da Bus Selector si loggano col nome tra parentesi (es. '<depth_m>').
% Confrontiamo togliendo '< >' e spazi, cosi' funziona in entrambi i casi.
strip   = @(s) erase(string(s), ["<",">"," "]);
avail   = arrayfun(@(i) strip(lg{i}.Name), 1:lg.numElements);
findsig = @(want) lg{find(avail==want,1)}.Values;

need    = ["depth_m","phase","completed_profiles"];
missing = need(~ismember(need, avail));
if ~isempty(missing)
    fprintf('Elementi loggati trovati: %s\n', strjoin(avail, ', '));
    error('Manca(no) nel log: %s.', strjoin(missing, ', '));
end

depth = findsig("depth_m");
phase = findsig("phase");
comp  = findsig("completed_profiles");

t  = depth.Time;
d  = double(depth.Data);
ph = double(phase.Data);

% Diagnostica: quali codici di fase compaiono davvero nel log.
% CorePhase: 0=NONE 1=PRE_DESCEND 2=DESCEND_HOLD 3=ASCEND_HOLD 4=SURFACE_REST 5=DONE
fprintf('Fasi (phase) presenti nel log: %s\n', mat2str(unique(ph(:))'));

DESCEND = 2; ASCEND = 3;
tol = double(P.depth_tolerance_m);
top_off = double(PL.sensor_to_bottom) + double(PL.sensor_to_top); % 0.48

safemin = @(x) min([x(:); nan]);   % NaN se vuoto (evita crash su fasi assenti)

deep_closest    = safemin(abs(d(ph==DESCEND) - double(P.descent_target_m)));
shallow_closest = safemin(abs(d(ph==ASCEND)  - double(P.ascent_target_bottom_m)));
max_depth       = max(d);
top_depth       = d - top_off;
min_top_depth   = safemin(top_depth(ph==ASCEND));
completed       = double(comp.Data(end));

fprintf('\n===== MIL SUMMARY (Simulink) =====\n');
fprintf('profiles completed        : %d / %d\n', completed, P.profile_count);
fprintf('deep  closest to %.2f m   : %.3f m (tol %.2f)\n', double(P.descent_target_m), deep_closest, tol);
fprintf('shallow closest to %.2f m : %.3f m (tol %.2f)\n', double(P.ascent_target_bottom_m), shallow_closest, tol);
fprintf('max depth (overshoot)     : %.3f m\n', max_depth);
fprintf('min top-depth (surface)   : %.3f m  (>=0 = non rompe la superficie)\n', min_top_depth);

pass = completed == double(P.profile_count) && ...
       deep_closest < tol && shallow_closest < tol && min_top_depth >= 0;
if pass
    fprintf('RESULT: PASS\n');
else
    fprintf('RESULT: CHECK (vedi metriche/tuning)\n');
end

% Confronto rapido con la traccia del MIL nativo (tools/mil/build/mission.csv),
% se disponibile: le due profondita' dovrebbero sovrapporsi (equivalenza MIL).
figure; plot(t, d, 'LineWidth',1.2); grid on; hold on;
yline(double(P.descent_target_m),'--'); yline(double(P.ascent_target_bottom_m),'--');
xlabel('t [s]'); ylabel('depth [m]'); title('MIL Simulink - profondita di missione');

%% --- go_trigger: incolla in un blocco MATLAB Function alimentato da Clock ---
%  function event = go_trigger(t)
%      event = int32(0);
%      if t >= 30 && t < 30.05, event = int32(1); end  % pulse GO di 1 tick
%  end
