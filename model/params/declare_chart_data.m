%% declare_chart_data.m
%  Dichiara/aggiorna IN BLOCCO tutti i dati LOCALI del chart float_mil/Chart via
%  API Stateflow: scope Local, tipo corretto, Size 1 e valore iniziale (evita il
%  "read before write"). Idempotente: se un dato esiste gia', lo aggiorna.
%
%  USO: apri float_mil.slx, poi in MATLAB esegui  declare_chart_data
%  (In/Out/P e le FUNZIONI restano da fare a mano: qui solo i locali.)
%
%  Maintainers: Colabella Davide, Benevenga Filippo - Team PoliTOcean

mdl = 'float_mil';
if ~bdIsLoaded(mdl), load_system(mdl); end

rt = sfroot;
charts = rt.find('-isa','Stateflow.Chart');
ch = [];
for k = 1:numel(charts)
    if strcmp(charts(k).Path, [mdl '/Chart'])
        ch = charts(k); break;
    end
end
assert(~isempty(ch), 'Chart "%s/Chart" non trovato. Apri float_mil.slx.', mdl);

% nome ; tipo ; valore iniziale
locals = { ...
  'st_completed'   , 'uint8'   , '0'     ; ...
  'st_phase_t'     , 'double'  , '0'     ; ...
  'st_stable_t'    , 'double'  , '0'     ; ...
  'st_home_t'      , 'double'  , '0'     ; ...
  'st_log_t'       , 'double'  , '0'     ; ...
  'pid_integral'   , 'double'  , '0'     ; ...
  'pid_dfilt'      , 'double'  , '0'     ; ...
  'pid_last_depth' , 'double'  , '0'     ; ...
  'pid_has_last'   , 'boolean' , 'false' ; ...
  'last_cmd_target', 'double'  , '0'     ; ...
  'at_ext_limit'   , 'boolean' , 'false' ; ...
  'tof_oor'        , 'uint8'   , '0'     ; ...
  'home_confirm'   , 'uint8'   , '0'     ; ...
  'req_safe'       , 'boolean' , 'false' ; ...
  'home_after'     , 'boolean' , 'false' ; ...
  'phase_r'        , 'double'  , '0'     ; ...
};

existing = ch.find('-isa','Stateflow.Data');
for i = 1:size(locals,1)
    name = locals{i,1};  dtype = locals{i,2};  init = locals{i,3};
    d = [];
    for j = 1:numel(existing)
        if strcmp(existing(j).Name, name), d = existing(j); break; end
    end
    if isempty(d)
        d = Stateflow.Data(ch);
        d.Name = name;
    end
    d.Scope = 'Local';
    d.DataType = dtype;
    d.Props.Array.Size = '1';
    d.Props.InitialValue = init;
    fprintf('  %-16s %-8s init=%s\n', name, dtype, init);
end
fprintf('declare_chart_data: %d locali dichiarati/aggiornati su %s/Chart.\n', ...
        size(locals,1), mdl);
