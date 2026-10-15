%% Scenario tests for the docking Stateflow chart (blueprint §4.2, §9, §11)
% Each scenario drives the chart's inputs with timed sensor signals and checks
% the final state plus safety invariants on every sample:
%   I1  move_permit => seated && latch_locked           (never move with unlocked agent)
%   I2  drive_cmd==+1 => latch_open_cmd                 (never drive into a closed latch)
%   I3  state FAULT  => drive_cmd==0 && ~move_permit
here = fileparts(mfilename('fullpath')); cd(here);
mdl = 'atlas_docking_fsm';
if ~isfile(fullfile(here,[mdl '.slx'])), build_docking_fsm(); end
load_system(mdl);
names = {'cmd_dock','cmd_undock','seated','latch_locked','id_ok','comm_ok', ...
         'carrier_stationary','agent_clear','estop','reset'};
T = 0:0.01:30;
S = {};  % {name, signal struct, expected final state, stopTime}

% helper: step(t0[,t1]) -> boolean vector true on [t0,t1)
on = @(t0,t1) (T >= t0) & (T < t1);

% 1 nominal dock, transport, undock
s = base(T); s.cmd_dock = on(1,1.2); s.seated = on(4,20); s.latch_locked = on(4.3,14.2);
s.cmd_undock = on(14,14.2); s.agent_clear = on(21,99);
S(end+1,:) = {'nominal dock + undock', s, 0, 30};

% 2 agent never seats -> 3 retries -> FAULT
s = base(T); s.cmd_dock = on(1,1.2);
S(end+1,:) = {'never seats -> FAULT after retries', s, 9, 30};   % may need longer horizon
S{end,4} = 80;

% 3 latch never confirms -> back off, retry; third attempt succeeds
s = base(T); s.cmd_dock = on(1,1.2); s.seated = on(4,6) | on(9.6,99); s.latch_locked = on(10,99);
S(end+1,:) = {'latch miss then success', s, 4, 30};

% 4 lock lost while docked (false-lock / vibration) -> FAULT
s = base(T); s.cmd_dock = on(1,1.2); s.seated = on(4,99); s.latch_locked = on(4.3,12);
S(end+1,:) = {'lock lost in transport -> FAULT', s, 9, 30};

% 5 wrong agent ID -> verify timeout -> backoff
s = base(T); s.cmd_dock = on(1,1.2); s.seated = on(4,6.5); s.latch_locked = on(4.3,6.2); s.id_ok = false(size(T));
S(end+1,:) = {'bad ID -> not docked', s, -1, 12};   % -1: any state except DOCKED

% 6 e-stop during approach, then reset
s = base(T); s.cmd_dock = on(1,1.2); s.estop = on(2,3); s.reset = on(5,5.2);
S(end+1,:) = {'estop then reset -> IDLE', s, 0, 10};

% 7 latch stuck on undock -> FAULT
s = base(T); s.cmd_dock = on(1,1.2); s.seated = on(4,99); s.latch_locked = on(4.3,99); s.cmd_undock = on(10,10.2);
S(end+1,:) = {'latch stuck on undock -> FAULT', s, 9, 20};

% 8 dock requested while carrier moving -> ignored
s = base(T); s.cmd_dock = on(1,1.2); s.carrier_stationary = ~on(0,5);
S(end+1,:) = {'dock cmd while moving ignored', s, 0, 10};

results = table('Size',[0 5],'VariableTypes',{'string','double','double','logical','string'}, ...
    'VariableNames',{'Scenario','Expected','Final','Pass','Notes'});
for k = 1:size(S,1)
    [nm, s, expState, tStop] = S{k,:};
    Tk = 0:0.01:tStop;
    ds = Simulink.SimulationData.Dataset;
    for j = 1:numel(names)
        v = s.(names{j});
        if numel(v) < numel(Tk), v = [v, repmat(v(end), 1, numel(Tk)-numel(v))]; end
        s.(names{j}) = v(1:numel(Tk));
        ds = ds.addElement(timeseries(s.(names{j})', Tk'), names{j});
    end
    in = Simulink.SimulationInput(mdl);
    in = in.setExternalInput(ds);
    in = in.setModelParameter('StopTime', num2str(tStop));
    out = sim(in);
    y = out.yout;
    % root outports in order: state_id, drive_cmd, latch_open_cmd, move_permit
    st  = y{1}.Values.Data;  dr = y{2}.Values.Data;
    lo  = y{3}.Values.Data;  mp = y{4}.Values.Data;
    tt  = y{1}.Values.Time;
    seat = interp1(Tk, double(s.seated), tt, 'previous') > 0.5;
    lock = interp1(Tk, double(s.latch_locked), tt, 'previous') > 0.5;
    i1 = all(~mp | (seat & lock));
    i2 = all(~(dr == 1) | lo);
    i3 = all(~(st == 9) | (dr == 0 & ~mp));
    fin = st(end);
    if expState == -1, okState = fin ~= 4; else, okState = fin == expState; end
    ok = okState && i1 && i2 && i3;
    note = sprintf('I1=%d I2=%d I3=%d max_retries_state=%d', i1, i2, i3, max(st==7));
    results(end+1,:) = {string(nm), expState, fin, ok, string(note)}; %#ok<SAGROW>
    % export trace for the Python twin parity test (05_simulation/python/tests)
    trDir = fullfile(here, 'results', 'docking_traces'); if ~exist(trDir,'dir'), mkdir(trDir); end
    tr = table(tt, 'VariableNames', {'t'});
    for j = 1:numel(names), tr.(names{j}) = interp1(Tk, double(s.(names{j})), tt, 'previous'); end
    tr.state_id = double(st(:)); tr.drive_cmd = double(dr(:)); tr.latch_open_cmd = double(lo(:)); tr.move_permit = double(mp(:));
    writetable(tr, fullfile(trDir, sprintf('scenario_%d.csv', k)));
end
disp(results);
fprintf('\n%d / %d docking scenarios passed\n', sum(results.Pass), height(results));
writetable(results, fullfile(here, 'results', 'docking_fsm_tests.csv'));

function s = base(T)
    f = false(size(T)); t = true(size(T));
    s = struct('cmd_dock',f,'cmd_undock',f,'seated',f,'latch_locked',f,'id_ok',t,'comm_ok',t, ...
               'carrier_stationary',t,'agent_clear',f,'estop',f,'reset',f);
end
