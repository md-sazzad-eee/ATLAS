function build_docking_fsm()
%BUILD_DOCKING_FSM  Programmatically build the ATLAS docking Stateflow chart.
%   Creates atlas_docking_fsm.slx next to this file. The chart is the reference
%   implementation of blueprint §4 / §9; the C firmware and the Python sim
%   (05_simulation/python/atlas/docking.py) must implement the same transitions.
%
%   Outputs: state_id, drive_cmd (+1 slow in, 0 stop, -1 back out),
%            latch_open_cmd (latch is spring fail-LOCKED: true = retract pin),
%            move_permit (carrier may drive with this bay occupied).

here = fileparts(mfilename('fullpath'));
P = jsondecode(fileread(fullfile(here,'..','..','00_requirements','atlas_params.json')));
mdl = 'atlas_docking_fsm';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if isfile(f), delete(f); end

sfnew(mdl);
rt = sfroot;
m  = rt.find('-isa','Simulink.BlockDiagram','Name',mdl);
ch = m.find('-isa','Stateflow.Chart');
ch.ActionLanguage = 'MATLAB';
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime  = '0.01';
ch.Name = 'DockingFSM';

% ---- data
ins  = {'cmd_dock','cmd_undock','seated','latch_locked','id_ok','comm_ok', ...
        'carrier_stationary','agent_clear','estop','reset'};
for k = 1:numel(ins)
    d = Stateflow.Data(ch); d.Name = ins{k}; d.Scope = 'Input'; d.DataType = 'boolean';
end
outs = {'state_id','double'; 'drive_cmd','double'; 'latch_open_cmd','boolean'; 'move_permit','boolean'};
for k = 1:size(outs,1)
    d = Stateflow.Data(ch); d.Name = outs{k,1}; d.Scope = 'Output'; d.DataType = outs{k,2};
end
d = Stateflow.Data(ch); d.Name = 'retries';     d.Scope = 'Local';     d.DataType = 'double'; d.Props.InitialValue = '0';
d = Stateflow.Data(ch); d.Name = 'RETRY_LIMIT'; d.Scope = 'Parameter'; d.DataType = 'double';
hws = get_param(mdl, 'ModelWorkspace');
assignin(hws, 'RETRY_LIMIT', P.dock.retry_limit);

% ---- states
ACTIVE = Stateflow.State(ch); ACTIVE.Position = [20 20 900 520];
ACTIVE.LabelString = 'ACTIVE';
mk = @(parent, name, pos, body) local_state(parent, name, pos, body);
IDLE  = mk(ACTIVE,'IDLE',      [60  80 170 90], 'entry: state_id=0; drive_cmd=0; latch_open_cmd=false; move_permit=false;');
APPR  = mk(ACTIVE,'APPROACH',  [300 80 170 90], 'entry: state_id=1; drive_cmd=1; latch_open_cmd=true; move_permit=false;');
LATCH = mk(ACTIVE,'LATCHING',  [540 80 170 90], 'entry: state_id=2; drive_cmd=0; latch_open_cmd=false;');
VER   = mk(ACTIVE,'VERIFY',    [740 80 160 90], 'entry: state_id=3; drive_cmd=0;');
DOCK  = mk(ACTIVE,'DOCKED',    [740 260 160 110], ['entry: state_id=4; drive_cmd=0; latch_open_cmd=false;' newline ...
                                                     'during: move_permit = seated && latch_locked;' newline 'exit: move_permit=false;']);
UNL   = mk(ACTIVE,'UNLATCHING',[540 400 170 90], 'entry: state_id=5; latch_open_cmd=true; move_permit=false;');
REL   = mk(ACTIVE,'RELEASING', [300 400 170 90], 'entry: state_id=6; drive_cmd=-1;');
BACK  = mk(ACTIVE,'BACKOFF',   [300 250 200 110], ['entry: state_id=7; latch_open_cmd=true; drive_cmd=0; retries=retries+1;' newline ...
                                                     'during: if ~latch_locked, drive_cmd=-1; else, drive_cmd=0; end']);
FAULT = Stateflow.State(ch); FAULT.Position = [960 200 200 120];
FAULT.LabelString = sprintf('FAULT\nentry: state_id=9; drive_cmd=0; latch_open_cmd=false; move_permit=false;');

% ---- transitions
tr = @(src, dst, lbl) local_tr(ch, src, dst, lbl);
local_default(ch, ACTIVE, [10 40]);
local_default(ch, IDLE,   [70 60]);
tr(IDLE,  APPR,  '[cmd_dock && carrier_stationary]{retries=0;}');
tr(APPR,  LATCH, '[seated]');
tr(APPR,  BACK,  'after(15,sec)');
tr(LATCH, VER,   '[seated && latch_locked]');
tr(LATCH, BACK,  '[~seated || after(2,sec)]');
tr(VER,   DOCK,  '[seated && latch_locked && id_ok && comm_ok]');
tr(VER,   BACK,  'after(3,sec)');
tr(DOCK,  UNL,   '[cmd_undock && carrier_stationary]');
tr(DOCK,  FAULT, '[~seated || ~latch_locked]');          % lost lock with agent onboard
tr(UNL,   REL,   '[~latch_locked]');
tr(UNL,   FAULT, 'after(2,sec)');                         % latch stuck
tr(REL,   IDLE,  '[~seated && agent_clear]');
tr(REL,   FAULT, 'after(10,sec)');
tr(BACK,  FAULT, '[~seated && after(3,sec) && retries >= RETRY_LIMIT]');
tr(BACK,  APPR,  '[~seated && after(3,sec) && retries < RETRY_LIMIT]');
tr(BACK,  FAULT, 'after(10,sec)');                        % cannot back out
tr(ACTIVE, FAULT,'[estop]');
tr(FAULT, ACTIVE,'[reset && ~estop]');

% ---- wire root I/O
blk = [mdl '/DockingFSM'];
set_param(blk, 'Position', [400 80 600 420]);
for k = 1:numel(ins)
    b = add_block('simulink/Sources/In1', [mdl '/' ins{k}], 'Position', [200 60+30*k 230 74+30*k]);
    set_param(b, 'OutDataTypeStr', 'boolean');
    add_line(mdl, [ins{k} '/1'], sprintf('DockingFSM/%d', k), 'autorouting', 'on');
end
for k = 1:size(outs,1)
    add_block('simulink/Sinks/Out1', [mdl '/' outs{k,1}], 'Position', [750 100+60*k 780 114+60*k]);
    add_line(mdl, sprintf('DockingFSM/%d', k), [outs{k,1} '/1'], 'autorouting', 'on');
end
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', 'StopTime', '30', ...
    'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl, f);
fprintf('Built %s\n', f);
end

function s = local_state(parent, name, pos, body)
s = Stateflow.State(parent);
s.Position = pos;
s.LabelString = sprintf('%s\n%s', name, body);
end

function t = local_tr(ch, src, dst, lbl)
t = Stateflow.Transition(ch);
t.Source = src; t.Destination = dst; t.LabelString = lbl;
end

function t = local_default(ch, dst, xy)
t = Stateflow.Transition(ch);
t.Destination = dst;
t.DestinationOClock = 0;
t.SourceEndpoint = [dst.Position(1)+20, dst.Position(2)-25];
t.MidPoint       = [dst.Position(1)+20, dst.Position(2)-12];
end
