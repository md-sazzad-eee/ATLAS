%% ATLAS-S wheel-speed loop: model, tune, simulate (Simulink)
% Builds atlas_wheel_speed_loop.slx:
%   step speed reference -> discrete PI (100 Hz, clamped anti-windup, +/-Vbat)
%   -> DC gearmotor + reflected wheel/robot inertia (State-Space, inputs V and load torque)
%   -> wheel angle -> encoder quantisation -> finite-difference speed estimate (feedback)
% A dock-ramp load torque step is applied at t = 1.0 s.
%
% MOTOR PARAMETERS ARE ASSUMED for a 25 mm-class 6 V brushed gearmotor, 1:150.
% Replace with datasheet values, or better, identify them from a step test
% (System Identification Toolbox) once the motor is on the bench.

clear; close all;
here = fileparts(mfilename('fullpath')); cd(here);
P = jsondecode(fileread(fullfile(here,'..','..','00_requirements','atlas_params.json')));
A = P.agent; g = P.environment.gravity; Crr = P.environment.rolling_resistance_coeff;

% ---- motor (assumed) ----
M.R = 6.0; M.L = 1.5e-3; M.Kt = 4.5e-3; M.Ke = 4.5e-3; M.Jm = 1.0e-7; M.b = 1.0e-7;
M.N = 150; M.eta = A.gearbox_efficiency;
r   = A.wheel_diameter_mm/2000;
Vb  = A.battery.nominal_voltage;
Ts  = 0.01;                                     % control period
cpr = A.encoder_cpr_motor_shaft*4*M.N;          % quadrature counts per wheel rev
Jl  = (A.mass_loaded_kg/A.driven_wheels)*r^2;   % robot mass reflected to one wheel
J   = M.Jm + Jl/(M.N^2*M.eta);

% states x = [i; w_motor; theta_wheel], inputs u = [V; T_load_wheel], outputs [w_wheel; theta_wheel]
Ass = [-M.R/M.L, -M.Ke/M.L, 0;  M.Kt/J, -M.b/J, 0;  0, 1/M.N, 0];
Bss = [1/M.L, 0;  0, -1/(M.N*M.eta*J);  0, 0];
Css = [0, 1/M.N, 0;  0, 0, 1];
Dss = zeros(2,2);

% ---- tune PI on the discretised V -> wheel-speed plant ----
G  = minreal(ss(Ass, Bss(:,1), Css(1,:), 0));
Gd = c2d(G, Ts, 'zoh') * tf(1, [1 0], Ts);     % +1 sample for the finite-difference estimate
wc = 20;                                        % rad/s target crossover
[Cpi, info] = pidtune(Gd, 'PI', wc);
Kp = Cpi.Kp; Ki = Cpi.Ki;
fprintf('PI tuned: Kp = %.3f V/(rad/s), Ki = %.3f V/rad, PM = %.1f deg, wc = %.1f rad/s\n', Kp, Ki, info.PhaseMargin, info.CrossoverFrequency);

% ---- operating point ----
w_ref  = A.max_speed_mps / r;                                                    % rad/s at wheel
T_ramp = A.mass_loaded_kg*g*(sind(P.dock.ramp_angle_deg) + Crr*cosd(P.dock.ramp_angle_deg))/A.driven_wheels*r;
q_ang  = 2*pi/cpr;
stall_T = M.Kt*Vb/M.R*M.N*M.eta;  stall_I = Vb/M.R;
fprintf('Wheel ref %.2f rad/s (%.1f rpm), ramp load %.4f N.m, stall %.3f N.m @ %.2f A, J_eq %.3g kg.m^2\n', ...
        w_ref, w_ref*60/2/pi, T_ramp, stall_T, stall_I, J);

% ---- build Simulink model ----
mdl = 'atlas_wheel_speed_loop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile([mdl '.slx']), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
ab = @(lib, name, pos, varargin) add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
ab('simulink/Sources/Step', 'w_ref',   [30 100 60 130], 'Time','0.1','Before','0','After','w_ref','SampleTime','Ts');
ab('simulink/Math Operations/Sum', 'err', [110 100 130 130], 'Inputs','+-');
ab('simulink/Discrete/Discrete PID Controller', 'PI', [170 95 250 135], ...
   'Controller','PI','P','Kp','I','Ki','SampleTime','Ts','LimitOutput','on', ...
   'UpperSaturationLimit','Vb','LowerSaturationLimit','-Vb','AntiWindupMode','clamping');
ab('simulink/Sources/Step', 'T_load',  [170 180 200 210], 'Time','1.0','Before','0','After','T_ramp');
ab('simulink/Signal Routing/Mux', 'mux', [290 100 295 200], 'Inputs','2');
ab('simulink/Continuous/State-Space', 'motor_wheel', [330 130 430 170], 'A','Ass','B','Bss','C','Css','D','Dss');
ab('simulink/Signal Routing/Demux', 'demux', [470 130 475 170], 'Outputs','2');
ab('simulink/Discrete/Zero-Order Hold', 'sample', [510 180 540 210], 'SampleTime','Ts');
ab('simulink/Discontinuities/Quantizer', 'encoder', [570 180 610 210], 'QuantizationInterval','q_ang');
ab('simulink/Discrete/Difference', 'dtheta', [640 180 680 210]);
ab('simulink/Math Operations/Gain', 'per_Ts', [700 180 740 210], 'Gain','1/Ts');
ab('simulink/Sinks/Out1', 'w_wheel', [520 120 550 134]);
ab('simulink/Sinks/Out1', 'w_meas',  [790 188 820 202]);
ab('simulink/Sinks/Out1', 'V_cmd',   [300 40 330 54]);
ab('simulink/Sinks/Out1', 'w_ref_o', [110 40 140 54]);
L = @(a,b) add_line(mdl, a, b, 'autorouting', 'on');
L('w_ref/1','err/1'); L('err/1','PI/1'); L('PI/1','mux/1'); L('T_load/1','mux/2');
L('mux/1','motor_wheel/1'); L('motor_wheel/1','demux/1');
L('demux/1','w_wheel/1'); L('demux/2','sample/1'); L('sample/1','encoder/1');
L('encoder/1','dtheta/1'); L('dtheta/1','per_Ts/1'); L('per_Ts/1','w_meas/1'); L('per_Ts/1','err/2');
L('PI/1','V_cmd/1'); L('w_ref/1','w_ref_o/1');
set_param(mdl, 'StopTime','2','Solver','ode23t','MaxStep','1e-3','SaveOutput','on','SaveFormat','Dataset','ReturnWorkspaceOutputs','on');
% store parameters in the model workspace so the .slx runs standalone
hws = get_param(mdl,'ModelWorkspace');
vars = struct('Ts',Ts,'Kp',Kp,'Ki',Ki,'Vb',Vb,'w_ref',w_ref,'T_ramp',T_ramp,'q_ang',q_ang, ...
              'Ass',Ass,'Bss',Bss,'Css',Css,'Dss',Dss);
fn = fieldnames(vars); for k = 1:numel(fn), assignin(hws, fn{k}, vars.(fn{k})); end
save_system(mdl);

% ---- simulate & evaluate ----
out = sim(mdl);
y = out.yout;
w  = y{1}.Values;  wm = y{2}.Values;  V = y{3}.Values;
t = w.Time; wd = squeeze(w.Data);
si = stepinfo(wd(t<1.0), t(t<1.0), w_ref, 'SettlingTimeThreshold', 0.05);
pre  = mean(wd(t>0.8 & t<1.0));
post = mean(wd(t>1.8));
dip  = w_ref - min(wd(t>1.0));
Vd = squeeze(V.Data); Vmax = max(abs(Vd(V.Time>0.5)));
fprintf('\nStep (0 -> %.2f rad/s): rise %.3f s, settle(5%%) %.3f s, overshoot %.1f %%\n', w_ref, si.RiseTime-0.1*0, si.SettlingTime-0.1, si.Overshoot);
fprintf('Steady-state error flat: %.2f %%, on ramp: %.2f %%; speed dip at load step %.2f rad/s (%.1f %%)\n', ...
        100*(w_ref-pre)/w_ref, 100*(w_ref-post)/w_ref, dip, 100*dip/w_ref);
fprintf('Steady voltage on ramp %.2f V of %.1f V available (headroom %.0f %%)\n', mean(Vd(V.Time>1.8)), Vb, 100*(1-mean(Vd(V.Time>1.8))/Vb));

f = figure('Visible','off','Position',[100 100 800 500]);
subplot(2,1,1); plot(t, wd*60/2/pi, 'LineWidth',1.4); hold on;
stairs(wm.Time, squeeze(wm.Data)*60/2/pi, ':'); yline(w_ref*60/2/pi,'--');
xline(1.0, ':', 'ramp load'); grid on; ylabel('Wheel speed (rpm)');
legend('true','encoder estimate','reference','Location','southeast'); title('ATLAS-S wheel-speed loop (PI, 100 Hz)');
subplot(2,1,2); stairs(V.Time, Vd, 'LineWidth',1.2); yline(Vb,'--','V_{bat}'); grid on;
xlabel('Time (s)'); ylabel('Motor voltage (V)');
if ~exist('results','dir'), mkdir results; end
exportgraphics(f, fullfile('results','speed_loop_step.png'), 'Resolution', 150);
R = struct('Kp',Kp,'Ki',Ki,'Ts',Ts,'phase_margin_deg',info.PhaseMargin,'rise_s',si.RiseTime, ...
    'overshoot_pct',si.Overshoot,'ss_err_ramp_pct',100*(w_ref-post)/w_ref,'dip_pct',100*dip/w_ref, ...
    'motor_assumed',M,'encoder_counts_per_wheel_rev',cpr);
fid = fopen(fullfile('results','speed_loop_results.json'),'w'); fprintf(fid,'%s',jsonencode(R,'PrettyPrint',true)); fclose(fid);
close_system(mdl, 1);
