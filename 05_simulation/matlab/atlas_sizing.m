%% ATLAS drivetrain + energy sizing  (Phase 1)
% Reads 00_requirements/atlas_params.json (generated from atlas_params.yaml)
% and computes, for ATLAS-S (agent) and ATLAS-M (carrier):
%   * required gearmotor output torque & speed per driven wheel, per load case
%   * traction check (can the wheels transmit that force without slipping?)
%   * electrical power / current per motor -> motor-driver requirement
%   * mission runtime from battery energy, and carrier energy spent charging agents
% Outputs: results/sizing_results.json, results/*.png, results/sizing_report.txt
%
% All inputs are proposed targets. Re-run whenever the YAML changes.

clear; close all;
here   = fileparts(mfilename('fullpath'));
root   = fullfile(here, '..', '..');
P      = jsondecode(fileread(fullfile(root, '00_requirements', 'atlas_params.json')));
outDir = fullfile(here, 'results');  if ~exist(outDir, 'dir'), mkdir(outDir); end

g   = P.environment.gravity;
Crr = P.environment.rolling_resistance_coeff;
mu  = 0.6;          % tyre-floor friction (rubber on sealed floor, conservative)
eta_motor = 0.60;   % small brushed DC motor efficiency near rated point

%% ---------------- ATLAS-S ----------------
A = P.agent;
A.r = A.wheel_diameter_mm/2000;
% caster at rear: driven axle carries most of the weight
A.frac_on_driven = 0.75;
casesA = struct( ...
  'name', {'Flat accel to v_max', 'Floor slope at v_max', 'Dock ramp climb (docking speed)'}, ...
  'v',    {A.max_speed_mps, A.max_speed_mps, A.docking_speed_mps}, ...
  'a',    {A.design_accel_mps2, 0, 0.20}, ...
  'th',   {0, P.environment.max_floor_slope_deg, P.dock.ramp_angle_deg});
RA = size_vehicle(A, A.mass_loaded_kg, casesA, g, Crr, mu, eta_motor, 1.0);

%% ---------------- ATLAS-M ----------------
C = P.carrier;
C.r = C.wheel_diameter_mm/2000;
C.frac_on_driven = 1.0;   % 4WD: all weight on driven wheels
mC = C.mass_empty_kg + C.agents_carried*A.mass_loaded_kg;
casesC = struct( ...
  'name', {'Flat accel to v_max (loaded)', 'Floor slope at v_max (loaded)', 'Skid-steer turn in place (loaded)'}, ...
  'v',    {C.max_speed_mps, C.max_speed_mps, 0.15}, ...
  'a',    {C.design_accel_mps2, 0, 0.2}, ...
  'th',   {0, P.environment.max_floor_slope_deg, 0});
RC = size_vehicle(C, mC, casesC, g, Crr, mu, eta_motor, 1.0);
% Skid-steer in-place turn (Wong, Theory of Ground Vehicles): turning-resistance
% moment M_r = mu_t*W*L/4 must be balanced by opposing side thrusts over the track B.
% Per-wheel thrust = M_r/(2B) + rolling resistance share. This usually governs 4WD sizing.
Wt = mC*g; L = C.wheelbase_mm/1000; B = C.track_width_mm/1000;
Mr = C.lateral_friction_coeff*Wt*L/4;
Fw = Mr/(2*B) + Crr*Wt/4 + mC*0.2/4;           % + small angular-accel share
vT = 0.15;  % wheel rim speed during spin (m/s)
k = 3; RC(k).case = 'Skid-steer turn in place (loaded)';
RC(k).force_N = 4*Fw; RC(k).T_wheel_Nm = Fw*C.r; RC(k).T_req_Nm = Fw*C.r*C.torque_safety_factor;
RC(k).rpm = vT/C.r*60/(2*pi); RC(k).P_mech_W = Fw*vT;
RC(k).P_elec_W = RC(k).P_mech_W/(C.gearbox_efficiency*eta_motor); RC(k).I_motor_A = RC(k).P_elec_W/C.battery.min_voltage;
RC(k).traction_ok = Fw <= mu*Wt/4;

%% ---------------- Energy / runtime ----------------
EA = energy_agent(A, g, Crr, eta_motor);
EC = energy_carrier(C, mC, g, Crr, eta_motor, A);

%% ---------------- Report ----------------
fid = fopen(fullfile(outDir, 'sizing_report.txt'), 'w');
cleanup = onCleanup(@() fclose(fid));
w = @(varargin) (fprintf(fid, varargin{:}) + fprintf(varargin{:}));
w('ATLAS sizing report  (%s)  params v%s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')), P.meta.version);
w('All values are design requirements derived from proposed targets.\n\n');
print_vehicle(w, 'ATLAS-S agent', A.mass_loaded_kg, RA);
print_vehicle(w, 'ATLAS-M carrier', mC, RC);
w('\n=== Energy ===\n');
w('ATLAS-S  pack %.1f Wh usable | avg power scout %.2f W, research %.2f W\n', EA.E_usable_Wh, EA.P_scout_W, EA.P_research_W);
w('         runtime scout %.0f min, research %.0f min  (target %d min)  -> %s\n', ...
  EA.t_scout_min, EA.t_research_min, A.runtime_target_min, pass(min(EA.t_scout_min,EA.t_research_min) >= A.runtime_target_min));
w('ATLAS-M  pack %.1f Wh usable | avg power %.1f W | runtime %.0f min (target %d) -> %s\n', ...
  EC.E_usable_Wh, EC.P_avg_W, EC.t_min, C.runtime_target_min, pass(EC.t_min >= C.runtime_target_min));
w('         full recharge of %d agents costs %.1f Wh = %.0f%% of carrier usable energy\n', ...
  C.agents_carried, EC.E_charge_agents_Wh, 100*EC.E_charge_agents_Wh/EC.E_usable_Wh);
w('         runtime if it also fully recharges both agents: %.0f min\n', EC.t_with_charging_min);
w('         peak dock charge power: %.1f W\n', EC.P_charge_peak_W);

%% ---------------- Plots ----------------
f1 = figure('Visible','off','Position',[100 100 900 380]);
subplot(1,2,1); plot_req(RA, 'ATLAS-S: per-wheel requirement');
subplot(1,2,2); plot_req(RC, 'ATLAS-M: per-wheel requirement');
exportgraphics(f1, fullfile(outDir, 'torque_speed_requirements.png'), 'Resolution', 150);

f2 = figure('Visible','off','Position',[100 100 900 380]);
capA = linspace(1000, 5000, 50);  capC = linspace(4000, 20000, 50);
tA = (A.battery.nominal_voltage*capA/1000*A.battery.usable_fraction)./EA.P_scout_W*60;
tC = (C.battery.nominal_voltage*capC/1000*C.battery.usable_fraction)./EC.P_avg_W*60;
subplot(1,2,1); plot(capA, tA, 'LineWidth', 1.5); hold on; yline(A.runtime_target_min,'--','target');
xline(A.battery.capacity_mAh, ':', 'selected'); grid on;
xlabel('Pack capacity (mAh, 2S)'); ylabel('Runtime (min)'); title('ATLAS-S scout runtime');
subplot(1,2,2); plot(capC, tC, 'LineWidth', 1.5); hold on; yline(C.runtime_target_min,'--','target');
xline(C.battery.capacity_mAh, ':', 'selected'); grid on;
xlabel('Pack capacity (mAh, 4S LiFePO4)'); ylabel('Runtime (min)'); title('ATLAS-M runtime (no agent charging)');
exportgraphics(f2, fullfile(outDir, 'runtime_vs_capacity.png'), 'Resolution', 150);

S = struct('agent', RA, 'carrier', RC, 'energy_agent', EA, 'energy_carrier', EC, 'carrier_loaded_mass_kg', mC);
fidj = fopen(fullfile(outDir, 'sizing_results.json'), 'w'); fprintf(fidj, '%s', jsonencode(S, 'PrettyPrint', true)); fclose(fidj);
fprintf('\nResults written to %s\n', outDir);

%% ======================= local functions =======================
function R = size_vehicle(V, m, cases, g, Crr, mu, eta_motor, extra)
  if isscalar(extra), extra = repmat(extra, 1, numel(cases)); end
  n = V.driven_wheels; r = V.r;
  N_driven = m*g*V.frac_on_driven;
  R = struct('case', {}, 'force_N', {}, 'T_wheel_Nm', {}, 'T_req_Nm', {}, 'rpm', {}, ...
             'P_mech_W', {}, 'P_elec_W', {}, 'I_motor_A', {}, 'traction_ok', {});
  for k = 1:numel(cases)
    c = cases(k); th = deg2rad(c.th);
    F = m*(c.a + g*sin(th) + Crr*g*cos(th)) * extra(k);
    Tw = F*r/n;                                   % output torque per wheel
    Treq = Tw * V.torque_safety_factor;           % with margin
    w = c.v/r;  rpm = w*60/(2*pi);
    Pm = Tw*w;  Pe = Pm/(V.gearbox_efficiency*eta_motor);
    I = Pe / V.battery.min_voltage;               % worst case at low battery
    R(k) = struct('case', c.name, 'force_N', F, 'T_wheel_Nm', Tw, 'T_req_Nm', Treq, 'rpm', rpm, ...
                  'P_mech_W', Pm, 'P_elec_W', Pe, 'I_motor_A', I, 'traction_ok', F <= mu*N_driven*cos(th));
  end
end

function E = energy_agent(A, g, Crr, eta_motor)
  Pdrive = A.mass_loaded_kg*g*Crr*A.max_speed_mps/(A.gearbox_efficiency*eta_motor);
  Pe = A.electronics_power_W;
  base = Pe.mcu_and_radio + Pe.motor_drivers_idle;
  E.P_drive_cruise_W = Pdrive;
  E.P_scout_W    = base + Pe.sensors_scout    + A.duty_cycle_driving*Pdrive;
  E.P_research_W = base + Pe.sensors_research + A.duty_cycle_driving*Pdrive;
  E.E_pack_Wh    = A.battery.nominal_voltage*A.battery.capacity_mAh/1000;
  E.E_usable_Wh  = E.E_pack_Wh*A.battery.usable_fraction;
  E.t_scout_min    = E.E_usable_Wh/E.P_scout_W*60;
  E.t_research_min = E.E_usable_Wh/E.P_research_W*60;
end

function E = energy_carrier(C, m, g, Crr, eta_motor, A)
  Pdrive = m*g*Crr*C.max_speed_mps/(C.gearbox_efficiency*eta_motor);
  Pe = C.electronics_power_W;
  E.P_drive_cruise_W = Pdrive;
  E.P_avg_W = Pe.sbc + Pe.mcu + Pe.lidar + Pe.dock_actuators_avg + C.duty_cycle_driving*Pdrive;
  E.E_pack_Wh   = C.battery.nominal_voltage*C.battery.capacity_mAh/1000;
  E.E_usable_Wh = E.E_pack_Wh*C.battery.usable_fraction;
  E.t_min = E.E_usable_Wh/E.P_avg_W*60;
  eta_charger = 0.85;
  E.E_charge_agents_Wh = C.agents_carried * A.battery.nominal_voltage*A.battery.capacity_mAh/1000 / eta_charger;
  E.t_with_charging_min = max(E.E_usable_Wh - E.E_charge_agents_Wh, 0)/E.P_avg_W*60;
  E.P_charge_peak_W = C.agents_carried * A.battery.max_voltage * C.agent_charge_current_A / eta_charger;
end

function print_vehicle(w, name, m, R)
  w('\n=== %s  (mass %.2f kg) ===\n', name, m);
  w('%-36s %8s %10s %10s %7s %8s %8s %9s\n', 'Load case', 'F [N]', 'Tw [N.m]', 'Treq[N.m]', 'rpm', 'Pe [W]', 'I [A]', 'traction');
  for k = 1:numel(R)
    w('%-36s %8.2f %10.3f %10.3f %7.1f %8.2f %8.2f %9s\n', R(k).case, R(k).force_N, R(k).T_wheel_Nm, ...
      R(k).T_req_Nm, R(k).rpm, R(k).P_elec_W, R(k).I_motor_A, pass(R(k).traction_ok));
  end
  [Tmax, iT] = max([R.T_req_Nm]);  rpmMax = max([R.rpm]);
  w('-> Gearmotor spec per wheel: continuous output torque >= %.3f N.m (%.2f kg.cm) [%s],\n', Tmax, Tmax/0.0981, R(iT).case);
  w('   loaded output speed >= %.0f rpm (no-load ~%.0f rpm at nominal voltage),\n', 1.25*rpmMax, 1.6*rpmMax);
  w('   stall torque >= %.3f N.m. Driver: continuous >= %.1f A/ch AND >= measured motor stall current.\n', ...
    3*Tmax, max(2*max([R.I_motor_A]), 1.0));
end

function s = pass(tf), if tf, s = 'PASS'; else, s = 'FAIL'; end, end

function plot_req(R, ttl)
  hold on; grid on;
  for k = 1:numel(R)
    plot(R(k).rpm, R(k).T_wheel_Nm, 'o', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', R(k).case);
    plot(R(k).rpm, R(k).T_req_Nm, 'x', 'MarkerSize', 9, 'LineWidth', 1.5, 'HandleVisibility', 'off');
  end
  xlabel('Wheel speed (rpm)'); ylabel('Torque per wheel (N.m)'); title(ttl);
  legend('Location', 'northoutside', 'FontSize', 7); text(0.02, 0.05, 'o = load,  x = with safety factor', 'Units', 'normalized', 'FontSize', 8);
end
