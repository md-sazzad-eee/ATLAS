%% Dock contact Joule heating — axisymmetric FE check (PDE Toolbox)
% Question (blueprint §6 Board C): does a charging contact need temperature monitoring,
% and what can the printed contact block be made of?
%
% Model: one pogo pin (brass, r = tip radius) standing on the carrier PCB (FR4 + 1 oz Cu,
% homogenised). ALL interface heat I^2*(Rc + Rpin) enters the pin tip (conservative: in
% reality ~half flows into the agent pad). Natural convection on every exposed surface,
% board edge adiabatic (conservative), 40 degC inside the enclosure.
% The problem is linear, so we solve once for 1 W and scale: dT = P * theta.
%
% Cross-check target for the COMSOL model in 05_simulation/comsol/.

clear; close all;
here = fileparts(mfilename('fullpath'));
P  = jsondecode(fileread(fullfile(here,'..','..','00_requirements','atlas_params.json')));
Cn = P.dock.contacts;
r_pin = Cn.pin_tip_diameter_mm/2*1e-3;  L_pin = 8e-3;       % protrusion + barrel above board
r_brd = 12e-3;  t_brd = 1.6e-3;
k_pin = 110;                                               % brass
k_brd = (385*35e-6 + 0.3*(t_brd-35e-6))/t_brd;             % in-plane homogenised FR4+Cu
h = 10; Tamb = 40;

model = createpde('thermal','steadystate-axisymmetric');
R1 = [3 4 0 r_pin r_pin 0, 0 0 L_pin L_pin]';                 % pin: r in [0,r_pin], z in [0,L_pin]
R2 = [3 4 0 r_brd r_brd 0, L_pin L_pin L_pin+t_brd L_pin+t_brd]';
gd = [R1 R2]; ns = char('pin','brd')'; sf = 'pin+brd';
[dl, bt] = decsg(gd, sf, ns);
g = geometryFromEdges(model, dl);
generateMesh(model, 'Hmax', 0.25e-3);

% identify faces by a point inside each
fp = nearestFace(g, [r_pin/2, L_pin/2]);  fb = nearestFace(g, [r_brd/2, L_pin+t_brd/2]);
thermalProperties(model, 'Face', fp, 'ThermalConductivity', k_pin);
thermalProperties(model, 'Face', fb, 'ThermalConductivity', k_brd);
eTip  = nearestEdge(g, [r_pin/2, 0]);
eSide = nearestEdge(g, [r_pin, L_pin/2]);
eTopB = nearestEdge(g, [(r_pin+r_brd)/2, L_pin]);
eBotB = nearestEdge(g, [r_brd/2, L_pin+t_brd]);
P1 = 1.0;                                                   % W, for the unit solution
thermalBC(model, 'Edge', eTip,  'HeatFlux', P1/(pi*r_pin^2));
thermalBC(model, 'Edge', [eSide eTopB eBotB], 'ConvectionCoefficient', h, 'AmbientTemperature', 0);
res = solve(model);
theta = max(res.Temperature);                               % K/W  (ambient = 0 in the unit solution)
fprintf('Pin-tip thermal resistance to ambient: %.0f K/W\n', theta);

% ---- sweep current and contact resistance
Rpin = 0.020;                                               % pin bulk + spring (ohm)
I  = linspace(0.25, 3, 60);
Rc = [0.010 0.030 0.100 0.200];                             % new, worst-case EOL (params), worn, contaminated
f = figure('Visible','off','Position',[100 100 760 420]); hold on; grid on;
for k = 1:numel(Rc)
    Tmax = Tamb + theta*I.^2*(Rc(k)+Rpin);
    plot(I, Tmax, 'LineWidth', 1.5, 'DisplayName', sprintf('R_c = %.0f m\\Omega', 1e3*Rc(k)));
end
yline(55,'--','PLA softening ~55 \circC','HandleVisibility','off'); yline(80,'--','PETG/ASA limit ~80 \circC','HandleVisibility','off');
xline(P.carrier.agent_charge_current_A, ':', 'design charge current','HandleVisibility','off');
xline(Cn.max_current_A, ':', 'pin rating','HandleVisibility','off');
xlabel('Charge current per contact (A)'); ylabel('Max pin temperature (\circC)');
title(sprintf('Dock contact heating (\\theta = %.0f K/W, T_{amb} = %d \\circC)', theta, Tamb));
legend('Location','northwest'); ylim([Tamb 140]);
exportgraphics(f, fullfile(here,'results','contact_heating.png'), 'Resolution', 150);

% ---- design-point report
Id = P.carrier.agent_charge_current_A;  Rw = Cn.contact_resistance_mohm*1e-3;
T_design = Tamb + theta*Id^2*(Rw+Rpin);
T_rating = Tamb + theta*Cn.max_current_A^2*(Rw+Rpin);
T_worn   = Tamb + theta*Cn.max_current_A^2*(0.2+Rpin);
I_PLA    = sqrt((55-Tamb)/(theta*(Rw+Rpin)));
I_PLA_worn = sqrt((55-Tamb)/(theta*(0.2+Rpin)));
fprintf('Design point  (%.1f A, %.0f mOhm): Tmax = %.1f degC\n', Id, Rw*1e3, T_design);
fprintf('Pin rating    (%.1f A, %.0f mOhm): Tmax = %.1f degC\n', Cn.max_current_A, Rw*1e3, T_rating);
fprintf('Contaminated  (%.1f A, 200 mOhm): Tmax = %.1f degC\n', Cn.max_current_A, T_worn);
fprintf('Current at which a PLA block reaches 55 degC: %.2f A (nominal Rc), %.2f A (200 mOhm)\n', I_PLA, I_PLA_worn);
R = struct('theta_K_per_W',theta,'T_design_C',T_design,'T_at_rating_C',T_rating,'T_contaminated_C',T_worn, ...
    'I_PLA_limit_A',I_PLA,'I_PLA_limit_contaminated_A',I_PLA_worn,'assumptions', ...
    'all heat into pin, adiabatic board edge, h=10 W/m2K, Tamb=40 C, Rpin=20 mOhm');
fid = fopen(fullfile(here,'results','contact_heating.json'),'w'); fprintf(fid,'%s',jsonencode(R,'PrettyPrint',true)); fclose(fid);
