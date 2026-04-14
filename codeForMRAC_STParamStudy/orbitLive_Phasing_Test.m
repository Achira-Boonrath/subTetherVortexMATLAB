% orbitLive_Phasing_Test.m
% Showcases a rendezvous phasing maneuver between a chaser and a target satellite.
% The chaser alters its initial orbit to a phasing orbit for kOrbit revolutions
% to catch up with the target ahead.

clear; clc; close all;

%% Parameters
muEarth = 398600; % km^3/s^2
orbitSemiMajorAxis0 = 8000; % km, Initial orbit half major-axis
orbitEccentricity0 = 0.0001; % initial orbit eccentricity
orbitArgPeriapsis0 = 20; % initial orbit argument of periapsis (degrees)

targetPhaseAngleDeg = 35; % degrees, target is this far ahead of chaser in Mean Anomaly
kOrbit = 1; % number of phasing orbits the chaser completes

satellitePlotStyle = 'boxwing'; % 'boxwing' or 'square'
showAxesAndGrid = false; % Toggle for x/y ticks and grid lines

% Calculations
T0 = 2*pi * sqrt(orbitSemiMajorAxis0^3 / muEarth);
deltaM = deg2rad(targetPhaseAngleDeg);

% Total time until rendezvous (Target travels kOrbit*2*pi minus its initial lead in Mean Anomaly)
T_total = (kOrbit*2*pi - deltaM) / (2*pi) * T0;

% The chaser must complete kOrbit orbits in exactly T_total time
T_phase = T_total / kOrbit;

% Compute phasing semi-major axis using Kepler's 3rd Law
a_phase = orbitSemiMajorAxis0 * (T_phase / T0)^(2/3);

% Velocity at periapsis in both orbits
rp = orbitSemiMajorAxis0 * (1 - orbitEccentricity0);
v0_peri = sqrt(muEarth * (2/rp - 1/orbitSemiMajorAxis0));
v_phase_peri = sqrt(muEarth * (2/rp - 1/a_phase));

fprintf('================ Phasing Maneuver ================\n');
fprintf('Initial/Target Orbit : a = %.2f km, e = %.4f, w = %.2f deg, Period = %.2f s\n', orbitSemiMajorAxis0, orbitEccentricity0, orbitArgPeriapsis0, T0);
fprintf('Phasing Orbit        : a = %.2f km, Period = %.2f s\n', a_phase, T_phase);
fprintf('Target phase diff    : %.2f deg (Mean Anomaly Head Start)\n', targetPhaseAngleDeg);
fprintf('Orbits to Catch-up   : %d\n', kOrbit);
fprintf('Time to Rendezvous   : %.2f s (%.2f hours)\n', T_total, T_total/3600);
fprintf('Delta-V required     : %.2f km/s (Burn 1), %.2f km/s (Burn 2)\n', abs(v_phase_peri-v0_peri), abs(v_phase_peri-v0_peri));
fprintf('==================================================\n');

%% Trajectory Simulation using ODE45
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
tspan1 = linspace(-T0, 0, 100);                  % Pre-burn
tspan2 = linspace(0, T_total, 350);                % Phasing maneuver
tspan3 = linspace(T_total, T_total + T0/2, 100);   % Post-rendezvous

target_mean_motion = sqrt(muEarth / orbitSemiMajorAxis0^3);
t_all = [tspan1(1:end-1), tspan2(1:end-1), tspan3];

% Rotation matrix for argument of periapsis
omega0 = deg2rad(orbitArgPeriapsis0);
R_omega = [cos(omega0) -sin(omega0); sin(omega0) cos(omega0)];

% --- Target Pre-calculation (Exact Kepler Propagation) ---
M_target = deltaM + target_mean_motion * t_all;
target_x = zeros(1, length(t_all)); target_y = zeros(1, length(t_all));
target_vx = zeros(1, length(t_all)); target_vy = zeros(1, length(t_all));

e0 = orbitEccentricity0;
p0 = orbitSemiMajorAxis0 * (1 - e0^2);
for i = 1:length(t_all)
    M_wrapped = mod(M_target(i), 2*pi);
    E = M_wrapped;
    for j=1:20
        E = E - (E - e0*sin(E) - M_wrapped)/(1 - e0*cos(E));
    end
    nu = 2*atan2(sqrt(1+e0)*sin(E/2), sqrt(1-e0)*cos(E/2));
    r = p0 / (1 + e0*cos(nu));
    pos_peri = [r*cos(nu); r*sin(nu)];
    vel_peri = [-sqrt(muEarth/p0)*sin(nu); sqrt(muEarth/p0)*(e0+cos(nu))];

    pos = R_omega * pos_peri; vel = R_omega * vel_peri;
    target_x(i) = pos(1); target_y(i) = pos(2);
    target_vx(i) = vel(1); target_vy(i) = vel(2);
end

% --- Chaser ODE45 Propagation ---
% Phase 1 (Pre-burn)
% Chaser starts at t = -T0, its Mean Anomaly M = 0 (Periapsis of initial orbit)
M_start = -target_mean_motion * T0;
M_wrapped = mod(M_start, 2*pi);
E_start = M_wrapped;
for j=1:20; E_start = E_start - (E_start - e0*sin(E_start) - M_wrapped)/(1 - e0*cos(E_start)); end
nu_start = 2*atan2(sqrt(1+e0)*sin(E_start/2), sqrt(1-e0)*cos(E_start/2));
r_start = p0 / (1 + e0*cos(nu_start));
p_peri_start = [r_start*cos(nu_start); r_start*sin(nu_start)];
v_peri_start = [-sqrt(muEarth/p0)*sin(nu_start); sqrt(muEarth/p0)*(e0+cos(nu_start))];
state_c1_0 = [R_omega * p_peri_start; R_omega * v_peri_start];
[~, chaser_state1] = ode45(@(t, y) twobody(t, y, muEarth), tspan1, state_c1_0, options);

% Phase 2 (Phasing maneuver, t=0 to T_total)
% Burn 1: exactly at periapsis
pos_peri = [rp; 0];
vel_peri = [0; v_phase_peri];
state_c2_0 = [R_omega * pos_peri; R_omega * vel_peri];
[~, chaser_state2] = ode45(@(t, y) twobody(t, y, muEarth), tspan2, state_c2_0, options);

% Phase 3 (Post-rendezvous from t=T_total to T_total + T0/2)
% Burn 2: exactly at periapsis again
pos_peri = [rp; 0];
vel_peri = [0; v0_peri];
state_c3_0 = [R_omega * pos_peri; R_omega * vel_peri];
[~, chaser_state3] = ode45(@(t, y) twobody(t, y, muEarth), tspan3, state_c3_0, options);

chaser_x = [chaser_state1(1:end-1, 1); chaser_state2(1:end-1, 1); chaser_state3(:, 1)]';
chaser_y = [chaser_state1(1:end-1, 2); chaser_state2(1:end-1, 2); chaser_state3(:, 2)]';
chaser_vx = [chaser_state1(1:end-1, 3); chaser_state2(1:end-1, 3); chaser_state3(:, 3)]';
chaser_vy = [chaser_state1(1:end-1, 4); chaser_state2(1:end-1, 4); chaser_state3(:, 4)]';

%% Setup Animation
figure;
set(gcf, 'Color', 'w');
set(gcf, 'Position',  [0, 0, 1080, 1080]*0.9);
hold on; axis equal;

if exist('showAxesAndGrid', 'var') && showAxesAndGrid
    grid on;
    xlabel('x (km)'); ylabel('y (km)');
else
    set(gca, 'XTick', [], 'YTick', []);
    grid off;
    xlabel(''); ylabel('');
end
% title(sprintf('Satellite Phasing Maneuver (%d Orbits to Rendezvous)', kOrbit));

% Bounding box for axes
max_r = max(max(sqrt(chaser_x.^2 + chaser_y.^2)), max(sqrt(target_x.^2 + target_y.^2)));
x_min = -max_r; x_max = max_r;
y_min = -max_r; y_max = max_r;
dx = x_max - x_min; if dx==0, dx=1; end
dy = y_max - y_min; if dy==0, dy=1; end

% Draw Earth
earthImgFile = 'earth_PNG3.png';
if exist(earthImgFile, 'file')
    [img, ~, alpha] = imread(earthImgFile);
    rEarth = 6378; % km
    hEarth = image('CData', img, 'XData', [-rEarth rEarth], 'YData', [rEarth -rEarth]);
    if ~isempty(alpha)
        set(hEarth, 'AlphaData', alpha);
    end
    set(gca, 'YDir', 'normal');
else
    warning('Earth image file "%s" not found.', earthImgFile);
end

% Plot static orbits
th_plot = linspace(0, 2*pi, 150);
r0_plot = p0 ./ (1 + e0 * cos(th_plot));
pos0_peri = [r0_plot .* cos(th_plot); r0_plot .* sin(th_plot)];
pos0_inertial = R_omega * pos0_peri;
plot(pos0_inertial(1,:), pos0_inertial(2,:), 'b--', 'LineWidth', 1.5,'DisplayName', 'Main Orbit');

% Phasing orbit ellipse plot
if a_phase >= rp
    e_phase = 1 - rp/a_phase;
    p_phase = a_phase * (1 - e_phase^2);
    r_phase_plot = p_phase ./ (1 + e_phase * cos(th_plot));
    peri_pos = [r_phase_plot .* cos(th_plot); r_phase_plot .* sin(th_plot)];
else
    e_phase = rp/a_phase - 1;
    p_phase = a_phase * (1 - e_phase^2);
    r_phase_plot = p_phase ./ (1 + e_phase * cos(th_plot));
    peri_pos = [r_phase_plot .* cos(th_plot + pi); r_phase_plot .* sin(th_plot + pi)];
end
plot_pos = R_omega * peri_pos;
plot(plot_pos(1,:), plot_pos(2,:), 'r--', 'LineWidth', 2.5, 'DisplayName', 'Phasing Orbit');

% Plot Trajectories via Animated Line
hTrajTarget = animatedline('LineWidth', 2.5, 'Color', 'g', 'DisplayName', 'Target');
hTrajChaser = animatedline('LineWidth', 2.5, 'Color', 'b', 'DisplayName', 'Chaser');
% legend('Location', 'best');

% vWriter = VideoWriter('satellite_phasing_maneuver.mp4', 'MPEG-4');
% vWriter.FrameRate = 30;
% vWriter.Quality = 100;

vWriter = VideoWriter('satellite_maneuver_phasing.avi','Motion JPEG AVI');
% vWriter = VideoWriter('satellite_maneuver.mp4','MPEG-4');
vWriter.FrameRate = 20;   % Set to 20 frames per second
% vWriter.Quality = 100;
open(vWriter);
open(vWriter);

params.BodySize = 234;
params.PanelLength = 248;
params.PanelWidth = 222;
avg_span = (dx + dy) / 2;

nSteps = length(t_all);
h_square_t = [];
h_sat_c = [];

for k = 1:nSteps
    xlim([x_min - 0.1*dx, x_max + 0.1*dx]);
    ylim([y_min - 0.1*dy, y_max + 0.1*dy]);

    addpoints(hTrajChaser, chaser_x(k), chaser_y(k));
    addpoints(hTrajTarget, target_x(k), target_y(k));

    theta_rot_c = atan2(chaser_vy(k), chaser_vx(k)) + pi/2;
    theta_rot_t = atan2(target_vy(k), target_vx(k)) + pi/2;

    % Satellite Drawings
    % Target (Green Square)
    sq_half = max(0.01*avg_span, params.BodySize*0.5);
    corners = sq_half * [-1 -1; 1 -1; 1 1; -1 1]';
    Rt = [cos(theta_rot_t), -sin(theta_rot_t); sin(theta_rot_t), cos(theta_rot_t)];
    rot_t = Rt * corners;
    if k == 1
        h_square_t = patch(rot_t(1,:) + target_x(k), rot_t(2,:) + target_y(k), 'g', 'EdgeColor', 'g', 'DisplayName', 'Target');
    else
        set(h_square_t, 'XData', rot_t(1,:) + target_x(k), 'YData', rot_t(2,:) + target_y(k));
    end

    % Chaser (Boxwing)
    if k == 1
        h_sat_c = draw_boxwing_satellite(chaser_x(k), chaser_y(k), theta_rot_c, params);
    else
        h_sat_c = draw_boxwing_satellite(chaser_x(k), chaser_y(k), theta_rot_c, params, h_sat_c);
    end

    drawnow;
    writeVideo(vWriter, getframe(gcf));
end

close(vWriter);
disp('[Success] Phasing maneuver animation generated and saved as satellite_phasing_maneuver.mp4');

%% ODE Helper for Two-body Propagation
function dstate = twobody(~, state, mu)
r = norm(state(1:2));
dstate = [state(3);
    state(4);
    -mu*state(1)/r^3;
    -mu*state(2)/r^3];
end
