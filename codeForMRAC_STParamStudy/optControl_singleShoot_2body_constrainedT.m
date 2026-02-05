function optControl_singleShoot_2body
% optControl_singleShoot_2body
% Solves a single-shootingConstT optimal control problem
% The Hamiltonian system (state + costate) is integrated and fsolve is used
% to find the initial costates that satisfy the terminal state constraint.
%
% Notes:
% - State X = [x; y; z; vx; vy; vz; m] (Position, Velocity, Mass)
% - Control U = [ux; uy; uz; u_throttle] (Thrust direction unit vector and throttle)
% - Costates L = [lambda_r; lambda_v; lambda_m] (7x1 vector)
% - Problem: drive x0 -> xf in time tf minimizing fuel consumption
%   (Bang-bang control via smoothing strategy)
%
% This script expects helper functions in the path:
% - odeDynAndLag_constT : returns symbolic xdot and Ldot expressions (dynamics and costate ODEs)
% - hamiltonian_odeConstT : evaluates the combined ODE for [lambda; x] given numeric z
% - shootingConstT        : residual function for terminal constraints, used by fsolve

%%
close all; clear all; clc
% Parameters
mu = 3.98600;     % Earth [m^3/s^2]
at = 7000;             % semi-major axis [m]
et = 0.1;                % eccentricity
it = deg2rad(30);        % inclination
Omega_t = deg2rad(40);   % RAAN
omega_t = deg2rad(60);   % argument of periapsis
theta_f = deg2rad(120);  % true anomaly

% Compute terminal state
[xf, rf, vf] = StatesInECI( ...
    theta_f, at, et, it, Omega_t, omega_t, mu);

%% Problem data
close all; clear all; clc
r0 = 780+6378;
rf = 1770+6378;
muVal = 398600;
aTrans = (r0 + rf)/2;
period = 2*pi*sqrt( (aTrans^3) /muVal  );
tf = period*0.5;                % Final time (seconds)

x0 = [r0 0 0 0 sqrt(muVal/r0) 0 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]';             % Desired terminal state at t = tf (Mass is free)

% Initial guess for costates (at t=0). fsolve will update this.
% lambda0_guess = [0; 0; 0; 0; 0; 0; 0];
lambda0_guess = 1e-5*ones(length(x0),1);

%% Define symbolic variables for deriving equations
% State symbols (position and velocity)
syms x y z vx vy vz n muEarth ...
    ax ay az u mC Tmax Isp g0 real

% Build state and control symbolic vectors for clarity
X = [x; y; z; vx; vy; vz; mC];            % symbolic state vector (4x1)
Xnum = sym('x', [length(x0) 1],'real');
U = [ax; ay; az; u];

% Define the dynamics f = dX/dt (Two-Body Keplerian dynamics with variable mass)
f = [vx; ...
    vy; ...
    vz; ...
    -x*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + ax*(Tmax/mC)*u; ...
    -y*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + ay*(Tmax/mC)*u; ...
    -z*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + az*(Tmax/mC)*u; ...
    -(Tmax/Isp*g0)*u; ...
    ];
% Symbolic costate vector (7x1)
Lvec = sym('L', [length(x0) 1],'real');

%% Build symbolic ODEs for states and costates using helper
optUSet =[ - Lvec(4)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
             - Lvec(5)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...  
             - Lvec(6)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...  
             U(end)];

V = U(end)'*U(end); % the instantaneous cost on the control (scalar).
% The function odeDynAndLag should return symbolic expressions for xdot and Ldot
[star_xdot, star_Ldot] = odeDynAndLag_constT(Lvec, X, Xnum, U, f, V, optUSet);

% Compose the equations array:
% - First the costate dynamics (Ldot)
% - Then the state dynamics (xdot)
% This ordering matches how we will stack z = [lambda; x] for integration.
eqns = [
    (star_Ldot).',         % costate ODEs (row)
    (star_xdot).'          % state ODEs (row)
    ];
eqns = simplify(eqns);

% Substitute the numeric value for mean motion to simplify expressions
uTest = 1;
eqns = subs( (eqns), muEarth, muVal);
eqns = subs( (eqns), Tmax, 425);
eqns = subs( (eqns), Isp, 230);
eqns = subs( (eqns), g0, 9.81);
eqns = subs( (eqns), U(end), uTest);
% eqns = subs( (eqns), mC, 1000);

eqns = simplify(eqns);
%% Build a function handle that substitutes numeric z = [lambda; x] into the
% symbolic eqns. This handle is passed to the ODE evaluator and shootingConstT.
funcSubs = @(z) subs(eqns, [Lvec, Xnum], [z(1:length(x0)), z((length(x0)+1):end)] );

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
% dzdt = hamiltonian_ode(0, ones(length(x0)+length(x0), 1), funcSubs);
ds = hamiltonian_odeConstT(0, [x0;x0*0.1], muVal, 425, 230, 9.81, 1, uTest);

%check 
dsDiff = double( funcSubs( [x0;x0*0.1]) ) - ds;

muEarth=muVal;
Tmax=425;
Isp=230; 
g0=9.81;
epsilon=1;
%% Solve two-point boundary value problem via shootingConstT
% First try a single invocation of shootingConstT with the initial guess
F = shootingConstT(lambda0_guess, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon);

%%  Use particleswarm to find lambda0 that makes the terminal state match xf
objfun = @(lam0) norm(shootingConstT(lam0', x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon))^2;
nvars = length(lambda0_guess);
% lb = -[1e-2*ones(6,1)', 1e2*ones(nvars-6,1)']'; % Lower bounds (adjust as needed)
% ub =  [1e-2*ones(6,1)', 1e2*ones(nvars-6,1)']'; % Upper bounds (adjust as needed)
lb = -1e-5*ones(nvars,1);
ub =  1e-5*ones(nvars,1);
opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 3*nvars, 'MaxIterations', 90); %, "UseParallel", true);
lambda0_guess = particleswarm(objfun, nvars, lb, ub, opts);
lambda0_guess = lambda0_guess';
%% Now use fsolve to find lambda0 that makes the terminal state match xf
lambda0 = fsolve(@(lam0) shootingConstT(lam0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon), ...
    lambda0_guess, ...
    optimoptions('fsolve', 'Display', 'iter'));
    % optimoptions('fsolve', 'Display', 'iter', 'MaxIterations', 90));

%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
z0 = [lambda0; x0];
[t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), [0 tf], z0);

% Extract state and costate trajectories from integrated z
x = z(:, (length(x0)+1):end);               % states (columns correspond to [x y vx vy])
lambda = z(:, 1:length(x0));            % costates (columns correspond to [L(1) L(2) L(3) L(4)])

% Compute control from costates. In this formulation the optimal control is
u = zeros(length(t), 4);
epsilon = 1;
for i = 1:length(t)
    L_t = lambda(i, :)';
    x_t = x(i, :)';
    
    % Direction (from optUSet structure: -L_v / norm(L_v))
    L_v_norm = sqrt(L_t(4)^2 + L_t(5)^2 + L_t(6)^2);
    if L_v_norm ~= 0
        u_dir = -L_t(4:6)' / L_v_norm;
    else
        u_dir = [0 0 0];
    end
    
    % Throttle (Switching function)
    rho = 1 - (Isp * g0 * L_v_norm)/(x_t(7)) - L_t(7);
    uSwitch = 0.5 - rho/(2*epsilon);
    if rho > epsilon
        uSwitch = 0;
    elseif rho < - epsilon
        uSwitch = 1;
    end
    
    u(i, :) = [u_dir, uSwitch];
end

%% Plot results: positions and control history
figure;
subplot(3, 1, 1)
plot(t, x(:, 1), 'LineWidth', 1.5), ylabel('x_1'), grid on
title('State and Control Histories')

subplot(3, 1, 2)
plot(t, x(:, 2), 'LineWidth', 1.5), ylabel('x_2'), grid on

subplot(3, 1, 3)
plot(t, u, 'LineWidth', 1.5), ylabel('u'), xlabel('Time (s)'), grid on

% figure
% plot( x(:, 1), x(:, 2), 'LineWidth', 1.5 )
% grid on

%% Animation
figure;
set(gcf, 'Color', 'w');
hold on; axis equal; grid on;
% Circle parameters
h = 0; % x-coordinate of center
k = 0; % y-coordinate of center

% Implicit equation: (x-h)^(2) + (y-k)^(2) - r^(2) = 0

xlabel('x (m)'); ylabel('y (m)');
title('Satellite Maneuvering Animation');

% Plot the target state (origin)
plot(-rf, 0, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');

% Create animated line for trajectory and marker for satellite
hTraj = animatedline('LineWidth', 1.5, 'Color', 'b', 'DisplayName', 'Trajectory');
hSat = plot(x(1, 1), x(1, 2), 'k.', 'MarkerSize', 20, 'DisplayName', 'Satellite');

% Calculate scaling for thrust vector visualization
% We want the max thrust vector to correspond to roughly 15% of the plot span
x_span = max(x(:,1)) - min(x(:,1));
y_span = max(x(:,2)) - min(x(:,2));
avg_span = (x_span + y_span) / 2;
if avg_span == 0, avg_span = 1; end

% u is [nx, ny, nz, throttle]
% Thrust vector components for plotting: direction * throttle
u_vec_plot = u(:, 1:3) .* u(:, 4);
u_mag = u(:, 4); % Throttle is the magnitude factor (assuming unit direction)

max_u = max(u_mag);
if max_u == 0, scale_inv = 1; else, scale_inv = (0.15 * avg_span) / max_u; end

% Initialize thrust vector (scaled)
% Plotting x-y components of thrust
hThrust = quiver(x(1, 1), x(1, 2), u_vec_plot(1, 1)*scale_inv, u_vec_plot(1, 2)*scale_inv, ...
    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'DisplayName', 'Thrust Direction');

legend('show', 'Location', 'best');

% Set axis limits with some padding
x_min = min(x(:,1)); x_max = max(x(:,1));
y_min = min(x(:,2)); y_max = max(x(:,2));
dx = x_max - x_min; if dx==0, dx=1; end
dy = y_max - y_min; if dy==0, dy=1; end
xlim([x_min - 0.2*dx, x_max + 0.2*dx]);
ylim([y_min - 0.2*dy, y_max + 0.2*dy]);

% Setup Video Writer
videoFilename = 'satellite_maneuver.mp4';
vWriter = VideoWriter(videoFilename, 'MPEG-4');
vWriter.FrameRate = 30; % Adjust frame rate as needed
open(vWriter);

% Loop through time steps to animate
% Depending on the number of steps in 't', we might want to skip frames to keep video short
nSteps = length(t);
frameStep = max(1, floor(nSteps / 300)); % Target approx 300 frames max (~10s at 30fps)

for k = 1:frameStep:nSteps
    addpoints(hTraj, x(k, 1), x(k, 2));
    set(hSat, 'XData', x(k, 1), 'YData', x(k, 2));

    % Update thrust vector
    set(hThrust, 'XData', x(k, 1), 'YData', x(k, 2), ...
        'UData', u_vec_plot(k, 1)*scale_inv, 'VData', u_vec_plot(k, 2)*scale_inv);

    drawnow;

    viscircles([0 0; 0 0],[r0 rf])
    axis equal;
    % Capture frame for video
    frame = getframe(gcf);
    writeVideo(vWriter, frame);
end

addpoints(hTraj, x(end, 1), x(end, 2));
set(hSat, 'XData', x(end, 1), 'YData', x(end, 2));
set(hThrust, 'XData', x(end, 1), 'YData', x(end, 2), ...
    'UData', u_vec_plot(end, 1)*scale_inv, 'VData', u_vec_plot(end, 2)*scale_inv);
drawnow;
writeVideo(vWriter, getframe(gcf));

close(vWriter);
fprintf('Animation saved to %s\n', videoFilename);

end

function ds = hamiltonian_odeConstT(t, s, muEarth, Tmax, Isp, g0, epsilon, uFixed)

L = s(1:7);               % states (columns correspond to [x y vx vy])
x = s(8:end);            % costates (columns correspond to [L(1) L(2) L(3) L(4)])
nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));

rho = 1 - (Isp * g0 * nLv)/(x(7)) - L(7);

uSwitch = 0.5 - rho/(2*epsilon);
if rho > epsilon
uSwitch = 0;
elseif rho < - epsilon
uSwitch = 1;
end

if nargin == 8
    uSwitch = uFixed;
end

Tmag = Tmax*uSwitch;

ds = zeros(length(s),1);
ds(1) = -(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2);
ds(2) = -(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2);
ds(3) = -(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2);
ds(4) = -L(1);
ds(5) = -L(2);
ds(6) = -L(3);
ds(7) = -(Tmag*nLv^(1/2))/x(7)^(2);
ds(8) = x(4);
ds(9) = x(5);
ds(10) =x(6);
ds(11) =L(4)/nLv - L(4)*((Tmag)/(x(7)*nLv^(1/2)) - (L(4)^(2)*Tmag)/(x(7)*nLv^(3/2))) - L(4)^3/nLv^(2)...
    - (L(4)*L(5)^(2))/nLv^(2) - (L(4)*L(6)^(2))/nLv^(2) - (muEarth*x(1))/nPos^(3/2)...
    - (L(4)*Tmag)/(x(7)*nLv^(1/2)) + (L(4)*L(5)^(2)*Tmag)/(x(7)*nLv^(3/2)) + (L(4)*L(6)^(2)*Tmag)/(x(7)*nLv^(3/2));
ds(12) =L(5)/nLv - L(5)*((Tmag)/(x(7)*nLv^(1/2)) - (L(5)^(2)*Tmag)/(x(7)*nLv^(3/2))) - L(5)^3/nLv^(2)...
    - (L(4)^(2)*L(5))/nLv^(2) - (L(5)*L(6)^(2))/nLv^(2) - (muEarth*x(2))/nPos^(3/2)...
    - (L(5)*Tmag)/(x(7)*nLv^(1/2)) + (L(4)^(2)*L(5)*Tmag)/(x(7)*nLv^(3/2)) + (L(5)*L(6)^(2)*Tmag)/(x(7)*nLv^(3/2));
ds(13) =L(6)/nLv - L(6)*((Tmag)/(x(7)*nLv^(1/2)) - (L(6)^(2)*Tmag)/(x(7)*nLv^(3/2))) - L(6)^3/nLv^(2)...
    - (L(4)^(2)*L(6))/nLv^(2) - (L(5)^(2)*L(6))/nLv^(2) - (muEarth*x(3))/nPos^(3/2)...
    - (L(6)*Tmag)/(x(7)*nLv^(1/2)) + (L(4)^(2)*L(6)*Tmag)/(x(7)*nLv^(3/2)) + (L(5)^(2)*L(6)*Tmag)/(x(7)*nLv^(3/2));
ds(14) =-(Tmax*g0*uSwitch)/Isp;

end

function F = shootingConstT(lambda0,x0,xf,tf, muEarth, Tmax, Isp, g0, epsilon)

    z0 = [lambda0; x0];
    
    [~,z] = ode45(@(t,z) hamiltonian_odeConstT(t,z, muEarth, Tmax, Isp, g0, epsilon),[0 tf],z0);
    
    x_tf = z(end,end-length(x0)+1:end).';
    F = [x_tf(1:end-1) - xf(1:end-1); z(end,length(x0))];  % terminal error

end

