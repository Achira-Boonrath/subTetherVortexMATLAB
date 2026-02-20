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
close all; clear all; clc
propulsionType = 'chemical'; % Options: 'chemical', 'electric'
%% 

% System Parameters:
%     Tmax = 0.1 N
%     Isp = 3000 s
%     mu = 398600.4418 km3/s2 
%     m0 = 1000 kg
% 
% Initial state (cartesian)
%     x0 = [41797.671293047104, 5214.9122634261175, 184.49268401666282, -0.3809149562899149, 3.052197624291683, 0.10656315631268225]
% Target orbital elements [sma, ecc, inc, ape, ran] (angles in degrees)
%     oet = [42164.0, 0.001, 2.0, 0, 0, 0]
% 
% Unknowns corresponding to solution:
%     u0= [-1.043291413209001e-5, -1.2687541712307916e-5, 0.00022528860478491966, -0.04557325107502537, -0.055654206847527754, 0.9974094630699921, 5.166976235952729e-7, 0.8069069128203797, 9345.775854692882]
% 
%     NOTE: first 7 elements are the initial time costates, the 8th element is the insertion true anomaly, the 9th is the time-of-flight

%%

% Parameters
% muVal = 398600.4418;     % Earth 
% at = 42164.0;             % semi-major axis [km]
% et = 0.001;                % eccentricity
% it = deg2rad(2.0);        % inclination
% Omega_t = deg2rad(0);   % RAAN
% omega_t = deg2rad(0);   % argument of periapsis
% theta_f = deg2rad(46);  % true anomaly
% 
% % Compute terminal state
% [xf, rf, vf] = StatesInECI( ...
%     theta_f, at, et, it, Omega_t, omega_t, muVal);

%% Problem data

r0 = 780+6378;
rf = 5770+6378;
muVal = 398600;
aTrans = (r0 + rf)/2;
period = 2*pi*sqrt( (aTrans^3) /muVal  );
tf = period*0.5;                % Final time (seconds)

x0 = [r0 0 0 0 sqrt(muVal/r0) 0 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]';             % Desired terminal state at t = tf (Mass is free)

%% Problem data
% x0 = [41797.671293047104, 5214.9122634261175, 184.49268401666282, -0.3809149562899149, 3.052197624291683, 0.10656315631268225, 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
% xf = [xf; 800];             % Desired terminal state at t = tf (Mass is free)
% tf = 9345.775854692882*1.25;
% 
% r0 = norm(x0(1:3));
% rf = norm(xf(1:3));
% % Initial guess for costates (at t=0). fsolve will update this.
% % lambda0_guess = [0; 0; 0; 0; 0; 0; 0];
% lambda0_guess = 1e-5*ones(length(x0),1);

%% Define symbolic variables for deriving equations
lambda0_guess = 1e-5*ones(length(x0),1);

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

V = 0.5*U(end)'*U(end); % the instantaneous cost on the control (scalar).
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

% Symbolic substitution based on propulsion type
switch propulsionType
    case 'chemical'
        % chem prop
        eqns = subs( (eqns), Tmax, 425*10);
        eqns = subs( (eqns), Isp, 230);
    case 'electric'
        % electric prop
        eqns = subs( (eqns), Tmax, 0.1);
        eqns = subs( (eqns), Isp, 3000);
end

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

muEarth=muVal;

switch propulsionType
    case 'chemical'
        Tmax=425*10;
        Isp=230; 
    case 'electric'
        Tmax=0.1;
        Isp=3000;
        
        % Additional parameters for electric prop
        at = 42164.0;             % semi-major axis [km]
        et = 0.001;                % eccentricity
        it = deg2rad(2.0);        % inclination
        Omega_t = deg2rad(0);   % RAAN
        omega_t = deg2rad(0);   % argument of periapsis
end 

g0=9.81;
epsilon=1;
ds = hamiltonian_odeConstT(0, [x0;x0*0.1], muVal, Tmax, Isp, g0, epsilon, uTest);

%check 
dsDiff = double( funcSubs( [x0;x0*0.1]) ) - ds;

%% Solve two-point boundary value problem via shootingConstT
% First try a single invocation of shootingConstT with the initial guess

switch propulsionType
    case 'chemical'
        F = shootingConstT(lambda0_guess, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon);
    case 'electric'
        % Update lambda0_guess for electric case (adds theta_f)
        lambda0_guess = [lambda0_guess; pi/2];
        F = shootingConstTFreeTheta(lambda0_guess,x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
end
%%  Use particleswarm to find lambda0 that makes the terminal state match xf

switch propulsionType
    case 'chemical'
        objfun = @(lam0) norm(shootingConstT(lam0', x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon))^2;
    case 'electric'
        objfun = @(lam0) norm(shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t))^2;
end

nvars = length(lambda0_guess);

switch propulsionType
    case 'chemical'
        lb = -1.0e-3*ones(nvars,1);
        ub =  1.0e-3*ones(nvars,1);
    case 'electric'
        lb = -[1e-0*ones(nvars-1,1)', 0]';
        ub =  [1e-0*ones(nvars-1,1)', pi/2]';
end
opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 100, 'MaxIterations', 90); %, "UseParallel", true);
lambda0_guess = particleswarm(objfun, nvars, lb, ub, opts);
lambda0_guess = lambda0_guess';
%% Now use fsolve to find lambda0 that makes the terminal state match xf
switch propulsionType
    case 'chemical'
        lambda0 = fsolve(@(lam0) shootingConstT(lam0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon), ...
            lambda0_guess, ...
            optimoptions('fsolve', 'Display', 'iter'));
    case 'electric'
        lambda0 = fsolve(@(lam0) shootingConstTFreeTheta(lam0, x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t), ...
            lambda0_guess, ...
            optimoptions('fsolve', 'Display', 'iter'));
end

%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
% z0 = [lambda0; x0];
switch propulsionType
    case 'chemical'
        z0 = [lambda0; x0];
    case 'electric'
        z0 = [lambda0(1:end-1); x0];
end

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

    viscircles([0 0; 0 0],[r0 rf],'LineStyle','--');
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

function F = shootingConstTFreeTheta(lambda0,x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)

    z0 = [lambda0(1:end-1); x0];
    theta_f = lambda0(end);
    [~,z] = ode45(@(t,z) hamiltonian_odeConstT(t,z, muEarth, Tmax, Isp, g0, epsilon),[0 tf],z0);
    
    zdot0 = hamiltonian_odeConstT(0, z0, muEarth, Tmax, Isp, g0, epsilon);
    lambdaHat0 = [lambda0(1:end-1); - lambda0(1:end-1)'*zdot0(end-length(x0)+1:end)];
    unitCostate = norm(lambdaHat0) -1;

    [xf, rf, vf] = StatesInECI(theta_f, at, et, it, Omega_t, omega_t, muVal);
    cFinal = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, muVal);

    x_tf = z(end,end-length(x0)+1:end).';
    % F = [x_tf(1:end-1) - xf; z(end,length(x0));...
    %     cFinal];  % terminal error
    F = [x_tf(1:end-1) - xf; z(end,length(x0));...
        cFinal];  % terminal error

end

function F = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, mu)

F = [-(at*(et^2 - 1)*(et*cos(Omega_t)*cos(omega_t) + cos(Omega_t)*sin(omega_t)*sin(theta_f) - et*sin(Omega_t)*cos(it)*sin(omega_t) + sin(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f)) + at*cos(theta_f)*(cos(Omega_t)*cos(omega_t) - sin(Omega_t)*cos(it)*sin(omega_t))*(et^2 - 1))/(et*cos(theta_f) + 1)^2;...
-(at*(et^2 - 1)*(et*sin(Omega_t)*cos(omega_t) + sin(Omega_t)*sin(omega_t)*sin(theta_f) + et*cos(Omega_t)*cos(it)*sin(omega_t) - cos(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f)) + at*cos(theta_f)*(sin(Omega_t)*cos(omega_t) + cos(Omega_t)*cos(it)*sin(omega_t))*(et^2 - 1))/(et*cos(theta_f) + 1)^2;...
                                                                                                                                                                                                          -(at*sin(it)*(sin(omega_t - theta_f) + et*sin(omega_t))*(et^2 - 1))/(et*cos(theta_f) + 1)^2;...
                                                                                 ((mu/at)^(1/2)*(cos(Omega_t)*sin(omega_t)*sin(theta_f) - cos(Omega_t)*cos(omega_t)*cos(theta_f) + sin(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f) + sin(Omega_t)*cos(it)*cos(theta_f)*sin(omega_t)))/(1 - et^2)^(1/2);...
                                                                                -((mu/at)^(1/2)*(sin(Omega_t)*cos(omega_t)*cos(theta_f) - sin(Omega_t)*sin(omega_t)*sin(theta_f) + cos(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f) + cos(Omega_t)*cos(it)*cos(theta_f)*sin(omega_t)))/(1 - et^2)^(1/2);...
                                                                                                                                                                                                                                     -(sin(omega_t + theta_f)*sin(it)*(mu/at)^(1/2))/(1 - et^2)^(1/2)];
end