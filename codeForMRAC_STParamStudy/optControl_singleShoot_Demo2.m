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
% propulsionType = 'electric'; % Options: 'chemical', 'electric'
%%

%% Problem data

% Symbolic substitution based on propulsion type
muVal = 398600.4418;     % Earth
uTest = 1;
minT = 1;
switch propulsionType
    case 'chemical'
        % chem prop

        r0 = 780+6378;
        rf = 2970+6378;
        aTrans = (r0 + rf)/2;
        period = 2*pi*sqrt( (aTrans^3) /muVal  );
        tf = period*0.5;                % Final time (seconds)

        x0 = [r0 0 0 0 sqrt(muVal/r0) 0 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
        xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]';             % Desired terminal state at t = tf (Mass is free)

                % if minT ==0
        lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
            ]'; % thetaf
        ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
            ]';% thetaf
        % else
        %     lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
        %         0,    -1]'; % thetaf
        %     ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
        %         2*pi, 1]';% thetaf
        % end

        % lambda0_guess = ub;
        lambda0_guess = [0.060776702600281   0.000298627734904   1.000667203169005   0.500000000000000   0.500000000000000   0.069330582561975   0.672142996295746]';

    case 'electric'
        % electric prop

        x0 = [41797.671293047104, 5214.9122634261175, 184.49268401666282, -0.3809149562899149, 3.052197624291683, 0.10656315631268225, 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
        disp("initial radius")
        norm( x0(1:3) )

        tf = 11682.21981822656;%9345.775854692882*1.25;
        r0 = norm(x0(1:3));

        % if minT ==0
        lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
            0]'; % thetaf
        ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
            2*pi]';% thetaf
        % else
        %     lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
        %         0,    -1]'; % thetaf
        %     ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
        %         2*pi, 1]';% thetaf
        % end

        lambda0_guess = lb;
end

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
% dzdt = hamiltonian_ode(0, ones(length(x0)+length(x0), 1), funcSubs);

muEarth=muVal;

switch propulsionType
    case 'chemical'
        % Tmax=425*80*1e-3;
        Tmax=40000*1e-3;
        Isp=230;
    case 'electric'
        Tmax=0.1*1e-3; %in kN
        Isp=3000;

        % Additional parameters for electric prop
        at = 42164.0;             % semi-major axis [km]
        rf = at;
        et = 0.001;                % eccentricity
        it = deg2rad(2.0);        % inclination
        Omega_t = deg2rad(0);   % RAAN
        omega_t = deg2rad(0);   % argument of periapsis
end

g0=9.81*1e-3; %in km/s^2
epsilon=1;
ds = hamiltonian_odeConstT(0, [x0*10.1;x0], muVal, Tmax, Isp, g0, epsilon, uTest);

%% Solve two-point boundary value problem via shootingConstT
% First try a single invocation of shootingConstT with the initial guess

switch propulsionType
    case 'chemical'
        F = shootingConstT(lambda0_guess, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon);
    case 'electric'
        % Update lambda0_guess for electric case (adds theta_f)
        if minT ==0
            F = shootingConstTFreeTheta(lambda0_guess,x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        else
            F = shootingConstTFreeTheta_minT(lambda0_guess,x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        end
        diagF_Inv = diag(1);

        % F((log10(abs(F))< 0)) = 1;
        % % diagF_Inv = diag(1./abs(F) ).^2;
        % diagF_Inv = diag(1./abs(F) );
end
%%  Use particleswarm to find lambda0 that makes the terminal state match xf

switch propulsionType
    case 'chemical'
        objfun = @(lam0) norm(shootingConstT(lam0', x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon))^2;
    case 'electric'
        % objfun = @(lam0) norm(shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t))^2;
        if minT ==0
            objfun = @(lam0) shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)' ...
                * diagF_Inv * shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        else
            objfun = @(lam0) shootingConstTFreeTheta_minT(lam0', x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)' ...
                * diagF_Inv * shootingConstTFreeTheta_minT(lam0', x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        end
end

nvars = length(lambda0_guess);

% opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 500, 'MaxIterations', 3, "UseParallel", true);
% opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 500, 'MaxIterations', 50, "UseParallel", true);
% lambda0_guess = particleswarm(objfun, nvars, lb, ub, opts);
% lambda0_guess = lambda0_guess';
%% Now use fsolve to find lambda0 that makes the terminal state match xf
switch propulsionType
    case 'chemical'
        lambda0 = fsolve(@(lam0) shootingConstT(lam0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon), ...
            lambda0_guess, ...
            optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 3000));
    case 'electric'
        if minT == 0
            L0_guess = costate_from_D( lambda0_guess(1:end-1) );
            lambda0_guess = [L0_guess(2:end); lambda0_guess(end)] ;

            lambda0 = fsolve(@(lam0) shootingConstTFreeTheta_Trust(lam0, x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0_guess(1)), ...
                lambda0_guess, ...
                optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 3000));
        else
            % L0_guess = costate_from_D( lambda0_guess(1:end-2) );
            % lambda0_guess = [L0_guess(2:end); lambda0_guess(end-1:end)] ;

            % lambda0_guess = u0';
            % L0_guess(1) = 1.1745500794600847e-7;

            L0_guess= costate_from_D( lambda0_guess(1:end-1) );
            % Call helper to integrate and compute terminal constraints
            [~, ~, ~, ~, H, tDone]= integrateAndComputeTerminal_minT([L0_guess(2:end); x0], 1e+6, muEarth, Tmax, Isp, g0, epsilon, L0_guess, lambda0_guess(end), ...
                at, et, it, Omega_t, omega_t, muVal);

            lambda0_guess = [L0_guess(2:end); lambda0_guess(end); tDone ];

            lambda0 = fsolve(@(lam0) shootingConstTFreeTheta_Trust_minT(lam0, x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0_guess(1)), ...
                lambda0_guess, ...
                optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 2000));
        end
end
lambda0-u0'
%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
% z0 = [lambda0; x0];
switch propulsionType
    case 'chemical'
        z0 = [lambda0; x0];
        [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon ), [0 tf], z0);
    case 'electric'

        if minT ==0
            L0= costate_from_D( lambda0(1:end-1) );
            z0 = [L0(2:end); x0];
            [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1)), [0 tf], z0);
        else
            lambda0 = lambda0(1:end-1);
            L0= costate_from_D( lambda0(1:end-1) );
            z0 = [L0(2:end); x0];

            [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1), 1), [0 tf], z0);
        end
end

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

    [Tmag,rho,uSwitch] = ThrottleSwitchingFunc(Isp , g0, L_v_norm, x_t, L_t, L0(1), epsilon, Tmax);

    u(i, :) = [u_dir, uSwitch];
end

switch propulsionType
    case 'chemical'
        z0 = [lambda0; x0];
    case 'electric'

        theta_f = 0;
        % Semi-latus rectum
        p = at*(1 - et^2);

        % Radius magnitude
        r_mag = p / (1 + et*cos(theta_f));

        % Perifocal position
        r_pf = r_mag * ...
            [ cos(theta_f);
            sin(theta_f);
            0 ];

        v_scale = sqrt(muVal / p);

        v_pf = v_scale * ...
            [ -sin(theta_f);
            et + cos(theta_f);
            0 ];

        periodF = 2*pi*sqrt( (at^3) /muVal  );
end

L0= costate_from_D( lambda0(1:end-1) );
z0F = [L0(2:end); r_pf; v_pf; 1000];
timeFinalF = linspace(0, periodF, 405) ;
[tF, zF] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, 1e-11*Tmax, Isp, g0, epsilon, L0(1)), timeFinalF, z0F, odeset('RelTol', 1e-11,'AbsTol', 1e-11,'Stats','off'));
xF = zF(:, (length(x0)+1):end);

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

plot(xF(:, 1), xF(:, 2), 'k-.', 'DisplayName', 'Target Orbit');

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

legend('show', 'Location', 'bestoutside');

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

    viscircles([0 0],[r0 ],'LineStyle','--');
    % viscircles([0 0],[rf],'LineStyle','-.');
    axis equal;
    % Capture frame for video
    frame = getframe(gcf);
    writeVideo(vWriter, frame);
end

% addpoints(hTraj, x(end, 1), x(end, 2));
% set(hSat, 'XData', x(end, 1), 'YData', x(end, 2));
% set(hThrust, 'XData', x(end, 1), 'YData', x(end, 2), ...
%     'UData', u_vec_plot(end, 1)*scale_inv, 'VData', u_vec_plot(end, 2)*scale_inv);
drawnow;
writeVideo(vWriter, getframe(gcf));

close(vWriter);
fprintf('Animation saved to %s\n', videoFilename);

end

function F = shootingConstTFreeTheta(lambda0,x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)

L0= costate_from_D( lambda0(1:end-1) );

z0 = [L0(2:end); x0];
theta_f = lambda0(end);

% Call helper to integrate and compute terminal constraints
[z, xf, x_tf, cFinal]= integrateAndComputeTerminal(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, ...
    at, et, it, Omega_t, omega_t, muVal);

%compute hamiltonian

F = [x_tf(1:end-1) - xf; z(end,length(x0));...
    cFinal];  % terminal error

end

function F = shootingConstTFreeTheta_Trust(lambda0, x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0)

z0 = [lambda0(1:end-1); x0];
theta_f = lambda0(end);

% Call helper to integrate and compute terminal constraints
[z, xf, x_tf, cFinal] = integrateAndComputeTerminal(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, ...
    at, et, it, Omega_t, omega_t, muVal);

F = [x_tf(1:end-1) - xf; z(end,length(x0));...
    cFinal];  % terminal error

end

function F = shootingConstTFreeTheta_minT(lambda0Aug,x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)

% lambda0 = lambda0Aug(1:end-1);
% tf = 10000*(10^(lambda0Aug(end)));

lambda0 = lambda0Aug;
tf = 1e+6;
L0= costate_from_D( lambda0(1:end-1) );

z0 = [L0(2:end); x0];
theta_f = lambda0(end);

% Call helper to integrate and compute terminal constraints
[z, xf, x_tf, cFinal, H, tDone]= integrateAndComputeTerminal_minT(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, ...
    at, et, it, Omega_t, omega_t, muVal);

%compute hamiltonian

F = [x_tf(1:end-1) - xf; H;...
    cFinal];  % terminal error

end

function F = shootingConstTFreeTheta_Trust_minT(lambda0Aug, x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0)

lambda0 = lambda0Aug(1:end-1);
tf = lambda0Aug(end);
% tf = 10000*(10^(lambda0Aug(end)));

z0 = [lambda0(1:end-1); x0];
theta_f = lambda0(end);

% Call helper to integrate and compute terminal constraints
[z, xf, x_tf, cFinal, H, tDone] = integrateAndComputeTerminal_minT(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, ...
    at, et, it, Omega_t, omega_t, muVal);

F = [x_tf(1:end-1) - xf; z(end,length(x0));...
    norm([z(1,1:length(x0)), L0])-1; cFinal];  % terminal error

end