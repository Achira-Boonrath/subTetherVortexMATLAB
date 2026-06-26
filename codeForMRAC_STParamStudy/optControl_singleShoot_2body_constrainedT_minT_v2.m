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
% Unknowns corresponding to solution, min T:
u0= [-1.043291413209001e-5, -1.2687541712307916e-5, 0.00022528860478491966, -0.04557325107502537, -0.055654206847527754, 0.9974094630699921, 5.166976235952729e-7, 0.8069069128203797, 9345.775854692882]
% lambda_0 = 1.1745500794600847e-7
% test call: shootingConstT_unified(u0', x0, argsStruct, 1.1745500794600847e-7)

% Unknowns corresponding to solution, min E:
% tf = 11682.21981822656
% lambda_0 = 0.504414962632399
% u0 = [-0.00032088069738992316
%  -0.0003785551362760347
%   0.00505738958019216
%  -2.075153067111718
%  -2.123950091808782
%  28.33270169497307
%   1.1498524710626637e-5
%   0.9774988736510167];

%     NOTE: first 7 elements are the initial time costates, the 8th element is the insertion true anomaly, the 9th is the time-of-flight

%% Problem data

% Symbolic substitution based on propulsion type
muVal = 398600.4418;     % Earth
uTest = 1;
minT = 1 ;
tol = 1e-9;
tf_max = 5e+4;

r0 = 780+6378;
rf = 2000+6378;
aTrans = (r0 + rf)/2;
period = 2*pi*sqrt( (aTrans^3) /muVal  );
tf = period*0.5*1.0;                % Final time (seconds)
xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]'; 
switch propulsionType
    case 'chemical'
        % chem prop
        x0 = [r0 0 0 0 sqrt(muVal/r0) 0 3000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)

        lb =  0*[1, 1, 1, 1, 1, 1, 1,...% costates D
            ]';%
        ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
            ]';% thetaf

        lambda0_guess = lb;

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
       
        lambda0_guess = lb;
end

[eqns, Lvec, Xnum] = buildHamiltonianEquations(x0, muVal, 'electric',uTest, minT);

%% Build a function handle that substitutes numeric z = [lambda; x] into the
% symbolic eqns. This handle is passed to the ODE evaluator and shootingConstT.
funcSubs = @(z) subs(eqns, [Lvec, Xnum], [z(1:length(x0)), z((length(x0)+1):end)] );

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
% dzdt = hamiltonian_ode(0, ones(length(x0)+length(x0), 1), funcSubs);

muEarth=muVal;
% Additional parameters for electric prop
at = 42164.0;             % semi-major axis [km]
rf = at;
et = 0.001;                % eccentricity
it = deg2rad(2.0);        % inclination
Omega_t = deg2rad(0);   % RAAN
omega_t = deg2rad(0);   % argument of periapsis

switch propulsionType
    case 'chemical'
        Tmax=5000*1e-3; %in kN
        Isp=230;
        FixedFinalPos = 1;
    case 'electric'
        Tmax=0.1*1e-3; %in kN
        Isp=3000;
        FixedFinalPos = 0;
end

g0=9.81*1e-3; %in km/s^2
epsilon=1;
ds = hamiltonian_odeConstT(0, [x0*10.1;x0], muVal, Tmax, Isp, g0, epsilon, uTest);

%check
dsDiff = norm( double( funcSubs([x0*10.1;x0]) ) - ds )

%% Solve two-point boundary value problem via shootingConstT
% First try a single invocation of shootingConstT with the initial guess
rho_s = 00;
a = 1+6378;
b = 1300+6378;

argsStruct = struct('xf', xf, 'tf', tf, 'muEarth', muEarth, 'Tmax', Tmax, 'Isp', Isp, 'g0', g0, 'epsilon', epsilon,...
    'at', at, 'et', et, 'it', it, 'Omega_t', Omega_t, 'omega_t', omega_t, 'muVal', muVal,...
    'TrustSolve', 0, 'minT', minT, 'FixedFinalPos', FixedFinalPos, ...
        'rho_s', rho_s, 'a', a, 'b', b, 'tf_max', tf_max);


diagF_Inv = diag(1);
F = shootingConstT_unified(lambda0_guess, x0, argsStruct);
%%  Use particleswarm to find lambda0 that makes the terminal state match xf

objfun = @(lam0) shootingConstT_unified(lam0', x0, argsStruct)' ...
            * diagF_Inv * shootingConstT_unified(lam0', x0, argsStruct);

nvars = length(lambda0_guess);

opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 300, 'MaxIterations', 150, "UseParallel", true);
lambda0_guess = particleswarm(objfun, nvars, lb, ub, opts);
lambda0_guess = lambda0_guess';
%% Now use fsolve to find lambda0 that makes the terminal state match xf
switch propulsionType
    case 'chemical'
        if minT == 0
            L0_guess = costate_from_D( lambda0_guess(1:end) );
            lambda0_guess = [L0_guess(2:end)] ;
        else

            L0_guess= costate_from_D( lambda0_guess(1:end) );
            % Call helper to integrate and compute terminal constraints
            [~, ~, ~, ~, H, tDone]= integrateAndComputeTerminal_minT([L0_guess(2:end); x0], tf_max, muEarth, Tmax, Isp, g0, epsilon, L0_guess, 1e-6, ...
                at, et, it, Omega_t, omega_t, muVal);

            lambda0_guess = [L0_guess(2:end); tDone ];
        end
    case 'electric'
        if minT == 0
            L0_guess = costate_from_D( lambda0_guess(1:end-1) );
            lambda0_guess = [L0_guess(2:end); lambda0_guess(end)] ;
        else
            % L0_guess = costate_from_D( lambda0_guess(1:end-2) );
            % lambda0_guess = [L0_guess(2:end); lambda0_guess(end-1:end)] ;

            % lambda0_guess = u0';
            % L0_guess(1) = 1.1745500794600847e-7;

            L0_guess= costate_from_D( lambda0_guess(1:end-1) );
            % Call helper to integrate and compute terminal constraints
            [~, ~, ~, ~, H, tDone]= integrateAndComputeTerminal_minT([L0_guess(2:end); x0], tf_max, muEarth, Tmax, Isp, g0, epsilon, L0_guess, lambda0_guess(end), ...
                at, et, it, Omega_t, omega_t, muVal);

            lambda0_guess = [L0_guess(2:end); lambda0_guess(end); tDone ];
        end
end

argsStruct.L0 = L0_guess(1);
argsStruct.TrustSolve = 1;
lambda0 = fsolve(@(lam0) shootingConstT_unified(lam0, x0, argsStruct), ...
    lambda0_guess, ...
    optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 9000));
% lambda0-u0'
%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
% z0 = [lambda0; x0];
switch propulsionType
    case 'chemical'
        if minT ==0
        z0 = [lambda0; x0];
        else
        z0 = [lambda0(1:end-1); x0];
        end
    case 'electric'
        z0 = [lambda0(1:end-1); x0];
end

if minT ==0
    [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0_guess(1)), [0 tf], z0);
else
    [z, ~, x_tf, ~, ~, tDone] = integrateAndComputeTerminal_minT([lambda0(1:end-1); x0], tf_max, muEarth, Tmax, Isp, g0, epsilon, L0_guess, 1e-9, at, et, it, Omega_t, omega_t, muVal);
    figure; plot(z(:, 8), z(:,9 ))
end
figure; plot(z(:, 8), z(:,9 ))

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

    [Tmag,rho,uSwitch] = ThrottleSwitchingFunc(Isp , g0, L_v_norm, x_t, L_t, L0_guess(1), epsilon, Tmax);

    u(i, :) = [u_dir, uSwitch];
end

switch propulsionType
    case 'chemical'

        z0F = [lambda0; xf];
        periodF = 2*pi*sqrt( (rf^3) /muVal  );
        timeFinalF = linspace(0, periodF, 405) ;
        [tF, zF] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, 1e-13*Tmax, Isp, g0, epsilon, L0_guess(1)), timeFinalF, z0F, odeset('RelTol',1e-9,'AbsTol',1e-9,'Stats','off'));
        xF = zF(:, (length(x0)+1):end);
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

        z0F = [lambda0; r_pf; v_pf; 1000];
        timeFinalF = linspace(0, periodF, 405) ;
        [tF, zF] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth,1e-9*Tmax, Isp, g0, epsilon, L0_guess(1)), timeFinalF, z0F, odeset('RelTol',1e-9,'AbsTol',1e-9,'Stats','off'));
        xF = zF(:, (length(x0)+1):end);
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
% hThrust = quiver(x(1, 1), x(1, 2), u_vec_plot(1, 1)*scale_inv, u_vec_plot(1, 2)*scale_inv, ...
%     'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'DisplayName', 'Thrust Direction');

legend('show', 'Location', 'bestoutside');

% Set axis limits with some padding
x_min = min(x(:,1)); x_max = max(x(:,1));
y_min = min(x(:,2)); y_max = max(x(:,2));
dx = x_max - x_min; if dx==0, dx=1; end
dy = y_max - y_min; if dy==0, dy=1; end
xlim([x_min - 0.2*dx, x_max + 0.2*dx]);
ylim([y_min - 0.2*dy, y_max + 0.2*dy]);

% Setup Video Writer
videoFilename = 'satellite_maneuver.avi';
vWriter = VideoWriter(videoFilename, 'Uncompressed AVI');
vWriter.FrameRate = 30; % Adjust frame rate as needed
% vWriter.Quality = 100;
open(vWriter);

% Loop through time steps to animate
% Depending on the number of steps in 't', we might want to skip frames to keep video short
nSteps = length(t);
frameStep = max(1, floor(nSteps / 300)); % Target approx 300 frames max (~10s at 30fps)
a = 1+6378;
b = 1400+6378;
fimplicit(@(x, y) x.^2/a.^2 + y.^2/b.^2 - 1)

for k = 1:frameStep:nSteps
    addpoints(hTraj, x(k, 1), x(k, 2));
    set(hSat, 'XData', x(k, 1), 'YData', x(k, 2));

    % Update thrust vector
    % set(hThrust, 'XData', x(k, 1), 'YData', x(k, 2), ...
    %     'UData', u_vec_plot(k, 1)*scale_inv, 'VData', u_vec_plot(k, 2)*scale_inv);

    drawnow;

    % viscircles([0 0],[r0 ],'LineStyle','--');
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

%%
