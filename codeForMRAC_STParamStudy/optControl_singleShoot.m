function optControl_singleShoot
% optControl_singleShoot
% Solves a single-shooting optimal control problem for a 2D linearized
% relative motion (Clohessy-Wiltshire/Hill) model using costate shooting.
% The Hamiltonian system (state + costate) is integrated and fsolve is used
% to find the initial costates that satisfy the terminal state constraint.
%
% Notes:
% - State X = [x; y; vx; vy]
% - Control U = [ax; ay] (accelerations)
% - Costates L = [L1; L2; L3; L4] (co-vector associated with X)
% - Problem: drive x0 -> xf in time tf minimizing quadratic control (implied
%   by u = -lambda(3:4) in this formulation)
%
% This script expects helper functions in the path:
% - odeDynAndLag    : returns symbolic xdot and Ldot expressions (dynamics and costate ODEs)
% - hamiltonian_ode : evaluates the combined ODE for [lambda; x] given numeric z
% - shooting        : residual function for terminal constraints, used by fsolve

close all; clear all; clc

%% Problem data
tf = 3600*0.5;                % Final time (seconds) 
x0 = [0 -1500 0 0 ]';           % Initial state: x (m), y (m), vx (m/s), vy (m/s)
xf = [0  0 0 0 ]';             % Desired terminal state at t = tf

% Initial guess for costates (at t=0). fsolve will update this.
lambda0_guess = [0; 0; 0; 0];

%% Physical parameter
meanMotion = 0.001;            % Mean motion n (rad/s) for CWH model

%% Define symbolic variables for deriving equations
% State symbols (position and velocity)
syms x y z vx vy vz n ...
    ax ay az real

% Build state and control symbolic vectors for clarity
X = [x; y; vx; vy];            % symbolic state vector (4x1)
Xnum = sym('x', [length(xf) 1],'real');      % corresponding numeric placeholders
U = [ax; ay];                  % symbolic control vector (2x1)

% Define the dynamics f = dX/dt (Clohessy-Wiltshire linearized relative motion)
f = [vx; ...
     vy; ...
     3*n^2*x + 2*n*vy + ax; ...
     -2*n*vx + ay; ...
     ];
% Symbolic costate vector (4x1)
Lvec = sym('L', [length(xf) 1],'real');

%% Build symbolic ODEs for states and costates using helper
V = 0.5*U'*U; % the instantaneous cost on the control (scalar).
% The function odeDynAndLag should return symbolic expressions for xdot and Ldot
[star_xdot, star_Ldot] = odeDynAndLag_constT(Lvec, X, Xnum, U, f, V);

% Compose the equations array:
% - First the costate dynamics (Ldot)
% - Then the state dynamics (xdot)
% This ordering matches how we will stack z = [lambda; x] for integration.
eqns = [
        (star_Ldot).',         % costate ODEs (row)
        (star_xdot).'          % state ODEs (row)
        ];
% Substitute the numeric value for mean motion to simplify expressions
eqns = subs(simplify(eqns), n, meanMotion);

% Build a function handle that substitutes numeric z = [lambda; x] into the
% symbolic eqns. This handle is passed to the ODE evaluator and shooting.
funcSubs = @(z) subs(eqns, [Lvec, Xnum], [z(1:length(xf)), z((length(xf)+1):end)] );

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
dzdt = hamiltonian_ode(0, ones(length(xf)+length(xf), 1), funcSubs);

%% Solve two-point boundary value problem via shooting
% First try a single invocation of shooting with the initial guess
F = shooting(lambda0_guess, x0, xf, tf, funcSubs);

% Now use fsolve to find lambda0 that makes the terminal state match xf
lambda0 = fsolve(@(lam0) shooting(lam0, x0, xf, tf, funcSubs), ...
    lambda0_guess, ...
    optimoptions('fsolve', 'Display', 'iter'));

%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
z0 = [lambda0; x0];
[t, z] = ode23(@(t, z) hamiltonian_ode(t, z, funcSubs), [0 tf], z0);

% Extract state and costate trajectories from integrated z
x = z(:, (length(xf)+1):end);               % states (columns correspond to [x y vx vy])
lambda = z(:, 1:length(xf));            % costates (columns correspond to [L1 L2 L3 L4])

% Compute control from costates. In this formulation the optimal control is
u = -lambda(:, 3:4);

%% exact solu for u
n = meanMotion;
% A = [[zeros(3,3),eye(3,3)];...
%     [3*n^2, 0 , 0, 0, 2*n, 0];...
%     [0, 0, 0, -2*n, 0, 0];...
%     [0, 0, -n^2, 0, 0, 0]];
% B = [zeros(3,3);eye(3,3)];

% Define the system matrices for the 2D CWH model
A = [[zeros(2,2), eye(2,2)]; ...      % Top: zeros and identity for position/velocity coupling
    [3*n^2, 0, 0, 2*n]; ...           % Bottom: CWH dynamics for x
    [0, 0, -2*n, 0]];                 % Bottom: CWH dynamics for y
B = [zeros(2,2); eye(2,2)];           % Control input matrix (affects acceleration states)

t0 = 0.0;                             % Initial time
syms tau real                         % Symbolic variable for integration

% Compute the controllability Gramian g for [t0, tf] using matrix exponentials
fun = @(tau) expm(A*(t0-tau)) * B * (B.') * expm(A.'*(t0-tau));
g = integral(fun, 0, tf, 'ArrayValued', true);

% Compute the controllability Gramian g2 for [tf, t0] 
fun2 = @(tau) expm(A*(tf-tau)) * B * (B.') * expm(A.'*(tf-tau));
g2 = integral(fun2, 0, tf, 'ArrayValued', true);

% Compute the exact optimal control using the analytical solution for LQR with fixed final state
for jj = 1:length(t)
    % solution (from t0)
    uExact(jj, :) = - (B') * expm(A' * (0 - t(jj))) * pinv(g) * (x0 - expm(A * (0 - tf)) * xf);
    % solution (from tf)
    uExact2(jj, :) = - (B') * expm(A' * (tf - t(jj))) * pinv(g2) * (expm(A * (tf - t0)) * x0 - xf);
end

% Plot the relative error between the numerically computed and exact controls
figure;
plot((u - uExact) ./ uExact)
hold on
plot((u - uExact2) ./ uExact2, "--")
grid minor
title('Relative Error: (u - uExact) / uExact')

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
xlabel('x (m)'); ylabel('y (m)');
title('Satellite Maneuvering Animation');

% Plot the target state (origin)
plot(0, 0, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');

% Create animated line for trajectory and marker for satellite
hTraj = animatedline('LineWidth', 1.5, 'Color', 'b', 'DisplayName', 'Trajectory');
hSat = plot(x(1, 1), x(1, 2), 'k.', 'MarkerSize', 20, 'DisplayName', 'Satellite');

% Calculate scaling for thrust vector visualization
% We want the max thrust vector to correspond to roughly 15% of the plot span
x_span = max(x(:,1)) - min(x(:,1));
y_span = max(x(:,2)) - min(x(:,2));
avg_span = (x_span + y_span) / 2;
if avg_span == 0, avg_span = 1; end
u_mag = sqrt(sum(u.^2, 2));
max_u = max(u_mag);
if max_u == 0, scale_inv = 1; else, scale_inv = (0.15 * avg_span) / max_u; end

% Initialize thrust vector (scaled)
hThrust = quiver(x(1, 1), x(1, 2), u(1, 1)*scale_inv, u(1, 2)*scale_inv, ...
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
        'UData', u(k, 1)*scale_inv, 'VData', u(k, 2)*scale_inv);
    
    drawnow;
    
    % Capture frame for video
    frame = getframe(gcf);
    writeVideo(vWriter, frame);
end

addpoints(hTraj, x(end, 1), x(end, 2));
set(hSat, 'XData', x(end, 1), 'YData', x(end, 2));
set(hThrust, 'XData', x(end, 1), 'YData', x(end, 2), ...
        'UData', u(end, 1)*scale_inv, 'VData', u(end, 2)*scale_inv);
drawnow;
writeVideo(vWriter, getframe(gcf));

close(vWriter);
fprintf('Animation saved to %s\n', videoFilename);

end
