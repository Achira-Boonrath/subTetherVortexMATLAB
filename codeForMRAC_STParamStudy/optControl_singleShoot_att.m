function optControl_singleShoot_att
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
tf = 3600*1.5;                % Final time (seconds) - quarter hour
x0 = [[0 -500 0 0 0 0], rotm2quat(rotz(0)), [0 0 0]]';           % Initial state: x (m), y (m), vx (m/s), vy (m/s) quat2rotm([1 0 0 0])
xf = [[0 0 0 0 0 0], rotm2quat(rotz(180)), [0 0 0]]';             % Desired terminal state at t = tf

% x0 = [ rotm2quat(rotz(0)), [0 0 0]]';           % Initial state: x (m), y (m), vx (m/s), vy (m/s) quat2rotm([1 0 0 0])
% xf = [ rotm2quat(rotz(180)), [0 0 0]]';             % Desired terminal state at t = tf

% Initial guess for costates (at t=0). fsolve will update this.
lambda0_guess = zeros(length(xf),1);
% lambda0_guess = 1e-4*ones(length(xf),1);

% xx = load("optRndAtt_solu_attOnly.mat");
% lambda0_guess = [0*ones(6,1)', xx.lambda(1,:)].';
%% Physical parameter
meanMotion = 0.001;            % Mean motion n (rad/s) for CWH model

%% Define symbolic variables for deriving equations
% State symbols (position and velocity)
% Define symbolic variables
syms x y z vx vy vz n ...
    ax ay az ...
    Jx Jy Jz real 

syms tx ty tz q1 q2 q3 q4 wx wy wz real

% 1. Define the state vector (Position and Velocity)
% States and Controls
X = [x; y; z; vx; vy; vz; q1; q2; q3; q4; wx; wy; wz];
% X = [q1; q2; q3; q4; wx; wy; wz];
Xnum = sym('x', [length(xf) 1],'real');
J = diag([Jx, Jy, Jz]);
omega = [wx; wy; wz];

% 2. Define the control vector (Accelerations)
U = [ax; ay; az; tx; ty; tz];
% U = [tx; ty; tz];

% 3. Define the dynamics vector function f (dX/dt)
% From CWH equations:
f_trans = [vx; ...
     vy; ...
     vz; ...
     3*n^2*x + 2*n*vy + ax; ...
     -2*n*vx + ay; ...
     -n^2*z + az];

% Omega matrix for q_dot = 0.5 * Omega * q
% Omega_mat = [ 0,   wz, -wy,  wx; ...
%              -wz,  0,   wx,  wy; ...
%               wy, -wx,  0,   wz; ...
%              -wx, -wy, -wz,  0];
% f_quat = 0.5 * Omega_mat * [q1; q2; q3; q4];
f_quat =  0.5*[[-q2 -q3 -q4];[q1 -q4 q3];...
        [q4 q1 -q2];[-q3 q2 q1]]*omega;

% tau = J*w_dot + w x (J*w)  => w_dot = J^-1 * (tau - w x Jw)
% f_omega = inv(J) * ([tx; ty; tz] - cross(omega, J*omega));
f_omega = (J)\([tx; ty; tz] - cross(omega,J*omega ) );

% Combined System
f = [f_trans; f_quat; f_omega];
% f = [f_quat; f_omega];
Lvec = sym('L', [length(xf) 1],'real');

%% Build symbolic ODEs for states and costates using helper
% The function odeDynAndLag should return symbolic expressions for xdot and Ldot
[star_xdot, star_Ldot] = odeDynAndLag(Lvec, X, Xnum, U, f);

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
eqns = subs(eqns,  J,  diag([100, 100, 100]) );

% Build a function handle that substitutes numeric z = [lambda; x] into the
% symbolic eqns. This handle is passed to the ODE evaluator and shooting.
funcSubs = @(z) double( subs(eqns, [Lvec, Xnum], [z(1:length(xf)), z((length(xf)+1):end)] ) );

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
dzdt = hamiltonian_ode(0, ones(2*length(xf), 1), funcSubs);

%% Solve two-point boundary value problem via shooting
% First try a single invocation of shooting with the initial guess
% F = shooting(lambda0_guess, x0, xf, tf, funcSubs);

% Now use fsolve to find lambda0 that makes the terminal state match xf
lambda0 = fsolve(@(lam0) shooting(lam0, x0, xf, tf, funcSubs), ...
    lambda0_guess, ...
    optimoptions('fsolve', 'Display', 'iter', 'MaxIterations', 40));

% Use particleswarm to find lambda0 that makes the terminal state match xf
% objfun = @(lam0) norm(shooting(lam0', x0, xf, tf, funcSubs));
% nvars = length(lambda0_guess);
% lb = -[1e-2*ones(6,1)', 1e2*ones(nvars-6,1)']'; % Lower bounds (adjust as needed)
% ub =  [1e-2*ones(6,1)', 1e2*ones(nvars-6,1)']'; % Upper bounds (adjust as needed)
% opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 2*nvars, 'MaxIterations', 40); %, "UseParallel", true);
% lambda0 = particleswarm(objfun, nvars, lb, ub, opts);

%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 2*length(xf)-D ODE
z0 = [lambda0; x0];
[t, z] = ode23(@(t, z) hamiltonian_ode(t, z, funcSubs), [0 tf], z0);

% Extract state and costate trajectories from integrated z
x = z(:, (length(xf)+1):end);               % states (columns correspond to [x y vx vy])
lambda = z(:, 1:length(xf));            % costates (columns correspond to [L1 L2 L3 L4])
save("optRndAtt_solu.mat")

%% Plot results: positions and control history
% load("optRndAtt_solu_ref.mat")
clear all
clc
load("optRndAtt_solu.mat")

% Compute control from costates. In this formulation the optimal control is
u = -lambda(:, 4:6);
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
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Satellite Maneuvering Animation (3D)');
view(3); hold on;

% Plot the target state (origin)
plot3(0, 0, 0, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');

% Create animated line for trajectory
hTraj = animatedline('LineWidth', 1.5, 'Color', 'b', 'DisplayName', 'Trajectory');

% Calculate scaling for thrust vector visualization
x_span = max(x(:,1)) - min(x(:,1));
y_span = max(x(:,2)) - min(x(:,2));
z_span = max(x(:,3)) - min(x(:,3));
avg_span = (x_span + y_span + z_span) / 3;
if avg_span == 0, avg_span = 1; end
u_mag = sqrt(sum(u.^2, 2));
max_u = max(u_mag);
if max_u == 0, scale_inv = 1; else, scale_inv = (0.25 * avg_span) / max_u; end

% Initialize thrust vector (scaled)
hThrust = quiver3(x(1, 1), x(1, 2), x(1, 3), ...
    u(1, 1)*scale_inv, u(1, 2)*scale_inv, u(1, 3)*scale_inv, ...
    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'DisplayName', 'Thrust Direction');

% Define Cube Vertices (Unit cube scaled to look reasonable)
cubeScale = avg_span * 0.05; % 5% of span
% Define vertices relative to center
v_template = cubeScale * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                          -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
% Define faces
faces = [1 2 3 4; 2 6 7 3; 6 5 8 7; 5 1 4 8; 1 5 6 2; 4 3 7 8];

% Create patch for satellite (initially at first step)
q1 = x(1, 7:10);
rotM1 = quat2rotm(q1);
v_rot1 = (rotM1 * v_template')';
v_trans1 = v_rot1 + x(1, 1:3);
hSat = patch('Vertices', v_trans1, 'Faces', faces, ...
    'FaceColor', 'c', 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'DisplayName', 'Satellite');

legend([hTraj, hThrust, hSat], {'Trajectory', 'Thrust', 'Satellite'}, 'Location', 'best');

% Set axis limits
x_min = min(x(:,1)); x_max = max(x(:,1));
y_min = min(x(:,2)); y_max = max(x(:,2));
z_min = min(x(:,3)); z_max = max(x(:,3));
pad = 0.2 * max([x_max-x_min, y_max-y_min, z_max-z_min]);
if pad==0, pad=1; end
xlim([x_min - pad, x_max + pad]);
ylim([y_min - pad, y_max + pad]);
zlim([z_min - pad, z_max + pad]);

% Setup Video Writer
videoFilename = 'satellite_maneuver_3d.mp4';
vWriter = VideoWriter(videoFilename, 'MPEG-4');
vWriter.FrameRate = 30;
open(vWriter);

% Loop through time steps
nSteps = length(t);
frameStep = max(1, floor(nSteps / 300));

for k = 1:frameStep:nSteps
    currPos = x(k, 1:3);
    currQuat = x(k, 7:10); 
    currU = u(k, :);
    
    % Update trajectory
    addpoints(hTraj, currPos(1), currPos(2), currPos(3));
    
    % Update Satellite Cube
    rotM = quat2rotm(currQuat);
    v_rot = (rotM * v_template')';
    v_trans = v_rot + currPos;
    set(hSat, 'Vertices', v_trans);
    
    % Update thrust vector
    set(hThrust, 'XData', currPos(1), 'YData', currPos(2), 'ZData', currPos(3), ...
        'UData', currU(1)*scale_inv, 'VData', currU(2)*scale_inv, 'WData', currU(3)*scale_inv);
    
    drawnow;
    frame = getframe(gcf);
    writeVideo(vWriter, frame);
end

% Final frame
currPos = x(end, 1:3);
currQuat = x(end, 7:10);
currU = u(end, :);
addpoints(hTraj, currPos(1), currPos(2), currPos(3));
rotM = quat2rotm(currQuat);
v_trans = (rotM * v_template')' + currPos;
set(hSat, 'Vertices', v_trans);
set(hThrust, 'XData', currPos(1), 'YData', currPos(2), 'ZData', currPos(3), ...
        'UData', currU(1)*scale_inv, 'VData', currU(2)*scale_inv, 'WData', currU(3)*scale_inv);
drawnow;
writeVideo(vWriter, getframe(gcf));

close(vWriter);
fprintf('Animation saved to %s\n', videoFilename);

end
