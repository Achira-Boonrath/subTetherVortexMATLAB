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
tf = 3600*0.5;                % Final time (seconds) - quarter hour
x0 = [0 -500 0 0 ]';           % Initial state: x (m), y (m), vx (m/s), vy (m/s)
xf = [0  0 0 0 ]';             % Desired terminal state at t = tf

% Initial guess for costates (at t=0). fsolve will update this.
lambda0_guess = [0; 0; 0; 0];

%% Physical parameter
meanMotion = 0.001;            % Mean motion n (rad/s) for CWH model

%% Define symbolic variables for deriving equations
% State symbols (position and velocity)
syms x y z vx vy vz n ...
    ax ay az L1 L2 L3 L4 L5 L6 real

% Numeric symbols that will be used to build function handles later
syms x1 x2 x3 x4 x5 x6 real

% Build state and control symbolic vectors for clarity
X = [x; y; vx; vy];            % symbolic state vector (4x1)
Xnum = [x1 x2 x3 x4].';        % corresponding numeric placeholders
U = [ax; ay];                  % symbolic control vector (2x1)

% Define the dynamics f = dX/dt (Clohessy-Wiltshire linearized relative motion)
f = [vx; ...
     vy; ...
     3*n^2*x + 2*n*vy + ax; ...
     -2*n*vx + ay; ...
     ];
% Symbolic costate vector (4x1)
Lvec = [L1 L2 L3 L4].';

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

% Build a function handle that substitutes numeric z = [lambda; x] into the
% symbolic eqns. This handle is passed to the ODE evaluator and shooting.
funcSubs = @(z) subs(eqns, [Lvec, Xnum], [z(1:4), z(5:end)] );

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
dzdt = hamiltonian_ode(0, ones(8, 1), funcSubs);

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
x = z(:, 5:end);               % states (columns correspond to [x y vx vy])
lambda = z(:, 1:4);            % costates (columns correspond to [L1 L2 L3 L4])

% Compute control from costates. In this formulation the optimal control is
u = -lambda(:, 3:4);

%% Plot results: positions and control history
figure;
subplot(3, 1, 1)
plot(t, x(:, 1), 'LineWidth', 1.5), ylabel('x_1'), grid on
title('State and Control Histories')

subplot(3, 1, 2)
plot(t, x(:, 2), 'LineWidth', 1.5), ylabel('x_2'), grid on

subplot(3, 1, 3)
plot(t, u, 'LineWidth', 1.5), ylabel('u'), xlabel('Time (s)'), grid on

figure 
plot( x(:, 1), x(:, 2), 'LineWidth', 1.5 )
grid on

end
