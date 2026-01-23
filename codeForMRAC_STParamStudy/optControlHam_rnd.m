% Close any open figure windows, clear workspace variables, and clear the command window.
% This ensures a clean environment before running the script.
close all; clear all; clc

%% Symbolic state + costate (Lagrange multiplier) definitions
% Define symbolic time-dependent functions for the state variables:
% x1, x2: relative position components (e.g., along-track and radial)
% x3, x4: corresponding velocity components
% L1..L4: costate (adjoint) variables associated with x1..x4
% ux, uy: declared here if control variables are later required (not used in this snippet)
syms x1(t) x2(t) x3(t) x4(t) L1(t) L2(t) L3(t) L4(t) ux(t) uy(t)

% Physical / problem parameters
% mu: standard gravitational parameter (keplerian), kept here for reference (not used below)
% n: mean motion (rad/s) used in the linearized relative-motion (Clohessy-Wiltshire) terms
mu = 398600;
n = 0.001;

% System of ordinary differential equations (ODEs)
% The first four are the costate (adjoint) differential equations:
%   diff(Li,t) = - dH/dxi  (from Pontryagin Minimum Principle / Hamiltonian)
% The last four are the state differential equations (dynamics).
% Comments indicate the physical meaning of each equation where applicable.
eqns = [
        % Costate dynamics:
        diff(L1,t) == - L3*3*(n^2),   % L1' = -3 n^2 * L3
        diff(L2,t) == - 0,            % L2' = 0  
        diff(L3,t) == - (L1 + L4*(-2*n)), % L3' = -(L1 - 2 n L4)
        diff(L4,t) == - (L2 + L3*(2*n)),  % L4' = -(L2 + 2 n L3)

        % State dynamics (linearized relative motion with costate forcing terms):
        diff(x1,t) == x3,                             % x1' = x3  
        diff(x2,t) == x4,                             % x2' = x4
        diff(x3,t) == 3*(n^2)*x1  + (2*n)*x4 - L3,    % x3' = 3 n^2 x1 + 2 n x4 - L3  
        diff(x4,t) == -(2*n)*x3 - L4,                 % x4' = -2 n x3 - L4
];

% Solve the system of linear ODEs symbolically.
% dsolve returns the general solution with arbitrary integration constants (C1, C2, ...).
S = dsolve(eqns);

%% Boundary conditions and determination of integration constants
% Define initial and final times
t0 = 0;
tf = 0.5*3600; % final time in seconds (0.5 hours = 1800 s). Adjust as required.

% Specify boundary conditions for the state at t0 and tf.
% Here the initial state is: x1(t0)=0, x2(t0)=-100, x3(t0)=0, x4(t0)=0
% and the terminal state at tf is required to be zero for all state components.
% These are algebraic equations for the unknown integration constants C1..C8.
eqnsSub = [
        subs( S.x1, t0 ) == 0,...
        subs( S.x2, t0 ) == -500,...
        subs( S.x3, t0 ) == 0,...
        subs( S.x4, t0 ) == 0,...
        subs( S.x1, tf ) == 0,... 
        subs( S.x2, tf ) == 0,... 
        subs( S.x3, tf ) == 0,... 
        subs( S.x4, tf ) == 0,...
];

% Solve the algebraic system for the integration constants.
% SS will contain fields C1..C8 corresponding to the constants from dsolve.
SS = solve(eqnsSub);

d1 = digits(24);

% Extract named constants for convenience. SS.C1 etc. are symbolic values.
C1 = SS.C1;
C2 = SS.C2;
C3 = SS.C3;
C4 = SS.C4;
C5 = SS.C5;
C6 = SS.C6;
C7 = SS.C7;
C8 = SS.C8;

%% Construct callable functions for the optimized state trajectories

xt = @(t) ((subs(simplify(S.x1))));
yt = @(t) ((subs(simplify(S.x2))));

% xt = @(t) vpa( subs(S.x1) );
% yt = @(t) vpa( subs(S.x2) );

% yfunc = @(t) yt;
% figure
% fplot(@(t) xt(t))
% hold on 
% fplot(@(t) yt(t))
% hold off
% grid on

% t = 0:10:tf;
% x = double(subs(S.x1));
% y = double(subs(S.x2));
% 
figure 
plot(double( xt( linspace(0,tf,30) )), double( yt( linspace(0,tf,30) )))
grid on
