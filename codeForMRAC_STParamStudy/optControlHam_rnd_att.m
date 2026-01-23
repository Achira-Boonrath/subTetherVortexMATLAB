% Close any open figure windows, clear workspace variables, and clear the command window.
% This ensures a clean environment before running the script.
close all; clear all; clc


%% Compute Jacobians

meanMotion = 0.001;
% Define symbolic variables
syms x(t) y(t) z(t) vx(t) vy(t) vz(t) n ...
    ax ay az L1(t) L2(t) L3(t) L4(t) L5(t) L6(t) ...
    Jx Jy Jz L7(t) L8(t) L9(t) L10(t) L11(t) L12(t) L13(t) real 

syms x1(t) x2(t) x3(t) x4(t) x5(t) x6(t) real 

% syms tx(t) ty(t) tz(t) q1(t) q2(t) q3(t) q4(t) wx(t) wy(t) wz(t) real
syms tx ty tz q1(t) q2(t) q3(t) q4(t) wx(t) wy(t) wz(t) real
%% xyz
% 1. Define the state vector (Position and Velocity)
% States and Controls
X = [x; y; z; vx; vy; vz; q1; q2; q3; q4; wx; wy; wz];
Xnum = [x1 x2 x3 x4 x5 x6 q1 q2 q3 q4 wx wy wz].';
J = diag([Jx, Jy, Jz]);
omega = [wx; wy; wz];

% 2. Define the control vector (Accelerations)
U = [ax; ay; az; tx; ty; tz];

% 3. Define the dynamics vector function f (dX/dt)
% From CWH equations:
f_trans = [vx; ...
     vy; ...
     vz; ...
     3*n^2*x + 2*n*vy + ax; ...
     -2*n*vx + ay; ...
     -n^2*z + az];

% Omega matrix for q_dot = 0.5 * Omega * q
Omega_mat = [ 0,   wz, -wy,  wx; ...
             -wz,  0,   wx,  wy; ...
              wy, -wx,  0,   wz; ...
             -wx, -wy, -wz,  0];
f_quat = 0.5 * Omega_mat * [q1; q2; q3; q4];

% tau = J*w_dot + w x (J*w)  => w_dot = J^-1 * (tau - w x Jw)
f_omega = inv(J) * ([tx; ty; tz] - cross(omega, J*omega));

% Combined System
f = [f_trans; f_quat; f_omega];
Lvec = [L1 L2 L3 L4 L5 L6...
    L7 L8 L9 L10 L11 L12 L13].';
%%

[star_xdot,star_Ldot] = odeDynAndLag(Lvec,X,Xnum,U,f);

%% System of ordinary differential equations (ODEs)
% The first four are the costate (adjoint) differential equations:
%   diff(Li,t) = - dH/dxi  (from Pontryagin Minimum Principle / Hamiltonian)

eqns = [
        % Costate dynamics:
        (diff(Lvec, t).'  == star_Ldot).',
        % State dynamics (linearized relative motion with costate forcing terms):
        (diff(Xnum, t).'  == star_xdot).',                 
        ];

% Solve the system of linear ODEs symbolically.
% dsolve returns the general solution with arbitrary integration constants (C1, C2, ...).
S = dsolve(eqns);

% Recursively simplify symbolic expressions in struct S
function out = simplifyStruct(in)
        out = in;
        if isstruct(in)
                fn = fieldnames(in);
                for k = 1:numel(fn)
                        out.(fn{k}) = simplifyStruct(in.(fn{k}));
                end
        elseif isa(in,'sym')
                out = simplify(in);
        elseif isa(in,'symfun')
                out = symfun(simplify(formula(in)), argnames(in));
        elseif iscell(in)
                for j = 1:numel(in)
                        in{j} = simplifyStruct(in{j});
                end
                out = in;
        else
                % leave other types unchanged
                out = in;
        end
end

S.ax = -S.L3;
S.ay = -S.L4;
S = subs(simplifyStruct(S), n, meanMotion);
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

uxt = @(t) ((subs(simplify(S.ax))));
uyt = @(t) ((subs(simplify(S.ay))));

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
plot(double( xt( linspace(0,tf,35) )), double( yt( linspace(0,tf,35) )))
grid on

figure
plot(double( uxt( linspace(0,tf,35) ))) 
hold on
plot(double( uyt( linspace(0,tf,35) )))
grid on
