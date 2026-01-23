function optControl_singleShoot

close all; clear all; clc

%% Problem data
tf = 3600*0.25;
x0 = [0 -500 0 0 ]';
xf = [0  0 0 0 ]';

% Initial guess for costates
lambda0_guess = [0; 0;0; 0];

%%

meanMotion = 0.001;
% Define symbolic variables
syms x y z vx vy vz n ...
    ax ay az L1 L2 L3 L4 L5 L6 real

syms x1 x2 x3 x4 x5 x6 real

% 1. Define the state vector (Position and Velocity)
X = [x; y; vx; vy];
Xnum = [x1 x2 x3 x4].';

% 2. Define the control vector (Accelerations)
U = [ax; ay];

% 3. Define the dynamics vector function f (dX/dt)
% From CWH equations:
f = [vx; ...
     vy; ...
     3*n^2*x + 2*n*vy + ax; ...
     -2*n*vx + ay; ...
     ];
Lvec = [L1 L2 L3 L4].';
%% -------------------------------------------------------------------------

[star_xdot,star_Ldot] = odeDynAndLag(Lvec,X,Xnum,U,f);

eqns = [
        % Costate dynamics:
        (star_Ldot).',
        % State dynamics (linearized relative motion with costate forcing terms):
        (star_xdot).',                 
        ];
eqns = subs(simplify(eqns), n, meanMotion);

funcSubs = @(z) subs(eqns, [Lvec,Xnum], [z(1:4), z(5:end)] );

%%

dzdt = hamiltonian_ode(0, ones(8, 1),funcSubs);

%%

F = shooting(lambda0_guess,x0,xf,tf,funcSubs);

%% Solve shooting equations
lambda0 = fsolve(@(lam0) shooting(lam0,x0,xf,tf,funcSubs), ...
    lambda0_guess, ...
    optimoptions('fsolve','Display','iter'));

% Integrate once more with optimal lambda0
z0 = [lambda0;x0];
[t,z] = ode23(@(t,z) hamiltonian_ode(t,z,funcSubs),[0 tf],z0);

x = z(:,5:end);
lambda = z(:,1:4);
u = -lambda(:,3:4);

% Plot
figure;
subplot(3,1,1)
plot(t,x(:,1),'LineWidth',1.5), ylabel('x_1'), grid on

subplot(3,1,2)
plot(t,x(:,2),'LineWidth',1.5), ylabel('x_2'), grid on

subplot(3,1,3)
plot(t,u,'LineWidth',1.5), ylabel('u'), xlabel('Time'), grid on


end


