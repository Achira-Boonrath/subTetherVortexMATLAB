function inverted_pendulum_HS_collocation_fixed
% Hermite–Simpson direct collocation + fmincon for inverted pendulum (fixed)
%
% EXPLANATION OF METHOD:
% This code Solves the Optimal Control Problem (OCP) by discretizing it into
% a Nonlinear Programming (NLP) problem using the Hermite-Simpson (HS)
% Direct Collocation method.
%
% 1. Discretization: The time domain [0, T] is divided into N intervals.
%    The state trajectories x(t) and control trajectories u(t) are approximated
%    by discrete values at "knot points" (boundaries of intervals) and
%    "collocation points" (midpoints of intervals).
%
% 2. Decision Variables (z):
%    The optimizer solves for a large vector 'z' containing:
%    - States X at all N+1 knot points.
%    - States Xm at all N midpoint intervals (improves accuracy to 3rd order).
%    - Controls U at all N+1 knot points.
%
% 3. Constraints (Defects):
%    Instead of integrating ODEs sequentially (shooting), we enforce algebraic
%    constraints at every interval that equate the derivative of the polynomial
%    spline to the systems dynamics f(x,u).
%    - Simpson Defect: Enforces dynamics across the whole interval.
%    - Midpoint Consistency: Enforces the state value at the midpoint based on
%      knots and dynamics.
%
% 4. Optimization:
%    fmincon finds the optimal 'z' that minimizes the Cost Function while
%    satisfying these defect constraints and boundary conditions.
%
% Everything (objective & constraints) is implemented as nested functions so
% they can access shared variables like nX, nXm, dt, etc.

clear; close all; clc;

%% Pendulum parameters
m = 1.0;        % mass (kg)
l = 1.0;        % length (m)
g = 9.81;       % gravity (m/s^2)
I = m*l^2;      % moment of inertia

%% Problem setup
T = 2.0;         % total time (s)
N = 40;          % number of intervals
dt = T / N;
tgrid = linspace(0,T,N+1);
tgrid_mid = tgrid(1:end-1) + dt/2;

nx = 2;  % states: [theta; theta_dot]
nu = 1;  % control: torque

% Initial condition (pendulum down)
theta0 = 0;             % start at 0 rad (downwards)
xd0 = [theta0; 0];

% Desired final condition (upright)
theta_des = pi;         % upright
xf_des = [theta_des; 0];

% control bounds
u_min = -20;
u_max = 20;

% state bounds (wide)
theta_min = -4*pi;
theta_max = 4*pi;
thetadot_min = -50;
thetadot_max = 50;

%% Decision variables ordering:
% The generic NLP solver (fmincon) sees a single vector 'z'. We structure it as:
% z = [X_knots(:); X_mid(:); U_knots(:)]
%   X_knots: (nx) x (N+1)  -> States at t_0, t_1, ..., t_N
%   X_mid:   (nx) x (N)    -> States at t_0.5, t_1.5, ..., t_(N-0.5)
%   U_knots: (nu) x (N+1)  -> Controls at t_0, t_1, ..., t_N
%
% Total variables = nx*(N+1) + nx*N + nu*(N+1)
% z = [X_knots(:); X_mid(:); U_knots(:)]
nX = nx*(N+1);
nXm = nx * N;
nU = nu*(N+1);
nz = nX + nXm + nU;

% initial guess
X0 = zeros(nx, N+1);
for k=1:N+1
    s = (k-1)/N;
    X0(1,k) = (1-s)*xd0(1) + s*xf_des(1);  % theta linear interp
    X0(2,k) = 0;
end
Xm0 = zeros(nx, N); % midpoints (linear)
for k=1:N
    Xm0(:,k) = 0.5*(X0(:,k)+X0(:,k+1));
end
U0 = zeros(nu, N+1);

z0 = [X0(:); Xm0(:); U0(:)];

%% Bounds
lb = -inf(nz,1);
ub =  inf(nz,1);

% knots states bounds
for k=1:N+1
    idx_x = (k-1)*nx + (1:nx);
    lb(idx_x) = [theta_min; thetadot_min];
    ub(idx_x) = [theta_max; thetadot_max];
end
% mid states bounds
offset_Xm = nX;
for k=1:N
    idx_xm = offset_Xm + (k-1)*nx + (1:nx);
    lb(idx_xm) = [theta_min; thetadot_min];
    ub(idx_xm) = [theta_max; thetadot_max];
end
% control bounds
offset_U = nX + nXm;
for k=1:N+1
    idx_u = offset_U + (k-1)*nu + (1:nu);
    lb(idx_u) = u_min;
    ub(idx_u) = u_max;
end

%% Cost weights
R_u = 1e-2;            % running cost weight on control squared
Qf = diag([200, 20]);  % terminal penalty

%% fmincon options
options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'MaxFunctionEvaluations', 5e5, ...
    'MaxIterations', 3000, ...
    'OptimalityTolerance',1e-6, ...
    'StepTolerance',1e-9);

%% Call fmincon using nested objective & constraint functions
fprintf('Starting HS optimization with N=%d intervals, T=%.2f s\n', N, T);
tic;
[z_opt, J_opt, exitflag, output] = fmincon(@obj, z0, [], [], [], [], lb, ub, @nonlcon, options);
toc;
fprintf('Finished. exitflag=%d, final J=%.6f\n', exitflag, J_opt);

%% Extract solution and plot (same as before)
X_opt = reshape(z_opt(1:nX), nx, N+1);
Xm_opt = reshape(z_opt(nX+1:nX+nXm), nx, N);
U_opt = reshape(z_opt(nX+nXm+1:end), nu, N+1);

figure;
subplot(4,1,1);
plot(tgrid, wrapToPi(X_opt(1,:)), '-','LineWidth',1.2); hold on;
plot(tgrid_mid, wrapToPi(Xm_opt(1,:)), 'o','MarkerSize',4);
ylabel('\theta (rad)'); grid on; title('Hermite–Simpson Collocation Solution');
plot(tgrid, ones(size(tgrid))*pi, '--k','HandleVisibility','off');

subplot(4,1,2);
plot(tgrid, X_opt(2,:), '-','LineWidth',1.2); hold on;
plot(tgrid_mid, Xm_opt(2,:), 'o','MarkerSize',4);
ylabel('theta dot (rad/s)'); grid on;

subplot(4,1,3);
stairs(tgrid, U_opt,'-','LineWidth',1.2);
ylabel('u (Nm)'); grid on;

Umid = zeros(1,N);
for k=1:N
    Umid(k) = 0.5*(U_opt(:,k) + U_opt(:,k+1));
end
subplot(4,1,4);
plot(tgrid_mid, Umid,'-o','LineWidth',1.1);
ylabel('u_{mid} (Nm)'); xlabel('time (s)'); grid on;

%% Animation
figure;
for k = 1:2:length(tgrid)
    theta = X_opt(1,k);
    x_p = l * sin(theta);
    y_p = -l * cos(theta);
    plot([0 x_p], [0 y_p], '-o','LineWidth',2,'MarkerSize',8);
    axis equal;
    xlim([-1.4*l,1.4*l]); ylim([-1.4*l,1.4*l]);
    title(sprintf('t = %.2f s, theta = %.2f rad', tgrid(k), wrapToPi(theta)));
    drawnow;
end

%% ---------------- Nested functions ----------------

    function J = obj(z)
        % COST FUNCTION
        % We want to minimize J = Integral( u(t)^2 * R ) + FinalErrorPenalty
        %
        % Since we have discretized u(t), we approximate the integral using
        % Simpson's Rule for higher accuracy than a simple Riemann sum.
        % Integral ~ (dt/6) * (u_k^2 + 4*u_mid^2 + u_{k+1}^2)

        % Objective uses variables from parent scope: nX, nXm, N, dt, R_u, Qf, xf_des
        X = reshape(z(1:nX), nx, N+1);
        Xm = reshape(z(nX+1:nX+nXm), nx, N);
        U = reshape(z(nX+nXm+1:end), nu, N+1);
        % Simpson integrate u^2 over each interval
        J_run = 0;
        for kk = 1:N
            uk = U(:,kk);
            uk1 = U(:,kk+1);
            um = 0.5*(uk + uk1);
            J_run = J_run + (dt/6)*(uk'*uk + 4*(um'*um) + uk1'*uk1);
        end
        J_run = R_u * J_run;
        % terminal penalty (angle wrapped)
        xN = X(:,end);
        ang_err = wrapToPi(xN(1) - xf_des(1));
        xerr = [ang_err; xN(2)-xf_des(2)];
        J_final = xerr' * Qf * xerr;
        J = J_run + J_final;
    end

    function [c, ceq] = nonlcon(z)
        % NON-LINEAR CONSTRAINTS
        % This is where the physics are enforced.
        % The solver must find X and U such that the spline interpolants
        % match the system dynamics: x_dot = f(x,u).

        X = reshape(z(1:nX), nx, N+1);
        Xm = reshape(z(nX+1:nX+nXm), nx, N);
        U = reshape(z(nX+nXm+1:end), nu, N+1);
        ceq = zeros(nx*(2*N) + nx, 1);
        cnt = 0;
        for kk = 1:N
            xk = X(:,kk);
            xk1 = X(:,kk+1);
            xm = Xm(:,kk);
            uk = U(:,kk);
            uk1 = U(:,kk+1);
            um = 0.5*(uk + uk1);
            fk = pendulum_dynamics(xk, uk);
            fk1 = pendulum_dynamics(xk1, uk1);
            fm = pendulum_dynamics(xm, um);
            % 1. Midpoint Consistency Constraint:
            %    Value at midpoint x_m must match the Hermite interpolation
            %    from x_k and x_{k+1} using derivatives f_k and f_{k+1}.
            %    x_m = 0.5*(x_k + x_{k+1}) + (dt/8)*(f_k - f_{k+1})
            mid_cons = xm - 0.5*(xk + xk1) - (dt/8)*(fk - fk1);
            ceq(cnt + (1:nx)) = mid_cons;
            cnt = cnt + nx;

            % 2. Hermite-Simpson Defect Constraint:
            %    The change in state x_{k+1} - x_k must match the integral
            %    of the dynamics, approximated by Simpson's rule.
            %    x_{k+1} - x_k = (dt/6) * (f_k + 4*f_{mid} + f_{k+1})
            defect = xk1 - xk - (dt/6)*(fk + 4*fm + fk1);
            ceq(cnt + (1:nx)) = defect;
            cnt = cnt + nx;
        end
        % initial state equality
        ceq(cnt + (1:nx)) = X(:,1) - xd0;
        c = [];
    end

    function xd = pendulum_dynamics(x,u)
        theta = x(1);
        theta_dot = x(2);
        theta_ddot = (u - m*g*l*sin(theta)) / I;
        xd = [theta_dot; theta_ddot];
    end

    function a_wrapped = wrapToPi(x)
        a_wrapped = mod(x + pi, 2*pi) - pi;
    end

end % end main function
