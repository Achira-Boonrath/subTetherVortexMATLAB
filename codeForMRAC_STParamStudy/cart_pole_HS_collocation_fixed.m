function cart_pole_HS_collocation_fixed
% Hermite–Simpson direct collocation + fmincon for cart-pole stabilization
%
% This script solves an optimal control problem using Hermite-Simpson direct
% collocation, a method that discretizes both states and controls over a time grid.
%
% Problem: Swing up a cart-pole system from downright (theta=pi) to upright (theta=0)
% while keeping the cart position near zero, subject to force constraints.
%
% Method: 
%   - Discretize time into N intervals with N+1 collocation knots
%   - At each interval, use 3 points: start (k), midpoint (m), end (k+1)
%   - Enforce dynamics at both Hermite-Simpson constraints (midpoint consistency 
%     and interval defect constraints)
%   - Minimize control effort + terminal state error using fmincon (Sequential Quadratic Programming)
%
% States: x = [xc; xc_dot; theta; theta_dot]
%   - xc: cart position (m)
%   - xc_dot: cart velocity (m/s)
%   - theta: pendulum angle from vertical (rad) [0=upright, pi=down]
%   - theta_dot: pendulum angular velocity (rad/s)
%
% Control: u = horizontal force applied to cart (N)

clear; close all; clc;

%% Cart-Pole System Parameters
M = 1.0;      % cart mass (kg)
m = 0.1;      % pendulum bob mass (kg)
l = 0.5;      % pendulum length from pivot to center of mass (m)
g = 9.81;     % gravitational acceleration (m/s^2)

%% Optimal Control Problem Setup
T = 2.5;         % total time horizon (s)
N = 60;          % number of collocation intervals (creates 61 knot points)
dt = T / N;      % time step between knots (s)
tgrid = linspace(0,T,N+1);              % time at knot points
tgrid_mid = tgrid(1:end-1) + dt/2;      % time at midpoints (for visualization)

nx = 4;  % number of states
nu = 1;  % number of controls (single force input)

% Initial Condition (pendulum hanging downward, cart at rest at origin)
xc0 = 0;
xdotc0 = 0;
theta0 = pi;   % downright position
thetadot0 = 0;
xd0 = [xc0; xdotc0; theta0; thetadot0];

% Desired Final Condition (pendulum upright, cart near origin)
xc_des = 0;
xdotc_des = 0;
theta_des = 0;    % upright (target angle)
thetadot_des = 0; % stopped swinging
xf_des = [xc_des; xdotc_des; theta_des; thetadot_des];

% Control Input Bounds (physical actuator limits)
u_min = -50;   % max backward force (N)
u_max = 50;    % max forward force (N)

% State Variable Bounds (keep solution physically reasonable)
xc_min = -5;        xc_max = 5;              % cart position limits
xc_dot_min = -20;   xc_dot_max = 20;        % cart velocity limits
theta_min = -4*pi;  theta_max = 4*pi;       % allow multiple rotations
thetadot_min = -50; thetadot_max = 50;      % angular velocity limits

%% Decision Variable Stacking
% The optimization variable z contains all unknowns stacked as a vector:
%   z = [X_knots(:); X_mid(:); U_knots(:)]
% where:
%   - X_knots: state values at N+1 time knots (size nx × (N+1))
%   - X_mid: state values at N midpoints (size nx × N)
%   - U_knots: control values at N+1 time knots (size nu × (N+1))
% This stacking is typical for fmincon-based collocation methods.

nX = nx*(N+1);      % total decision variables for knot states
nXm = nx * N;       % total decision variables for midpoint states
nU = nu*(N+1);      % total decision variables for control knots
nz = nX + nXm + nU; % total problem size

% Generate initial guess via linear interpolation from initial to final state
X0 = zeros(nx, N+1);
for k=1:N+1
    s = (k-1)/N;  % parameter: 0 at start, 1 at end
    X0(:,k) = (1-s)*xd0 + s*xf_des;  % linear interpolation
end
Xm0 = zeros(nx, N);  % midpoint states: average of endpoints
for k=1:N
    Xm0(:,k) = 0.5*(X0(:,k)+X0(:,k+1));
end
U0 = zeros(nu, N+1);  % start with zero control effort

z0 = [X0(:); Xm0(:); U0(:)];  % stack into column vector

%% Variable Bounds for fmincon
lb = -inf(nz,1);
ub =  inf(nz,1);

% Set bounds for knot point states: X(:,1) through X(:,N+1)
for k=1:N+1
    idx_x = (k-1)*nx + (1:nx);
    lb(idx_x) = [xc_min; xc_dot_min; theta_min; thetadot_min];
    ub(idx_x) = [xc_max; xc_dot_max; theta_max; thetadot_max];
end

% Set bounds for midpoint states: Xm(:,1) through Xm(:,N)
offset_Xm = nX;  % offset where midpoint states begin in z vector
for k=1:N
    idx_xm = offset_Xm + (k-1)*nx + (1:nx);
    lb(idx_xm) = [xc_min; xc_dot_min; theta_min; thetadot_min];
    ub(idx_xm) = [xc_max; xc_dot_max; theta_max; thetadot_max];
end

% Set bounds for control: U(:,1) through U(:,N+1)
offset_U = nX + nXm;  % offset where control begins in z vector
for k=1:N+1
    idx_u = offset_U + (k-1)*nu + (1:nu);
    lb(idx_u) = u_min;
    ub(idx_u) = u_max;
end

%% Objective Function Weights
% Balance two competing goals:
R_u = 1e-3;  % penalty on control effort: \int (u^2)dt
             % (small value: control cost is secondary to reaching the goal)
Qf = diag([500, 50, 400, 40]);  % terminal state penalty (only applied at t=T)
% High weights on theta (400) and position (500) to strongly enforce reaching the goal
% Lower weights on velocities (50, 40) to allow them to vary more freely

%% fmincon Solver Options
% Use Sequential Quadratic Programming (SQP), which is robust for constrained optimization
options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...           % Sequential Quadratic Programming
    'Display','iter', ...             % print progress each iteration
    'MaxFunctionEvaluations', 8e5, ... % allow up to 800,000 function calls
    'MaxIterations', 500, ...          % allow up to 500 iterations
    'OptimalityTolerance',1e-5, ...    % tolerance on gradient (lower=tighter)
    'StepTolerance',1e-7);             % tolerance on step size

%% Solve the Optimal Control Problem
fprintf('Starting HS optimization for cart-pole with N=%d, T=%.2f s\n', N, T);
tic;
[z_opt, J_opt, exitflag, output] = fmincon(@obj, z0, [], [], [], [], lb, ub, @nonlcon, options);
toc;
fprintf('Finished. exitflag=%d, final J=%.6f\n', exitflag, J_opt);

%% Extract Optimal Solution
% Unstack the decision vector into state and control trajectories
X_opt = reshape(z_opt(1:nX), nx, N+1);           % optimal knot states
Xm_opt = reshape(z_opt(nX+1:nX+nXm), nx, N);    % optimal midpoint states
U_opt = reshape(z_opt(nX+nXm+1:end), nu, N+1);  % optimal control trajectory

%% Plot Optimal Trajectories
figure;
subplot(4,1,1);
plot(tgrid, X_opt(1,:), '-','LineWidth',1.2); hold on;
plot(tgrid_mid, Xm_opt(1,:), 'o','MarkerSize',4);
ylabel('x_c (m)'); grid on; title('Cart-Pole Hermite–Simpson Solution');
legend('Knots','Midpoints');

subplot(4,1,2);
plot(tgrid, X_opt(2,:), '-','LineWidth',1.2); hold on;
plot(tgrid_mid, Xm_opt(2,:), 'o','MarkerSize',4);
ylabel('x_c dot (m/s)'); grid on;

subplot(4,1,3);
plot(tgrid, wrapToPi(X_opt(3,:)), '-','LineWidth',1.2); hold on;
plot(tgrid_mid, wrapToPi(Xm_opt(3,:)), 'o','MarkerSize',4);
ylabel('\theta (rad)'); grid on;
plot(tgrid, ones(size(tgrid))*0, '--k','HandleVisibility','off');  % target angle

subplot(4,1,4);
stairs(tgrid, U_opt,'-','LineWidth',1.2);
ylabel('u (N)'); xlabel('time (s)'); grid on;

%% Animation of Cart-Pole System
% Visualize the motion frame-by-frame to verify physical plausibility
figure('Name','Cart-Pole Animation','NumberTitle','off');
cart_w = 0.3;      % cart width (m)
cart_h = 0.15;     % cart height (m)
track_y = -0.2;    % ground level (m)
axis_lim = 1.2*max(1, max(abs(X_opt(1,:))) + l);  % set axis limits
for k = 1:2:length(tgrid)  % plot every other frame for speed
    clf;
    xc = X_opt(1,k);
    theta = X_opt(3,k);
    
    % Draw cart as a rectangle
    rectangle('Position',[xc-cart_w/2, track_y, cart_w, cart_h], 'FaceColor',[0.2 0.6 0.8]);
    hold on;
    
    % Draw pole as a line from cart center to pendulum tip
    x_p = xc + l * sin(theta);              % pendulum bob x position
    y_p = track_y + cart_h + l * cos(theta); % pendulum bob y position
    plot([xc, x_p], [track_y+cart_h, y_p], '-k', 'LineWidth',2);
    plot(x_p, y_p, 'ok', 'MarkerSize',6, 'MarkerFaceColor','k');  % bob
    
    % Draw ground reference line
    plot([-axis_lim axis_lim], [track_y track_y], '-','Color',[0.5 0.5 0.5]);
    
    xlim([-axis_lim axis_lim]);
    ylim([track_y-0.2, axis_lim]);
    title(sprintf('t = %.2f s, x_c=%.2f m, theta=%.2f rad', tgrid(k), xc, wrapToPi(theta)));
    axis off;
    drawnow;
end

%% ============ NESTED FUNCTION DEFINITIONS ============

    function J = obj(z)
        % Objective function: minimize control effort + terminal state error
        %
        % J = \int _0^T u^2 dt + (x(T) - x_des)^T Q_f (x(T) - x_des)
        %
        % Uses Simpson's rule for numerical integration of u^2
        
        % Extract decision variables
        X = reshape(z(1:nX), nx, N+1);
        Xm = reshape(z(nX+1:nX+nXm), nx, N);
        U = reshape(z(nX+nXm+1:end), nu, N+1);
        
        % Running cost: \int (u^2)dt integrated via Simpson's rule
        % Simpson's rule: \int _a^b f(x)dx  approx. (b-a)/6 * [f(a) + 4*f((a+b)/2) + f(b)]
        J_run = 0;
        for kk = 1:N
            uk = U(:,kk);         % control at left endpoint
            uk1 = U(:,kk+1);      % control at right endpoint
            um = 0.5*(uk + uk1);  % approximate midpoint control
            % Simpson's formula applied to (u^2) over interval [kk, kk+1]
            J_run = J_run + (dt/6)*(uk'*uk + 4*(um'*um) + uk1'*uk1);
        end
        J_run = R_u * J_run;  % scale by weight R_u
        
        % Terminal cost: penalize error from desired final state
        xN = X(:,end);  % final state at time T
        % Handle angle wraparound: ensure we measure error modulo 2π
        ang_err = wrapToPi(xN(3) - xf_des(3));
        xerr = [xN(1)-xf_des(1); xN(2)-xf_des(2); ang_err; xN(4)-xf_des(4)];
        J_final = xerr' * Qf * xerr;
        
        % Total cost
        J = J_run + J_final;
    end

    function [c, ceq] = nonlcon(z)
        % Nonlinear constraint function: enforce dynamics via collocation constraints
        %
        % Two constraint types per interval k:
        %   1) Midpoint consistency: x_m  approx. (x_k + x_{k+1})/2 + (dt/8)(f_k - f_{k+1})
        %   2) Simpson defect: x_{k+1} - x_k - (dt/6)(f_k + 4*f_m + f_{k+1}) = 0
        % Plus initial condition: x(0) = x_d0
        %
        % All constraints are equality (c is empty, ceq contains all constraints)
        
        X = reshape(z(1:nX), nx, N+1);
        Xm = reshape(z(nX+1:nX+nXm), nx, N);
        U = reshape(z(nX+nXm+1:end), nu, N+1);
        
        % Total number of equality constraints:
        % - 2*N*nx from interval constraints (2 per interval × N intervals)
        % - 1*nx from initial condition
        ceq = zeros(nx*(2*N) + nx, 1);
        cnt = 0;
        
        for kk = 1:N
            % Retrieve states and controls for this interval
            xk = X(:,kk);
            xk1 = X(:,kk+1);
            xm = Xm(:,kk);
            uk = U(:,kk);
            uk1 = U(:,kk+1);
            um = 0.5*(uk + uk1);  % approximate midpoint control
            
            % Evaluate dynamics at three points
            fk = cartpole_dynamics(xk, uk);     % dynamics at left endpoint
            fk1 = cartpole_dynamics(xk1, uk1);  % dynamics at right endpoint
            fm = cartpole_dynamics(xm, um);     % dynamics at midpoint
            
            % Constraint 1: Midpoint consistency
            % The midpoint state should be consistent with cubic Hermite interpolation
            % x_m = (x_k + x_{k+1})/2 + (dt/8)*(f_k - f_{k+1})
            mid_cons = xm - 0.5*(xk + xk1) - (dt/8)*(fk - fk1);
            ceq(cnt + (1:nx)) = mid_cons;
            cnt = cnt + nx;
            
            % Constraint 2: Simpson integration defect
            % Enforce that the next state is consistent with Simpson's integration:
            % x_{k+1} = x_k + (dt/6)*(f_k + 4*f_m + f_{k+1})
            defect = xk1 - xk - (dt/6)*(fk + 4*fm + fk1);
            ceq(cnt + (1:nx)) = defect;
            cnt = cnt + nx;
        end
        
        % Constraint 3: Initial condition (must match x(0) = xd0)
        ceq(cnt + (1:nx)) = X(:,1) - xd0;
        
        c = [];  % no inequality constraints
    end

    function xd = cartpole_dynamics(x,u)
        % Cart-pole system dynamics: xdot = f(x, u)
        %
        % State: x = [xc; xc_dot; theta; theta_dot]
        % Control: u = force applied to cart (horizontal)
        % Output: xd = [xc_dot; xc_ddot; theta_dot; theta_ddot]
        
        xc = x(1);          % cart position
        xc_dot = x(2);      % cart velocity
        theta = x(3);       % pendulum angle from vertical
        theta_dot = x(4);   % pendulum angular velocity
        
        s = sin(theta);
        cth = cos(theta);
        
        % Standard cart-pole equation of motion:
        % Denominator in acceleration expressions
        denom = (M + m*(1 - cth^2));
        
        % Temporary variable (reduces code repetition)
        temp = (u + m*l*(theta_dot^2)*s) / denom;
        
        % Angular acceleration (standard derivation using Lagrangian mechanics)
        theta_ddot = (g*s - cth*temp) / (l*(4/3 - (m*(cth^2))/denom));
        
        % Linear acceleration of cart
        xc_ddot = temp - (m*l*theta_ddot*cth)/denom;
        
        % Return state derivative as column vector
        xd = [xc_dot; xc_ddot; theta_dot; theta_ddot];
    end

    function a_wrapped = wrapToPi(x)
        % Wrap angle to [-π, π] range
        % This prevents discontinuities when plotting angles
        % Example: 2π becomes 0, 3π/2 becomes -π/2
        a_wrapped = mod(x + pi, 2*pi) - pi;
    end

end % end main function