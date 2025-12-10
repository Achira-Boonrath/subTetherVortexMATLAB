function cart_pole_TPBVP_indirect
% cart_pole_TPBVP_indirect
% Indirect optimal control via TPBVP (bvp4c) for cart-pole swing-up / stabilization.
%
% OVERVIEW:
%   This script converts the optimal control problem into necessary conditions
%   using Pontryagin's Minimum Principle (PMP) and solves the resulting 
%   two-point boundary-value problem (TPBVP) using MATLAB's bvp4c solver.
%
% OPTIMAL CONTROL PROBLEM:
%   Minimize:  J = \int_0^T R_u * u^2 dt + (x(T)-x_f)' Qf (x(T)-x_f)
%   Subject to: xdot = f(x,u)  (cart-pole dynamics)
%               u_min <= u <= u_max
%               x(0) = x0, x(T) free
%
% SOLUTION APPROACH (Indirect Method):
%   1. Form Hamiltonian: H = 0.5*R_u*u^2 + lambda' * f(x,u)
%   2. PMP optimality: dH/du = 0  =>  u* = -0.5*(lambda' * f_u) / R_u
%   3. Costate dynamics: lambda_dot = -(dH/dx)' = -(df/dx)' * lambda
%   4. Boundary conditions: x(0)=x0, lambda(T)=2*Qf*(x(T)-x_f)
%   5. Solve augmented system [x; lambda] as TPBVP via bvp4c
%
% NUMERICAL APPROACH:
%   - Jacobians df/dx and df/du are computed with central finite differences
%   - Control saturation is applied (clipping). Strict bang-bang switching 
%     structure is NOT explicitly enforced; convergence depends on initial guess
%
% Author: adapted from user's HS collocation code
% Date: 2025

clear; close all; clc;

%% ==================== SYSTEM PARAMETERS ====================
% Physical parameters of the cart-pole system
M = 1.0;      % cart mass (kg)
m = 0.1;      % pendulum bob mass (kg)
l = 0.5;      % pendulum length (m)
g = 9.81;     % gravitational acceleration (m/s^2)

%% ==================== OPTIMAL CONTROL PROBLEM SETUP ====================
% Time horizon and discretization
T = 2.5;         % total time horizon (s)
N = 120;         % number of grid points for initial mesh (increase for finer resolution)
tmesh = linspace(0, T, N+1);  % initial mesh [0, T] with N+1 points

nx = 4;  % number of states: [x_c, x_c_dot, theta, theta_dot]
nu = 1;  % number of control inputs: u (cart force)

% INITIAL CONDITION: pendulum hanging down, cart at rest
xc0 = 0;           % cart position (m)
xdotc0 = 0;        % cart velocity (m/s)
theta0 = pi;       % pendulum angle (rad) - pi = downward
thetadot0 = 0;     % pendulum angular velocity (rad/s)
x0 = [xc0; xdotc0; theta0; thetadot0];

% DESIRED FINAL STATE: upright position, at rest
xc_des = 0;        % desired cart position (m)
xdotc_des = 0;     % desired cart velocity (m/s)
theta_des = 0;     % desired angle (rad) - 0 = upright
thetadot_des = 0;  % desired angular velocity (rad/s)
xf_des = [xc_des; xdotc_des; theta_des; thetadot_des];

% CONTROL CONSTRAINTS
u_min = -50;  % minimum force (N)
u_max = 50;   % maximum force (N)

% COST FUNCTION WEIGHTS
R_u = 1e-3;                      % penalty on control magnitude (running cost)
Qf = diag([500,50,400,40]);      % terminal state penalty: diag([x_c, x_c_dot, theta, theta_dot])
                                 % Large weights on position (500) and angle (400) errors

%% ==================== bvp4c SETUP ====================
% The augmented state vector for bvp4c is:
%   y = [x1, x2, x3, x4, lambda1, lambda2, lambda3, lambda4]'
% where x = [x_c, x_c_dot, theta, theta_dot] and lambda is the costate vector

% Initial guess for STATE: linear interpolation from initial to desired state
Xinit = zeros(nx, numel(tmesh));
for k=1:numel(tmesh)
    s = (k-1)/(numel(tmesh)-1);  % normalized time parameter [0,1]
    Xinit(:,k) = (1-s)*x0 + s*xf_des;  % linear path from x0 to xf_des
end

% Initial guess for COSTATE: assume zero everywhere (typically a rough guess)
Laminit = zeros(nx, numel(tmesh));

% Create initial solution structure for bvp4c using interpolation function
solinit = bvpinit(tmesh, @(t) guess_y(t, tmesh, Xinit, Laminit));

% Configure bvp4c solver options
options = bvpset('RelTol',1e-5, 'AbsTol',1e-6, 'NMax',5000);
% RelTol, AbsTol: relative and absolute tolerance for solution accuracy
% NMax: maximum number of mesh points to prevent excessive refinement

fprintf('Solving TPBVP with bvp4c (may take a moment)...\n');
sol = bvp4c(@odesys, @bcs, solinit, options);
fprintf('bvp4c finished.\n');

% Extract solution on dense grid for plotting/analysis
t_dense = linspace(0,T,300);  % 300 points for smooth plotting
Y = deval(sol, t_dense);      % evaluate solution structure at t_dense
X_sol = Y(1:nx, :);           % extract state component [x_c, x_c_dot, theta, theta_dot]
Lam_sol = Y(nx+1:end, :);     % extract costate component [lambda1, ..., lambda4]

% Recover the optimal control u(t) along the entire trajectory
U_sol = zeros(1, size(X_sol,2));
for k = 1:size(X_sol,2)
    xk = X_sol(:,k);      % state at time t_dense(k)
    lamk = Lam_sol(:,k);  % costate at time t_dense(k)
    u_uncl = optimal_control_from_lambda(xk, lamk, R_u);  % unconstrained optimal u
    U_sol(k) = max(u_min, min(u_max, u_uncl));  % apply saturation constraints
end

%% ==================== PLOTTING RESULTS ====================
figure('Name','Cart-Pole Indirect TPBVP Solution','NumberTitle','off');

subplot(4,1,1);  % Cart position
plot(t_dense, X_sol(1,:), 'LineWidth', 1.2); 
ylabel('x_c (m)'); grid on; hold on;

subplot(4,1,2);  % Cart velocity
plot(t_dense, X_sol(2,:), 'LineWidth', 1.2); 
ylabel('x_c dot (m/s)'); grid on; hold on;

subplot(4,1,3);  % Pendulum angle (wrapped to [-pi, pi])
plot(t_dense, wrapToPi(X_sol(3,:)), 'LineWidth', 1.2); 
ylabel('\theta (rad)'); grid on; hold on;

subplot(4,1,4);  % Optimal control input
plot(t_dense, U_sol, 'LineWidth', 1.2); 
ylabel('u (N)'); xlabel('time (s)'); grid on; hold on;

sgtitle('Cart-Pole Indirect TPBVP Solution (bvp4c)');

%% ==================== ANIMATION ====================
figure('Name','Cart-Pole Animation (Indirect)','NumberTitle','off');
cart_w = 0.3;           % cart width (m)
cart_h = 0.15;          % cart height (m)
track_y = -0.2;         % y-position of the track
axis_lim = 1.2*max(1, max(abs(X_sol(1,:))) + l);  % symmetric axis limits

% Select subset of frames for animation (80 frames)
idxs = round(linspace(1,length(t_dense),80));

for ii = idxs
    clf;
    xc = X_sol(1,ii);      % cart position at frame ii
    theta = X_sol(3,ii);   % pendulum angle at frame ii
    
    % Draw cart as a rectangle
    rectangle('Position',[xc-cart_w/2, track_y, cart_w, cart_h], ...
              'FaceColor',[0.2 0.6 0.8]);
    hold on;
    
    % Compute pendulum bob position
    x_p = xc + l * sin(theta);         % bob x-coordinate
    y_p = track_y + cart_h + l * cos(theta);  % bob y-coordinate
    
    % Draw rod (line from cart to bob)
    plot([xc, x_p], [track_y+cart_h, y_p], '-k', 'LineWidth',2);
    
    % Draw bob (point mass)
    plot(x_p, y_p, 'ok', 'MarkerSize',6, 'MarkerFaceColor','k');
    
    % Draw ground/track line
    plot([-axis_lim axis_lim], [track_y track_y], '-', 'Color',[0.5 0.5 0.5]);
    
    xlim([-axis_lim axis_lim]); 
    ylim([track_y-0.2, axis_lim]);
    title(sprintf('t = %.2f s', t_dense(ii)));
    axis off; drawnow;
end

%% ==================== NESTED / LOCAL FUNCTIONS ====================

    function dydt = odesys(t, y)
        % ODE system for the augmented state [x; lambda]
        % Implements the necessary conditions from Pontryagin's Minimum Principle
        %
        % Inputs:
        %   t: current time (used for event detection, not explicit t-dependence here)
        %   y: augmented state [x(1:4); lambda(5:8)]
        %
        % Outputs:
        %   dydt: time derivative [xdot; lambda_dot]
        
        x = y(1:nx);           % extract state vector
        lam = y(nx+1:end);     % extract costate vector
        
        % STEP 1: Compute df/du (how dynamics respond to control)
        % Using central finite differences: df/du ≈ [f(x,eps) - f(x,-eps)] / (2*eps)
        eps_fd = 1e-6;
        fu_plus = cartpole_dynamics(x, eps_fd);      % f(x, +eps)
        fu_minus = cartpole_dynamics(x, -eps_fd);    % f(x, -eps)
        df_du = (fu_plus - fu_minus)/(2*eps_fd);     % df/du (4x1 vector)
        
        % STEP 2: Compute unconstrained optimal control from PMP
        % Optimality condition: dH/du = R_u*u + (df/du)'*lambda = 0
        % Therefore: u* = -0.5*(lambda' * df/du) / R_u
        u_uncl = - (lam' * df_du) / (2*R_u);
        
        % Apply control saturation constraints
        u = max(u_min, min(u_max, u_uncl));
        
        % STEP 3: Evaluate dynamics at current (x, u)
        f_xu = cartpole_dynamics(x, u);
        
        % STEP 4: Compute Jacobian df/dx (how dynamics respond to state changes)
        % Using central finite differences for each state variable
        J = zeros(nx,nx);
        for ii = 1:nx
            dxi = zeros(nx,1); dxi(ii) = eps_fd;  % perturbation in i-th state
            f_plus = cartpole_dynamics(x + dxi, u);   % f(x+eps*e_i, u)
            f_minus = cartpole_dynamics(x - dxi, u);  % f(x-eps*e_i, u)
            J(:,ii) = (f_plus - f_minus) / (2*eps_fd);  % column ii of df/dx
        end
        
        % STEP 5: Propagate state and costate
        % State derivative: simply the dynamics xdot = f(x,u)
        xdot = f_xu;
        
        % Costate derivative: lambda_dot = -(df/dx)' * lambda
        % (no running state cost, only control cost and terminal state cost)
        lam_dot = - J' * lam;
        
        % Return time derivatives for both state and costate
        dydt = [xdot; lam_dot];
    end

    function res = bcs(ya, yb)
        % Boundary conditions for the TPBVP
        % Enforces initial state constraint and terminal costate constraint
        %
        % Inputs:
        %   ya: augmented state at t=0: [x(0); lambda(0)]
        %   yb: augmented state at t=T: [x(T); lambda(T)]
        %
        % Outputs:
        %   res: 8x1 residual vector (should be zero at solution)
        
        xa = ya(1:nx);                  % initial state
        xb = yb(1:nx);                  % final state
        lambdab = yb(nx+1:end);         % final costate
        
        % INITIAL CONDITION: x(0) = x0
        bc1 = xa - x0;
        
        % TERMINAL CONDITION (from PMP transversality): 
        % lambda(T) = gradient of terminal cost = 2*Qf*(x(T) - x_f_des)
        % Special handling for angle error: wrap to [-pi, pi] to avoid large jumps
        ang_err = wrapToPi(xb(3) - xf_des(3));
        x_err = [xb(1)-xf_des(1); xb(2)-xf_des(2); ang_err; xb(4)-xf_des(4)];
        bc2 = lambdab - 2*(Qf * x_err);
        
        % Combine all boundary condition residuals
        res = [bc1; bc2];
    end

    function yguess = guess_y(t, tmesh_in, Xinit_in, Laminit_in)
        % Provide an initial guess for the solution at an arbitrary time t
        % Used by bvpinit to create the initial solution structure for bvp4c
        %
        % Inputs:
        %   t: time at which to evaluate the guess
        %   tmesh_in, Xinit_in, Laminit_in: precomputed arrays defining guess trajectory
        %
        % Outputs:
        %   yguess: [x(t); lambda(t)] interpolated at time t
        
        xg = zeros(nx,1);   % state component of guess
        lg = zeros(nx,1);   % costate component of guess
        
        % Linear interpolation for each state and costate component
        for i=1:nx
            xg(i) = interp1(tmesh_in, Xinit_in(i,:), t, 'linear');
            lg(i) = interp1(tmesh_in, Laminit_in(i,:), t, 'linear');
        end
        
        yguess = [xg; lg];
    end

    function u = optimal_control_from_lambda(x, lam, R_u_local)
        % Recover the optimal control from current state and costate
        % (applies same formula as in odesys, used for post-processing)
        %
        % Inputs:
        %   x: current state vector (4x1)
        %   lam: current costate vector (4x1)
        %   R_u_local: control cost weight
        %
        % Outputs:
        %   u: saturated optimal control input
        
        % Compute df/du numerically
        eps_fd = 1e-6;
        fu_plus = cartpole_dynamics(x, eps_fd);
        fu_minus = cartpole_dynamics(x, -eps_fd);
        df_du = (fu_plus - fu_minus)/(2*eps_fd);
        
        % Apply PMP optimality: u* = -0.5*(lambda'*df/du) / R_u
        u_uncl = - (lam' * df_du) / (2*R_u_local);
        
        % Enforce control bounds
        u = max(u_min, min(u_max, u_uncl));
    end

    function xd = cartpole_dynamics(x,u)
        % Computes the state derivative for the cart-pole system
        % using nonlinear dynamics equations
        %
        % Inputs:
        %   x: state vector [x_c, x_c_dot, theta, theta_dot]'
        %   u: control input (force on cart, N)
        %
        % Outputs:
        %   xd: state derivative [x_c_dot, x_c_ddot, theta_dot, theta_ddot]'
        %
        % Dynamics derived from Lagrangian mechanics with constraints
        
        xc = x(1);         % cart position
        xc_dot = x(2);     % cart velocity
        theta = x(3);      % pendulum angle from vertical
        theta_dot = x(4);  % pendulum angular velocity
        
        s = sin(theta);    % sine of angle
        cth = cos(theta);  % cosine of angle
        
        % Common denominator in acceleration expressions
        denom = (M + m*(1 - cth^2));
        
        % Intermediate term (appears in both x_c_ddot and theta_ddot)
        temp = (u + m*l*(theta_dot^2)*s) / denom;
        
        % Pendulum angular acceleration
        theta_ddot = (g*s - cth*temp) / (l*(4/3 - (m*(cth^2))/denom));
        
        % Cart acceleration
        xc_ddot = temp - (m*l*theta_ddot*cth)/denom;
        
        % Return state derivatives
        xd = [xc_dot; xc_ddot; theta_dot; theta_ddot];
    end

    function a_wrapped = wrapToPi(x)
        % Wrap angle x to the interval [-pi, pi]
        % Useful for handling periodic variables like pendulum angle
        %
        % Inputs:
        %   x: angle in radians (can be any value)
        %
        % Outputs:
        %   a_wrapped: angle in [-pi, pi]
        
        a_wrapped = mod(x + pi, 2*pi) - pi;
    end

end
