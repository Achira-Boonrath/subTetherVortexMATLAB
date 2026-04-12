function optControlSingleShooting
% SINGLE-SHOOTING OPTIMAL CONTROL USING ode45 FOR INVERTED PENDULUM
%
% This code solves the same OCP as the previous HS collocation version,
% but now uses a single-shooting approach.
%
% EXPLANATION OF METHOD:
% This code Solves the Optimal Control Problem (OCP) using the "Single Shooting"
% method.
%
% 1. Decision Variables (z):
%    The ONLY decision variables are the control inputs U at each time step.
%    The states X are NOT decision variables.
%    z = [u_0; u_1; ...; u_N]
%
% 2. Propagation (Forward Shooting):
%    To evaluate the cost or constraints for a given guess of U, we must
%    simulate (integrate) the ODE system forward from the initial condition x_0.
%    This is done using ode45 sequentially across all N intervals.
%
% 3. Constraints:
%    - Control bounds are handled directly by fmincon.
%    - Terminal Constraints (x_N == x_final) are calculated by checking the
%      final state of the forward propagation.
%
% 4. Comparison to Collocation:
%    - Pros: Much smaller optimization vector (only U).
%    - Cons: The dynamics constraints are satisfied implicitly by ode45, but
%      this creates a highly nonlinear relationship between u_0 and x_N.
%      This "tail-wagging-the-dog" sensitivity often makes convergence difficult
%      for unstable systems or long time horizons compared to collocation.
%
% The states are propagated with ode45 from initial condition using the control decision variables.

    clear; close all; clc;
    
    %% Pendulum parameters
    m = 1.0; l = 1.0; g = 9.81;
    I = m*l^2;
    
    %% Problem setup
    T = 2.0;     % total time
    N = 40;      % intervals
    dt = T/N;
    tgrid = linspace(0,T,N+1);
    
    nx = 2; nu = 1;
    
    % initial and desired final states
    xd0 = [0; 0];      % down
    xf_des = [pi; 0];  % upright
    
    % control bounds
    u_min = -20; u_max = 20;
    
    % cost weights
    R_u = 1e-2;
    Qf = diag([200, 20]);
    
    %% Initial guess for controls
    U0 = zeros(N+1,1);  % zero torque
    
    lb = u_min*ones(N+1,1);
    ub = u_max*ones(N+1,1);
    
    %% fmincon options
    options = optimoptions('fmincon', ...
        'Algorithm','sqp', ...
        'Display','iter', ...
        'MaxFunctionEvaluations', 5e4, ...
        'MaxIterations', 5000, ...
        'OptimalityTolerance',1e-5, ...
        'StepTolerance',1e-9);
    
    %% Call fmincon using nested objective & constraint functions
    fprintf('Starting HS optimization with N=%d intervals, T=%.2f s\n', N, T);
    [z_opt, J_opt, exitflag, output] = fmincon(@objFun, U0, [],[],[],[], lb, ub, @nonlcon, options);
    
    fprintf('Finished. exitflag=%d, J_opt=%.6f\n', exitflag, J_opt);
    
    %% Reconstruct states using ode45
    X = zeros(nx,N+1);
    X(:,1) = xd0;
    for k = 1:N
        u = z_opt(k);
        [~,xint] = ode45(@(t,x) pendulum_dynamics(x,u), [0 dt], X(:,k));
        X(:,k+1) = xint(end,:)';
    end
    
    %% Plot
    figure;
    subplot(3,1,1); plot(tgrid, wrapToPi(X(1,:)),'-o'); ylabel('\theta'); grid on;
    subplot(3,1,2); plot(tgrid, X(2,:),'-o'); ylabel('theta dot'); grid on;
    subplot(3,1,3); stairs(tgrid, z_opt,'-o'); ylabel('u'); xlabel('t'); grid on;
    
    %% Animation
    figure;
    for k = 1:2:length(tgrid)
        theta = X(1,k);
        x_p = l * sin(theta);
        y_p = -l * cos(theta);
        plot([0 x_p], [0 y_p], '-o','LineWidth',2,'MarkerSize',8);
        axis equal;
        xlim([-1.4*l,1.4*l]); ylim([-1.4*l,1.4*l]);
        title(sprintf('t = %.2f s, theta = %.2f rad', tgrid(k), wrapToPi(theta)));
        drawnow;
    end
    %% ---------------- Nested Functions ----------------
    function J = objFun(U)
        % COST FUNCTION (Objective for fmincon)
        %
        % This function computes the scalar cost J for a given control
        % sequence U = [u_0; u_1; ...; u_N]. The cost has two components:
        %  1) Running cost: integral over time of (R_u * u(t)^2). Since the
        %     control is piecewise-constant over each interval of length dt,
        %     the integral reduces to dt * sum(U.^2), then scaled by R_u.
        %  2) Terminal cost: quadratic penalty on the deviation of the
        %     final state x_N from the desired final state xf_des, using
        %     weight matrix Qf. The pendulum angle error is wrapped to
        %     [-pi,pi] before computing the penalty to avoid artificial
        %     large-angle errors due to periodicity.
        %
        % Implementation details:
        %  - The states are not optimization variables in single shooting.
        %    To evaluate the terminal cost we must propagate the dynamics
        %    forward starting from xd0, applying each control U(k) for a
        %    duration dt. Propagation is done with ode45 over each interval.
        %  - Using ode45 for each interval gives a more accurate state
        %    propagation than simple Euler stepping, but is more expensive.
        %
        % Inputs:
        %  - U : (N+1)x1 vector of control inputs applied sequentially.
        % Output:
        %  - J : scalar objective value

        % Running cost: trapezoidal rule is unnecessary for piecewise-constant
        % control; simple rectangular integration (dt * sum(U.^2)) suffices.
        J_run = dt * sum(U.^2);
        J_run = R_u * J_run;  % scale by control weight

        % Forward propagate to obtain terminal state x_N
        x = xd0;  % start at initial condition
        for k = 1:N
            u_k = U(k);                                 % control on interval k
            % Integrate dynamics from t=0 to t=dt with constant input u_k.
            % Using a short integration window (0..dt) for each interval
            % preserves the correct timing while keeping step sizes small.
            [~, xint] = ode45(@(t,x) pendulum_dynamics(x, u_k), [0 dt], x);
            x = xint(end, :)';                          % state at end of interval
        end
        xN = x;  % final state after applying all controls up to t = T-dt

        % Compute terminal state error.
        % - For the angular component use wrapToPi to handle 2*pi periodicity.
        ang_err = wrapToPi(xN(1) - xf_des(1));
        vel_err = xN(2) - xf_des(2);
        xerr = [ang_err; vel_err];

        % Quadratic terminal cost
        J_final = xerr' * Qf * xerr;

        % Total cost
        J = J_run + J_final;
    end

    function [c, ceq] = nonlcon(U)
        % NONLINEAR CONSTRAINTS for fmincon
        %
        % This function returns inequality constraints c(U) <= 0 and
        % equality constraints ceq(U) == 0. For the single-shooting
        % formulation:
        %  - There are no inequality constraints here (control bounds are
        %    supplied separately via lb/ub).
        %  - We impose the terminal equality constraint: the propagated
        %    final state x_N must equal the desired final state xf_des.
        %
        % Implementation details:
        %  - Because the final state depends on the entire control
        %    sequence in a highly nonlinear way, we propagate the system
        %    forward exactly as done in objFun to compute x_N for the
        %    provided U. The equality constraint is then x_N - xf_des = 0.
        %
        % Inputs:
        %  - U : (N+1)x1 vector of control inputs.
        % Outputs:
        %  - c   : inequality constraints (empty here)
        %  - ceq : equality constraints (2x1 vector enforcing final state)

        % Forward propagate to obtain terminal state x_N (same as in objFun)
        x = xd0;
        for k = 1:N
            u_k = U(k);
            [~, xint] = ode45(@(t,x) pendulum_dynamics(x, u_k), [0 dt], x);
            x = xint(end, :)';
        end

        % Equality constraint: final state matches desired final state.
        % Note: we do not wrap the angle inside ceq because fmincon expects
        % continuous equality constraints; wrapping would produce
        % discontinuities. If angle wrapping is important here, one should
        % instead enforce sin/cos consistency or use inequality tolerances.
        ceq = x - xf_des;

        % No inequality constraints
        c = [];
    end

    function xd = pendulum_dynamics(x, u)
        % PENDULUM DYNAMICS (continuous-time ODE)
        %
        % Simple inverted pendulum about a pivot with control torque u.
        % State vector x = [theta; theta_dot], where theta is the angle
        % measured from the downward vertical (xd0 = [0;0]). The dynamics:
        %   theta_dot = x(2)
        %   theta_ddot = (u - m*g*l*sin(theta))/I
        %
        % Inputs:
        %  - x : 2x1 state vector
        %  - u : scalar control torque applied at the pivot
        % Output:
        %  - xd : 2x1 derivative [theta_dot; theta_ddot]

        theta = x(1);
        theta_dot = x(2);
        theta_ddot = (u - m*g*l*sin(theta)) / I;
        xd = [theta_dot; theta_ddot];
    end

    function a_wrapped = wrapToPi(x)
        % WRAP ANGLE TO [-pi, pi)
        %
        % Utility function to map any real angle x into the principal
        % interval [-pi, pi). This avoids large artificial angular errors
        % when computing differences between angles that are equivalent
        % modulo 2*pi.
        a_wrapped = mod(x + pi, 2*pi) - pi;
    end
end
