function optControl_DirectUnify(method_choice, system_choice, enforced_term_states, cost_choice)
    % UNIFIED DIRECT OPTIMAL CONTROL
    % Solves the Optimal Control Problem (OCP) using either:
    %  Method:
    %  1) 'shooting'      - Direct Single Shooting
    %  2) 'collocation'   - Hermite-Simpson Direct Collocation
    %
    %  System:
    %  1) 'pendulum'      - Simple Inverted Pendulum
    %  2) 'cartpole'      - Cart-Pole
    %  3) 'ejection'      - Restore Ejection
    %  4) 'CWH'      - CWH RND
    %
    %  enforced_term_states: Array of state indices to strictly enforce at the final time.
    %  cost_choice: 'effort' or 'time'. Setting 'time' makes the final time T an optimization variable.
    %
    % Usage:
    %   optControl_DirectUnify('shooting', 'pendulum')
    %   optControl_DirectUnify('collocation', 'cartpole', [1, 3]) % Enforce only position & angle
    %   optControl_DirectUnify() % Defaults to collocation, pendulum, all states enforced

    if nargin < 1
        method_choice = 'collocation'; % default method
    end
    if nargin < 2
        system_choice = 'pendulum';    % default system
    end
    if nargin < 3
        enforced_term_states = [];     % Flag for default initialization after nx is known
    end
    if nargin < 4
        cost_choice = 'effort';        % 'effort' or 'time'
    end
    
    close all; clc;
    fprintf('=== Unified Optimal Control ===\n');
    fprintf('Method: %s\n', upper(method_choice));
    fprintf('System: %s\n', upper(system_choice));
    
    %% Common Problem Setup
    T_guess = 3600*0.5;         % default time guess (s)
    N = 300;                % number of intervals
    
    % Depending on cost_choice, T may vary:
    if strcmp(cost_choice, 'time')
        lb_T = 0.05;       % minimum physical time allowed
        ub_T = 10.0;       % maximum physical time allowed
    else
        lb_T = T_guess;    % strictly fix time 
        ub_T = T_guess;    
    end
    
    %% System-specific settings
    if strcmp(system_choice, 'pendulum')
        % Pendulum parameters
        sys_params.m = 1.0;        % mass (kg)
        sys_params.l = 1.0;        % length (m)
        sys_params.g = 9.81;       % gravity (m/s^2)
        sys_params.I = sys_params.m * sys_params.l^2; % moment of inertia
        
        nx = 2;  % states: [theta; theta_dot]
        nu = 1;  % control: torque
        
        % Initial and desired states
        xd0 = [0; 0];
        xf_des = [pi; 0];
        
        % Bounds
        u_min = -20; u_max = 20;
        theta_min = -4*pi; theta_max = 4*pi;
        thetadot_min = -50; thetadot_max = 50;
        
        lb_state = [theta_min; thetadot_min];
        ub_state = [theta_max; thetadot_max];
        
        % Cost weights
        R_u = 1e-2;
        Qf = diag([200, 20]);
        
    elseif strcmp(system_choice, 'cartpole')
        % Cartpole parameters
        sys_params.M = 2.0; % cart mass (kg)
        sys_params.m = 0.5; % pole mass (kg)
        sys_params.l = 0.5; % pole length from cart to center of mass (m)
        sys_params.g = 9.81; % gravity (m/s^2)
        sys_params.I = sys_params.m * sys_params.l^2; % pole inertia

        dist = 0.8;  %How far must the cart translate during its swing-up
        maxForce = 100;  %Maximum actuator forces
                
        nx = 4;  % states: [x; x_dot; theta; theta_dot]
        nu = 1;  % control: cart linear force
        
        % Initial condition (pendulum down, at x=0)
        xd0 = [0; 0; 0; 0];
        % Desired final condition (upright, at x=0)
        xf_des = [dist; 0; pi; 0];
        
        % Bounds
        u_min = -maxForce; u_max = maxForce; % higher force bounds for cartpole

        x_min = -2*dist; x_max = 2*dist;
        xdot_min = -inf; xdot_max = inf;
        theta_min = -2*pi; theta_max = 2*pi;
        thetadot_min = -inf; thetadot_max = inf;
        
        lb_state = [x_min; xdot_min; theta_min; thetadot_min];
        ub_state = [x_max; xdot_max; theta_max; thetadot_max];
        
        % Cost weights
        R_u = 1;
        Qf = (1e-9)*diag([1, 1, 1, 1]); % Penalize position, velocity, angle, angular velocity
        
    elseif strcmp(system_choice, 'ejection')
        m_net05 = 0.5458517235612266/2; % Half of net mass
        numMU  = 2;
        
        % Length parameter
        % Mass H
        % Mass P
        % Force parameter
        sys_params.l_0 = 28.28427125;              
        % sys_params.m_H = 0.5 + m_net05;  
        sys_params.m_H = 3 + m_net05;   
        sys_params.m_P = 7 + m_net05/2;            
        sys_params.F_T = numMU.*22;                 
        
        nx = 4;  % states: [yH; yH_dot; psi; psi_dot]
        nu = 1;  % control: phi
        
        xd0 = [0; 0; deg2rad(15); 0];
        % xf_des = [0; 0; pi/2; 0];
        xf_des = [0; -36; pi/2; 0];
        
        u_min = 0;   
        u_max = pi/2;    
        
        x_min = -inf;        x_max = inf;              
        xdot_min = -inf;    xdot_max = inf;        
        theta_min = 0;  theta_max = pi/2;       
        thetadot_min = -inf;  thetadot_max = inf;      
        
        lb_state = [x_min; xdot_min; theta_min; thetadot_min];
        ub_state = [x_max; xdot_max; theta_max; thetadot_max];
        
        R_u = 1;
        Qf = (1e-9)*diag([1e-3, 1e-3, 1e-3, 1e-3]);
    elseif strcmp(system_choice, 'CWH')
        % Cartpole parameters
        meanMotion = 0.001;
        sys_params.n = meanMotion; 
                
        nx = 4;  % states: [x; y; x_dot; y_dot]
        nu = 2;  % control: cart linear force
        
        % Initial condition (pendulum down, at x=0)
        xd0 = [0; -1500; 0; 0];
        % Desired final condition (upright, at x=0)
        xf_des = [0; 0; 0; 0];
        
        % Bounds
        maxForce = 1e+6;  % No force limits for CWH in this example
        u_min = -[maxForce; maxForce]; u_max = [maxForce; maxForce]; % higher force bounds for cartpole

        x_min = -1e+6; x_max = 1e+6;
        y_min = -1e+6; y_max = 1e+6;
        vx_min = -1e+6; vx_max = 1e+6;
        vy_min = -1e+6; vy_max = 1e+6;
        
        lb_state = [x_min; y_min; vx_min; vy_min];
        ub_state = [x_max; y_max; vx_max; vy_max];
        
        % Cost weights
        R_u = 1;
        Qf = (1e-9)*diag([1, 1, 1, 1]); % Penalize position, velocity, angle, angular velocity
        
    else
        error('Unknown system_choice. ');
    end

    % Set default enforced terminal states if not provided (enforce everything)
    if isempty(enforced_term_states)
        enforced_term_states = 1:nx;
    end

    %% fmincon options
    % 'Algorithm','interior-point', ...
            % 'Display','iter');
    options = optimoptions('fmincon', ...
        'Algorithm','interior-point', ...
        'Display','iter', ...
        'MaxFunctionEvaluations', 1e5, ...
        'MaxIterations', 5000, ...
        'OptimalityTolerance',1e-5, ...
        'StepTolerance',1e-9);
        
    %% Method Selection & Optimization Execution
    if strcmp(method_choice, 'shooting')
        % --- DIRECT SINGLE SHOOTING SETUP ---
        % (we use N intervals but include controls at both endpoints to allow
        % simple reconstruction/plotting). z = [U(1:N+1); T_var].
        U0 = zeros(nu, N+1);              % initial guess: zero torque/force
        Z0 = [U0(:); T_guess];             % full decision variable
        lb = [repmat(u_min, N+1, 1); lb_T]; % lower bound on each control knot + Time bounds
        ub = [repmat(u_max, N+1, 1); ub_T]; % upper bound on each control knot + Time bounds
        
        % Solve with fmincon:
        % - objective computes integral cost by simulating forward with the
        %   candidate control sequence (using same integrator as reconstruction)
        % - nonlinear constraints enforce terminal state penalty or equality
        %   conditions as implemented in nonlcon_shooting
        tic;
        [z_opt, J_opt, exitflag, output] = fmincon(@objFun_shooting, Z0, [],[],[],[], lb, ub, @nonlcon_shooting, options);
        toc;
        
        fprintf('Finished. exitflag=%d, final J=%.6f\n', exitflag, J_opt);
        
        U_opt = reshape(z_opt(1:end-1), nu, N+1);  % store controls
        T_opt = z_opt(end);
        dt = T_opt / N;
        tgrid = linspace(0, T_opt, N+1);
        tgrid_mid = tgrid(1:end-1) + dt/2;

        % Reconstruct state trajectory from optimal control using numerical
        % integration (ode23) over each interval of length dt. This produces
        % X_opt(:,k) for k=1..N+1 corresponding to knots t=0:dt:T_opt.
        X_opt = zeros(nx,N+1);
        X_opt(:,1) = xd0;
        for k = 1:N
            u_k = U_opt(:,k);  % apply optimal control 
            [~,xint] = ode23(@(t_var,x_ode) system_dynamics(x_ode, u_k), [0 dt], X_opt(:,k));
            X_opt(:,k+1) = xint(end,:)'; % take state at end of interval
        end
        Xm_opt = [];     % midpoints are not used in shooting approach

    elseif strcmp(method_choice, 'collocation')
        % --- HERMITE-SIMPSON COLLOCATION SETUP ---
        % Decision vector z comprises:
        %   - X at N+1 knots: nX = nx*(N+1)
        %   - Xm at N midpoints: nXm = nx*N
        %   - U at N+1 knots: nU = nu*(N+1)
        % Total variables: nz = nX + nXm + nU
        nX = nx*(N+1);
        nXm = nx * N;
        nU = nu*(N+1);
        nz = nX + nXm + nU;
        
        % Build a continuous initial guess:
        % - X0: linear interpolation between xd0 and xf_des across knots
        X0 = zeros(nx, N+1);
        for k=1:N+1
            s = (k-1)/N;
            X0(:,k) = (1-s)*xd0 + s*xf_des;  % linear interpolation for all states
        end
        % - Xm0: midpoint initial guess as average of neighboring knots
        Xm0 = zeros(nx, N);
        for k=1:N
            Xm0(:,k) = 0.5*(X0(:,k)+X0(:,k+1));
        end
        % - U0_colloc: zero control initial guess at knots
        U0_colloc = zeros(nu, N+1);
        
        % Decision vector includes T_var at the very end
        nz_total = nz + 1;
        z0 = [X0(:); Xm0(:); U0_colloc(:); T_guess];
        
        % Prepare bounds for the full decision vector:
        % Default to unbounded and then assign meaningful bounds blockwise.
        lb = -inf(nz_total,1);
        ub =  inf(nz_total,1);
        
        % Knot state bounds: apply physical limits for each state at knots
        for k=1:N+1
            idx_x = (k-1)*nx + (1:nx);   % linear index into z for knot k
            lb(idx_x) = lb_state;        % lower bounds per state
            ub(idx_x) = ub_state;        % upper bounds per state
        end
        % Midpoint state bounds: same state limits apply at midpoints
        offset_Xm = nX;
        for k=1:N
            idx_xm = offset_Xm + (k-1)*nx + (1:nx);
            lb(idx_xm) = lb_state;
            ub(idx_xm) = ub_state;
        end
        % Control bounds at all knot points
        offset_U = nX + nXm;
        for k=1:N+1
            idx_u = offset_U + (k-1)*nu + (1:nu);
            lb(idx_u) = u_min;
            ub(idx_u) = u_max;
        end
        % Time bounds
        lb(end) = lb_T;
        ub(end) = ub_T;
        
        % Call fmincon with objective and nonlinear constraints tailored to
        % collocation. The objective integrates cost over the trajectory
        % using the collocated states/controls, while nonlcon_collocation
        % enforces Hermite-Simpson defect constraints and boundary conditions.
        tic;
        [z_opt, J_opt, exitflag, output] = fmincon(@objFun_collocation, z0, [], [], [], [], lb, ub, @nonlcon_collocation, options);
        toc;
        
        fprintf('Finished. exitflag=%d, final J=%.6f\n', exitflag, J_opt);
        
        % Unpack the optimized decision vector 
        X_opt = reshape(z_opt(1:nX), nx, N+1);                     
        Xm_opt = reshape(z_opt(nX+1:nX+nXm), nx, N);               
        U_opt = reshape(z_opt(nX+nXm+1:end-1), nu, N+1);           
        
        T_opt = z_opt(end);
        dt = T_opt / N;
        tgrid = linspace(0, T_opt, N+1);
        tgrid_mid = tgrid(1:end-1) + dt/2;
    else
        error('Unknown method_choice. Use ''shooting'' or ''collocation''.');
    end
    
    %% Plot
    figure('Position', [100, 100, 800, 800]); % make larger for subplots
    
    if strcmp(system_choice, 'pendulum')
        subplot(4,1,1)
        plot(tgrid, wrapToPi(X_opt(1,:)), '-o', 'LineWidth', 1.2); hold on;
        if strcmp(method_choice, 'collocation')
            plot(tgrid_mid, wrapToPi(Xm_opt(1,:)), 'o', 'MarkerSize', 4);
        end
        ylabel('\theta (rad)'); grid on; title(sprintf('Pendulum Solution using %s', method_choice));
        plot(tgrid, ones(size(tgrid))*pi, '--k', 'HandleVisibility', 'off');
        
        subplot(4,1,2);
        plot(tgrid, X_opt(2,:), '-o', 'LineWidth', 1.2); hold on;
        if strcmp(method_choice, 'collocation')
            plot(tgrid_mid, Xm_opt(2,:), 'o', 'MarkerSize', 4);
        end
        ylabel('theta dot (rad/s)'); grid on;
        
        subplot(4,1,3);
        stairs(tgrid, U_opt', '-o', 'LineWidth', 1.2);
        ylabel('u (Nm)'); grid on;
        
        subplot(4,1,4);
        if strcmp(method_choice, 'collocation')
            Umid = zeros(nu,N);
            for k=1:N
                Umid(:,k) = 0.5*(U_opt(:,k) + U_opt(:,k+1));
            end
            plot(tgrid_mid, Umid', '-o', 'LineWidth', 1.1);
            ylabel('u_{mid} (Nm)'); xlabel('time (s)'); grid on;
        else
            stairs(tgrid(1:end-1), U_opt(:, 1:end-1)', '-o', 'LineWidth', 1.1);
            ylabel('u piecewise (Nm)'); xlabel('time (s)'); grid on;
        end
        
    elseif strcmp(system_choice, 'cartpole') || strcmp(system_choice, 'ejection') || strcmp(system_choice, 'CWH')
        subplot(3,2,1);
        plot(tgrid, X_opt(1,:), '-o', 'LineWidth', 1.2); hold on;
        if strcmp(method_choice, 'collocation')
            plot(tgrid_mid, Xm_opt(1,:), 'o', 'MarkerSize', 4);
        end
        if strcmp(system_choice, 'CWH')
            ylabel('x (m)'); title('CWH: x-position');
        else
            ylabel('x (m)'); title(sprintf('%s (%s): Position', upper(system_choice), method_choice));
        end
        grid on;
        
        subplot(3,2,3);
        plot(tgrid, X_opt(2,:), '-o', 'LineWidth', 1.2); hold on;
        if strcmp(method_choice, 'collocation')
            plot(tgrid_mid, Xm_opt(2,:), 'o', 'MarkerSize', 4);
        end
        if strcmp(system_choice, 'CWH')
            ylabel('y (m)'); title('CWH: y-position');
        else
            ylabel('x dot (m/s)');
        end
        grid on;
        
        subplot(3,2,2);
        if strcmp(system_choice, 'CWH')
            plot(tgrid, X_opt(3,:), '-o', 'LineWidth', 1.2); hold on;
            if strcmp(method_choice, 'collocation')
                plot(tgrid_mid, Xm_opt(3,:), 'o', 'MarkerSize', 4);
            end
            ylabel('vx (m/s)'); title('CWH: x-velocity');
        else
            plot(tgrid, wrapToPi(X_opt(3,:)), '-o', 'LineWidth', 1.2); hold on;
            if strcmp(method_choice, 'collocation')
                plot(tgrid_mid, wrapToPi(Xm_opt(3,:)), 'o', 'MarkerSize', 4);
            end
            ylabel('\theta (rad)'); title('Pole Angle');
            plot(tgrid, ones(size(tgrid))*pi, '--k', 'HandleVisibility', 'off');
        end
        grid on;
        
        subplot(3,2,4);
        plot(tgrid, X_opt(4,:), '-o', 'LineWidth', 1.2); hold on;
        if strcmp(method_choice, 'collocation')
            plot(tgrid_mid, Xm_opt(4,:), 'o', 'MarkerSize', 4);
        end
        if strcmp(system_choice, 'CWH')
            ylabel('vy (m/s)'); title('CWH: y-velocity');
        else
            ylabel('theta dot (rad/s)');
        end
        grid on;
        
        % controls span bottom row
        subplot(3,2,[5,6]);
        stairs(tgrid, U_opt', '-o', 'LineWidth', 1.2);
        ylabel('Force u (N)'); xlabel('time (s)'); grid on; title('Control Output');
    end
    
    %% exact solu for u
    if strcmp(system_choice, 'CWH')
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
        % syms tau real                         % Symbolic variable for integration
        
        % Compute the controllability Gramian g for [t0, tgrid(end)] using matrix exponentials
        fun = @(tau) expm(A*(t0-tau)) * B * (B.') * expm(A.'*(t0-tau));
        g = integral(fun, 0, tgrid(end), 'ArrayValued', true);

        % Compute the exact optimal control using the analytical solution for LQR with fixed final state
        for jj = 1:length(tgrid)
            % solution (from t0)
            uExact(jj, :) = - (B') * expm(A' * (0 - tgrid(jj))) * pinv(g) * (xd0 - expm(A * (0 - tgrid(end))) * xf_des);
        end
        
        % Plot the relative error between the numerically computed and exact controls
        figure;
        % plot((U_opt' - uExact) ./ U_opt')
        plot((U_opt' - uExact));
        grid minor
        title('Relative Error: (u - uExact) / uExact')
    end
    %% Animation
    figure;
    for k = 1:2:length(tgrid)
        if strcmp(system_choice, 'pendulum')
            theta_val = X_opt(1,k);
            x_p = sys_params.l * sin(theta_val);
            y_p = -sys_params.l * cos(theta_val);
            
            plot([0 x_p], [0 y_p], '-o', 'LineWidth', 2, 'MarkerSize', 8);
            axis equal;
            xlim([-1.4*sys_params.l, 1.4*sys_params.l]); ylim([-1.4*sys_params.l, 1.4*sys_params.l]);
            title(sprintf('Pendulum: t=%.2f s, \\theta=%.2f rad', tgrid(k), wrapToPi(theta_val)));
            
        elseif strcmp(system_choice, 'cartpole')
            x_cart = X_opt(1,k);
            theta_val = X_opt(3,k);
            x_p = x_cart + sys_params.l * sin(theta_val);
            y_p = -sys_params.l * cos(theta_val);
            
            % Draw cart
            cw = 0.8; ch = 0.4; % cart width/height
            cart_rect = [x_cart - cw/2, -ch/2, cw, ch];
            rectangle('Position', cart_rect, 'FaceColor', [0.3 0.5 0.7]); hold on;
            
            % Draw pole
            plot([x_cart x_p], [0 y_p], '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', 'k');
            
            % Draw path track reference
            plot([-10 10], [-ch/2 -ch/2], 'k-', 'LineWidth', 1);
            hold off;
            
            axis equal;
            xlim([-4, 4]); ylim([-2, 2]); % Adjust view boundaries logic depending on results
            title(sprintf('Cartpole: t=%.2f s, x=%.2f m, \\theta=%.2f rad', tgrid(k), x_cart, wrapToPi(theta_val)));
        elseif strcmp(system_choice, 'ejection')
            yH = X_opt(1,k);
            psi = X_opt(3,k);
            x_p = yH + sys_params.l_0 * sin(psi);
            y_p = -sys_params.l_0 * cos(psi);
            
            % plot([yH x_p], [0 y_p], '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', 'k'); hold on;
            % plot([-50 50], [0 0], 'k-', 'LineWidth', 1); hold off;
            % axis equal;
            % xlim([-40, 40]); ylim([-40, 40]); 
            % title(sprintf('Ejection: t=%.2f s, yH=%.2f m, \\psi=%.2f rad', tgrid(k), yH, psi));
        elseif strcmp(system_choice, 'CWH')
            x_pos = X_opt(1,k);
            y_pos = X_opt(2,k);
            
            plot(x_pos, y_pos, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); hold on;
            plot(0, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % target
            hold off;
            axis equal;
            xlim([-2000, 2000]); ylim([-2000, 2000]); 
            title(sprintf('CWH: t=%.2f s, x=%.2f m, y=%.2f m', tgrid(k), x_pos, y_pos));
        end
        drawnow;
    end
    
    %% ---------------- Common Nested System Dynamics ----------------
    function xd = system_dynamics(x, u)
        if strcmp(system_choice, 'pendulum')
            theta_val = x(1);
            theta_dot = x(2);
            theta_ddot = (u - sys_params.m * sys_params.g * sys_params.l * sin(theta_val)) / sys_params.I;
            xd = [theta_dot; theta_ddot];
            
        elseif strcmp(system_choice, 'cartpole') % cartpole
            % states: [x, x_dot, theta, theta_dot]
            theta_val = x(3);
            theta_dot = x(4);
            
            M = sys_params.M;
            m = sys_params.m;
            l = sys_params.l;
            g = sys_params.g;
            I = sys_params.I;
            
            % Equations of motion derived via Lagrangian for Cart-pole
            % A * [x_ddot; theta_ddot] = B
            A = [M + m,           m*l*cos(theta_val);
                 m*l*cos(theta_val), I];
             
            B = [u + m*l*theta_dot^2*sin(theta_val);
                 -m*g*l*sin(theta_val)];
                 
            % solve for accelerations
            accel = A \ B;
            x_ddot = accel(1);
            theta_ddot = accel(2);
            
            xd = [x(2); x_ddot; theta_dot; theta_ddot];
            
        elseif strcmp(system_choice, 'ejection')
            phi = u; 
            psi = x(3);       
            psi_dot = x(4);  
            
            m_P = sys_params.m_P;
            l_0 = sys_params.l_0;
            F_T = sys_params.F_T;
            m_H = sys_params.m_H;

            A_eq = -2 .* m_P .* l_0 .* psi_dot.^2 .* cos(psi) - 2 .* F_T .* cos(phi);
            B_eq = 2 .* F_T .* l_0 .* (sin(phi) .* cos(psi) - cos(phi) .* sin(psi));
            common_den = l_0 .* (-2 .* m_P .* sin(psi).^2 + m_H + 2 .* m_P);
            yH_ddot = - (B_eq .* sin(psi) - A_eq .* l_0)./ common_den;
            psi_ddot = (B_eq .* m_H + 2 .* B_eq .* m_P - 2 .* A_eq .* l_0 .* m_P .* sin(psi))./ ...
               (2 .* m_P .* l_0.* common_den);
            
            xd = [x(2); yH_ddot; psi_dot; psi_ddot];
        elseif strcmp(system_choice, 'CWH')
            xx =  x(1);
            yy =  x(2);
            vx = x(3);
            vy = x(4);

            ax = u(1);
            ay = u(2);

            n = sys_params.n;
            f = [vx; ...
                vy; ...
                3*n^2*xx + 2*n*vy + ax; ...
                -2*n*vx + ay; ...
                ];
            
            xd = f;            
        end
        
    end

    function a_wrapped = wrapToPi(ang)
        if strcmp(system_choice, 'ejection')
            a_wrapped = ang;
        else
            a_wrapped = mod(ang + pi, 2*pi) - pi;
        end
    end

    function xerr = get_xerr(xN)
        % Helper to correctly wrap theta for terminal cost penalty
        if strcmp(system_choice, 'pendulum')
            ang_err = wrapToPi(xN(1) - xf_des(1));
            vel_err = xN(2) - xf_des(2);
            xerr = [ang_err; vel_err];
        elseif strcmp(system_choice, 'CWH')
            xerr = xN - xf_des;
        else
            ang_err = wrapToPi(xN(3) - xf_des(3));
            xerr = [xN(1)-xf_des(1); xN(2)-xf_des(2); ang_err; xN(4)-xf_des(4)];
        end
    end
    
    %% ---------------- Single Shooting Nested Functions ----------------
    function J = objFun_shooting(Z)
        U = reshape(Z(1:end-1), nu, N+1);
        T_var = Z(end);
        dt_var = T_var / N;
        
        % Running cost (sum of squared controls over all intervals)
        J_run = dt_var * sum(U(:).^2);
        J_run = R_u * J_run; 

        % Propagate system forward using piecewise-constant controls U
        x = xd0;  % initialize state at known initial condition
        for k_ss = 1:N
            u_ss = U(:, k_ss);                                
            % Integrate system dynamics over one interval [0, dt_var] with control u_ss
            [~, xint_ss] = ode23(@(t_var,x_ode) system_dynamics(x_ode, u_ss), [0 dt_var], x);
            % Take the state at the end of the interval as the next initial state
            x = xint_ss(end, :)';                          
        end
        
        % Terminal cost: compute error with desired final state (with angle wrapping)
        xerr = get_xerr(x);
        J_final = xerr' * Qf * xerr;
        
        if strcmp(cost_choice, 'time')
            J = 1.0 * T_var;% + J_run ;%+ J_final; % min time heavily penalized
        else
            J = J_run ;%+ J_final;
        end
    end

    function [c, ceq] = nonlcon_shooting(Z)
        U = reshape(Z(1:end-1), nu, N+1);
        T_var = Z(end);
        dt_var = T_var / N;
        
        % Propagate state forward using the control sequence U
        x = xd0;
        for k_ss = 1:N
            u_ss = U(:, k_ss);
            [~, xint_ss] = ode23(@(t_ode,x_ode) system_dynamics(x_ode, u_ss), [0 dt_var], x);
            x = xint_ss(end, :)';
        end
        
        % Enforce final state equality constraint only on selected states
        ceq = x(enforced_term_states) - xf_des(enforced_term_states);
        c = [];
    end

    %% ---------------- Collocation Nested Functions ----------------
    function J = objFun_collocation(z)
        % Objective function for direct collocation transcription.
        % Input:
        %   z - optimization vector containing:
        %       - X_coll flattened (nx*(N+1) entries): state at nodes
        %       - Xm_coll flattened (nx*N entries): state at collocation midpoints (not used here)
        %       - U_coll flattened (nu*(N+1) entries): control at nodes
        % Output:
        %   J - scalar cost composed of running cost (numeric quadrature over control)
        %       plus terminal cost on final state.
        %
        % Running cost approximation uses Simpson-like (3-point) rule per interval:
        % (dt_var/6)*(u_k^2 + 4*u_mid^2 + u_{k+1}^2) summed over intervals, scaled by R_u.
        
        % Extract variables from optimization vector
        X_coll = reshape(z(1:nX), nx, N+1);                     
        U_coll = reshape(z(nX+nXm+1:end-1), nu, N+1);             
        
        T_var = z(end);
        dt_var = T_var / N;

        % Numerical quadrature of control effort over all intervals
        J_run = 0;
        for kk = 1:N
            uk  = U_coll(:,kk);        
            uk1 = U_coll(:,kk+1);      
            um_coll  = 0.5*(uk + uk1); % midpoint control (for Simpson-like rule)
            J_run = J_run + (dt_var/6)*(uk'*uk + 4*(um_coll'*um_coll) + uk1'*uk1);
        end
        J_run = R_u * J_run; 

        % Terminal cost: penalty on deviation of final state from desired final state
        xN = X_coll(:,end);
        xerr = get_xerr(xN);           
        J_final = xerr' * Qf * xerr;           

        if strcmp(cost_choice, 'time')
            J = 1.0 * T_var; %+ J_run ;%+ J_final;
        else
            J = J_run ;%+ J_final;
        end
    end
    
    function [c, ceq] = nonlcon_collocation(z)
        % Nonlinear constraints for direct collocation transcription.
        % Enforces collocation (midpoint) constraints and defect constraints so
        % the discrete trajectory approximates continuous dynamics.
        %
        % Input:
        %   z - optimization vector containing X_coll, Xm_coll, U_coll
        % Outputs:
        %   c   - inequality constraints (empty)
        %   ceq - equality constraints stacking:
        %         - midpoint consistency: Xm - 0.5*(Xk + Xk+1) - (dt_var/8)*(fk - fk1) = 0
        %         - defect (integration) constraint: Xk+1 - Xk - (dt_var/6)*(fk + 4*fm + fk1) = 0
        %         - initial condition constraint: X(:,1) - xd0 = 0
        
        % Unpack optimization vector into state/node and control variables
        X_coll = reshape(z(1:nX), nx, N+1);                     
        Xm_coll = reshape(z(nX+1:nX+nXm), nx, N);               
        U_coll = reshape(z(nX+nXm+1:end-1), nu, N+1);             
        
        T_var = z(end);
        dt_var = T_var / N;

        % Preallocate equality constraint vector:
        % For each interval we have nx midpoint constraints and nx defect constraints,
        % so total nx*(2*N). Plus nx initial condition constraints,
        % and finally the selected terminal condition constraints.
        n_term = length(enforced_term_states);
        ceq = zeros(nx*(2*N) + nx + n_term, 1);
        cnt = 0;

        % Loop over each interval and build constraints
        for kk = 1:N
            xk  = X_coll(:,kk);        
            xk1 = X_coll(:,kk+1);      
            xm_c  = Xm_coll(:,kk);       
            uk  = U_coll(:,kk);        
            uk1 = U_coll(:,kk+1);      
            um_c  = 0.5*(uk + uk1); % midpoint control

            % Evaluate dynamics at the node and midpoint states
            fk  = system_dynamics(xk,  uk);   
            fk1 = system_dynamics(xk1, uk1);  
            fm_c  = system_dynamics(xm_c,  um_c);   

            % Midpoint consistency constraint:
            % xm = 0.5*(xk + xk1) + (dt_var/8)*(fk - fk1)
            % Rearranged to standard form: xm - 0.5*(xk + xk1) - (dt_var/8)*(fk - fk1) = 0
            mid_cons = xm_c - 0.5*(xk + xk1) - (dt_var/8)*(fk - fk1);
            ceq(cnt + (1:nx)) = mid_cons;
            cnt = cnt + nx;

            % Defect constraint (Simpson / Hermite collocation style):
            % xk+1 - xk - (dt_var/6)*(fk + 4*fm + fk1) = 0
            defect = xk1 - xk - (dt_var/6)*(fk + 4*fm_c + fk1);
            ceq(cnt + (1:nx)) = defect;
            cnt = cnt + nx;
        end

        % Initial condition equality constraint
        ceq(cnt + (1:nx)) = X_coll(:,1) - xd0;
        cnt = cnt + nx;
        
        % Terminal condition equality constraint
        if n_term > 0
            ceq(cnt + (1:n_term)) = X_coll(enforced_term_states, end) - xf_des(enforced_term_states);
        end
        c = [];
    end
end
