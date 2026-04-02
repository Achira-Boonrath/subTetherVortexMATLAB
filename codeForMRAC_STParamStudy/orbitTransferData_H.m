function [t, x, lambda, u] = orbitTransferData_H(at, theta_f, r0)
    close all; 

    %% Problem data
    
    % r0 = 780+6378;
    % rf = 1770+6378;
    muVal = 398600;
    % aTrans = (r0 + rf)/2;
    % period = 2*pi*sqrt( (aTrans^3) /muVal  );
    theta_f = deg2rad(theta_f);
    et = 0;
    %%
    x0 = [r0 0 0 0 sqrt(muVal/r0) 0 1000]';           % Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
    % xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]';             % Desired terminal state at t = tf (Mass is free)
    rf = at;
    % Radius magnitude
    p = at * (1 - et^2);
    r_mag = p / (1 + et*cos(theta_f));
    
    % Position in perifocal frame
    r_pf = r_mag * ...
    [ cos(theta_f);
      sin(theta_f);
      0 ];
    
    % Velocity scaling
    v_scale = sqrt(muVal / p);
    
    % Velocity in perifocal frame
    v_pf = v_scale * ...
    [ -sin(theta_f);
      et + cos(theta_f);
      0 ];

    % Terminal state
    xf = [r_pf; v_pf;800];

    aTrans = (rf + r0)/2;
    period = 2*pi*sqrt( (aTrans^3) /muVal  );
    tf = period*(theta_f/(2*pi));                % Final time (seconds)
    
    %% Define symbolic variables for deriving equations
    lambda0_guess = 1e-4*ones(length(x0),1);
    
    nvars = length(lambda0_guess);

    theta_f = 0;
    et = (rf - r0)/(rf + r0);
    p = aTrans * (1 - et^2);

    % Velocity scaling
    v_scale = sqrt(muVal / p);

    % Velocity in perifocal frame
    v_pf = v_scale * ...
    [ -sin(theta_f);
      et + cos(theta_f);
      0 ];
    % sqrt(muVal/rf)
    x0(4:6) = v_pf;

    muEarth=muVal;
    
    % chem prop
    Tmax=425*4;
    Isp=230; 

    g0=9.81e-3;
    epsilon=1;

        %% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
    % z0 = [lambda0; x0];
    % chem prop
    z0 = [1e-8*lambda0_guess; x0];

    % electric prop
    % z0 = [lambda0(1:end-1); x0];

    % options = odeset('RelTol', 1e-7,'AbsTol', 1e-9,'Stats','off');
    options = odeset('Stats','off');
    % Init orbit
    z0Init = [1e-8*lambda0_guess; [r0 0 0 0 sqrt(muVal/r0) 0 1000]'];
    periodInit = 2*pi*sqrt( (r0^3) /muVal  );
    timeFinalInit = linspace(0, periodInit, 45);
    [tInit, zInit] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinalInit, z0Init, options);

    % transfer orbit
    timeFinal = linspace(0, tf, 45) ;
    [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinal, z0, options);
    t = t + tInit(end);

    % F orbit
    z0F = [1e-8*lambda0_guess; [-rf 0 0 0 -sqrt(muVal/rf) 0 1000]'];
    periodF = 2*pi*sqrt( (rf^3) /muVal  );
    timeFinalF = linspace(0, periodF, 45) ;
    [tF, zF] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinalF, z0F, odeset('AbsTol', 1e-9,'Stats','off'));
    tF = tF + t(end);
    
    z = [zInit; z; zF];
    t = [tInit; t; tF];
    %%
    % Extract state and costate trajectories from integrated z
    x = z(:, (length(x0)+1):end);               % states (columns correspond to [x y vx vy])
    lambda = z(:, 1:length(x0));            % costates (columns correspond to [L(1) L(2) L(3) L(4)])
    
    % Compute control from costates. In this formulation the optimal control is
    u = zeros(length(t), 4);
    epsilon = 1;
    for i = 1:length(t)
        L_t = lambda(i, :)';
        x_t = x(i, :)';
        
        % Direction (from optUSet structure: -L_v / norm(L_v))
        L_v_norm = sqrt(L_t(4)^2 + L_t(5)^2 + L_t(6)^2);
        if L_v_norm ~= 0
            u_dir = -L_t(4:6)' / L_v_norm;
        else
            u_dir = [0 0 0];
        end
        
        % Throttle (Switching function)
        rho = 1 - (Isp * g0 * L_v_norm)/(x_t(7)) - L_t(7);
        uSwitch = 0.5 - rho/(2*epsilon);
        if rho > epsilon
            uSwitch = 0;
        elseif rho < - epsilon
            uSwitch = 1;
        end
        
        u(i, :) = [u_dir, uSwitch];
    end
end