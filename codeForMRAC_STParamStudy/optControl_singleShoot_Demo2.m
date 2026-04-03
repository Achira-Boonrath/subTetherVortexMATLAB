function [t,z] = optControl_singleShoot_Demo2(lambda0_guess,x0,tf,muEarth,Tmax,Isp,g0,epsilon,options,tInit)
% optControl_singleShoot_2body
% Solves a single-shootingConstT optimal control problem
% The Hamiltonian system (state + costate) is integrated and fsolve is used
% to find the initial costates that satisfy the terminal state constraint.
%
% Notes:
% - State X = [x; y; z; vx; vy; vz; m] (Position, Velocity, Mass)
% - Control U = [ux; uy; uz; u_throttle] (Thrust direction unit vector and throttle)
% - Costates L = [lambda_r; lambda_v; lambda_m] (7x1 vector)
% - Problem: drive x0 -> xf in time tf minimizing fuel consumption
%   (Bang-bang control via smoothing strategy)
%
% This script expects helper functions in the path:
% - odeDynAndLag_constT : returns symbolic xdot and Ldot expressions (dynamics and costate ODEs)
% - hamiltonian_odeConstT : evaluates the combined ODE for [lambda; x] given numeric z
% - shootingConstT        : residual function for terminal constraints, used by fsolve
% close all; clear all; clc
propulsionType = 'chemical'; % Options: 'chemical', 'electric'
% propulsionType = 'electric'; % Options: 'chemical', 'electric'
%%

%% Problem data

% Symbolic substitution based on propulsion type
muVal = muEarth;     % Earth
uTest = 1;
minT = 1;
switch propulsionType
    case 'chemical'
        % chem prop

        r0 = norm(x0(1:3));
        aTrans = ( (tf/pi)^2 * muVal )^(1/3);
        rf = 2*aTrans - r0;
        xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 x0(7)-200]';             % Desired terminal state at t = tf (Mass is free)

        % if minT ==0
        lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
            ]'; % thetaf
        ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
            ]';% thetaf
        % else
        %     lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
        %         0,    -1]'; % thetaf
        %     ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
        %         2*pi, 1]';% thetaf
        % end

        % lambda0_guess = ub;

    case 'electric'
        % electric prop

        disp("initial radius")
        norm( x0(1:3) )
        r0 = norm(x0(1:3));

        % if minT ==0
        lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
            0]'; % thetaf
        ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
            2*pi]';% thetaf
        % else
        %     lb =  [0, 0, 0, 0, 0, 0, 0,... % costates D
        %         0,    -1]'; % thetaf
        %     ub =  [1, 1, 1, 1, 1, 1, 1,...% costates D
        %         2*pi, 1]';% thetaf
        % end

        lambda0_guess = lb;
end

%% Quick test: evaluate the Hamiltonian ODE at a sample state vector
% This calls the numeric ODE wrapper to produce dz/dt for a sample input.
% dzdt = hamiltonian_ode(0, ones(length(x0)+length(x0), 1), funcSubs);

% muEarth is passed in

switch propulsionType
    case 'chemical'
        % Tmax and Isp are passed in
    case 'electric'
        Tmax=0.1*1e-3; %in kN
        Isp=3000;

        % Additional parameters for electric prop
        at = 42164.0;             % semi-major axis [km]
        rf = at;
        et = 0.001;                % eccentricity
        it = deg2rad(2.0);        % inclination
        Omega_t = deg2rad(0);   % RAAN
        omega_t = deg2rad(0);   % argument of periapsis
end

% g0=9.81*1e-3; %in km/s^2
% epsilon=1;
ds = hamiltonian_odeConstT(0, [x0*10.1;x0], muVal, Tmax, Isp, g0, epsilon, uTest);

%% Solve two-point boundary value problem via shootingConstT
% First try a single invocation of shootingConstT with the initial guess

switch propulsionType
    case 'chemical'
        F = shootingConstT(lambda0_guess, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon);
    case 'electric'
        % Update lambda0_guess for electric case (adds theta_f)
        if minT ==0
            F = shootingConstTFreeTheta(lambda0_guess,x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        else
            F = shootingConstTFreeTheta_minT(lambda0_guess,x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        end
        diagF_Inv = diag(1);

        % F((log10(abs(F))< 0)) = 1;
        % % diagF_Inv = diag(1./abs(F) ).^2;
        % diagF_Inv = diag(1./abs(F) );
end
%%  Use particleswarm to find lambda0 that makes the terminal state match xf

switch propulsionType
    case 'chemical'
        objfun = @(lam0) norm(shootingConstT(lam0', x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon))^2;
    case 'electric'
        % objfun = @(lam0) norm(shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t))^2;
        if minT ==0
            objfun = @(lam0) shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)' ...
                * diagF_Inv * shootingConstTFreeTheta(lam0', x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        else
            objfun = @(lam0) shootingConstTFreeTheta_minT(lam0', x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t)' ...
                * diagF_Inv * shootingConstTFreeTheta_minT(lam0', x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t);
        end
end

nvars = length(lambda0_guess);

% opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 500, 'MaxIterations', 3, "UseParallel", true);
% opts = optimoptions('particleswarm', 'Display', 'iter', "SwarmSize", 500, 'MaxIterations', 50, "UseParallel", true);
% lambda0_guess = particleswarm(objfun, nvars, lb, ub, opts);
% lambda0_guess = lambda0_guess';
%% Now use fsolve to find lambda0 that makes the terminal state match xf
switch propulsionType
    case 'chemical'
        lambda0 = fsolve(@(lam0) shootingConstT(lam0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon), ...
            lambda0_guess, ...
            optimoptions('fsolve', 'Display', 'off', "MaxFunctionEvaluations", 3000));
    case 'electric'
        if minT == 0
            L0_guess = costate_from_D( lambda0_guess(1:end-1) );
            lambda0_guess = [L0_guess(2:end); lambda0_guess(end)] ;

            lambda0 = fsolve(@(lam0) shootingConstTFreeTheta_Trust(lam0, x0, tf, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0_guess(1)), ...
                lambda0_guess, ...
                optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 3000));
        else
            % L0_guess = costate_from_D( lambda0_guess(1:end-2) );
            % lambda0_guess = [L0_guess(2:end); lambda0_guess(end-1:end)] ;

            % lambda0_guess = u0';
            % L0_guess(1) = 1.1745500794600847e-7;

            L0_guess= costate_from_D( lambda0_guess(1:end-1) );
            % Call helper to integrate and compute terminal constraints
            [~, ~, ~, ~, H, tDone]= integrateAndComputeTerminal_minT([L0_guess(2:end); x0], 1e+6, muEarth, Tmax, Isp, g0, epsilon, L0_guess, lambda0_guess(end), ...
                at, et, it, Omega_t, omega_t, muVal);

            lambda0_guess = [L0_guess(2:end); lambda0_guess(end); tDone ];

            lambda0 = fsolve(@(lam0) shootingConstTFreeTheta_Trust_minT(lam0, x0, muEarth, Tmax, Isp, g0, epsilon, muVal, at, et, it ,Omega_t, omega_t, L0_guess(1)), ...
                lambda0_guess, ...
                optimoptions('fsolve', 'Display', 'iter', "MaxFunctionEvaluations", 2000));
        end
end
% lambda0-u0'

%% Integrate the Hamiltonian system with the solved initial costates
% Stack initial condition z0 = [lambda0; x0] to integrate the 8-D ODE
% z0 = [lambda0; x0];
switch propulsionType
    case 'chemical'
        L0= costate_from_D( lambda0(1:end) );
        z0 = [L0(2:end); x0];
        timeFinal = linspace(0, tf, 300);
        [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon ), timeFinal, z0, options);
    case 'electric'

        if minT ==0
            L0= costate_from_D( lambda0(1:end-1) );
            z0 = [L0(2:end); x0];
            timeFinal = linspace(0, tf, 300);
            [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1)), timeFinal, z0, options);
        else
            lambda0 = lambda0(1:end-1);
            L0= costate_from_D( lambda0(1:end-1) );
            z0 = [L0(2:end); x0];

            timeFinal = linspace(0, tf, 300);
            [t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1), 1), timeFinal, z0, options);
        end
end
t = t + tInit(end);