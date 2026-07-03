function F = shootingConstT_unified(lambda0, x0, argsStruct)
% shootingConstT_unified  Compute the shooting residual for the constant-thrust
% optimal control problem.
%
%   F = shootingConstT_unified(lambda0, x0, argsStruct)
%
%   This function builds the residual vector for the boundary value problem
%   solver. It supports both minimum-time and fixed-time formulations, and
%   handles fixed and free final position cases.
%
% Inputs:
%   lambda0    - initial guess for costates and optionally final anomaly/time
%   x0         - initial state vector [rx; ry; rz; vx; vy; vz; m]
%   argsStruct - structure containing problem data and solver options
%
% Output:
%   F          - vector of residuals for the shooting problem

    %% Unpack constants used throughout the function
    muEarth = argsStruct.muEarth;
    Tmax = argsStruct.Tmax;
    g0 = argsStruct.g0;
    Isp = argsStruct.Isp;

    %% Construct the initial augmented state z0 and final anomaly theta_f
    if argsStruct.TrustSolve == 0
        % Standard solve mode: costates are recovered from the decision vector
        if argsStruct.minT == 1
            tf_minT = argsStruct.tf_max; % maximum candidate final time for min-time
        end

        if argsStruct.FixedFinalPos == 1
            % Fixed final position: only costates are part of lambda0
            L0 = costate_from_D(lambda0(1:end));
            theta_f = 1e-9; % no free anomaly variable in this case
        else
            % Free final position: last element of lambda0 is the target anomaly
            L0 = costate_from_D(lambda0(1:end-1));
            theta_f = lambda0(end);
        end

        z0 = [L0(2:end); x0]; % initial costates (without L0(1)) plus initial state
    else
        % Trust-region solve mode: initial costates are provided directly
        L0 = argsStruct.L0;
        if argsStruct.minT == 1
            tf_minT = lambda0(end);     % final time is a decision variable here
            lambda0 = lambda0(1:end-1);
        end

        if argsStruct.FixedFinalPos == 1
            z0 = [lambda0(1:end); x0];
            theta_f = 1e-9; % fixed final position does not use theta_f
        else
            z0 = [lambda0(1:end-1); x0];
            theta_f = lambda0(end); % free final position uses theta_f
        end
    end

    %% Minimum-time Hamiltonian or fixed-time integration
    if argsStruct.minT == 1
        %% Minimum-time problem: evaluate the Hamiltonian and terminal residuals
        L = z0(1:7);              % costate vector [Lrx; Lry; Lrz; Lvx; Lvy; Lvz; Lm]
        x = z0(8:end);            % state vector [rx; ry; rz; vx; vy; vz; m]
        nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));   % square of velocity costate norm
        nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));  % square of position magnitude

        % Thrust direction from costates (normalized velocity costate)
        ax = L(4)/sqrt(nLv);
        ay = L(5)/sqrt(nLv);
        az = L(6)/sqrt(nLv);
        mC = x(end); % instantaneous spacecraft mass

        % Penalty parameters for safety or path constraint enforcement
        p_min = 1;
        rho_s = argsStruct.rho_s;
        a = argsStruct.a;
        b = argsStruct.b;

        % Unpack state and costate components for readability
        rx = x(1); ry = x(2); rz = x(3);
        vx = x(4); vy = x(5); vz = x(6);
        Lrx = L(1); Lry = L(2); Lrz = L(3);
        Lvx = L(4); Lvy = L(5); Lvz = L(6);
        p = (rx^2)/(a^2) + (ry^2)/(b^2);

        kSig = 100; % smoothing gain for the penalty function

        % Hamiltonian for the minimum-time problem with a smooth path penalty
        H = Lrx*vx + Lry*vy + Lrz*vz ...
            - Lvx*((muEarth*rx)/nPos^(3/2) - (Tmax*ax)/mC) ...
            - Lvy*((muEarth*ry)/nPos^(3/2) - (Tmax*ay)/mC) ...
            - Lvz*((muEarth*rz)/nPos^(3/2) - (Tmax*az)/mC) ...
            - (L(7)*Tmax)/(Isp*g0) ...
            + L0(1)*(rho_s*(rx^2/a^2 - p_min + ry^2/b^2)^2) ...
              /(exp(kSig*(rx^2/a^2 - p_min + ry^2/b^2)) + 1) ...
            + L0(1);
        % Note: the penalty term is a smooth approximation and L0(1) enters as
        % an additional constant contribution.

        if abs(H) < 0.1
            % If the Hamiltonian is nearly zero, integrate at the candidate final
            % time to compute terminal conditions for the shooting residual.
            [z, ~, x_tf, ~, ~, tDone, L0_final] = integrateAndComputeTerminal_minT(z0, tf_minT, ...
                muEarth, argsStruct.Tmax, Isp, g0, argsStruct.epsilon, L0, ...
                theta_f, argsStruct.at, argsStruct.et, argsStruct.it, ...
                argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal, argsStruct.TrustSolve);

            if abs(tDone - tf_minT) < 1e-3
                % Avoid a false zero residual when the integration ends at the
                % same final time as the candidate.
                H = 1e+6;
            end
        else
            % If Hamiltonian is not near zero, use a short integration 
            [z, ~, x_tf, ~, ~, tDone, L0_final] = integrateAndComputeTerminal_minT(z0, 2, ...
                muEarth, argsStruct.Tmax, Isp, g0, argsStruct.epsilon, L0, ...
                theta_f, argsStruct.at, argsStruct.et, argsStruct.it, ...
                argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal, argsStruct.TrustSolve);
            H = H * 2e+9;
        end

    else
        % Fixed-time problem: integrate the unified Hamiltonian dynamics directly
        xf_fixed = argsStruct.xf;
        [x_tf, z] = solveODE_unify(z0, x0, xf_fixed(1:end-1), argsStruct.tf, ...
            muEarth, argsStruct.Tmax, Isp, g0, argsStruct.epsilon, L0);
    end

    %% Build terminal shoot residuals for free or fixed final position
    if argsStruct.FixedFinalPos == 0
        % Free final position: compute target state in ECI and transversality term
        [xf_free, ~, ~] = StatesInECI(theta_f, argsStruct.at, argsStruct.et, ...
            argsStruct.it, argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal);
        [F1, F2] = dxf_dtheta(theta_f, argsStruct.at, argsStruct.et, ...
            argsStruct.it, argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal);

        cFinal = z(end, 1:3) * F1 + z(end, 4:6) * F2;
        F = [x_tf(1:end-1) - xf_free; cFinal];
    else
        % Fixed final position: only state matching residual is required
        cFinal = 1e-9;
        F1 = 1e-9;
        F2 = 1e-9;
        xf_free = 1e-9;

        xf_fixed = argsStruct.xf;
        F = [x_tf(1:end-1) - xf_fixed(1:end-1)];
    end

    %% Append final conditions depending on formulation and solve mode
    if argsStruct.minT == 1
        if argsStruct.TrustSolve == 0
            % Append Hamiltonian residual for minimum-time solutions
            F = [F; H];
        else
            % Trust-region mode adds final mass and costate normalization residuals
            F = [F; z(end,length(x0)); ...
                    norm([z(1,1:length(x0)), L0_final]) - 1];
        end
    else
        % For fixed-time problems, append final mass costate residual
        F = [F; z(end,length(x0))];
    end
end

function [x_tf, z] = solveODE_unify(z0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon, L0)
% solveODE_unify  Integrate the unified Hamiltonian ODE for a fixed-time
% constant-thrust problem and return the final state.

    options = odeset('RelTol',1e-11, 'AbsTol',1e-11, 'Stats','off');
    % Suppress ode45 warning messages during the solve
    warning('off', 'MATLAB:ode45:IntegrationTolNotMet');
    warning('off', 'MATLAB:ode45:IntegrationFailed');
    warning('off', 'MATLAB:odewarn:IntegrationTolNotMet');

    [~, z] = ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1)), ...
        [0 tf], z0, options);

    % Extract the final state vector from the integrated trajectory
    x_tf = z(end, length(xf) + 2 : end)';
end
