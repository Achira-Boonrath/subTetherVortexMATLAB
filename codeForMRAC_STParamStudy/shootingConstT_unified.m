function F = shootingConstT_unified(lambda0, x0, argsStruct)

    %%
    muEarth = argsStruct.muEarth;
    Tmax = argsStruct.Tmax;
    g0 = argsStruct.g0;
    Isp = argsStruct.Isp;
    if argsStruct.TrustSolve == 0
        if argsStruct.minT == 1
            tf_minT = argsStruct.tf_max;
        end

        if argsStruct.FixedFinalPos == 1
            L0= costate_from_D( lambda0(1:end) );
            theta_f = 1e-9;
        else
            L0= costate_from_D( lambda0(1:end-1) );
            theta_f = lambda0(end);
        end
        z0 = [L0(2:end); x0];
    else
        L0 = argsStruct.L0;
        if argsStruct.minT == 1
            tf_minT = lambda0(end);
            lambda0 = lambda0(1:end-1);
        end

        if argsStruct.FixedFinalPos == 1
            z0 = [lambda0(1:end); x0];
            theta_f = 1e-9;
        else
            z0 = [lambda0(1:end-1); x0];
            theta_f = lambda0(end);
        end
    end

    %%
    if argsStruct.minT == 1
        % Call helper to integrate and compute terminal constraints
        [z, ~, x_tf, ~, ~, tDone] = integrateAndComputeTerminal_minT(z0, tf_minT, ...
            muEarth, argsStruct.Tmax, Isp, g0, argsStruct.epsilon, L0, ...
            theta_f, argsStruct.at, argsStruct.et, argsStruct.it, argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal);
        
        %% hamiltonian for minT
        L = z0(1:7);              % costates (columns correspond to [L...])
        x = z0(8:end);            % states (columns correspond to [x...])
        nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
        nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));        

        ax = L(4)/(nLv^0.5);
        ay = L(5)/(nLv^0.5);
        az = L(6)/(nLv^0.5);
        mC = x(end);        

        p_min = 1;
        rho_s = argsStruct.rho_s;
        a = argsStruct.a;
        b = argsStruct.b;

        rx =x(1);ry =x(2);rz =x(3);vx =x(4);vy =x(5);vz =x(6);
        Lrx =L(1);Lry =L(2);Lrz =L(3);Lvx =L(4);Lvy =L(5);Lvz =L(6);
        p = ( (rx^(2))/(a^(2))+(ry^(2))/(b^(2)) );

        H = L(1)*x(4) + L(2)*x(5) + L(3)*x(6) - L(4)*((muEarth*x(1))/nPos^(3/2)...
         - (Tmax*ax)/mC) - L(5)*((muEarth*x(2))/nPos^(3/2) - (Tmax*ay)/mC) ...
         - L(6)*((muEarth*x(3))/nPos^(3/2) - (Tmax*az)/mC) - (L(7)*Tmax*g0)/Isp ...
         + rho_s*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2 + L0(1);

    else
        xf_fixed = argsStruct.xf;
        [x_tf, z] = solveODE_unify(z0, x0, xf_fixed(1:end-1), argsStruct.tf, ...
            muEarth, argsStruct.Tmax, Isp, g0, argsStruct.epsilon, L0);
        
    end

    if argsStruct.FixedFinalPos == 0
        [xf_free, ~, ~] = StatesInECI(theta_f, argsStruct.at, argsStruct.et, argsStruct.it, argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal);
        [F1, F2] = dxf_dtheta(theta_f, argsStruct.at, argsStruct.et, argsStruct.it, argsStruct.Omega_t, argsStruct.omega_t, argsStruct.muVal);
        % cFinal is the transversality condition contribution
        cFinal = z(end, 1:3) * F1 + z(end, 4:6) * F2;

        F = [x_tf(1:end-1) - xf_free; cFinal];  % terminal error
    else
        cFinal = 1e-9;
        F1 = 1e-9;
        F2 = 1e-9;
        xf_free = 1e-9;
        
        xf_fixed = argsStruct.xf;
        F = [x_tf(1:end-1) - xf_fixed(1:end-1)];  % terminal error
    end

%%
    if argsStruct.minT == 1
        if argsStruct.TrustSolve == 0
            F = [F; H];
        else
            F = [F; z(end,length(x0));...
                    norm([z(1,1:length(x0)), L0])-1];   
        end
    else
        F = [F; z(end,length(x0))];
    end

end

function [x_tf, z] = solveODE_unify(z0, x0, xf, tf, muEarth, Tmax, Isp, g0, epsilon, L0)
    options = odeset('RelTol',1e-11, 'AbsTol',1e-11, 'Stats', 'off');
    % Disable all ode45-related warnings
    warning('off', 'MATLAB:ode45:IntegrationTolNotMet');
    warning('off', 'MATLAB:ode45:IntegrationFailed');
    warning('off', 'MATLAB:odewarn:IntegrationTolNotMet');
    warning('off', 'MATLAB:ode15s:IntegrationTolNotMet'); % sometimes triggered internally
    [~, z] = ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1)), [0 tf], z0, options);
    x_tf = z(end, length(xf) + 2  : end)'; % extract state portion at final time

end
