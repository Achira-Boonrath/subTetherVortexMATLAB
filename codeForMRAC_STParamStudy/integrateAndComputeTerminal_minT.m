function [z, xf, x_tf, cFinal, H, tDone, L0_final] = integrateAndComputeTerminal_minT(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, ...
    theta_f, at, et, it, Omega_t, omega_t, muVal, trustSolve)
% Helper that integrates the Hamiltonian ODE and computes terminal constraints.

if nargin <16
    trustSolve = 0;
end

    function [position,isterminal,direction] = appleEventsFcn(t,z)
        % position = (abs(z(7)) > 1e-6) && ~isnan(z(7)) && (z(end) > 0.1) && (length(z) == 14);...)
        position = (abs(z(7)) > 1e-4) && ~isnan(z(7)) && (z(end) > 0.1) && (length(z) == 14);...)

        position = (max( abs(z(1:7)) ) < 1e+7) && position;
        % 
        % argsCons = struct('a', a, 'b', b, 'p_min', p_min, 'rho_s', rho_s, ...
        %     'muEarth', muEarth, 'L0', L0(1));
        % 
        % if consOn ==1
        %     ds_costate = costate_Dyn(z(8:end), z(1:7), argsCons);
        %     position = (max( abs(ds_costate) ) < 1e+9) && position;
        % 
        %     % x = z(8:end);
        %     % rx =x(1);ry =x(2);rz =x(3);vx =x(4);vy =x(5);vz =x(6);
        %     % position= ( (rx^(2))/(a^(2))+(ry^(2))/(b^(2)) > 1 ) && position;
        % end

        % The value that we want to be zero
        if position == 0
            % disp(z(end))
        end

        isterminal = 1;  % Halt integration
        direction = 0;   % The zero can be approached from either direction
    end

L = z0(1:7);              % costates (columns correspond to [L...])
x = z0(8:end);            % states (columns correspond to [x...])
nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));

ax = L(4)/(nLv^0.5);
ay = L(5)/(nLv^0.5);
az = L(6)/(nLv^0.5);
mC = x(end);

% Set up the event function for the ODE solver
if trustSolve == 0 %tf > 1e+5
    L0_final = L0(1);
    options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Stats', 'off', 'Events', @appleEventsFcn);
    [~, z,te,~, ~] = ode23(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1), 1), [0 tf], z0, options);
    % disp(te)
    tDone = te;
else

    L0_trust = -( L(1)*x(4) + L(2)*x(5) + L(3)*x(6) - L(4)*((muEarth*x(1))/nPos^(3/2) - (Tmax*ax)/mC) - L(5)*((muEarth*x(2))/nPos^(3/2) ...
        - (Tmax*ay)/mC) - L(6)*((muEarth*x(3))/nPos^(3/2) - (Tmax*az)/mC) - (L(7)*Tmax)/(Isp*g0) );
    L0_final = L0_trust;
    options = odeset('RelTol', 1e-11, 'AbsTol', 1e-11, 'Stats', 'off', 'Events', @appleEventsFcn);
    [~, z] =         ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0_trust, 1), [0 tf], z0, options);
    tDone = tf;
end
[xf, ~, ~] = StatesInECI(theta_f, at, et, it, Omega_t, omega_t, muVal);
[F1, F2] = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, muVal);

% cFinal is the transversality condition contribution
cFinal = z(end, 1:3) * F1 + z(end, 4:6) * F2;

x_tf = z(end, length(xf) + 2 : end).'; % extract state portion at final time

%% hamiltonian for minT

H = L(1)*x(4) + L(2)*x(5) + L(3)*x(6) - L(4)*((muEarth*x(1))/nPos^(3/2) - (Tmax*ax)/mC) - L(5)*((muEarth*x(2))/nPos^(3/2) ...
    - (Tmax*ay)/mC) - L(6)*((muEarth*x(3))/nPos^(3/2) - (Tmax*az)/mC) - (L(7)*Tmax)/(Isp*g0) + L0(1);
if isempty(tDone) == 1
    tDone = tf;
    H = 1e+6;
end
end