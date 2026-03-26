function [z, xf, x_tf, cFinal, H, tDone] = integrateAndComputeTerminal_minT(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, at, et, it, Omega_t, omega_t, muVal)
% Helper that integrates the Hamiltonian ODE and computes terminal constraints.

    function [position,isterminal,direction] = appleEventsFcn(t,z)
        position = (abs(z(7)) > 1e-9 ); % The value that we want to be zero
        isterminal = 1;  % Halt integration
        direction = 0;   % The zero can be approached from either direction
    end
% Set up the event function for the ODE solver
if tf > 0.9e+6
    options = odeset('RelTol', 1e-11, 'AbsTol', 1e-11, 'Stats', 'off', 'Events', @appleEventsFcn);
    [~, z,te,~, ~] = ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1), 1), [0 tf], z0, options);
    tDone = te;
    if isempty(tDone) == 1
        tDone = tf;
    end
else
    options = odeset('RelTol', 1e-11, 'AbsTol', 1e-11, 'Stats', 'off');
    [~, z] = ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1), 1), [0 tf], z0, options);
    tDone = tf;
end
[xf, ~, ~] = StatesInECI(theta_f, at, et, it, Omega_t, omega_t, muVal);
[F1, F2] = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, muVal);

% cFinal is the transversality condition contribution
cFinal = z(end, 1:3) * F1 + z(end, 4:6) * F2;

x_tf = z(end, length(z0) - length(xf)  : end).'; % extract state portion at final time

% hamiltonian for minT
L = z(1,1:7);              % costates (columns correspond to [L...])
x = z(1,8:end);            % states (columns correspond to [x...])
nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));

ax = L(4)/(nLv^0.5);
ay = L(5)/(nLv^0.5);
az = L(6)/(nLv^0.5);
mC = x(end);
H = L(1)*x(4) + L(2)*x(5) + L(3)*x(6) - L(4)*((muEarth*x(1))/nPos^(3/2) - (Tmax*ax)/mC) - L(5)*((muEarth*x(2))/nPos^(3/2) ...
    - (Tmax*ay)/mC) - L(6)*((muEarth*x(3))/nPos^(3/2) - (Tmax*az)/mC) - (L(7)*Tmax*g0)/Isp + L0(1);
end