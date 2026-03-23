
function [z, xf, x_tf, cFinal] = integrateAndComputeTerminal(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, at, et, it, Omega_t, omega_t, muVal)
% Helper that integrates the Hamiltonian ODE and computes terminal constraints.
    options = odeset('RelTol', 1e-11, 'AbsTol', 1e-11, 'Stats', 'off');

    [~, z] = ode45(@(t,z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon, L0(1)), [0 tf], z0, options);

    [xf, ~, ~] = StatesInECI(theta_f, at, et, it, Omega_t, omega_t, muVal);
    [F1, F2] = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, muVal);

    % cFinal is the transversality condition contribution
    cFinal = z(end, 1:3) * F1 + z(end, 4:6) * F2;

    x_tf = z(end, length(z0) - length(xf)  : end).'; % extract state portion at final time
end