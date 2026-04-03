function F = shootingConstT(lambda0,x0,xf,tf, muEarth, Tmax, Isp, g0, epsilon)

% Additional parameters for electric prop
at = 42164.0;             % semi-major axis [km]
rf = at;
et = 0.001;                % eccentricity
it = deg2rad(2.0);        % inclination
Omega_t = deg2rad(0);   % RAAN
omega_t = deg2rad(0);   % argument of periapsis
% Symbolic substitution based on propulsion type
muVal = 398600.4418;     % Earth

L0= costate_from_D( lambda0(1:end) );

z0 = [L0(2:end); x0];
theta_f = 0;

% Call helper to integrate and compute terminal constraints
[z, ~, x_tf, cFinal]= integrateAndComputeTerminal(z0, tf, muEarth, Tmax, Isp, g0, epsilon, L0, theta_f, ...
    at, et, it, Omega_t, omega_t, muVal);

%compute hamiltonian

F = [x_tf(1:3) - xf(1:3); (x_tf(4:end-1) - xf(4:end-1))*1000; z(end,length(x0))];  % terminal error

if  abs( z(end,length(x0)) ) > 1e+1
    F(end) = F(end) + 1e+5;
end

end