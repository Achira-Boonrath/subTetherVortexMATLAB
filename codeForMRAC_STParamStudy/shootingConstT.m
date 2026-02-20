function F = shootingConstT(lambda0,x0,xf,tf, muEarth, Tmax, Isp, g0, epsilon)

    z0 = [lambda0; x0];
    % S = warning('MATLAB:ode45:IntegrationTolNotMet', 'off');
    options = odeset('RelTol', 1e-8,'AbsTol', 1e-8,'Stats','off');
    [~,z] = ode45(@(t,z) hamiltonian_odeConstT(t,z, muEarth, Tmax, Isp, g0, epsilon),[0 tf],z0,options);
    % warning(S);

    x_tf = z(end,end-length(x0)+1:end).';
    % F = [x_tf(1:end-1) - xf(1:end-1); z(end,length(x0))/(1.0e+4)];  % terminal error
    F = [x_tf(1:3) - xf(1:3); (x_tf(4:end-1) - xf(4:end-1))*(1e+2); z(end,length(x0))/(1.0e+4)]/(1.0e+3);  % terminal error

end