function F = shootingConstT(lambda0,x0,xf,tf, muEarth, Tmax, Isp, g0, epsilon)

    z0 = [lambda0; x0];
    % S = warning('MATLAB:ode45:IntegrationTolNotMet', 'off');
    [~,z] = ode45(@(t,z) hamiltonian_odeConstT(t,z, muEarth, Tmax, Isp, g0, epsilon),[0 tf],z0);
    % warning(S);

    x_tf = z(end,end-length(x0)+1:end).';
    F = [x_tf(1:end-1) - xf(1:end-1); z(end,length(x0))];  % terminal error

end