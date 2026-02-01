function F = shooting(lambda0,x0,xf,tf,funcSubs)
    % fixed BC for all states
    z0 = [lambda0; x0];
    
    [~,z] = ode23(@(t,z) hamiltonian_ode(t,z,funcSubs),[0 tf],z0);
    
    x_tf = z(end,end-length(x0)+1:end).';
    F = x_tf - xf;   % terminal error

end