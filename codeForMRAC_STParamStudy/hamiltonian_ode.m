function dzdt = hamiltonian_ode(t, z, funcSubs)

    % dzdt = ( funcSubs(z) );
    dzdt = double( funcSubs(z) );
end