function dzdt = hamiltonian_ode(t, z, funcSubs)

    dzdt = double( funcSubs(z) );

end