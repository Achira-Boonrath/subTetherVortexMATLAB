function f = objfunsm_rbf(x,x_in,rbf_coeff,rho)

f = rbf_approx(x_in,x,rbf_coeff,rho,'Multiquadric');
end