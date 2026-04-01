function [c,ceq] = nonlcon(x,x_in,rbf_coeff,rho)
% c = 0;
% ceq = 0;
% (x(1)-3)^3 +1 - x(2)
%  2 - x(1) - x(2)
c = [(x(1)-3)^3 +1 - x(2),  2 - x(1) - x(2)];
ceq = [0, 0];

end

