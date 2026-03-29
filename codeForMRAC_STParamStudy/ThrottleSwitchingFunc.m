function [Tmag,rho,uSwitch] = ThrottleSwitchingFunc(Isp , g0, normLV,x, L, L0, epsilon, Tmax)
rho = 1 - (Isp * g0 * normLV)/(x(7)*L0) - L(7)/L0;

uSwitch = 0.5 - rho/(2*epsilon);
if rho > epsilon
    uSwitch = 0;
elseif rho < - epsilon
    uSwitch = 1;
end

Tmag = Tmax*uSwitch;
end