function [ds] = costate_Dyn(x, L, argsCons)

a = argsCons.a;
b = argsCons.b;
p_min = argsCons.p_min;
rho_s = argsCons.rho_s;
muEarth = argsCons.muEarth;
% nPos =argsCons.nPos;
% nLv = argsCons.nLv;

rx =x(1);ry =x(2);rz =x(3);vx =x(4);vy =x(5);vz =x(6);
Lrx =L(1);Lry =L(2);Lrz =L(3);Lvx =L(4);Lvy =L(5);Lvz =L(6);

p = ( (rx^(2))/(a^(2))+(ry^(2))/(b^(2)) );
%%
% sec( clip( (pi/2)* (p_min/p),...
%     -pi/2+1e-9, pi/2-1e-9 ) )

% consX = Lvx*(1/(nPos)^(3/2) - (3*rx*x(1))/(nPos)^(5/2)) - (3*Lvy*ry*x(1))/(nPos)^(5/2) - (3*Lvz*rz*x(1))/(nPos)^(5/2) + (p_min*rho_s*rx*pi*sin((p_min*pi)/(2*(rx^2/a^2 + ry^2/b^2))))/(a^2*cos((p_min*pi)/((2*rx^2)/a^2 + (2*ry^2)/b^2))^2*(rx^2/a^2 + ry^2/b^2)^2);
% consY = Lvy*(1/(nPos)^(3/2) - (3*ry*x(2))/(nPos)^(5/2)) - (3*Lvx*rx*x(2))/(nPos)^(5/2) - (3*Lvz*rz*x(2))/(nPos)^(5/2) + (p_min*rho_s*ry*pi*sin((p_min*pi)/(2*(rx^2/a^2 + ry^2/b^2))))/(b^2*cos((p_min*pi)/((2*rx^2)/a^2 + (2*ry^2)/b^2))^2*(rx^2/a^2 + ry^2/b^2)^2);
% consZ =                                                                                                                                          Lvz*(1/(nPos)^(3/2) - (3*rz*x(3))/(nPos)^(5/2)) - (3*Lvx*rx*x(3))/(nPos)^(5/2) - (3*Lvy*ry*x(3))/(nPos)^(5/2);

% rho_s = 1e-13;
% lc = 0;

%%
ds= zeros(1,6);
consX =Lvx*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*rx*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) + (2*rho_s*rx*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/a^2 - (4*rho_s*rx*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/a^2 - (3*Lvy*muEarth*ry*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvz*muEarth*rz*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);
consY =Lvy*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*ry*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) + (2*rho_s*ry*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/b^2 - (4*rho_s*ry*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/b^2 - (3*Lvx*muEarth*rx*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvz*muEarth*rz*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);
consZ =                                                                                                                                                                                                                           Lvz*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*rz*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) - (3*Lvx*muEarth*rx*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvy*muEarth*ry*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);

% disp(consX - ds(1))
% disp(consX )
% disp(ds(1))
ds(1:3) = [consX, consY, consZ];

% ds(1) = (-(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2) )/(1) ...
%     + (2*rho_s*rx*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/a^2 - (4*rho_s*rx*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/a^2;
% ds(2) = (-(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2) )/(1)...
%     + (2*rho_s*ry*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/b^2 - (4*rho_s*ry*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/b^2;
% ds(3) = (-(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2) )/(1);

ds(4) = -L(1);
ds(5) = -L(2);
ds(6) = -L(3);

end