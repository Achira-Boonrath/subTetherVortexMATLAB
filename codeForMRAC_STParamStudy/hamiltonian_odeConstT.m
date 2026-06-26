function ds = hamiltonian_odeConstT(t, s, muEarth, Tmax, Isp, g0, epsilon, L0, uFixed)

L = s(1:7);              % costates (columns correspond to [L...]) 
x = s(8:end);            % states (columns correspond to [x...])
nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));
if nargin < 8
    L0 = 1;
elseif nargin > 8
    uSwitch = uFixed;
end

[Tmag,rho,uSwitch] = ThrottleSwitchingFunc(Isp , g0, (nLv^(1/2)), x, L, L0, epsilon, Tmax);

ds = zeros(length(s),1);

ds(7) = -(1)*(Tmag*nLv^(1/2))/x(7)^(2);
ds(8) = x(4);
ds(9) = x(5);
ds(10) =x(6);
% ds(11) =-(L(4)^3*Tmag*nPos^(3/2) + muEarth*x(1)*x(7)*nLv^(3/2) + L(4)*L(5)^2*Tmag*nPos^(3/2)...
%     + L(4)*L(6)^2*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
% ds(12) =-(L(5)^3*Tmag*nPos^(3/2) + muEarth*x(2)*x(7)*nLv^(3/2) + L(4)^2*L(5)*Tmag*nPos^(3/2)...
%     + L(5)*L(6)^2*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
% ds(13) =-(L(6)^3*Tmag*nPos^(3/2) + muEarth*x(3)*x(7)*nLv^(3/2) + L(4)^2*L(6)*Tmag*nPos^(3/2)...
%     + L(5)^2*L(6)*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
ds(11:13) = - (muEarth/nPos^(3/2) )*x(1:3) - ( L(4:6)/(nLv)^(1/2) )*Tmag/x(7);
ds(14) = (-(Tmag*g0)/Isp);

%% unconstrained state

ds(1) = (-(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2) )/(1);
ds(2) = (-(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2) )/(1);
ds(3) = (-(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2) )/(1);
% ds(4) = -L(1);
% ds(5) = -L(2);
% ds(6) = -L(3);

%% constrained state
r0 = 1+6378;
rf = 1300+6378;
a = r0;
b = rf;
p_min = 1;
rho_s = 00;

rx =x(1);ry =x(2);rz =x(3);vx =x(4);vy =x(5);vz =x(6);
Lrx =L(1);Lry =L(2);Lrz =L(3);Lvx =L(4);Lvy =L(5);Lvz =L(6);

p = ( (rx^(2))/(a^(2))+(ry^(2))/(b^(2)) );
% sec( clip( (pi/2)* (p_min/p),...
%     -pi/2+1e-9, pi/2-1e-9 ) )

% consX = Lvx*(1/(nPos)^(3/2) - (3*rx*x(1))/(nPos)^(5/2)) - (3*Lvy*ry*x(1))/(nPos)^(5/2) - (3*Lvz*rz*x(1))/(nPos)^(5/2) + (p_min*rho_s*rx*pi*sin((p_min*pi)/(2*(rx^2/a^2 + ry^2/b^2))))/(a^2*cos((p_min*pi)/((2*rx^2)/a^2 + (2*ry^2)/b^2))^2*(rx^2/a^2 + ry^2/b^2)^2);
% consY = Lvy*(1/(nPos)^(3/2) - (3*ry*x(2))/(nPos)^(5/2)) - (3*Lvx*rx*x(2))/(nPos)^(5/2) - (3*Lvz*rz*x(2))/(nPos)^(5/2) + (p_min*rho_s*ry*pi*sin((p_min*pi)/(2*(rx^2/a^2 + ry^2/b^2))))/(b^2*cos((p_min*pi)/((2*rx^2)/a^2 + (2*ry^2)/b^2))^2*(rx^2/a^2 + ry^2/b^2)^2);
% consZ =                                                                                                                                          Lvz*(1/(nPos)^(3/2) - (3*rz*x(3))/(nPos)^(5/2)) - (3*Lvx*rx*x(3))/(nPos)^(5/2) - (3*Lvy*ry*x(3))/(nPos)^(5/2);

% rho_s = 1e-13;
% lc = 0;
consX =Lvx*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*rx*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) + (2*rho_s*rx*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/a^2 - (4*rho_s*rx*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/a^2 - (3*Lvy*muEarth*ry*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvz*muEarth*rz*abs(rx)*sign(rx))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);
consY =Lvy*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*ry*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) + (2*rho_s*ry*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/b^2 - (4*rho_s*ry*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/b^2 - (3*Lvx*muEarth*rx*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvz*muEarth*rz*abs(ry)*sign(ry))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);
consZ =                                                                                                                                                                                                                           Lvz*(muEarth/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(3/2) - (3*muEarth*rz*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2)) - (3*Lvx*muEarth*rx*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2) - (3*Lvy*muEarth*ry*abs(rz)*sign(rz))/(abs(rx)^2 + abs(ry)^2 + abs(rz)^2)^(5/2);

% disp(consX - ds(1))
% disp(consX )
% disp(ds(1))
% ds(1:3) = [consX, consY, consZ];

% ds(1) = (-(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2) )/(1) ...
%     + (2*rho_s*rx*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/a^2 - (4*rho_s*rx*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/a^2;
% ds(2) = (-(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2) )/(1)...
%     + (2*rho_s*ry*dirac((- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2)^2)/b^2 - (4*rho_s*ry*heaviside(-(- p_min*a^2*b^2 + a^2*ry^2 + b^2*rx^2)/(a^2*b^2))*(rx^2/a^2 - p_min + ry^2/b^2))/b^2;
% ds(3) = (-(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2) )/(1);

ds(4) = -L(1);
ds(5) = -L(2);
ds(6) = -L(3);

end