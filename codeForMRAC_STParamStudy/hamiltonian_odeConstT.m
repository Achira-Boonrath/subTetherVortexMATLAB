function ds = hamiltonian_odeConstT(t, s, muEarth, Tmax, Isp, g0, epsilon, L0, uFixed)

L = s(1:7);               % states (columns correspond to [x y vx vy])
x = s(8:end);            % costates (columns correspond to [L(1) L(2) L(3) L(4)])
nLv = (L(4)^(2) + L(5)^(2) + L(6)^(2));
nPos = (x(1)^(2) + x(2)^(2) + x(3)^(2));

if nargin < 8
    L0 = 1;
elseif nargin > 8
    uSwitch = uFixed;
end

% rho = 1 - (Isp * g0 * nLv^(1/2))/(x(7)*L0) - L(7)/L0;
% 
% uSwitch = 0.5 - rho/(2*epsilon);
% if rho > epsilon
%     uSwitch = 0;
% elseif rho < - epsilon
%     uSwitch = 1;
% end
% 
% Tmag = Tmax*uSwitch;

[Tmag,rho,uSwitch] = ThrottleSwitchingFunc(Isp , g0, nLv^(1/2), x, L, L0, epsilon, Tmax);

ds = zeros(length(s),1);
ds(1) = -(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2);
ds(2) = -(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2);
ds(3) = -(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2);
ds(4) = -L(1);
ds(5) = -L(2);
ds(6) = -L(3);
ds(7) = -(Tmag*nLv^(1/2))/x(7)^(2);
ds(8) = x(4);
ds(9) = x(5);
ds(10) =x(6);
% ds(11) =-(L(4)^3*Tmag*nPos^(3/2) + muEarth*x(1)*x(7)*nLv^(3/2) + L(4)*L(5)^2*Tmag*nPos^(3/2)...
%     + L(4)*L(6)^2*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
% ds(12) =-(L(5)^3*Tmag*nPos^(3/2) + muEarth*x(2)*x(7)*nLv^(3/2) + L(4)^2*L(5)*Tmag*nPos^(3/2)...
%     + L(5)*L(6)^2*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
% ds(13) =-(L(6)^3*Tmag*nPos^(3/2) + muEarth*x(3)*x(7)*nLv^(3/2) + L(4)^2*L(6)*Tmag*nPos^(3/2)...
%     + L(5)^2*L(6)*Tmag*nPos^(3/2))/(x(7)*nLv^(3/2)*nPos^(3/2));
ds(11:13) = - (muEarth/nPos^(3/2) )*x(1:3) - ( L(4:6)/nLv^(1/2) )*Tmag/x(7);
ds(14) =-(Tmag*g0)/Isp;
% abs(ds(11:13))  - abs(-(muEarth/nPos^(3/2) )*x(1:3) - ( L(4:6)/nLv^(1/2) )*Tmag/x(7))

end