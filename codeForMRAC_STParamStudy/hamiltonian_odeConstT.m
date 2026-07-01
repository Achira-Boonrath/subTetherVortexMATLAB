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
ds(11:13) = - (muEarth/nPos^(3/2) )*x(1:3) - ( L(4:6)/(nLv)^(1/2) )*Tmag/x(7);
ds(14) = (-(Tmag)/(Isp*g0));

%% constrained state
r0 = 1+6378;
rf = 780+6378;
a = r0;
b = rf;
p_min = 1;
rho_s = 50;
consOn = 1;

argsCons = struct('a', a, 'b', b, 'p_min', p_min, 'rho_s', rho_s, ...
    'muEarth', muEarth,'nPos', nPos, 'nLv',nLv );

if consOn == 0
    % unconstrained state
    
    ds(1) = (-(muEarth*(2*L(4)*x(1)^(2) + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^(2) - L(4)*x(3)^(2)))/nPos^(5/2) )/(1);
    ds(2) = (-(muEarth*(- L(5)*x(1)^(2) + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^(2) + 3*L(6)*x(2)*x(3) - L(5)*x(3)^(2)))/nPos^(5/2) )/(1);
    ds(3) = (-(muEarth*(- L(6)*x(1)^(2) + 3*L(4)*x(1)*x(3) - L(6)*x(2)^(2) + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^(2)))/nPos^(5/2) )/(1);
    ds(4) = -L(1);
    ds(5) = -L(2);
    ds(6) = -L(3);
else
    ds(1:6) = costate_Dyn(x, L, argsCons);

end

end