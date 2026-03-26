function [eqns, Lvec, Xnum] = buildHamiltonianEquations(x0, muVal, propulsionType, uTest, minT)
% buildHamiltonianEquations Construct symbolic Hamiltonian ODE expressions
%   eqns = buildHamiltonianEquations(x0, muVal, propulsionType)
%   Returns a symbolic row vector of equations [Ldot'; xdot'] with numeric
%   substitutions for mu, Tmax, Isp, g0 and a nominal throttle u=1.
%
%   Inputs:
%     x0            - numeric initial state vector (used to size symbols)
%     muVal         - numeric gravitational parameter
%     propulsionType- 'chemical' or 'electric'
%
%   Output:
%     eqns - symbolic row vector containing the costate ODEs followed by
%            state ODEs, ready for numeric substitution.

if nargin < 5
    minT = 0;
end

% State symbols (position and velocity)
syms x y z vx vy vz muEarth ...
    ax ay az u mC Tmax Isp g0 real

% Build state and control symbolic vectors for clarity
X = [x; y; z; vx; vy; vz; mC];            % symbolic state vector
Xnum = sym('x', [length(x0) 1],'real');
if minT == 0
    U = [ax; ay; az; u];
else
    u = 1;
    U = [ax; ay; az];
end
% Define the dynamics f = dX/dt (Two-Body Keplerian dynamics with variable mass)
f = [vx; ...
    vy; ...
    vz; ...
    -x*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + ax*(Tmax/mC)*u; ...
    -y*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + ay*(Tmax/mC)*u; ...
    -z*muEarth/((x^(2) + y^(2) + z^(2))^(3/2)) + az*(Tmax/mC)*u; ...
    -(Tmax/Isp*g0)*u; ...
    ];

% Symbolic costate vector sized to state length
Lvec = sym('L', [length(x0) 1],'real');

% Optimal control direction (unit vector opposite costates of velocity)
if minT == 0
    optUSet =[ - Lvec(4)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
        - Lvec(5)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
        - Lvec(6)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
        U(end)];
    % Instantaneous control cost (here a simple throttle penalization)
    V = 0.5*U(end)'*U(end);
else
    optUSet =[ - Lvec(4)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
        - Lvec(5)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2);...
        - Lvec(6)/sqrt(Lvec(5)^2 + Lvec(4)^2 + Lvec(6)^2)];
    % Instantaneous control cost (here a simple throttle penalization)
    V = 1;
end

% Delegate symbolic derivation of xdot and Ldot to helper (expected on path)
[star_xdot, star_Ldot] = odeDynAndLag_constT(Lvec, X, Xnum, U, f, V, optUSet);

% Compose the equations array: [Ldot'; xdot']
eqns = [
    (star_Ldot).',         % costate ODEs (row)
    (star_xdot).'          % state ODEs (row)
    ];
eqns = simplify(eqns);

% Substitute numeric constants
eqns = subs(eqns, muEarth, muVal);

switch propulsionType
    case 'chemical'
        eqns = subs(eqns, Tmax, 425*4);
        eqns = subs(eqns, Isp, 230);
    case 'electric'
        eqns = subs(eqns, Tmax, 0.1*1e-3);
        eqns = subs(eqns, Isp, 3000);
end

eqns = subs(eqns, g0, 9.81*1e-3);
eqns = subs(eqns, U(end), uTest);

eqns = simplify(eqns);
end