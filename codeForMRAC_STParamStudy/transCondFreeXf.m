clear; clc;

% Symbolic variables
syms theta_f at et it Omega_t omega_t mu real
assume(et > 0 & et < 1)

cO = cos(Omega_t);  sO = sin(Omega_t);
ci = cos(it);       si = sin(it);
co = cos(omega_t);  so = sin(omega_t);

R_PI = [ ...
    cO*co - sO*ci*so,   -cO*so - sO*ci*co,   sO*si;
    sO*co + cO*ci*so,   -sO*so + cO*ci*co,  -cO*si;
    si*so,               si*co,              ci
];

% Semi-latus rectum
p = at*(1 - et^2);

% Radius magnitude
r_mag = p / (1 + et*cos(theta_f));

% Perifocal position
r_pf = r_mag * ...
    [ cos(theta_f);
      sin(theta_f);
      0 ];

% ECI position
rf = R_PI * r_pf;

v_scale = sqrt(mu / p);

v_pf = v_scale * ...
    [ -sin(theta_f);
       et + cos(theta_f);
       0 ];

vf = R_PI * v_pf;

% Terminal state
xf = [rf; vf];

% Symbolic derivative w.r.t. theta_f
dxf_dtheta = simplify( jacobian(xf, theta_f) );

dxf_dtheta = simplify(dxf_dtheta, 'Steps', 20)
% pretty(dxf_dtheta)