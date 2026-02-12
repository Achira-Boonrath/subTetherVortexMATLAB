function [xf, rf, vf] = StatesInECI(theta_f, at, et, it, Omega_t, omega_t, mu)
%TARGETTERMINALSTATE Terminal position and velocity in ECI
%
% Inputs:
%   theta_f  - true anomaly [rad]
%   at       - semi-major axis
%   et       - eccentricity
%   it       - inclination [rad]
%   Omega_t  - RAAN [rad]
%   omega_t  - argument of periapsis [rad]
%   mu       - gravitational parameter
%
% Outputs:
%   rf       - position in ECI (3x1)
%   vf       - velocity in ECI (3x1)
%   xf       - stacked state [rf; vf] (6x1)

% Rotation matrix (perifocal -> ECI)
R_PI = perifocalToECI(Omega_t, it, omega_t);

% Radius magnitude
p = at * (1 - et^2);
r_mag = p / (1 + et*cos(theta_f));

% Position in perifocal frame
r_pf = r_mag * ...
    [ cos(theta_f);
      sin(theta_f);
      0 ];

% Velocity scaling
v_scale = sqrt(mu / p);

% Velocity in perifocal frame
v_pf = v_scale * ...
    [ -sin(theta_f);
      et + cos(theta_f);
      0 ];

% Transform to ECI
rf = R_PI * r_pf;
vf = R_PI * v_pf;

% Terminal state
xf = [rf; vf];

end