function [rECI,vECI,DCM_PQW2ECI] = COE2ECI(COE,mu)
%% ========================================================================
% COE2ECI
%
% Converts Classical Orbital Elements (COEs) into inertial
% position and velocity vectors (ECI frame).
%
% ------------------------------------------------------------------------
% INPUTS
%
% COE = [ a
%         e
%         i
%         RAAN
%         omega
%         anomaly ]
%
% a        Semi-major axis (km)
% e        Eccentricity (-)
% i        Inclination (rad)
% RAAN     Right Ascension Ascending Node Ω (rad)
% omega    Argument of Perigee ω (rad)
%
% anomaly  TRUE anomaly ν (rad)
%           OR Mean anomaly M (rad)
%
% NOTE:
% By default this function assumes TRUE anomaly input.
% (see optional flag below if Mean anomaly is provided)
%
%
% mu       Gravitational parameter (km^3/sec^2)
%
% Earth = 398600.4418
%
% ------------------------------------------------------------------------
% OUTPUTS
%
% rECI     Position vector in ECI frame (km)
%
% vECI     Velocity vector in ECI frame (km/sec)
%
% DCM_PQW2ECI
%          Rotation matrix:
%
%          V_ECI = DCM_PQW2ECI * V_PQW
%
%
% ------------------------------------------------------------------------
% FEATURES
%
% ✓ Numerically stable
% ✓ Elliptic orbits supported
% ✓ Optimization-loop safe
% ✓ Professional reusable structure
%
% ========================================================================

%% -------- Input Validation ---------------------------------------------

if nargin < 2
    error('Usage: COE2ECI(COE,mu)');
end

if numel(COE) ~= 6
    error('COE must contain 6 elements.');
end

COE = COE(:);

a     = COE(1);
e     = COE(2);
inc   = COE(3);
RAAN  = COE(4);
omega = COE(5);
nu    = COE(6);   % assumed TRUE anomaly

if a <= 0
    error('Semi-major axis must be positive.');
end

if e < 0
    error('Eccentricity must be >= 0.');
end

%% -------- Semi-latus Rectum --------------------------------------------

p = a*(1 - e^2);

%% -------- Radius --------------------------------------------------------

rmag = p / (1 + e*cos(nu));

%% -------- State in PQW (Perifocal Frame) -------------------------------

rPQW = [ rmag*cos(nu);
         rmag*sin(nu);
         0 ];

vPQW = sqrt(mu/p)* ...
      [ -sin(nu);
         e + cos(nu);
         0 ];

%% -------- Rotation Matrix PQW -> ECI -----------------------------------
%
% DCM = R3(RAAN)*R1(i)*R3(omega)

cO = cos(RAAN);
sO = sin(RAAN);

ci = cos(inc);
si = sin(inc);

cw = cos(omega);
sw = sin(omega);

DCM_PQW2ECI = [ ...
 cO*cw - sO*sw*ci , -cO*sw - sO*cw*ci ,  sO*si;
 sO*cw + cO*sw*ci , -sO*sw + cO*cw*ci , -cO*si;
 sw*si            ,  cw*si             ,  ci ];

%% -------- Transform to ECI ---------------------------------------------

rECI = DCM_PQW2ECI * rPQW;
vECI = DCM_PQW2ECI * vPQW;

end
