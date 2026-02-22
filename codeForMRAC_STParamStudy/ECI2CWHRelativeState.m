function [rHill,vHill,DCM_ECI2Hill] = ECI2CWHRelativeState( ...
                                    rChief,vChief,...
                                    rDeputy,vDeputy,mu)
%% ========================================================================
% ECI2CWHRelativeState
%
% Computes Deputy relative position and velocity expressed in the
% Chief-centered Hill (LVLH / CWH / RTN) frame.
%
% INPUTS
%
% rChief   [3x1] Chief inertial position     (km)
% vChief   [3x1] Chief inertial velocity     (km/s)
%
% rDeputy  [3x1] Deputy inertial position    (km)
% vDeputy  [3x1] Deputy inertial velocity    (km/s)
%
% mu       gravitational parameter (km^3/s^2)
%          Earth = 398600.4418
%
% ------------------------------------------------------------------------
% OUTPUTS
%
% rHill    Relative position in Hill Frame (km)
% vHill    Relative velocity in Hill Frame (km/sec)
%
% DCM_ECI2Hill
%           Direction Cosine Matrix transforming vectors:
%
%           V_Hill = DCM_ECI2Hill * V_ECI
%
%
% Hill Frame Definition (RTN):
%
%   x-axis = Radial      (Chief -> Earth center outward)
%   y-axis = Along-track (velocity direction)
%   z-axis = Cross-track (orbit angular momentum)
%
% ------------------------------------------------------------------------
% NOTES
%
% • Exact nonlinear geometry (NOT linearized HCW).
% • Works for elliptical and inclined orbits.
% • Correctly includes rotating-frame velocity correction.
%
% ========================================================================

%% -------- Validation ----------------------------------------------------

rChief  = rChief(:);
vChief  = vChief(:);
rDeputy = rDeputy(:);
vDeputy = vDeputy(:);

if any([numel(rChief),numel(vChief), ...
        numel(rDeputy),numel(vDeputy)] ~= 3)

    error('All position/velocity vectors must be 3x1.');
end

%% -------- Relative State in ECI -----------------------------------------

drECI = rDeputy - rChief;
dvECI = vDeputy - vChief;

%% -------- Chief Hill Frame Axes -----------------------------------------
%
% R-axis (Radial)

rhat = rChief / norm(rChief);

%
% Orbit angular momentum

hVec = cross(rChief,vChief);
hhat = hVec / norm(hVec);

%
% T-axis (Along-track)

that = cross(hhat,rhat);

%% -------- DCM (ECI -> Hill) ---------------------------------------------

DCM_ECI2Hill = [ rhat.';
                 that.';
                 hhat.' ];

%% -------- Relative Position ---------------------------------------------

rHill = DCM_ECI2Hill * drECI;

%% -------- Angular Velocity of Hill Frame --------------------------------
%
% omega = h / r^2

rNorm = norm(rChief);

omegaHill = hVec / rNorm^2;     % rad/sec

%% -------- Relative Velocity (Rotating Frame Correction) -----------------
%
% v_rel_hill =
%   C*(dvECI) - omega x rHill
%
% NOTE:
% omega must be expressed in Hill frame.

omegaHill_Hill = DCM_ECI2Hill * omegaHill;

vHill = DCM_ECI2Hill * dvECI ...
        - cross(omegaHill_Hill , rHill);

end
