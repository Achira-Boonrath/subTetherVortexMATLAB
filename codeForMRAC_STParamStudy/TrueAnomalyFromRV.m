function nu = TrueAnomalyFromRV(rECI,vECI,mu)
%% ========================================================================
% TrueAnomalyFromRV
%
% Computes the TRUE ANOMALY from inertial position and velocity vectors.
%
% ------------------------------------------------------------------------
% INPUTS
%
% rECI   [3x1] Position vector (km)
% vECI   [3x1] Velocity vector (km/sec)
%
% mu     Gravitational parameter (km^3/sec^2)
%
% ------------------------------------------------------------------------
% OUTPUT
%
% nu     True anomaly (rad)   ∈ [0 , 2π)
%
% ------------------------------------------------------------------------
% METHOD
%
% 1) Compute angular momentum vector
% 2) Compute eccentricity vector
% 3) Compute angle between eccentricity vector and position vector
%
% Fully valid for elliptic or hyperbolic orbits (e ≠ 0).
%
% ========================================================================

%% ---------- Input Handling ---------------------------------------------

rECI = rECI(:);
vECI = vECI(:);

if numel(rECI) ~= 3 || numel(vECI) ~= 3
    error('rECI and vECI must be 3x1 vectors.');
end

%% ---------- Magnitudes --------------------------------------------------

r = norm(rECI);
v = norm(vECI);

%% ---------- Angular Momentum -------------------------------------------

hVec = cross(rECI,vECI);

%% ---------- Eccentricity Vector ----------------------------------------

eVec = ( (v^2 - mu/r)*rECI ...
        - dot(rECI,vECI)*vECI ) / mu;

e = norm(eVec);

if e < 1e-12
    error('Orbit nearly circular: true anomaly undefined from e-vector.');
end

%% ---------- True Anomaly (Angle Between e and r) ------------------------

cosNu = dot(eVec,rECI)/(e*r);

% numerical protection
cosNu = max(-1,min(1,cosNu));

nu = acos(cosNu);

%% ---------- Quadrant Check ---------------------------------------------
%
% If radial velocity < 0 → descending portion of orbit

if dot(rECI,vECI) < 0
    nu = 2*pi - nu;
end

end