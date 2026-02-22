function [nu,E] = Mean2TrueAnomaly(M,e,tol,maxIter)
% ========================================================================
% Mean2TrueAnomaly
%
% Computes TRUE ANOMALY from MEAN ANOMALY by solving Kepler’s Equation.
%
% Supports:
%   • Elliptical orbits   (0 ≤ e < 1)
%   • Hyperbolic orbits   (e > 1)
%
% ------------------------------------------------------------------------
% INPUTS
%
% M        Mean anomaly (rad)
% e        Eccentricity (-)
%
% OPTIONAL
%
% tol      Newton solver tolerance       (default = 1e-13)
% maxIter  Maximum iterations            (default = 100)
%
% ------------------------------------------------------------------------
% OUTPUTS
%
% nu       True anomaly (rad)
% E        Eccentric anomaly (ellipse)
%          OR Hyperbolic anomaly (hyperbola)
%
% ------------------------------------------------------------------------
% NOTES
%
% ✓ Robust Newton-Raphson solver
% ✓ Optimization-loop safe
% ✓ Handles scalar or vector inputs
%
% ========================================================================

%% ---------- Defaults ----------------------------------------------------

if nargin < 3 || isempty(tol)
    tol = 1e-9;
end

if nargin < 4 || isempty(maxIter)
    maxIter = 100;
end

M = M(:).';     % allow vector input

if any(e < 0)
    error('Eccentricity must be >= 0.');
end

% ========================================================================
%%                ELLIPTIC ORBIT ( e < 1 )
% ========================================================================

if e < 1

    % wrap mean anomaly
    M = mod(M,2*pi);

    % good initial guess
    E = M;

    for k = 1:maxIter

        f  = E - e*sin(E) - M;
        fp = 1 - e*cos(E);

        dE = -f./fp;

        E = E + dE;

        if max(abs(dE)) < tol
            break
        end
    end

    % True anomaly conversion
    %
    % tan(nu/2) =
    % sqrt((1+e)/(1-e)) tan(E/2)

    nu = 2*atan2( ...
          sqrt(1+e).*sin(E/2), ...
          sqrt(1-e).*cos(E/2) );

    nu = mod(nu,2*pi);

% ========================================================================
%%                HYPERBOLIC ORBIT ( e > 1 )
% ========================================================================

elseif e > 1

    % Initial guess (standard)
    H = asinh(M/e);

    for k = 1:maxIter

        f  = e*sinh(H) - H - M;
        fp = e*cosh(H) - 1;

        dH = -f./fp;

        H = H + dH;

        if max(abs(dH)) < tol
            break
        end
    end

    E = H;   % return hyperbolic anomaly

    % True anomaly
    nu = 2*atan2( ...
          sqrt(e+1).*sinh(H/2), ...
          sqrt(e-1).*cosh(H/2) );

else

    error('Parabolic case (e = 1) not supported.');

end

end
