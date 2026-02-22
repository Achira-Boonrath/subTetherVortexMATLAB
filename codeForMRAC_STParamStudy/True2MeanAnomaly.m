function [M,E] = True2MeanAnomaly(nu,e)
% ========================================================================
% True2MeanAnomaly
%
% Computes MEAN ANOMALY from TRUE ANOMALY.
%
% Supports:
%   ✓ Elliptical orbits   (0 ≤ e < 1)
%   ✓ Hyperbolic orbits   (e > 1)
%
% ------------------------------------------------------------------------
% INPUTS
%
% nu     True anomaly (rad)
% e      Eccentricity (-)
%
% ------------------------------------------------------------------------
% OUTPUTS
%
% M      Mean anomaly (rad)
%
% E      Eccentric anomaly (ellipse)
%        OR hyperbolic anomaly (hyperbola)
%
% ------------------------------------------------------------------------
% NOTES
%
% ✓ Numerically stable atan2 formulation.
% ✓ Vector input compatible.
% ✓ Optimization / propagation safe.
%
% ========================================================================

%% ---------- Input Handling ---------------------------------------------

nu = nu(:).';   % allow vector input

if any(e < 0)
    error('Eccentricity must be >= 0.');
end

% ========================================================================
%%                     ELLIPTIC ORBIT
% ========================================================================

if e < 1

    % wrap angle
    nu = mod(nu,2*pi);

    %% ----- True -> Eccentric anomaly -----
    %
    % tan(E/2) =
    % sqrt((1-e)/(1+e)) tan(nu/2)

    E = 2*atan2( ...
        sqrt(1-e).*sin(nu/2), ...
        sqrt(1+e).*cos(nu/2));

    E = mod(E,2*pi);

    %% ----- Mean anomaly -----

    M = E - e*sin(E);

    M = mod(M,2*pi);

% ========================================================================
%%                     HYPERBOLIC ORBIT
% ========================================================================

elseif e > 1

    %% ----- True -> Hyperbolic anomaly -----
    %
    % tanh(H/2) =
    % sqrt((e-1)/(e+1)) tan(nu/2)

    H = 2*atanh( ...
        sqrt((e-1)/(e+1)) .* tan(nu/2) );

    E = H;

    %% ----- Mean anomaly -----

    M = e*sinh(H) - H;

else

    error('Parabolic case (e = 1) not supported.');

end

end
