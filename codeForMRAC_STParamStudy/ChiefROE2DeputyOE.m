function DeputyOE = ChiefROE2DeputyOE(ChiefOE,ROE)
%% ========================================================================
%  ChiefROE2DeputyOE
%
%  Converts Chief Classical Orbital Elements (COEs) and Relative Orbital
%  Elements (ROEs) into Deputy Classical Orbital Elements.
%
%  IMPLEMENTATION:
%  Schaub–Alfriend Relative Orbital Elements formulation.
%
% ------------------------------------------------------------------------
% INPUTS
%
% ChiefOE  : [6x1] or [1x6]
%
%   ChiefOE =
%   [ aC      semi-major axis        (km)
%     eC      eccentricity           (-)
%     iC      inclination            (rad)
%     RAANC   RAAN                   (rad)
%     wC      argument of perigee    (rad)
%     MC      mean anomaly           (rad) ]
%
%
% ROE : [6x1] or [1x6]
%
%   ROE =
%   [ da          = (aD-aC)/aC
%     dLambda     = relative mean longitude
%     dex         = delta e_x
%     dey         = delta e_y
%     dix         = delta inclination
%     diy         = delta RAAN*sin(iC) ]
%
%
% ------------------------------------------------------------------------
% OUTPUT
%
% DeputyOE :
%
%   [ aD
%     eD
%     iD
%     RAAND
%     wD
%     MD ]
%
%
% ------------------------------------------------------------------------
% NOTES
%
% • Angles are radians.
% • Fully reusable for optimization loops and simulations.
% • Handles row or column vector inputs.
%
% ------------------------------------------------------------------------
% Author : (Reusable GNC Utility)
% ========================================================================

%% -------- Input Validation ---------------------------------------------

if numel(ChiefOE) ~= 6
    error('ChiefOE must contain 6 elements.');
end

if numel(ROE) ~= 6
    error('ROE must contain 6 elements.');
end

ChiefOE = ChiefOE(:);
ROE     = ROE(:);

%% -------- Unpack Chief OE ----------------------------------------------

aC    = ChiefOE(1);
eC    = ChiefOE(2);
iC    = ChiefOE(3);
RAANC = ChiefOE(4);
wC    = ChiefOE(5);
MC    = ChiefOE(6);

%% -------- Unpack ROE ---------------------------------------------------

da      = ROE(1);
dLambda = ROE(2);

dex = ROE(3);
dey = ROE(4);

dix = ROE(5);
diy = ROE(6);

%% -------- Safety Checks ------------------------------------------------

if abs(sin(iC)) < 1e-10
    error(['Chief inclination too close to zero.\n' ...
           'RAAN difference undefined when sin(i)=0.']);
end

%% -------- Chief Eccentricity Vector ------------------------------------

exC = eC*cos(wC);
eyC = eC*sin(wC);

%% -------- Deputy Semi-major Axis ---------------------------------------

aD = aC*(1 + da);

%% -------- Deputy Inclination + RAAN ------------------------------------

iD = iC + dix;

deltaRAAN = diy / sin(iC);

RAAND = RAANC + deltaRAAN;

%% -------- Deputy Eccentricity + Argument of Perigee --------------------

exD = exC + dex;
eyD = eyC + dey;

eD = hypot(exD,eyD);

wD = atan2(eyD,exD);

%% -------- Deputy Mean Anomaly ------------------------------------------

% Relative mean longitude definition:
%
% dLambda =
% deltaM + delta(w) + cos(iC)*deltaRAAN

delta_w = wrapToPi(wD - wC);

deltaM = dLambda ...
        - delta_w ...
        - cos(iC)*deltaRAAN;

MD = MC + deltaM;

%% -------- Wrap Angles --------------------------------------------------

RAAND = wrapTo2Pi(RAAND);
wD    = wrapTo2Pi(wD);
MD    = wrapTo2Pi(MD);

%% -------- Output -------------------------------------------------------

DeputyOE = [aD; eD; iD; RAAND; wD; MD];

end
