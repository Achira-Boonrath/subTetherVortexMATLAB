% ========================================================================
%  TWO–SPACECRAFT RELATIVE MOTION SIMULATION
%
%  PURPOSE
%  -------
%  This script propagates the motion of a CHIEF spacecraft and a DEPUTY
%  spacecraft in inertial space under two–body gravity and evaluates the
%  relative motion in the Hill (LVLH / CWH / RTN) frame.
%
%  The deputy spacecraft is controlled using a simple PD feedback law
%  designed to track a time-varying reference orbit derived from the chief.
%
%  Major Steps:
%
%   1) Define chief and deputy orbital elements.
%   2) Convert COEs → inertial Cartesian states (ECI).
%   3) Numerically propagate both spacecraft using ode45.
%   4) Convert relative states into Hill coordinates.
%   5) Visualize relative motion.
%
%  ASSUMPTIONS
%  -----------
%  • Two-body gravity only.
%  • Earth-centered inertial frame (ECI).
%  • Small deputy control accelerations.
%
% ========================================================================

close all
clear all
clc

% ========================================================================
%% CONSTANTS AND INITIAL CONDITIONS
% ========================================================================

% Earth's gravitational parameter
% Units : km^3/sec^2
mu = 398600.4418;

% ---------------- Chief Orbital Elements -------------------------------
%
% COE format:
%
% [ a , e , i , RAAN , omega , true anomaly ]
%
% a       = semi-major axis (km)
% e       = eccentricity
% i       = inclination (rad)
% RAAN    = right ascension ascending node (rad)
% omega   = argument of perigee (rad)
% nu      = true anomaly (rad)

% p1
% ChiefOE = [8500 , ...
%            0.1, ...
%            deg2rad(33), ...
%            deg2rad(45), ...
%            deg2rad(40), ...
%            deg2rad(85)];

%p3, rel orbit w/o ctrl
ChiefOE = [8500 , ...
           0, ...
           deg2rad(53), ...
           deg2rad(55), ...
           deg2rad(40), ...
           deg2rad(0)];

% %p4, rel orbit w/o ctrl
% ChiefOE = [8500 , ...
%            0.2, ...
%            deg2rad(53), ...
%            deg2rad(55), ...
%            deg2rad(40), ...
%            deg2rad(0)];

% ---------------- Deputy Orbital Elements ------------------------------
%
% Deputy initialized as a small perturbation of chief orbit.
%
% Small offsets approximate formation flying separation.

% p1
% DeputyOE = [0, ...
%              10/ChiefOE(1), ...
%              0, ...
%              10/ChiefOE(1), ...
%              0, ...
%              0] + ChiefOE;

% p3
DeputyOE = [0, ...
             10/ChiefOE(1), ...
             0, ...
             deg2rad(0.01), ...
             0, ...
             0] + ChiefOE;

%p4, rel orbit w/o ctrl
% DeputyOE = [0, ...
%              1e-3, ...
%              deg2rad(0.1), ...
%              deg2rad(0.1), ...
%              0, ...
%              0] + ChiefOE;

% Alternative test case:
% DeputyOE = [0,10/ChiefOE(1),0,deg2rad(0.01),0,0] + ChiefOE;

% ========================================================================
%% COE → INERTIAL STATE CONVERSION
% ========================================================================

% Convert chief orbital elements to ECI position and velocity.
[rECI,vECI,DCM_PQW2ECI] = COE2ECI(ChiefOE,mu);
% Convert deputy orbital elements.
[rECI_D,vECI_D,DCM_PQW2ECI_D] = COE2ECI(DeputyOE,mu);

% ========================================================================
%% NUMERICAL PROPAGATION (FORMATION MOTION)
% ========================================================================

% State vector definition:
%
% z =
% [ Chief Position (3)
%   Chief Velocity (3)
%   Deputy Position (3)
%   Deputy Velocity (3) ]
%

z0F = [rECI; vECI; rECI_D; vECI_D];

% Simulation timeline (seconds)
%
% Integration performed every 8 seconds.

timeFinalF = [0:8:4848];

% Propagate nonlinear dynamics using MATLAB adaptive RK solver.

[tF, zF] = ode45( ...
    @(t, z) relMotionCrtl(t, z, mu), ...
    timeFinalF, ...
    z0F, ...
    odeset('RelTol',1e-11,'AbsTol',1e-11,'Stats','off'));

% ========================================================================
%% RELATIVE MOTION VISUALIZATION (HILL FRAME)
% ========================================================================

figure

% Example time indices chosen for visualization.
% (beginning, middle, end of trajectory)

for i = [1:length(tF)]
% for i = [1,126,501]
% for i = [1,126,607]

    % Convert instantaneous inertial states → Hill frame.

    [rHill,vHill,DCM_ECI2Hill] = ...
        ECI2CWHRelativeState( ...
        zF(i,1:3),zF(i,4:6),...     % chief state
        zF(i,7:9),zF(i,10:12),mu)  % deputy state

    % Plot relative position in Hill coordinates.
    %
    % x : radial
    % y : along-track
    % z : cross-track

    scatter3(rHill(1), rHill(2), rHill(3))

    hold on

    xlabel("Radial, km")
    ylabel("Along-Track, km")
    zlabel("Cross-Track, km")

end

% axis equal    % optional scaling

% ========================================================================
%% DYNAMICS + CONTROL MODEL
% ========================================================================
%
% Propagates:
%
%   Chief → uncontrolled two-body motion.
%   Deputy → two-body motion + control acceleration.
%
% ========================================================================

function ds = relMotionCrtl(t, s, muEarth)

% ---------- STATE EXTRACTION -------------------------------------------

% Chief state
x = s(1:6);
% Deputy state
x_2 = s(7:end);

% ---------- REFERENCE ORBIT GENERATION ---------------------------------
%
% Construct a time-varying desired reference orbit.
%
% Procedure:
%   1) Compute chief true anomaly from inertial state.
%   2) Convert to mean anomaly.
%   3) Apply phase offset.
%   4) Convert back to true anomaly.
%
% This creates a slightly advanced reference trajectory.

% ecc = 0.1;
% trueAna = TrueAnomalyFromRV(x(1:3),x(4:6),muEarth);

% % p1
% [M,E] = True2MeanAnomaly(trueAna,ecc);
% % Desired phase lead
% M_dd = M + deg2rad(0.1);
% [nu_dd,E] = Mean2TrueAnomaly(M_dd,ecc);
% Desired reference orbit

% p1
% ChiefOE = [8500 , ecc , ...
%            deg2rad(33), ...
%            deg2rad(45), ...
%            deg2rad(40), ...
% 
% %p3
% ChiefOE = [8500+10, ...
%            1e-9, ...
%            deg2rad(53), ...
%            deg2rad(55), ...
%            deg2rad(40), ...
%            trueAna];
% [rECI,vECI,DCM_PQW2ECI] = COE2ECI(ChiefOE,muEarth);

rECI = 8510*(x(1:3)/norm(x(1:3)));
vECI = x(4:6);
% vECI = sqrt(muEarth/8510)*(x(4:6)/norm(x(4:6)));
% ---------- Tracking Errors --------------------------------------------

% Relative position error
dr = x_2(1:3) - rECI;
% Relative velocity error
drDot = x_2(4:6) - vECI;

% ---------- STATE DERIVATIVE INITIALIZATION ----------------------------

ds = zeros(12,1);

% ---------- Chief Dynamics (Two-body) ----------------------------------
% kinematics
ds(1) = x(4);
ds(2) = x(5);
ds(3) = x(6);

% Two-body gravitational acceleration model.
scAcc = @(x) [ ...
 -(muEarth*x(1))/(x(1)^2+x(2)^2+x(3)^2)^(3/2); ...
 -(muEarth*x(2))/(x(1)^2+x(2)^2+x(3)^2)^(3/2); ...
 -(muEarth*x(3))/(x(1)^2+x(2)^2+x(3)^2)^(3/2)];

% chief acceleration
ds(4:6) = scAcc(x);

% ---------- Deputy Dynamics --------------------------------------------

% kinematics
ds(7) = x_2(4);
ds(8) = x_2(5);
ds(9) = x_2(6);

% natural gravitational acceleration
ds(10:12) = scAcc(x_2);

% ---------- Reference Acceleration -------------------------------------

% acceleration of desired reference orbit
ds_dd = scAcc([rECI; vECI]');

% ---------- PD CONTROL LAW ---------------------------------------------
%
% Control objective:
%
%   drive deputy → desired reference orbit.
%
% Control:
%
%   u =
%   -(a_dep - a_ref)
%   - K1 * position error
%   - K2 * velocity error

K_1 = 0.00002;    % position gain
K_2 = 0.005;      % velocity gain

[rHill,vHill,DCM_ECI2Hill] = ...
    ECI2CWHRelativeState( ...
    x(1:3),x(4:6),...     % chief state
    x_2(1:3),x_2(4:6),muEarth);  % deputy state

ud = 0;%- K_1*( x(1:3) - 8510*(x(1:3)/norm(x(1:3))) );
% ud =  - K_1*DCM_ECI2Hill'*  [0.5480,-2.1957, 0]';
% ud =  - K_1*DCM_ECI2Hill'* [0.3405, -3.1924, 0]';
% ud = - (K_2*eye(3)*drDot + K_1*eye(3)*dr);%- K_1*( x(1:3) - 8510*(x(1:3)/norm(x(1:3))) );

u = -(ds(10:12) - ds_dd) ...
    - K_1*eye(3)*dr ...
    - K_2*eye(3)*drDot + ud;

% Apply control acceleration.

ds(10:12) = ds(10:12) + u;

end
