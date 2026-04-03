function [t, x, lambda, u] = orbitTransferData_H(af, ef, theta_f_deg, a0, e0)
    close all; 
%% ORBITTRANSFERDATA_H  Generates data for orbital transfer simulations.
%   [t, x, lambda, u] = orbitTransferData_H(af, ef, theta_f_deg, a0, e0)
% 
%   Inputs:
%       af          - Final orbit semi-major axis (km)
%       ef          - Final orbit eccentricity
%       theta_f_deg - Target true anomaly in degrees
%       a0          - Initial orbit semi-major axis (km)
%       e0          - Initial orbit eccentricity
%
%   Outputs:
%       t           - Time vector spanning the init, transfer, and final phases
%       x           - State trajectory [x, y, z, vx, vy, vz, m]
%       lambda      - Costate trajectory [lambda1 ... lambda7]
%       u           - Control history (currently a placeholder)

    %% Problem Data & Constants
    muVal = 398600; % Earth's gravitational parameter (km^3/s^2)
    theta_f = deg2rad(theta_f_deg); % Convert target true anomaly to radians
    
    %% Compute Initial State (Initial Orbit at Periapsis)
    % Compute the parameter (p0) and radius at periapsis (r0) of the initial orbit
    p0 = a0 * (1 - e0^2);
    r0 = p0 / (1 + e0);
    
    % Compute the velocity at periapsis of the initial elliptical orbit
    v_scale_0 = sqrt(muVal / p0);
    v0_mag = v_scale_0 * (e0 + 1);
    
    % Initial state vector format: [x, y, z, vx, vy, vz, mass]
    x0 = [r0 0 0 0 v0_mag 0 1000]';           
    
    et = ef; % Target eccentricity
    % Compute the radius magnitude of the target point at theta_f
    p = af * (1 - ef^2);
    r_mag = p / (1 + ef*cos(theta_f));
    
    %% Compute Final Orbit Parameters (Apoapsis Boundary)
    pf_orbit = af * (1 - ef^2);
    rf_apo = pf_orbit / (1 - ef); % Target orbit apoapsis distance

    % Determine a baseline orbital transfer duration (tf)
    aTrans = (rf_apo + r0)/2;
    period = 2*pi*sqrt( (aTrans^3) /muVal  );
    if theta_f == 0
        tf = period * 0.5; % Hohmann transfer baseline time (half period)
    else
        tf = period*(theta_f/(2*pi)); % Wait time defined by angular separation
    end
    
    %% Optimization & ODE Boundary Values
    % Initialize costate guess for optimal control tracking (using Hamiltonian formulation)
    % This is set near zero to evaluate basic or unpowered propagation.
    % lambda0_guess = 1e-4*ones(7,1);
    % for minE
    lambda0_guess = [0.060776702600281   0.000298627734904   1.000667203169005   0.500000000000000   0.500000000000000   0.069330582561975   0.672142996295746]';

    % Update x0 using a purely analytical transfer orbit injection velocity
    % Compute required parameters for Hohmann transfer intersecting r_mag
    p_trans = 2 * r0 * r_mag / (r0 + r_mag);
    v_trans = sqrt(muVal * p_trans) / r0;
    x0(4:6) = [0; v_trans; 0]; % Overwrite initial velocity with transfer velocity

    muEarth = muVal;
    
    % Propulsion constraints and characteristics (Used in optimal control switching)
    Tmax = 425*4e-3; % Max thrust (kN)
    Isp = 230; % Specific impulse (s)
    g0 = 9.81e-3; % Standard gravity constant (km/s^2)
    epsilon = 1; % Smoothing parameter for control law

    %% ODE Integration for Phasing
    
    options = odeset('Stats','off');
    
    % 1. Simulate the steady initial orbit (unpowered) over its orbital period
    periodInit = 2*pi*sqrt( (a0^3) /muVal  );
    z0Init = [1e-8*lambda0_guess; [r0 0 0 0 v0_mag 0 1000]'];
    timeFinalInit = linspace(0, periodInit, 300);
    [tInit, zInit] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinalInit, z0Init, options);

    % 2. Simulate the transfer orbit using the computed Hohmann injection velocity
     minEctrl = false;
    if minEctrl == false
    [t,z] = transferSimImpulsive(lambda0_guess,x0,tf,muEarth,Tmax,Isp,g0,epsilon,options,tInit);
    else
    [t,z] = optControl_singleShoot_Demo2(lambda0_guess,x0,tf,muEarth,Tmax,Isp,g0,epsilon,options,tInit);
    end

    % 3. Simulate the target (Final) orbit to visualize where the transfer terminates
    % Set boundary from the previously determined apoapsis condition
    pf_orbit = af * (1 - ef^2);
    rf_apo = pf_orbit / (1 - ef);
    v_scale_f = sqrt(muVal / pf_orbit);
    vf_apo = v_scale_f * (ef - 1);
    z0F = [1e-8*lambda0_guess; [-rf_apo 0 0 0 vf_apo 0 1000]'];
    
    periodF = 2*pi*sqrt( (af^3) /muVal  );
    timeFinalF = linspace(0, periodF, 300) ;
    [tF, zF] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinalF, z0F, odeset('RelTol', 1e-8, 'AbsTol', 1e-9,'Stats','off'));
    tF = tF + t(end); % Shift final orbit time array 
    
    %% Combine all phases
    % Concatenate the initial, transfer, and final phases into a single trajectory map
    z = [zInit; z; zF];
    t = [tInit; t; tF];
    
    %% Output Structuring
    % Extract state and costate trajectories from concatenated z states array
    % columns 1:7 are costates (lambdas), columns 8:14 are actual states (orbit variables)
    x = z(:, (length(x0)+1):end);       
    lambda = z(:, 1:length(x0));            
    
    % The control matrix formulation (u) is typically extracted via costates.
    % Currently deactivated in favor of unpowered phasing.
    u = 0;
end

function [t,z] = transferSimImpulsive(lambda0_guess,x0,tf,muEarth,Tmax,Isp,g0,epsilon,options,tInit)
% Stack initial condition z0 = [lambda0; x0] to integrate the ODE
z0 = [1e-8*lambda0_guess; x0];
timeFinal = linspace(0, tf, 300) ;
[t, z] = ode45(@(t, z) hamiltonian_odeConstT(t, z, muEarth, Tmax, Isp, g0, epsilon), timeFinal, z0, options);
t = t + tInit(end); % Shift transfer time array to proceed strictly after initial orbit phase
end