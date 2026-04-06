orbitSemiMajorAxis1 = 40700; %km
orbitEccentricity1 = 0.52; % final orbit eccentricity
orbitSemiMajorAxis0 = 8100; %km
orbitEccentricity0 = 0.31; % initial orbit eccentricity
orbitArgPeriapsis1 = 40; % final orbit argument of periapsis (degrees)

warning('off', 'MATLAB:ode45:IntegrationTolNotMet');

orbitTrueAnamoly = 180;
r0 = orbitSemiMajorAxis0 * (1 - orbitEccentricity0^2) / (1 + orbitEccentricity0); % initial periapsis
orbitSemiMajorAxis = (orbitSemiMajorAxis1 + orbitSemiMajorAxis0)/2;

[t, x, lambda, u] = orbitTransferData_H(orbitSemiMajorAxis1, orbitEccentricity1, orbitTrueAnamoly, orbitSemiMajorAxis0, orbitEccentricity0, orbitArgPeriapsis1);

%% Problem data
muVal = 398600;
% Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
v0_mag = sqrt(muVal / (orbitSemiMajorAxis0 * (1 - orbitEccentricity0^2))) * (orbitEccentricity0 + 1);
x0 = [r0 0 0 0 v0_mag 0 2000]';           

theta_f = deg2rad(orbitTrueAnamoly);
% Radius magnitude at periapsis of final orbit
rf = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2) / (1 + orbitEccentricity1);

figure;
set(gcf, 'Color', 'w');
set(gcf, 'Position',  [0, 0, 1080, 1080]*0.7)
hold on; axis equal; grid on;
% Circle parameters
h = 0; % x-coordinate of center
k = 0; % y-coordinate of center

% Implicit equation: (x-h)^(2) + (y-k)^(2) - r^(2) = 0

xlabel('x (m)'); ylabel('y (m)');
title('Satellite Maneuvering Animation');

% Set axis limits with some padding
% x_min = min(x(:,1)); x_max = max(x(:,1));
% y_min = min(x(:,2)); y_max = max(x(:,2));
x_min = -max(orbitSemiMajorAxis1, orbitSemiMajorAxis0); x_max = max(orbitSemiMajorAxis1, orbitSemiMajorAxis0);
y_min = -max(orbitSemiMajorAxis1, orbitSemiMajorAxis0); y_max = max(orbitSemiMajorAxis1, orbitSemiMajorAxis0);
dx = x_max - x_min; if dx==0, dx=1; end
dy = y_max - y_min; if dy==0, dy=1; end

% Import and display Earth image
    % Check if image exists
    earthImgFile = 'earth_PNG3.png';
    if exist(earthImgFile, 'file')
        % Read image and alpha channel if available
        [img, ~, alpha] = imread(earthImgFile);
   
        % Constants for Earth
        rEarth = 6378; % km
   
        % Plot image with correct extent
        % XData: [-rEarth, rEarth], YData: [rEarth, -rEarth] (top-down)
        hEarth = image('CData', img, 'XData', [-rEarth rEarth], 'YData', [rEarth -rEarth]);
   
        % Apply transparency if alpha channel exists
        if ~isempty(alpha)
            set(hEarth, 'AlphaData', alpha);
        end
   
        % Ensure axis direction is normal for cartesian plot
        set(gca, 'YDir', 'normal');
    else
        warning('Earth image file "%s" not found.', earthImgFile);
    end
   
    % Plot initial and final orbits accurately
    th = linspace(0, 2*pi, 100);
    p0 = orbitSemiMajorAxis0 * (1 - orbitEccentricity0^2);
    r0_orbit = p0 ./ (1 + orbitEccentricity0 * cos(th));
    hInit = plot(r0_orbit .* cos(th), r0_orbit .* sin(th), 'k--', 'DisplayName', 'Initial Orbit');
    
    omega_f = deg2rad(orbitArgPeriapsis1);
    pf = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2);
    rf_orbit = pf ./ (1 + orbitEccentricity1 * cos(th));
    hFinal = plot(rf_orbit .* cos(th + omega_f), rf_orbit .* sin(th + omega_f), 'r-.', 'DisplayName', 'Target Orbit');

    % Plot the target state
    r_target = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2) / (1 + orbitEccentricity1 * cos(theta_f));
    target_x = r_target * cos(theta_f + omega_f);
    target_y = r_target * sin(theta_f + omega_f);
    hTarget = plot(target_x, target_y, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target Point');
   
    % Create animated line for trajectory and marker for satellite
    hTraj = animatedline('LineWidth', 1.5, 'Color', 'b', 'DisplayName', 'Transfer Trajectory');

    % Create proxy for Satellite legend
    hSatProxy = patch(NaN, NaN, 'g', 'EdgeColor', 'k', 'DisplayName', 'Satellite');

    % % Main body (square)
    body_size = 1340;
    % Solar panels (rectangles)
    panel_length = 1480;
    panel_width  = 1220;
    params.BodySize    = body_size;
    params.PanelLength = panel_length;
    params.PanelWidth  = panel_width;

    x_span = max(x(:,1)) - min(x(:,1));
    y_span = max(x(:,2)) - min(x(:,2));
    avg_span = (x_span + y_span) / 2;
    if avg_span == 0, avg_span = 1; end

    % Define explicit legend
    lgd = legend([hInit, hFinal, hTarget, hTraj, hSatProxy], 'Location', 'best');
    set(lgd, 'AutoUpdate', 'off');
   
    % Setup Video Writer
    videoFilename = 'satellite_maneuver.mp4';
    vWriter = VideoWriter(videoFilename, 'MPEG-4');
    vWriter.FrameRate = 20; % Adjust frame rate as needed
    % open(vWriter);
   
    % Loop through time steps to animate
    % Depending on the number of steps in 't', we might want to skip frames to keep video short
    nSteps = length(t);
    frameStep = max(1, floor(nSteps / 200)); % Target approx 300 frames max (~10s at 30fps)
   
    for k = 1:frameStep:nSteps
        xlim([x_min - 0.35*dx, x_max + 0.35*dx]);
        ylim([y_min - 0.35*dy, y_max + 0.35*dy]);
        addpoints(hTraj, x(k, 1), x(k, 2));
        xc = x(k, 1);
        yc = x(k, 2);
        theta_rot = atan2(x(k, 5), x(k, 4)) + pi/2; % align with velocity
        % theta_rot = atan2(x(k, 4), x(k, 5)); % align with velocity

        % Draw/update satellite
        if k == 1
            h_sat = draw_boxwing_satellite(xc, yc, theta_rot, params);
        else
            h_sat = draw_boxwing_satellite(xc, yc, theta_rot, params, h_sat);
        end
        drawnow;
   
        axis equal;
    end