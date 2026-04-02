orbitSemiMajorAxis1 =30700; %km
orbitSemiMajorAxis0 =18100; %km

warning('off', 'MATLAB:ode45:IntegrationTolNotMet');

orbitTrueAnamoly = 180;
r0 = orbitSemiMajorAxis0;
orbitSemiMajorAxis = (orbitSemiMajorAxis1 + orbitSemiMajorAxis0)/2;

[t, x, lambda, u] = orbitTransferData_H(orbitSemiMajorAxis1, orbitTrueAnamoly, orbitSemiMajorAxis0);

%% Problem data
orbitEccentricity = 0;
% r0 = 780+6378;
muVal = 398600;
% Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
x0 = [r0 0 0 0 sqrt(muVal/r0) 0 2000]';           

theta_f = deg2rad(orbitTrueAnamoly);
% Radius magnitude
rf = orbitSemiMajorAxis1;

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
x_min = -orbitSemiMajorAxis1; x_max = orbitSemiMajorAxis1;
y_min = -orbitSemiMajorAxis1; y_max = orbitSemiMajorAxis1;
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
   
    % Plot the target state (origin)
    plot(-rf, 0, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');
   
    % Create animated line for trajectory and marker for satellite
    hTraj = animatedline('LineWidth', 1.5, 'Color', 'b', 'DisplayName', 'Trajectory');

    % Load satellite image
    chaserImgFile = 'chaser2D.png';
    if exist(chaserImgFile, 'file')
        [satImg, ~, satAlpha] = imread(chaserImgFile);
        rSat = 3000; % Visible radius on plot (not to scale)
        % Initial position
        hSat = image('CData', satImg, ...
            'XData', x(1, 1) + [-rSat rSat], ...
            'YData', x(1, 2) + [rSat -rSat]);
        if ~isempty(satAlpha)
            set(hSat, 'AlphaData', satAlpha);
        end
    else
        hSat = plot(x(1, 1), x(1, 2), 'k.', 'MarkerSize', 20, 'DisplayName', 'Satellite');
        warning('Satellite image file "%s" not found. Using default marker.', chaserImgFile);
    end

    x_span = max(x(:,1)) - min(x(:,1));
    y_span = max(x(:,2)) - min(x(:,2));
    avg_span = (x_span + y_span) / 2;
    if avg_span == 0, avg_span = 1; end
    % Thrust vector components for plotting: direction * throttle
    u_vec_plot = u(:, 1:3) .* u(:, 4);
    u_mag = u(:, 4); % Throttle is the magnitude factor (assuming unit direction)
   
    max_u = max(u_mag);
    if max_u == 0, scale_inv = 1; else, scale_inv = (0.15 * avg_span) / max_u; end
    legend('show', 'Location', 'best');
   
   
    % Setup Video Writer
    videoFilename = 'satellite_maneuver.mp4';
    vWriter = VideoWriter(videoFilename, 'MPEG-4');
    vWriter.FrameRate = 20; % Adjust frame rate as needed
    % open(vWriter);
   
    % Loop through time steps to animate
    % Depending on the number of steps in 't', we might want to skip frames to keep video short
    nSteps = length(t);
    frameStep = max(1, floor(nSteps / 200)); % Target approx 300 frames max (~10s at 30fps)
    % viscircles([0 0],[r0],'LineStyle','--','Color',"r");
    % viscircles([0 0],[rf],'LineStyle','--','Color',"k");
    circle(0,0,r0,"k",'--');
    circle(0,0,rf,"r",'-.');
    %plot(x(end-40:end,1), x(end-40:end,2),"r-.")
   
    for k = 1:frameStep:nSteps
        xlim([x_min - 0.2*dx, x_max + 0.2*dx]);
        ylim([y_min - 0.2*dy, y_max + 0.2*dy]);
        addpoints(hTraj, x(k, 1), x(k, 2));
        if exist(chaserImgFile, 'file')
            set(hSat, 'XData', x(k, 1) + [-rSat rSat], 'YData', x(k, 2) + [rSat -rSat]);
        else
            set(hSat, 'XData', x(k, 1), 'YData', x(k, 2));
        end
        drawnow;
   
        axis equal;
    end