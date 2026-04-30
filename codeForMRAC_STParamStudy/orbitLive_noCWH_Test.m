% Good for deorbiting show
% orbitSemiMajorAxis1 = 6900; %km
% orbitEccentricity1 = 0.1; % final orbit eccentricity
% orbitSemiMajorAxis0 = 8100; %km
% orbitEccentricity0 = 0.0001; % initial orbit eccentricity
% orbitArgPeriapsis1 = -150; % final orbit argument of periapsis (degrees)
% orbitArgPeriapsis0 = 0; % initial orbit argument of periapsis (degrees)
clear all
showAxesAndGrid = false; % Toggle for x/y ticks and grid lines

%% old set
% drag showcase
% orbitSemiMajorAxis1 = 7500; %km
% orbitEccentricity1 = 0.06; % final orbit eccentricity
% orbitSemiMajorAxis0 = 8000; %km
% orbitEccentricity0 = 0.0001; % initial orbit eccentricity
% orbitArgPeriapsis1 = -160; % final orbit argument of periapsis (degrees)
% orbitArgPeriapsis0 = 0; % initial orbit argument of periapsis (degrees)
% satellitePlotStyle = 'boxwing';

% deorbit showcase - debris
% orbitSemiMajorAxis1 = 7200; %km
% orbitEccentricity1 = 0.08; % final orbit eccentricity
% orbitSemiMajorAxis0 = 7800; %km
% orbitEccentricity0 = 0.0001; % initial orbit eccentricity
% orbitArgPeriapsis1 = -179; % final orbit argument of periapsis (degrees)
% orbitArgPeriapsis0 = 0; % initial orbit argument of periapsis (degrees)
% satellitePlotStyle = 'square';

% eject showcase - debris
% orbitSemiMajorAxis1 = 7300; %km
% orbitEccentricity1 = 0.085; % final orbit eccentricity
% orbitSemiMajorAxis0 = 7500; %km
% orbitEccentricity0 = 0.06; % initial orbit eccentricity
% orbitArgPeriapsis1 = -160; % final orbit argument of periapsis (degrees)
% orbitArgPeriapsis0 = -160.00001; % initial orbit argument of periapsis (degrees)
% satellitePlotStyle = 'square';

% % eject showcase - RESTORE
% orbitSemiMajorAxis1 = 7500; %km
% orbitEccentricity1 = 0.06; % final orbit eccentricity
% orbitSemiMajorAxis0 = 7500; %km
% orbitEccentricity0 = 0.06; % initial orbit eccentricity
% orbitArgPeriapsis1 = -160; % final orbit argument of periapsis (degrees)
% orbitArgPeriapsis0 = -160.00001; % initial orbit argument of periapsis (degrees)
% satellitePlotStyle = 'boxwing';

% % reboost showcase - RESTORE
orbitSemiMajorAxis1 = 8000; %km
orbitEccentricity1 = 0.0001; % final orbit eccentricity
orbitSemiMajorAxis0 = 7500 ; %km
orbitEccentricity0 = 0.06 ; % initial orbit eccentricity
orbitArgPeriapsis1 = 210; % final orbit argument of periapsis (degrees)
orbitArgPeriapsis0 = 200; % initial orbit argument of periapsis (degrees)
satellitePlotStyle = 'boxwing';

%%

warning('off', 'MATLAB:ode45:IntegrationTolNotMet');

orbitTrueAnamoly = 0;
r0 = orbitSemiMajorAxis0 * (1 - orbitEccentricity0^2) / (1 + orbitEccentricity0); % initial periapsis
orbitSemiMajorAxis = (orbitSemiMajorAxis1 + orbitSemiMajorAxis0)/2;

[t, x, lambda, u] = orbitTransferData_H(orbitSemiMajorAxis1, orbitEccentricity1, orbitTrueAnamoly, orbitSemiMajorAxis0, orbitEccentricity0, orbitArgPeriapsis1, orbitArgPeriapsis0);

%% Problem data
muVal = 398600;
% Initial state: x, y, z (m), vx, vy, vz (m/s), m (kg)
v0_mag = sqrt(muVal / (orbitSemiMajorAxis0 * (1 - orbitEccentricity0^2))) * (orbitEccentricity0 + 1);
omega_0 = deg2rad(orbitArgPeriapsis0);
r0_x = r0 * cos(omega_0);
r0_y = r0 * sin(omega_0);
v0_x = -v0_mag * sin(omega_0);
v0_y = v0_mag * cos(omega_0);
x0 = [r0_x r0_y 0 v0_x v0_y 0 2000]';           

theta_f = deg2rad(orbitTrueAnamoly);
% Radius magnitude at periapsis of final orbit
rf = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2) / (1 + orbitEccentricity1);

figure;
set(gcf, 'Color', 'w');
set(gcf, 'Position',  [0, 0, 1080, 1080]*0.9)
hold on; axis equal;

if exist('showAxesAndGrid', 'var') && showAxesAndGrid
    grid on;
    xlabel('x (m)'); ylabel('y (m)');
else
    set(gca, 'XTick', [], 'YTick', []);
    grid off;
    xlabel(''); ylabel('');
end

% Circle parameters
h = 0; % x-coordinate of center
k = 0; % y-coordinate of center

% Implicit equation: (x-h)^(2) + (y-k)^(2) - r^(2) = 0

% title('Satellite Maneuvering Animation');

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
    omega_0_plot = deg2rad(orbitArgPeriapsis0);
    hInit = plot(r0_orbit .* cos(th + omega_0_plot), r0_orbit .* sin(th + omega_0_plot), 'k--', 'DisplayName', 'Initial Orbit');
    
    omega_f = deg2rad(orbitArgPeriapsis1);
    pf = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2);
    rf_orbit = pf ./ (1 + orbitEccentricity1 * cos(th));
    hFinal = plot(rf_orbit .* cos(th + omega_f), rf_orbit .* sin(th + omega_f), 'r-.', 'LineWidth', 2.5, 'DisplayName', 'Target Orbit');

    % Plot the target state
    r_target = orbitSemiMajorAxis1 * (1 - orbitEccentricity1^2) / (1 + orbitEccentricity1 * cos(theta_f));
    target_x = r_target * cos(theta_f + omega_f);
    target_y = r_target * sin(theta_f + omega_f);
    % hTarget = plot(target_x, target_y, 'gx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target Point');
   
    % Create animated line for trajectory and marker for satellite
    hTraj = animatedline('LineWidth', 2.5, 'Color', 'b', 'DisplayName', 'Transfer Trajectory');

    % Create proxy for Satellite legend
    hSatProxy = patch(NaN, NaN, 'g', 'EdgeColor', 'k', 'DisplayName', 'Satellite');

    % % Main body (square)
    body_size = 2.4*134;
    % Solar panels (rectangles)
    panel_length = 2.4*148;
    panel_width  = 2.4*122;
    params.BodySize    = body_size;
    params.PanelLength = panel_length;
    params.PanelWidth  = panel_width;

    x_span = max(x(:,1)) - min(x(:,1));
    y_span = max(x(:,2)) - min(x(:,2));
    avg_span = (x_span + y_span) / 2;
    if avg_span == 0, avg_span = 1; end

    % Define explicit legend
    % lgd = legend([hInit, hFinal, hTarget, hTraj, hSatProxy], 'Location', 'best');
    % set(lgd, 'AutoUpdate', 'off');
   
    % Setup Video Writer
    % videoFilename = 'satellite_maneuver.avi';
    % vWriter = VideoWriter(videoFilename);
    % vWriter.FrameRate = 20; % Adjust frame rate as needed

    % vWriter = VideoWriter('satellite_maneuver.avi','Motion JPEG AVI');
    vWriter = VideoWriter('satellite_maneuver.mp4','MPEG-4');
    vWriter.FrameRate = 20;   % Set to 20 frames per second
    % vWriter.Quality = 100;
    open(vWriter);
   
    % Allow user to choose satellite plotting style:
    % 'boxwing' -> use draw_boxwing_satellite (existing behavior)
    % 'square'  -> draw a simple green square (patch) at satellite location
    % Set default style here; user can change before running animation
    if ~exist('satellitePlotStyle', 'var') || isempty(satellitePlotStyle)
        satellitePlotStyle = 'boxwing'; % options: 'boxwing' or 'square'
    end

    % Loop through time steps to animate
    % Depending on the number of steps in 't', we might want to skip frames to keep video short
    nSteps = length(t);
    frameStep = max(1, floor(nSteps / 200)); % Target approx 300 frames max (~10s at 30fps)
   
    % Pre-calculate burns (impulses) at Phase transitions
    burn_idx1 = 301;
    burn_idx2 = 401;
    % dv1_c = [0; x(burn_idx1, 5) - x(burn_idx1-1, 5)]
    dv1_c = [x(burn_idx1, 4) - x(burn_idx1-1, 4); x(burn_idx1, 5) - x(burn_idx1-1, 5)]
    % dv2_c = -[x(burn_idx2, 4) - x(burn_idx2-1, 4); x(burn_idx2, 5) - x(burn_idx2-1, 5)];
    arrowLength = avg_span * 0.15;
    dv1_norm = dv1_c / norm(dv1_c) * arrowLength;
    % dv2_norm = dv2_c / norm(dv2_c) * arrowLength;
    arrow_frames_left = 0;
    arrow_u = 0; arrow_v = 0;

    hArrow = quiver(nan, nan, nan, nan, 0, 'Color', 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Burn Direction');

    % Precreate graphics handle for square option to reuse
    h_square = [];
    for k = 1:frameStep:nSteps
        xlim([x_min - 0.1*dx, x_max + 0.1*dx]);
        ylim([y_min - 0.1*dy, y_max + 0.1*dy]);
        addpoints(hTraj, x(k, 1), x(k, 2));
        xc = x(k, 1);
        yc = x(k, 2);
        theta_rot = atan2(x(k, 5), x(k, 4)) + pi/2; % align with velocity

        % Draw/update satellite according to selected style
        switch lower(satellitePlotStyle)
            case 'square'
                % Square size scaled relative to orbital span for visibility
                sq_half = max(0.0015*avg_span, body_size*0.5);
                % Define rotated square corners
                R = [cos(theta_rot), -sin(theta_rot); sin(theta_rot), cos(theta_rot)];
                corners = sq_half * [-1 -1; 1 -1; 1 1; -1 1]';
                rotated = R * corners;
                verts_x = rotated(1, :) + xc;
                verts_y = rotated(2, :) + yc;
                if k == 1
                    h_square = patch(verts_x, verts_y, 'g', 'EdgeColor', 'k', 'FaceAlpha', 1, 'DisplayName', 'Satellite');
                else
                    set(h_square, 'XData', verts_x, 'YData', verts_y);
                end

            otherwise % 'boxwing' and any other values default to boxwing
                if k == 1
                    h_sat = draw_boxwing_satellite(xc, yc, theta_rot, params);
                    % If previously created square handle exists, delete it
                    if ~isempty(h_square) && isgraphics(h_square)
                        delete(h_square);
                        h_square = [];
                    end
                else
                    h_sat = draw_boxwing_satellite(xc, yc, theta_rot, params, h_sat);
                end
        end

        % Burn indication logic
        % if k > 1 && k <= burn_idx1 && k + frameStep > burn_idx1
        %     arrow_frames_left = 3;
        %     arrow_u = dv1_norm(1); arrow_v = dv1_norm(2);
        % elseif k > 1 && k <= burn_idx2 && k + frameStep > burn_idx2
        %     % arrow_frames_left = 10;
        %     % arrow_u = dv2_norm(1); arrow_v = dv2_norm(2);
        % end
        % 
        % if arrow_frames_left > 0
        %     set(hArrow, 'XData', xc, 'YData', yc, 'UData', arrow_u, 'VData', arrow_v);
        %     arrow_frames_left = arrow_frames_left - 1;
        % else
        %     set(hArrow, 'XData', nan, 'YData', nan, 'UData', nan, 'VData', nan);
        % end

        drawnow;
        
        frame = getframe(gcf);
        writeVideo(vWriter, frame);
   
        axis equal;
    end
    
    close(vWriter);