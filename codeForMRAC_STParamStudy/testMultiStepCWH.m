clear; clc; close all;

%% Constants
mu = 3.986e14;
Re = 6771e3;
n = sqrt(mu/Re^3);

%% Waypoints (LVLH frame)
% P0 = [-1000; -3000; 0];
% P1 = [-1000;  -1500;  0];
% P2 = [-500;  -500;  0];
% P3 = [-100;   -100;   0];
% Pf = [0; 0; 0];

%% Waypoints (LVLH frame)
P0 = [-1000; -2000; 0];
P1 = [-0;  -500;  0];
P2 = [-0;  -100;  0];
P3 = [-0;   -10;   0];
Pf = [0; 0; 0];

waypoints = {P0, P1, P2, P3, Pf};

%% Time per phase
t_phase = [0.8*2000, 1500, 1000, 500];

%% Initial velocity
v0 = [0;0;0];

traj = [];
t_all = [];
phase_id = [];

time_offset = 0;

%% --- Generate trajectory using STM targeting ---
for i = 1:length(t_phase)
    
    r0 = waypoints{i};
    rf = waypoints{i+1};
    tf = t_phase(i);
    
    % STM
    [Phi_rr, Phi_rv, ~, ~] = cw_stm(n, tf);
    
    % Required velocity
    v_req = Phi_rv \ (rf - Phi_rr*r0);
    
    % Delta-V
    dv = v_req - v0;
    fprintf('Phase %d ΔV: [%f %f %f] m/s\n', i, dv);
    
    % Apply impulse
    v0 = v0 + dv;
    
    % Propagate
    x0 = [r0; v0];
    [t, x] = ode45(@(t,x) cwh(t,x,n,[0;0;0]), [0 tf], x0);
    
    traj = [traj; x];
    t_all = [t_all; t + time_offset];
    phase_id = [phase_id; i*ones(length(t),1)];
    
    % Update state
    r_end = x(end,1:3)';
    v0 = x(end,4:6)';
    waypoints{i+1} = r_end;
    
    time_offset = t_all(end);
end

%% Extract position
x = traj(:,1);
y = traj(:,2);

%% --- Animation ---
%% --- Animation ---
figure;
hold on; grid on;
axis equal;

xlabel('R-bar [m]');
ylabel('V-bar [m]');
title('Impulsive Rendezvous Animation (Box-Wing Satellite)');

% Target (circle)
theta = linspace(0,2*pi,100);
r_target = 20;
plot(r_target*cos(theta), r_target*sin(theta), 'r', 'LineWidth', 2);

% Full trajectory
plot(x, y, '--', 'Color', [0.7 0.7 0.7]);

xlim([min(x)-200 max(x)+200]);
ylim([min(y)-200 max(y)+200]);

%% --- Define satellite geometry (body frame) ---

% Main body (square)
body_size = 40;
body = body_size/2 * [-1 -1;
                      1 -1;
                      1  1;
                     -1  1]';

% Solar panels (rectangles)
panel_length = 80;
panel_width  = 20;

left_panel = [ -body_size/2, -panel_width/2;
               -body_size/2 - panel_length, -panel_width/2;
               -body_size/2 - panel_length,  panel_width/2;
               -body_size/2,  panel_width/2 ]';

right_panel = [ body_size/2, -panel_width/2;
                body_size/2 + panel_length, -panel_width/2;
                body_size/2 + panel_length,  panel_width/2;
                body_size/2,  panel_width/2 ]';

%% --- Create patch objects ---
h_body = patch('XData', [], 'YData', [], 'FaceColor', 'g');
h_left = patch('XData', [], 'YData', [], 'FaceColor', 'b');
h_right = patch('XData', [], 'YData', [], 'FaceColor', 'b');

% Phase label
phase_text = text(0.05,0.95,'','Units','normalized');

%% --- Animation loop ---
for k = 1:10:length(x)
    
    % Current position
    xc = x(k);
    yc = y(k);
    
    % Optional: align satellite with velocity direction
    vx = traj(k,4);
    vy = traj(k,5);
    % theta_rot = atan2(vy, vx);
    theta_rot = 0;

    
    R = [cos(theta_rot) -sin(theta_rot);
         sin(theta_rot)  cos(theta_rot)];
    
    % Rotate and translate shapes
    body_rot = R * body + [xc; yc];
    left_rot = R * left_panel + [xc; yc];
    right_rot = R * right_panel + [xc; yc];
    
    % Update graphics
    set(h_body, 'XData', body_rot(1,:), 'YData', body_rot(2,:));
    set(h_left, 'XData', left_rot(1,:), 'YData', left_rot(2,:));
    set(h_right,'XData', right_rot(1,:), 'YData', right_rot(2,:));
    
    % Update phase text
    current_phase = phase_id(k);
    set(phase_text, 'String', sprintf('Phase %d', current_phase));
    
    drawnow;
    pause(0.1);
end

disp('Docking complete.');


disp('Docking complete.');

function [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, t)

nt = n*t;
c = cos(nt);
s = sin(nt);

Phi_rr = [4-3*c,      0,        0;
          6*(s-nt),   1,        0;
          0,          0,        c];

Phi_rv = [1/n*s,      2/n*(1-c),    0;
          2/n*(c-1),  1/n*(4*s-3*nt),0;
          0,          0,            1/n*s];

Phi_vr = [3*n*s,      0,        0;
          6*n*(c-1),  0,        0;
          0,          0,       -n*s];

Phi_vv = [c,          2*s,      0;
          -2*s,       4*c-3,    0;
          0,          0,        c];
end

function dxdt = cwh(~, x, n, u)

dxdt = zeros(6,1);

dxdt(1:3) = x(4:6);

dxdt(4) = 3*n^2*x(1) + 2*n*x(5) + u(1);
dxdt(5) = -2*n*x(4) + u(2);
dxdt(6) = -n^2*x(3) + u(3);

end
