%% Orbital Rendezvous Animation -- CWH / Hill Frame
% Reproduces the trajectory shape from the classic rendezvous diagram:
%
%  Phase 1-2 (Phasing)    : Two teardrop drift loops below V-bar.
%                           Chaser starts below-and-behind the target.
%  Phase 3   (Homing)     : Large arc rising above V-bar then returning.
%  Phase 4   (Closing)    : Small hook curving above V-bar to P3.
%  Phase 5   (Final appr.): Straight approach along V-bar to docking.
%
% Convention:  x = V-bar (+x = ahead of target)
%              z = R-bar (+z = toward Earth, plotted downward)
%
% CWH free-drift ODE:
%   xdd =  2n*zd + 3n^2*x
%   zdd = -2n*xd

clear; close all; clc;

%% ── Orbital parameters ────────────────────────────────────────────────────
mu    = 3.986004418e14;
R_E   = 6.3781e6;
h     = 400e3;
n     = sqrt(mu / (R_E + h)^3);
T_orb = 2*pi / n;

fprintf('n = %.6f rad/s,  T = %.2f min\n', n, T_orb/60);

odeOpts = odeset('RelTol',1e-11, 'AbsTol',1e-14);

%% ── Phase 1 & 2: Phasing loops ────────────────────────────────────────────
% Tangential burn from below V-bar; natural CWH motion traces a teardrop
% that drifts forward each revolution.
x0 = -52e3;   z0 = 9e3;        % initial position [m]  (z>0 = below V-bar)
vx1 = 18.5;   vz1 = 0;         % tangential burn [m/s]
T_loop = 0.97 * T_orb;         % slightly less than 1 period (non-resonant)

S1 = integ(n, [x0;z0], [vx1;vz1], T_loop, 500, odeOpts);

% Loop 2: restart from where loop 1 ended
S2 = integ(n, S1(end,1:2)', [19.0;0], T_loop, 500, odeOpts);

%% ── Phase 3: Homing  P1 -> P2 ────────────────────────────────────────────
P1 = S2(end,1:2)';
P2 = [-8e3; -0.3e3];            % slightly above V-bar (z < 0)
T_hom = 0.45 * T_orb;

v0_hom = dv_target(n, P1, P2, T_hom);
S3 = integ(n, P1, v0_hom, T_hom, 400, odeOpts);

%% ── Phase 4: Closing  P2 -> P3 ───────────────────────────────────────────
P3 = [-2.5e3; 0];               % on V-bar, just behind target
T_clos = 0.28 * T_orb;

v0_clos = dv_target(n, S3(end,1:2)', P3, T_clos);
S4 = integ(n, S3(end,1:2)', v0_clos, T_clos, 250, odeOpts);

%% ── Phase 5: Final approach  P3 -> Dock ──────────────────────────────────
T_fin = 0.06 * T_orb;

v0_fin = dv_target(n, S4(end,1:2)', [0;0], T_fin);
S5 = integ(n, S4(end,1:2)', v0_fin, T_fin, 100, odeOpts);

%% ── Assemble ──────────────────────────────────────────────────────────────
Sall   = [S1; S2; S3; S4; S5];
phases = [ones(size(S1,1),1);  2*ones(size(S2,1),1);
          3*ones(size(S3,1),1); 4*ones(size(S4,1),1);
          5*ones(size(S5,1),1)];

Tdurs = [T_loop, T_loop, T_hom, T_clos, T_fin];
szs   = [size(S1,1), size(S2,1), size(S3,1), size(S4,1), size(S5,1)];
Tbrk  = cumsum([0, Tdurs]);
t_cum = [];
for k = 1:5
    t_cum = [t_cum, linspace(Tbrk(k), Tbrk(k+1), szs(k))]; %#ok<AGROW>
end
t_cum = t_cum';

x_km = Sall(:,1)/1e3;
z_km = Sall(:,2)/1e3;
N    = size(Sall,1);
fprintf('Points: %d,  Mission: %.1f min\n', N, Tbrk(end)/60);

%% ── Styling ───────────────────────────────────────────────────────────────
phNames  = {'Phasing 1 (Ground)', 'Phasing 2 (Ground)', ...
            'Homing',             'Closing',  'Final Approach'};
phColors = {[0.25 0.55 1.00], [0.20 0.45 0.95], ...
            [0.20 0.85 0.45], [1.00 0.58 0.12], ...
            [1.00 0.22 0.22]};

%% ── Figure ────────────────────────────────────────────────────────────────
fig = figure('Name','Orbital Rendezvous Animation', ...
             'Color',[0.04 0.04 0.10], 'Position',[40 40 1340 700]);

ax = axes(fig, 'Color',[0.04 0.04 0.10], ...
    'XColor',[0.75 0.75 0.80], 'YColor',[0.75 0.75 0.80], ...
    'GridColor',[0.18 0.18 0.28], 'GridAlpha',0.55, ...
    'FontName','Consolas', 'FontSize',10, ...
    'Box','on', 'XGrid','on', 'YGrid','on', ...
    'YDir','reverse');          % R-bar points DOWN
hold(ax,'on');

xPad=4; zPad=1.8;
xL = [min(x_km)-xPad,  max(x_km)+xPad];
zL = [min(z_km)-zPad,  max(z_km)+zPad];
xlim(ax,xL);  ylim(ax,zL);

xlabel(ax, 'V-bar  (km)     \rightarrow     direction of orbital motion','FontSize',11);
ylabel(ax, 'R-bar  (km)     \downarrow     toward Earth',                 'FontSize',11);
title(ax,  'Orbital Rendezvous  |  LVLH Frame  (CWH equations + ode45)', ...
    'Color',[1 1 1], 'FontSize',13, 'FontWeight','bold');

%% Static decorations
line(ax, xL, [0 0], 'Color',[0.82 0.82 0.82], 'LineWidth',1.2, ...
    'LineStyle','--', 'HandleVisibility','off');
text(ax, xL(1)+0.4, -0.4, 'V-bar  (target orbit)', ...
    'Color',[0.78 0.78 0.78], 'FontSize',9, 'FontName','Consolas');
text(ax, xL(1)+0.4, zL(2)-0.5, '\oplus  Earth', ...
    'Color',[0.45 0.75 1.0], 'FontSize',11, 'FontName','Consolas');

for bx = [-40, -18, -8, -1.8]
    line(ax, [bx bx], zL, 'Color',[0.32 0.32 0.52], 'LineStyle',':', ...
        'LineWidth',0.9, 'HandleVisibility','off');
end

text(ax, -26,  zL(1)+0.40, '\leftarrow  Ground Control  \rightarrow', ...
    'Color',[0.50 0.72 1.0], 'FontSize',9, 'FontName','Consolas', ...
    'HorizontalAlignment','center');
text(ax, -5.5, zL(1)+0.40, '\leftarrow Automated \rightarrow', ...
    'Color',[0.45 1.0 0.60], 'FontSize',9, 'FontName','Consolas', ...
    'HorizontalAlignment','center');
text(ax, -9.5, zL(1)+0.95, 'Close-range rendezvous', ...
    'Color',[0.85 0.85 0.85], 'FontSize',9, 'FontName','Consolas', ...
    'HorizontalAlignment','center');

phLX = [-46, -28, -12, -5.2, -1.0];
for k = 1:5
    text(ax, phLX(k), zL(2)-0.30, phNames{k}, ...
        'Color',phColors{k}, 'FontSize',8, 'FontName','Consolas', ...
        'HorizontalAlignment','center', 'FontWeight','bold');
end

wpts  = {[0;0],    P3/1e3,  S3(end,1:2)'/1e3, ...
         P1/1e3,   S1(end,1:2)'/1e3,  [x0;z0]/1e3};
wLbls = {'Dock','P_3','P_2','P_1','P_{mid}','P_0'};
for k = 1:numel(wpts)
    plot(ax, wpts{k}(1), wpts{k}(2), 'o', 'MarkerSize',7, ...
        'MarkerFaceColor',[1.0 0.90 0.20], 'MarkerEdgeColor','w', ...
        'LineWidth',1.4, 'HandleVisibility','off');
    text(ax, wpts{k}(1)+0.35, wpts{k}(2)-0.30, wLbls{k}, ...
        'Color',[1.0 0.90 0.20], 'FontSize',9, 'FontName','Consolas');
end

plot(ax, x_km, z_km, '-', 'Color',[0.20 0.20 0.36], 'LineWidth',0.9, ...
    'HandleVisibility','off');

%% Animated objects
trailH = gobjects(5,1);
trailX = cell(5,1);  trailZ = cell(5,1);
for k = 1:5
    trailH(k) = line(ax, NaN, NaN, 'Color',phColors{k}, ...
                     'LineWidth',2.6, 'DisplayName',phNames{k});
end
hChaser = plot(ax, x_km(1), z_km(1), 's', 'MarkerSize',11, ...
    'MarkerFaceColor',[0.20 0.95 0.55], 'MarkerEdgeColor','w', ...
    'LineWidth',1.6, 'HandleVisibility','off');
hTarget = plot(ax, 0, 0, 'p', 'MarkerSize',18, ...
    'MarkerFaceColor',[1.0 0.80 0.20], 'MarkerEdgeColor','w', ...
    'LineWidth',1.5, 'HandleVisibility','off');

hx = xL(2)-0.3;  hz = zL(2);
hPhase = text(ax, hx, hz+0.15, '', 'Color',[1 1 1], 'FontSize',11, ...
    'FontName','Consolas', 'FontWeight','bold', 'HorizontalAlignment','right');
hRange = text(ax, hx, hz+0.65, '', 'Color',[0.80 0.92 1.00], 'FontSize',10, ...
    'FontName','Consolas', 'HorizontalAlignment','right');
hVrel  = text(ax, hx, hz+1.12, '', 'Color',[0.70 0.70 0.85], 'FontSize',9,  ...
    'FontName','Consolas', 'HorizontalAlignment','right');
hTime  = text(ax, hx, hz+1.55, '', 'Color',[0.55 0.55 0.70], 'FontSize',9,  ...
    'FontName','Consolas', 'HorizontalAlignment','right');

legend(ax, 'Location','northwest', 'TextColor',[1 1 1], ...
    'Color',[0.07 0.07 0.15], 'EdgeColor',[0.32 0.32 0.52], 'FontSize',9);

%% ── Animation loop ────────────────────────────────────────────────────────
skip   = max(1, floor(N/750));
frameN = 0;
fprintf('Animating %d frames...\n', ceil(N/skip));

for i = 1:skip:N
    if ~ishandle(fig), break; end
    ph = phases(i);

    trailX{ph}(end+1) = x_km(i);
    trailZ{ph}(end+1) = z_km(i);
    set(trailH(ph), 'XData',trailX{ph}, 'YData',trailZ{ph});
    set(hChaser, 'XData',x_km(i), 'YData',z_km(i));

    set(hPhase, 'String',['Phase: ' phNames{ph}], 'Color',phColors{ph});
    set(hRange, 'String', sprintf('Range  : %8.3f km',  sqrt(x_km(i)^2+z_km(i)^2)));
    set(hVrel,  'String', sprintf('Rel. v : %8.3f m/s', sqrt(Sall(i,3)^2+Sall(i,4)^2)));
    set(hTime,  'String', sprintf('T+     : %8.1f min', t_cum(i)/60));

    frameN = frameN + 1;
    set(hTarget, 'MarkerSize', 15+5*abs(sin(frameN*0.25)));
    drawnow limitrate;
end

set(hChaser, 'MarkerFaceColor',[1.0 0.35 0.35], 'MarkerSize',14);
set(hPhase,  'String','DOCKED', 'Color',[0.20 1.00 0.45], 'FontSize',13);
set(hRange,  'String','Range  :    0.000 km');
drawnow;
fprintf('Docking complete.\n');

%% ── Static summary ────────────────────────────────────────────────────────
figure('Name','Rendezvous Summary', 'Color',[0.04 0.04 0.10], ...
       'Position',[120 80 1100 580]);
ax2 = axes('Color',[0.04 0.04 0.10], 'XColor',[0.75 0.75 0.80], ...
    'YColor',[0.75 0.75 0.80], 'GridColor',[0.18 0.18 0.28], 'GridAlpha',0.5, ...
    'FontName','Consolas', 'FontSize',10, 'Box','on', ...
    'XGrid','on', 'YGrid','on', 'YDir','reverse');
hold(ax2,'on');
for k = 1:5
    idx = (phases == k);
    plot(ax2, x_km(idx), z_km(idx), '-', 'Color',phColors{k}, ...
        'LineWidth',2.8, 'DisplayName',phNames{k});
end
for k = 1:numel(wpts)
    plot(ax2, wpts{k}(1), wpts{k}(2), 'o', 'MarkerSize',8, ...
        'MarkerFaceColor',[1 0.9 0.2], 'MarkerEdgeColor','w', ...
        'LineWidth',1.4, 'HandleVisibility','off');
    text(ax2, wpts{k}(1)+0.5, wpts{k}(2)-0.28, wLbls{k}, ...
        'Color',[1 0.9 0.2], 'FontSize',9, 'FontName','Consolas');
end
line(ax2, xlim(ax2), [0 0], 'Color',[0.7 0.7 0.7], 'LineStyle','--', ...
    'HandleVisibility','off');
xlabel(ax2,'V-bar (km)');  ylabel(ax2,'R-bar (km)');
title(ax2, 'Rendezvous Trajectory  --  LVLH / Hill Frame', ...
    'Color',[1 1 1], 'FontSize',12, 'FontWeight','bold');
legend(ax2, 'Location','northwest', 'TextColor',[1 1 1], ...
    'Color',[0.07 0.07 0.15], 'EdgeColor',[0.32 0.32 0.52]);

%% ════════════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS  (n is passed explicitly -- no workspace variable capture)
%% ════════════════════════════════════════════════════════════════════════════

function S = integ(n, r0, v0, T, npts, odeOpts)
%INTEG  Integrate CWH free-drift from [r0;v0] over time T.
    cwh_ode = @(~,s) [s(3);
                      s(4);
                      2*n*s(4) + 3*n^2*s(1);
                     -2*n*s(3)];
    [~, S] = ode45(cwh_ode, linspace(0,T,npts), [r0(:); v0(:)], odeOpts);
end

function v0 = dv_target(n, r0, rf, T)
%DV_TARGET  Impulsive delta-v to reach rf from r0 in time T (via CWH STM).
    Phi = cwh_stm(n, T);
    Prr = Phi(1:2, 1:2);
    Prv = Phi(1:2, 3:4);
    rc  = rcond(Prv);
    assert(rc > 1e-10, ...
        'Phi_rv is singular (rcond=%.2e). T=%.4f*T_orb is resonant.', ...
        rc, T*n/(2*pi));
    v0 = Prv \ (rf(:) - Prr*r0(:));
end

function Phi = cwh_stm(n, T)
%CWH_STM  4x4 CWH state-transition matrix for free drift over time T.
    c  = cos(n*T);
    s_ = sin(n*T);
    Phi = [4-3*c,        0,   s_/n,         2*(1-c)/n;
           6*(s_-n*T),   1,   2*(c-1)/n,    (4*s_-3*n*T)/n;
           3*n*s_,       0,   c,             2*s_;
           6*n*(c-1),    0,  -2*s_,          4*c-3        ];
end
