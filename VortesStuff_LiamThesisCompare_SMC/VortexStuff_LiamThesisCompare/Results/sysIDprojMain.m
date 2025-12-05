close all
clear all
clc

%% tether prop
Et =  60e9;
ct =  48;

cVec = [ct];

At = pi*0.002^2;

l0_mt = 15.6875;
l0_1 = 17.25625;
l0_2 = 17.25625;
l0_3 = 17.25625;
l0_4 = 17.25625;

l0vec = [l0_mt,l0_1,l0_2,l0_3,l0_4];

% k_mt = Et*At/l0_mt;
% k_1 = Et*At/l0_1;
% k_2 = Et*At/l0_2;
% k_3 = Et*At/l0_3;
% k_4 = Et*At/l0_4;
% Kvec = [k_mt,k_1,k_2,k_3,k_4]

Kvec = [58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467];
%% chaser prop
chaserM = 500.003809;
chaserI = [[83.3 0 0];[0 83.3 0];[0 0 83.3]];

chaserSideLength = 1.000000;


pos0chaser = [0 0 0]'; 
vel0chaser = [0 0 0]'; 

q0chaser = [1 0 0 0]';
omega0chaser = [0 0 0]'; 


states0_chaser = [pos0chaser; vel0chaser; q0chaser; omega0chaser]; 

%% target prop
targetM = 3000.003809;

%original
% targetSideLengthX 	= 1.250000;
% targetSideLengthY 	= 1.750000;
% targetSideLengthZ 	= 1.250000;
% targetI = [[15000 0 0];[0 3000 0];[0 0 15000]];

targetSideLengthX 	= 1.750000;
targetSideLengthY 	= 1.250000;
targetSideLengthZ 	= 1.250000;
targetI = [[3000 0 0];[0 15000 0];[0 0 15000]];

% pos0target = [30+0.875,0 ,0]'; 
pos0target = [31.375,0 ,0]'; 
vel0target = [0 0 0]'; 

q0target = [1 0 0 0]';
omega0target = [0 0 0]'; 

states0_target = [pos0target; vel0target; q0target; omega0target]; 

%% connection point prop
pos0connPoint1 = [15.6875 0 0]'; 

vel0connPoint1 = [0 0 0]'; 

massPoint1 = 0.00761799980;

states0_connPoint1 = [pos0connPoint1;vel0connPoint1];
%% IC

s0 = [states0_chaser; states0_target;states0_connPoint1] ;

%% ODE45

toll = 1e-8;
options = odeset('RelTol',toll,'Stats','on');
t0 = 0;
t1 = 90.0;
dt = 0.01;
tspan = t0:dt:t1;

tStart = tic;
[t_mod_code2,s_mod_code2] = ode45(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);
t_ode45 = toc(tStart);
    
fprintf('It took %.4f s to integrate \n', t_ode45);

% [t_mod_code2,s_mod_code2] = ode15s(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);

%%
close all

X1 = s_mod_code2(:,1);
Y1 = s_mod_code2(:,2);
Z1 = s_mod_code2(:,3);

X2 = s_mod_code2(:,14);
Y2 = s_mod_code2(:,15);
Z2= s_mod_code2(:,16);
t_plot = t_mod_code2;

% figure
% plot(t_plot, X1, t_plot, Y1,t_plot, Z1)
% xlabel('Time, s'); 
% ylabel({'Chaser Position, m'})
% legend({"X","Y","Z"},"location","best");
% grid minor
% 
% figure
% plot(t_plot, X2, t_plot, Y2,t_plot, Z2)
% xlabel('Time, s'); 
% ylabel({'Target Position, m'})
% legend({"X","Y","Z"},"location","best");
% grid minor


%% Import data from Vortex 

path = ''

% X1 = importdata(fullfile(path, 'NodesX.txt'));
% Y1 = importdata(fullfile(path, 'NodesY.txt'));
% Z1 = importdata(fullfile(path, 'NodesZ.txt'));

tetherLength = importdata(fullfile(path, 'TetherDynamics.txt'));
chaserInfo = importdata(fullfile(path, 'ChaserDynamics.txt'));
targetInfo = importdata(fullfile(path, 'TargetDynamics.txt'));

t1 = [importdata(fullfile(path, 'IntegrationTime.txt'))];    

%% Plot
fig = figure

plot(t1,chaserInfo(1:1:end,1),'-',"LineWidth",2)
hold on
plot(t1,chaserInfo(1:1:end,2),'-',"LineWidth",2)
plot(t1,chaserInfo(1:1:end,3),'-',"LineWidth",2)
plot(t_plot, X1,'--', t_plot, Y1,'--',t_plot, Z1,'--')
hold off
xlabel("Time [s]")
ylabel("Chaser Position [m]")
% ylim([-3 3])
% xlim([25 t1(end)])
legend("x_{MATLAB}","y_{MATLAB}","z_{MATLAB}","x_{Vortex}","y_{Vortex}","z_{Vortex}")
grid minor
hold off
saveas(fig, 'ChaserPosition.png')


%%
fig = figure

plot(t1,targetInfo(1:1:end,1),'-',"LineWidth",2)
hold on
plot(t1,targetInfo(1:1:end,2),'-',"LineWidth",2)
plot(t1,targetInfo(1:1:end,3),'-',"LineWidth",2)
plot(t_plot, X2,'--', t_plot, Y2,'--',t_plot, Z2,'--')
hold off
xlabel("Time [s]")
ylabel("Target Position [m]")
% ylim([-3 3])
% xlim([25 t1(end)])
legend("x_{MATLAB}","y_{MATLAB}","z_{MATLAB}","x_{Vortex}","y_{Vortex}","z_{Vortex}")
grid minor
hold off
saveas(fig, 'ChaserPosition.png')