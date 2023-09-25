close all
clear all
clc


%% import data

% path = '12_X0_Y9_ZN50_25s_rFinal2_PID/'
path = ''

% X1 = importdata(fullfile(path, 'NodesX.txt'));
% Y1 = importdata(fullfile(path, 'NodesY.txt'));
% Z1 = importdata(fullfile(path, 'NodesZ.txt'));

tetherLength = importdata(fullfile(path, 'TetherDynamics.txt'));
chaserInfo = importdata(fullfile(path, 'ChaserDynamics.txt'));
targetInfo = importdata(fullfile(path, 'ChaserDynamics.txt'));

t1 = [importdata(fullfile(path, 'IntegrationTime.txt'))];    

%% Plot
fig = figure

plot(t1,chaserInfo(1:1:end,1),'-',"LineWidth",2)
hold on
plot(t1,chaserInfo(1:1:end,2),'-',"LineWidth",2)
plot(t1,chaserInfo(1:1:end,3),'-',"LineWidth",2)
hold off
xlabel("Time [s]")
ylabel("Chaser Position [m]")
% ylim([-3 3])
% xlim([25 t1(end)])
legend("x","y","z")
grid minor
hold off
saveas(fig, 'ChaserPosition.png')

%% Plot
fig = figure

plot(t1,chaserInfo(1:1:end,7),'-',"LineWidth",2)
hold on
plot(t1,chaserInfo(1:1:end,8),'-',"LineWidth",2)
plot(t1,chaserInfo(1:1:end,9),'-',"LineWidth",2)
hold off
xlabel("Time [s]")
ylabel("Chaser Augular Velocity [rad/s]")
% xlim([25 t1(end)])
legend("\omega_x","\omega_y","\omega_z")
grid minor
hold off
saveas(fig, 'ChaserAngVel.png')

%% Plot
fig = figure

plot(t1,targetInfo(1:1:end,1),'-',"LineWidth",2)
hold on
plot(t1,targetInfo(1:1:end,2),'-',"LineWidth",2)
plot(t1,targetInfo(1:1:end,3),'-',"LineWidth",2)
hold off
xlabel("Time [s]")
ylabel("Target Position [m]")
% ylim([-3 3])
% xlim([25 t1(end)])
legend("x","y","z")
grid minor
hold off
saveas(fig, 'ChaserPosition.png')

%% Plot
fig = figure

plot(t1,targetInfo(1:1:end,7),'-',"LineWidth",2)
hold on
plot(t1,targetInfo(1:1:end,8),'-',"LineWidth",2)
plot(t1,targetInfo(1:1:end,9),'-',"LineWidth",2)
hold off
xlabel("Time [s]")
ylabel("Target Augular Velocity [rad/s]")
% xlim([25 t1(end)])
legend("\omega_x","\omega_y","\omega_z")
grid minor
hold off
saveas(fig, 'ChaserAngVel.png')

%% Plot
% fig = figure
% 
% plot(t1,tetherLength(1:1:end,3),'-',"LineWidth",2)
% xlabel("Time [s]")
% ylabel("Main Tether Tension, N")
% xlim([25 t1(end)])
% grid minor
% hold off
% saveas(fig, 'Tether_Tension.png')
% 
% %% Plot
% fig = figure
% 
% plot(t1,tetherLength(1:1:end,2),'-',"LineWidth",2)
% xlabel("Time [s]")
% ylabel("Main Tether Elongation, m")
% xlim([25 t1(end)])
% % legend("\omega_x","\omega_y","\omega_z")
% grid minor
% hold off
% saveas(fig, 'Tether_Elongation.png')