close all
clear all
clc

%% tether prop
Et =  60e9;
ct =  48;

cVec = [ct];

At = pi*0.002^2;

% l0_mt = 15.6875;
% l0_1 = 17.25625;
% l0_2 = 17.25625;
% l0_3 = 17.25625;
% l0_4 = 17.25625;

l0_mt = 15.6875;
l0_1 = 15.6875;
l0_2 = 15.6875;
l0_3 = 15.6875;
l0_4 = 15.6875;


%for 1 Tether
% l0_1 = 15.6875;

l0vec = [l0_mt,l0_1,l0_2,l0_3,l0_4];

% k_mt = Et*At/l0_mt;
% k_1 = Et*At/l0_1;
% k_2 = Et*At/l0_2;
% k_3 = Et*At/l0_3;
% k_4 = Et*At/l0_4;
% Kvec = [k_mt,k_1,k_2,k_3,k_4]

% Kvec = [58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467];
Kvec = [58643.062867009467,58643.062867009467/4.0,58643.062867009467/4.0,58643.062867009467/4.0,58643.062867009467/4.0];
% Kvec = [14660.765716752367,14660.765716752367/4.0,14660.765716752367/4.0,14660.765716752367/4.0,14660.765716752367/4.0];
% Kvec = [3665.19,3665.19/4.0,3665.19/4.0,3665.19/4.0,3665.19/4.0];
cVec = [58643.062867009467/10.0];

%for 1 Tether
% Kvec = [4712.3889799999997,4712.3889799999997];
%% chaser prop
chaserM = 500.003809;
chaserI = [[83.3 0 0];[0 83.3 0];[0 0 83.3]];

chaserSideLength = 1.000000;


pos0chaser = [0 0 0]'; 
vel0chaser = [0 0 0]'; 

q0chaser = [1 0 0 0]';
omega0chaser = [0 0 0.0]'; 


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
targetI = [[781.25 0 0];[0 1156.25 0];[0 0 1156.25]];

pos0target = [31.375,0 ,0]'; 
vel0target = [0 0 0]'; 

q0target = [1 0 0 0]';
omega0target = [0.0 0 0.1]'; 

states0_target = [pos0target; vel0target; q0target; omega0target]; 

%% connection point prop
pos0connPoint1 = [15.6875 0 0]'; 

vel0connPoint1 = [0 0 0]'; 

% massPoint1 = 0.00761799980;
massPoint1 = 1.0;

states0_connPoint1 = [pos0connPoint1;vel0connPoint1];
%% IC

s0 = [states0_chaser; states0_target;states0_connPoint1] ;

%% ODE45

toll = 1e-9;
options = odeset('RelTol',toll,'AbsTol',1e-12,'Stats','on');
t0 = 0;
t1 = 90.0;
dt = 0.001;
tspan = t0:dt:t1;

tStart = tic; 
[t_mod_code2,s_mod_code2] = ode45(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);
% [t_mod_code2,s_mod_code2] = ode23s(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);
% [t_mod_code2,s_mod_code2] = ode78(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);
% [t_mod_code2,s_mod_code2] = ode15s(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0,options);

% s_mod_code2 = ode4(@(t,s) stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec),tspan,s0);
% t_mod_code2 = tspan;
t_ode45 = toc(tStart);
    
fprintf('It took %.4f s to integrate \n', t_ode45);
%%
close all

X1 = s_mod_code2(:,1);
Y1 = s_mod_code2(:,2);
Z1 = s_mod_code2(:,3);
VX1 = s_mod_code2(:,4);
VY1 = s_mod_code2(:,5);
VZ1 = s_mod_code2(:,6);

X2 = s_mod_code2(:,14);
Y2 = s_mod_code2(:,15);
Z2= s_mod_code2(:,16);
VX2 = s_mod_code2(:,17);
VY2 = s_mod_code2(:,18);
VZ2= s_mod_code2(:,19);

X3 = s_mod_code2(:,27);
Y3 = s_mod_code2(:,28);
Z3= s_mod_code2(:,29);
VX3 = s_mod_code2(:,30);
VY3 = s_mod_code2(:,31);
VZ3= s_mod_code2(:,32);

t_plot = t_mod_code2;

% chaser
quar1_B1 = s_mod_code2(:,7);
quar1_B2 = s_mod_code2(:,8);
quar1_B3 = s_mod_code2(:,9);
quar1_B4 = s_mod_code2(:,10);
omega1_BX = s_mod_code2(:,11);
omega1_BY = s_mod_code2(:,12);
omega1_BZ= s_mod_code2(:,13);

% target
quar2_B1 = s_mod_code2(:,20);
quar2_B2 = s_mod_code2(:,21);
quar2_B3 = s_mod_code2(:,22);
quar2_B4 = s_mod_code2(:,23);
omega2_BX = s_mod_code2(:,24);
omega2_BY = s_mod_code2(:,25);
omega2_BZ= s_mod_code2(:,26);

omega2_I = zeros(length(t_plot), 3);
omega1_I = zeros(length(t_plot), 3);
for i = 1:length(t_plot)
    
    qc = [quar1_B1(i),quar1_B2(i),quar1_B3(i),quar1_B4(i)];
    qt = [quar2_B1(i),quar2_B2(i),quar2_B3(i),quar2_B4(i)];

    rotMat_C_A_I=[ qc(1)^2+qc(2)^2-qc(3)^2-qc(4)^2  2*(qc(2)*qc(3)-qc(1)*qc(4))      2*(qc(2)*qc(4)+qc(1)*qc(3));
                      2*(qc(2)*qc(3)+qc(1)*qc(4))     qc(1)^2-qc(2)^2+qc(3)^2-qc(4)^2   2*(qc(4)*qc(3)-qc(1)*qc(2));
                      2*(qc(2)*qc(4)-qc(3)*qc(1))     2*(qc(3)*qc(4)+qc(1)*qc(2))      qc(1)^2-qc(2)^2-qc(3)^2+qc(4)^2]';
                  
    rotMat_D_A_I=[ qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2  2*(qt(2)*qt(3)-qt(1)*qt(4))      2*(qt(2)*qt(4)+qt(1)*qt(3));
                      2*(qt(2)*qt(3)+qt(1)*qt(4))     qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2   2*(qt(4)*qt(3)-qt(1)*qt(2));
                      2*(qt(2)*qt(4)-qt(3)*qt(1))     2*(qt(3)*qt(4)+qt(1)*qt(2))      qt(1)^2-qt(2)^2-qt(3)^2+qt(4)^2]';
              
    omega2_I(i,:) = rotMat_D_A_I'*[omega2_BX(i);omega2_BY(i);omega2_BZ(i)];
     
    omega1_I(i,:) = rotMat_C_A_I'*[omega1_BX(i);omega1_BY(i);omega1_BZ(i)];
end



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

%% Post compute MATLAB tension

for i = 1:length(t_plot)
    % Tension in links
    
%     posChaser = [X1(i),Y1(i),Z1(i)]';
%     velChaser = [VX1(i),VY1(i),VZ1(i)]';
%     quar_C = [quar1_B1(i),quar1_B2(i),quar1_B3(i),quar1_B4(i)]';
%     qc=quar_C;
%     omegaChaser = [omega1_BX(i), omega1_BY(i), omega1_BZ(i)]';
% 
%     posTarget = [X2(i),Y2(i),Z2(i)]';
%     velTarget = [VX2(i),VY2(i),VZ2(i)]';
%     quar_T = [quar2_B1(i),quar2_B2(i),quar2_B3(i),quar2_B4(i)]';
%     qt=quar_T;
%     omegaTarget = [omega2_BX(i), omega2_BY(i), omega2_BZ(i)]';
% 
%     posPoint1 = [X3(i),Y3(i),Z3(i)]';
%     velPoint1 = [VX3(i),VY3(i),VZ3(i)]';
%     
%     Lc = chaserSideLength;
%     
%         
%     %From Liam and SJ book
%     rotMat_C_A_I=[ qc(1)^2+qc(2)^2-qc(3)^2-qc(4)^2  2*(qc(2)*qc(3)-qc(1)*qc(4))      2*(qc(2)*qc(4)+qc(1)*qc(3));
%                       2*(qc(2)*qc(3)+qc(1)*qc(4))     qc(1)^2-qc(2)^2+qc(3)^2-qc(4)^2   2*(qc(4)*qc(3)-qc(1)*qc(2));
%                       2*(qc(2)*qc(4)-qc(3)*qc(1))     2*(qc(3)*qc(4)+qc(1)*qc(2))      qc(1)^2-qc(2)^2-qc(3)^2+qc(4)^2]';
% 
%     rotMat_D_A_I=[ qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2  2*(qt(2)*qt(3)-qt(1)*qt(4))      2*(qt(2)*qt(4)+qt(1)*qt(3));
%                       2*(qt(2)*qt(3)+qt(1)*qt(4))     qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2   2*(qt(4)*qt(3)-qt(1)*qt(2));
%                       2*(qt(2)*qt(4)-qt(3)*qt(1))     2*(qt(3)*qt(4)+qt(1)*qt(2))      qt(1)^2-qt(2)^2-qt(3)^2+qt(4)^2]';
%     
%     l_mt_vec = posPoint1 - (posChaser + rotMat_C_A_I'*(0.5*Lc*[1 0 0]'));
%     l_mt = norm(l_mt_vec);
%     evec_mt = l_mt_vec/l_mt;
%     VR_mt = velPoint1 - (velChaser + rotMat_C_A_I'*(cross(omegaChaser,0.5*Lc*[1 0 0]')));
% 
%     Tvec_mt = zeros(3,1);
%     if (l_mt > l0vec(1))
%         Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
%         Tvec_mt = Tmag_mt*evec_mt;
% %         if (Tmag_mt > 0)
% %             Tvec_mt = Tmag_mt*evec_mt;
% %         end
%     end
%     
%     appTorqueTarget = zeros(3,1) ;
%     Tvec_st = zeros(3,4);
% 
%     for j = [1:4]
%     % Tension in links
% 
%         if j == 1
%             distAttPt_to_D = (0.5*[-targetSideLengthX, -targetSideLengthY, -targetSideLengthZ ]');
%         elseif j == 2
%             distAttPt_to_D = (0.5*[-targetSideLengthX, -targetSideLengthY, targetSideLengthZ ]');
%         elseif j == 3
%             distAttPt_to_D = (0.5*[-targetSideLengthX, targetSideLengthY, -targetSideLengthZ ]');
%         elseif j == 4
%             distAttPt_to_D = (0.5*[-targetSideLengthX, targetSideLengthY, targetSideLengthZ ]');
%         end 
%         
%         l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
%         l_st = norm(l_st_vec);
%         evec_st = l_st_vec/l_st;
%         VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
%     
%         if (l_st > l0vec(j+1))
%             Tmag_st = Kvec(j+1)*(l_st - l0vec(j+1))+ cVec(1)* dot(VR_st,evec_st );
%             Tvec_st(:,j) = Tmag_st*evec_st;
% %             if (Tmag_st > 0)
% %                 Tvec_st(:,i) = Tmag_st*evec_st;
% %             end
%         end
% 
%     end
%     
%     Tx(i,:) = [Tvec_mt(1),Tvec_st(1,1),Tvec_st(1,2),Tvec_st(1,3),Tvec_st(1,4)];
%     Ty(i,:) = [Tvec_mt(2),Tvec_st(2,1),Tvec_st(2,2),Tvec_st(2,3),Tvec_st(2,4)];
%     Tz(i,:) = [Tvec_mt(3),Tvec_st(3,1),Tvec_st(3,2),Tvec_st(3,3),Tvec_st(3,4)];
    
end

%% Import data from Vortex 

path = ''

% X1 = importdata(fullfile(path, 'NodesX.txt'));
% Y1 = importdata(fullfile(path, 'NodesY.txt'));
% Z1 = importdata(fullfile(path, 'NodesZ.txt'));

tetherLength = importdata(fullfile(path, 'TetherDynamics.txt'));
chaserInfo = importdata(fullfile(path, 'ChaserDynamics.txt'));
targetInfo = importdata(fullfile(path, 'TargetDynamics.txt'));
nodeInfo = importdata(fullfile(path, 'NodesPos.txt'));
TensionInfo = importdata(fullfile(path, 'Tension.txt'));

t1 = [importdata(fullfile(path, 'IntegrationTime.txt'))];    

%% Post compute Vortex tension
clc
for i = 1:length(t1)
    % Tension in links
    
%     posChaser = [chaserInfo(i,1),chaserInfo(i,2),chaserInfo(i,3)]';
%     velChaser = [chaserInfo(i,4),chaserInfo(i,5),chaserInfo(i,6)]';
%     quar_C = [chaserInfo(i,13),chaserInfo(i,14),chaserInfo(i,15),chaserInfo(i,16)]';
%     qc=quar_C;
%     omegaChaser = [chaserInfo(i,7),chaserInfo(i,8),chaserInfo(i,9)]';
% 
%     posTarget = [targetInfo(i,1),targetInfo(i,2),targetInfo(i,3)]';
%     velTarget = [targetInfo(i,4),targetInfo(i,5),targetInfo(i,6)]';
%     quar_T = [targetInfo(i,13),targetInfo(i,14),targetInfo(i,15),targetInfo(i,16)]';
%     qt=quar_T;
%     omegaTarget = [targetInfo(i,7),targetInfo(i,8),targetInfo(i,9)]';
% 
%     posPoint1 = [nodeInfo(i,1),nodeInfo(i,2),nodeInfo(i,3)]';
%     velPoint1 = [nodeInfo(i,1),nodeInfo(i,2),nodeInfo(i,3)]';
%     
%     Lc = chaserSideLength;
%     
%     %From Liam and SJ book
%     rotMat_C_A_I=[ qc(1)^2+qc(2)^2-qc(3)^2-qc(4)^2  2*(qc(2)*qc(3)-qc(1)*qc(4))      2*(qc(2)*qc(4)+qc(1)*qc(3));
%                       2*(qc(2)*qc(3)+qc(1)*qc(4))     qc(1)^2-qc(2)^2+qc(3)^2-qc(4)^2   2*(qc(4)*qc(3)-qc(1)*qc(2));
%                       2*(qc(2)*qc(4)-qc(3)*qc(1))     2*(qc(3)*qc(4)+qc(1)*qc(2))      qc(1)^2-qc(2)^2-qc(3)^2+qc(4)^2]';
% 
%     rotMat_D_A_I=[ qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2  2*(qt(2)*qt(3)-qt(1)*qt(4))      2*(qt(2)*qt(4)+qt(1)*qt(3));
%                       2*(qt(2)*qt(3)+qt(1)*qt(4))     qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2   2*(qt(4)*qt(3)-qt(1)*qt(2));
%                       2*(qt(2)*qt(4)-qt(3)*qt(1))     2*(qt(3)*qt(4)+qt(1)*qt(2))      qt(1)^2-qt(2)^2-qt(3)^2+qt(4)^2]';
%                   
%     R = [0, -1, 0;1, 0, 0;0, 0, 1];               
%     rotMat_D_A_I=rotMat_D_A_I*R';
%     
%     omegaChaser = rotMat_C_A_I'*omegaChaser;
%     l_mt_vec = posPoint1 - (posChaser + rotMat_C_A_I*(0.5*Lc*[1 0 0]'));
%     l_mt = norm(l_mt_vec);
%     evec_mt = l_mt_vec/l_mt;
%     VR_mt = velPoint1 - (velChaser + rotMat_C_A_I*(cross(omegaChaser,0.5*Lc*[1 0 0]')));
% 
%     Tvec_mt = zeros(3,1);
%     if (l_mt > l0vec(1))
%         Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
%         Tvec_mt = Tmag_mt*evec_mt;
% %         if (Tmag_mt > 0)
% %             Tvec_mt = Tmag_mt*evec_mt;
% %         end
%     end
%     
%     omegaTarget = rotMat_D_A_I'*omegaTarget;
%     appTorqueTarget = zeros(3,1) ;
%     Tvec_st = zeros(3,4);
% 
%     for j = [1:4]
%     % Tension in links
% 
%         if j == 1
%             distAttPt_to_D = (0.5*[-targetSideLengthX, -targetSideLengthY, -targetSideLengthZ ]');
%         elseif j == 2
%             distAttPt_to_D = (0.5*[-targetSideLengthX, -targetSideLengthY, targetSideLengthZ ]');
%         elseif j == 3
%             distAttPt_to_D = (0.5*[-targetSideLengthX, targetSideLengthY, -targetSideLengthZ ]');
%         elseif j == 4
%             distAttPt_to_D = (0.5*[-targetSideLengthX, targetSideLengthY, targetSideLengthZ ]');
%         end 
%         
%         l_st_vec = posPoint1 - posTarget - rotMat_D_A_I*distAttPt_to_D;
%         l_st = norm(l_st_vec);
%         evec_st = l_st_vec/l_st;
%         VR_st = velPoint1 - velTarget - rotMat_D_A_I*(cross(omegaTarget,distAttPt_to_D));
%     
%         if (l_st > l0vec(j+1))
%             Tmag_st = Kvec(j+1)*(l_st - l0vec(j+1))+ cVec(1)* dot(VR_st,evec_st );
%             Tvec_st(:,j) = Tmag_st*evec_st;
% %             if (Tmag_st > 0)
% %                 Tvec_st(:,i) = Tmag_st*evec_st;
% %             end
%         end
% 
%     end
%     
%     Tx_Vortex(i,:) = [Tvec_mt(1),Tvec_st(1,1),Tvec_st(1,2),Tvec_st(1,3),Tvec_st(1,4)];
%     Ty_Vortex(i,:) = [Tvec_mt(2),Tvec_st(2,1),Tvec_st(2,2),Tvec_st(2,3),Tvec_st(2,4)];
%     Tz_Vortex(i,:) = [Tvec_mt(3),Tvec_st(3,1),Tvec_st(3,2),Tvec_st(3,3),Tvec_st(3,4)];
    
end



%% Plot
fig = figure

plot(t1,nodeInfo(1:1:end,1),'-',"LineWidth",2)
hold on
plot(t1,nodeInfo(1:1:end,2),'-',"LineWidth",2)
plot(t1,nodeInfo(1:1:end,3),'-',"LineWidth",2)
plot(t_plot, X3,'--', t_plot, Y3,'--',t_plot, Z3,'--')
xlabel("Time [s]")
ylabel("Connection Point Position [m]")
% ylim([-3 3])
xlim([0 t_plot(end)])
legend("x_{Vortex}","y_{Vortex}","z_{Vortex}","x_{MATLAB}","y_{MATLAB}","z_{MATLAB}")
grid minor
hold off
saveas(fig, 'ConnectionPosition.png')

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
xlim([0 t_plot(end)])
legend("x_{Vortex}","y_{Vortex}","z_{Vortex}","x_{MATLAB}","y_{MATLAB}","z_{MATLAB}")
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
xlim([0 t_plot(end)])
xlim([0 t_plot(end)])
legend("x_{Vortex}","y_{Vortex}","z_{Vortex}","x_{MATLAB}","y_{MATLAB}","z_{MATLAB}")
grid minor
hold off
saveas(fig, 'targetPosition.png')

%% Plot
fig = figure

plot(t1,chaserInfo(1:1:end,7),'-',"LineWidth",2)
hold on
plot(t1,chaserInfo(1:1:end,8),'-',"LineWidth",2)
plot(t1,chaserInfo(1:1:end,9),'-',"LineWidth",2)
plot(t_plot, omega1_I(:,1),'--', t_plot, omega1_I(:,2),'--',t_plot, omega1_I(:,3),'--')
hold off
xlabel("Time [s]")
ylabel("Chaser Augular Velocity [rad/s]")
xlim([0 t_plot(end)])
legend("\omega_x_{Vortex}","\omega_y_{Vortex}","\omega_z_{Vortex}","\omega_x_{MATLAB}","\omega_y_{MATLAB}","\omega_z_{MATLAB}")
grid minor
hold off
saveas(fig, 'ChaserAngVel.png')

%% Plot
fig = figure

plot(t1,targetInfo(1:1:end,7),'-',"LineWidth",2)
hold on
plot(t1,targetInfo(1:1:end,8),'-',"LineWidth",2)
plot(t1,targetInfo(1:1:end,9),'-',"LineWidth",2)
plot(t_plot, omega2_I(:,1),'--', t_plot, omega2_I(:,2),'--',t_plot, omega2_I(:,3),'--')
hold off
xlabel("Time [s]")
ylabel("Target Augular Velocity [rad/s]")
xlim([0 t_plot(end)])
legend("\omega_x_{Vortex}","\omega_y_{Vortex}","\omega_z_{Vortex}","\omega_x_{MATLAB}","\omega_y_{MATLAB}","\omega_z_{MATLAB}")
grid minor
hold off
saveas(fig, 'targetAngVel.png')

%% RMSE for the chaser over time
RMSEpos_c = sqrt( sum( (chaserInfo(1:end,1) - X1).^2 + (chaserInfo(1:end,2) - Y1).^2 + (chaserInfo(1:end,3) - Z1).^2 ,2 ) / size(X1,2) ); %creates a vector of the RMSE for position at every time interval

n=length(t1);
RMSEpos_fig1 = figure('DefaultAxesFontSize',12);
plot(t1, RMSEpos_c,'k',"linewidth",2)
xlabel('Time, s')
ylabel('RMSE_t_o_t_a_l, m')
% xlim([0.7 1.7])
% ylim([0.0 0.05])
grid on
% %title('Capture Without Contact Among Nodes vs Capture With Contact Among Nodes')
saveas(RMSEpos_fig1, 'RMSEposC.png')
saveas(RMSEpos_fig1, 'RMSEposC.eps')

RMSEangvel_c = sqrt( sum( (chaserInfo(1:end,7) - omega1_I(:,1)).^2 + (chaserInfo(1:end,8) - omega1_I(:,2)).^2 + (chaserInfo(1:end,9) - omega1_I(:,3)).^2 ,2 ) / size(omega1_I,2) ); %creates a vector of the RMSE for position at every time interval

RMSEangvel_fig1 = figure('DefaultAxesFontSize',12);
plot(t1, RMSEangvel_c,'k',"linewidth",2)
xlabel('Time, s')
ylabel('RMSE_t_o_t_a_l, rad/s')
% xlim([0.7 1.7])
% ylim([0.0 0.05])
grid on
% %title('Capture Without Contact Among Nodes vs Capture With Contact Among Nodes')
saveas(RMSEangvel_fig1, 'RMSEangvelC.png')
saveas(RMSEangvel_fig1, 'RMSEangvelC.eps')

%% RMSE for the target over time
RMSEpos_t = sqrt( sum( (targetInfo(1:end,1) - X2).^2 + (targetInfo(1:end,2) - Y2).^2 + (targetInfo(1:end,3) - Z2).^2 ,2 ) / size(X2,2) ); %creates a vector of the RMSE for position at every time interval

n=length(t1);
RMSEpos_fig2 = figure('DefaultAxesFontSize',12);
plot(t1, RMSEpos_t,'k',"linewidth",2)
xlabel('Time, s')
ylabel('RMSE_t_o_t_a_l, m')
% xlim([0.7 1.7])
% ylim([0.0 0.05])
grid on
% %title('Capture Without Contact Among Nodes vs Capture With Contact Among Nodes')
saveas(RMSEpos_fig2, 'RMSEposT.png')
saveas(RMSEpos_fig2, 'RMSEposT.eps')

RMSEangvel_t = sqrt( sum( (targetInfo(1:end,7) - omega2_I(:,1)).^2 + (targetInfo(1:end,8) - omega2_I(:,2)).^2 + (targetInfo(1:end,9) - omega2_I(:,3)).^2 ,2 ) / size(omega2_I,2) ); %creates a vector of the RMSE for position at every time interval

RMSEangvel_fig2 = figure('DefaultAxesFontSize',12);
plot(t1, RMSEangvel_t,'k',"linewidth",2)
xlabel('Time, s')
ylabel('RMSE_t_o_t_a_l, rad/s')
% xlim([0.7 1.7])
% ylim([0.0 0.05])
grid on
% %title('Capture Without Contact Among Nodes vs Capture With Contact Among Nodes')
saveas(RMSEangvel_fig2, 'RMSEangvelT.png')
saveas(RMSEangvel_fig2, 'RMSEangvelT.eps')


%% Plot Tension X

% figure
% plot(t_plot, Tx(:,1), t_plot, Tx(:,2),t_plot, Tx(:,3),t_plot, Tx(:,4),t_plot, Tx(:,5))
% hold on
% plot(t_plot, Tx_Vortex(:,1), "--", t_plot, Tx_Vortex(:,2), "--",t_plot, Tx_Vortex(:,3), "--",t_plot, Tx_Vortex(:,4), "--",t_plot, Tx_Vortex(:,5), "--")
% hold off
% xlabel("Time [s]")
% ylabel("Tension in the X direction, N")
% xlim([0 t_plot(end)])
% % legend("T_{x_{Vortex}","\omega_y_{Vortex}","\omega_z_{Vortex}","\omega_x_{MATLAB}","\omega_y_{MATLAB}","\omega_z_{MATLAB}")
% grid minor
% saveas(fig, 'Tx.png')
% 
% %% Plot Tension Y
% 
% figure
% plot(t_plot, Ty(:,1), t_plot, Ty(:,2),t_plot, Ty(:,3),t_plot, Ty(:,4),t_plot, Ty(:,5))
% hold on
% plot(t_plot, Ty_Vortex(:,1), "--", t_plot, Ty_Vortex(:,2), "--",t_plot, Ty_Vortex(:,3), "--",t_plot, Ty_Vortex(:,4), "--",t_plot, Ty_Vortex(:,5), "--")
% hold off
% xlabel("Time [s]")
% ylabel("Tension in the Y direction, N")
% xlim([0 t_plot(end)])
% % legend("T_{x_{Vortex}","\omega_y_{Vortex}","\omega_z_{Vortex}","\omega_x_{MATLAB}","\omega_y_{MATLAB}","\omega_z_{MATLAB}")
% grid minor
% hold off
% saveas(fig, 'Ty.png')
% 
% %% Plot Tension Z
% 
% figure
% plot(t_plot, Tz(:,1), t_plot, Tz(:,2),t_plot, Tz(:,3),t_plot, Tz(:,4),t_plot, Tz(:,5))
% hold on
% plot(t_plot, Tz_Vortex(:,1), "--", t_plot, Tz_Vortex(:,2), "--",t_plot, Tz_Vortex(:,3), "--",t_plot, Tz_Vortex(:,4), "--",t_plot, Tz_Vortex(:,5), "--")
% hold off
% xlabel("Time [s]")
% ylabel("Tension in the Z direction, N")
% xlim([0 t_plot(end)])
% % legend("T_{x_{Vortex}","\omega_y_{Vortex}","\omega_z_{Vortex}","\omega_x_{MATLAB}","\omega_y_{MATLAB}","\omega_z_{MATLAB}")
% grid minor
% hold off
% saveas(fig, 'Tz.png')



