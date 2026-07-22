close all
clear all
clc

%% tether properties

At = pi*0.002^2;

l0_mt = 15.6875;
l0_1 = 15.6875;
l0_2 = 15.6875;
l0_3 = 15.6875;
l0_4 = 15.6875;



l0vec = [l0_mt,l0_1,l0_2,l0_3,l0_4];

% At = pi*0.002^2;
% Et =  60e9;
% ct =  48;
% cVec = [ct];
% k_mt = Et*At/l0_mt;
% k_1 = Et*At/l0_1;
% k_2 = Et*At/l0_2;
% k_3 = Et*At/l0_3;
% k_4 = Et*At/l0_4;
% Kvec = [k_mt,k_1,k_2,k_3,k_4]

% Kvec = [58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467,58643.062867009467];
Kvec = [58643.062867009467,58643.062867009467/4.0,58643.062867009467/4.0,58643.062867009467/4.0,58643.062867009467/4.0];
cVec = [58643.062867009467/10.0];

%% for 1 Tether
% l0_1 = 15.6875;
% Kvec = [4712.3889799999997,4712.3889799999997];
%% chaser properties
chaserM = 500.003809;
chaserI = [[83.3 0 0];[0 83.3 0];[0 0 83.3]];

chaserSideLength = 1.000000;


pos0chaser = [0 0 0]'; 
vel0chaser = [0 0 0]'; 

q0chaser = [1 0 0 0]';
omega0chaser = [0 0 0.0]'; 


states0_chaser = [pos0chaser; vel0chaser; q0chaser; omega0chaser]; 

%% target properties
targetM = 3000.003809;

targetSideLengthX 	= 1.750000;
targetSideLengthY 	= 1.250000;
targetSideLengthZ 	= 1.250000;
targetI = [[781.25 0 0];[0 1156.25 0];[0 0 1156.25]];

pos0target = [31.375,0 ,0]'; 
vel0target = [0 0 0]'; 

q0target = [1 0 0 0]';
omega0target = [0.0 0 0.1]'; 

states0_target = [pos0target; vel0target; q0target; omega0target]; 

%% connection point properties
pos0connPoint1 = [15.6875 0 0]'; 

vel0connPoint1 = [0 0 0]'; 

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



figure
plot(t_plot, X1, t_plot, Y1,t_plot, Z1)
xlabel('Time, s'); 
ylabel({'Chaser Position, m'})
legend({"X","Y","Z"},"location","best");
grid minor

figure
plot(t_plot, X2, t_plot, Y2,t_plot, Z2)
xlabel('Time, s'); 
ylabel({'Target Position, m'})
legend({"X","Y","Z"},"location","best");
grid minor
