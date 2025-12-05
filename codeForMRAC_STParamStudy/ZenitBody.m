function Zenit = ZenitBody(plotNum)
%% Building Zenit
% Builds the Zenit spacecraft out of multiple cylinders and saves them in
% the .struct Zenit for manipulation later. The Zenit structure origin for positions are at the
% geometric center of the main body cylinder

% Inputs:
%   plotNum - plots the baseline zenit at the figure number required, 0 for
%   no plotting.



if plotNum > 0
figure(plotNum)
hold on
end 
% Positions of Bodies 1-6 in row order (1 = main, 2 = secondary, 3-6 =
% tertiary
% 
% CoM = [0 0 -.183]';
% R = [0 0 0; 0 0 -5.4; 0 1.5 -5.4; 0 -1.5 -5.4; 1.5 0 -5.4; -1.5 0  -5.4]-CoM';
% % R = [0 0 0; 0 0 -5.4; 0 1.5 -5.4; 0 -1.5 -5.4; 1.5 0 -5.4; -1.5 0  -5.4]+[ 0 0 -1];
% r_main = 1.95; h_main = 10.2;
% r_s = 1; h_s = 1;
% r_t = .3; h_t = 1;

CoM = [0 0 0.0]';
R = [0 0 0; 0 0 0; 0 1.5 -5.4; 0 -1.5 -5.4; 1.5 0 -5.4; -1.5 0  -5.4]-CoM';
% R = [0 0 0; 0 0 -5.4; 0 1.5 -5.4; 0 -1.5 -5.4; 1.5 0 -5.4; -1.5 0  -5.4]+[ 0 0 -1];
r_main = 1.95; h_main = 10.2;
r_s = 0; h_s = 0;
r_t = 0; h_t = 0;

%center of mass position

Zenit.body.CoM = CoM-CoM;
Zenit.body.rotationMatrix = eye(3);
%% Main Structure
 % [X,Y,Z] = cylinder2P(r_main,100,[0 0 h_main/2]+R(1,:),[0 0 -h_main/2]+R(1,:));
 [X,Y,Z] = cylinder2P(r_main,100,[0 0 h_main/2]+R(1,:),[0 0 -h_main/2]+R(1,:));
 % R is radius of cylider, difference between r1 and r2 is height

 if plotNum > 0
surf(X,Y,Z,'facecolor',[0.5 0.5 0.5],'LineStyle','none');
fill3(X(1,:),Y(1,:),Z(1,:),[0.5 0.5 0.5])
fill3(X(2,:),Y(2,:),Z(2,:),[0.5 0.5 0.5])   % draw cube
xlabel('X')
ylabel('Y')
zlabel('Z')
 end 
Zenit.body.cylinder.main.X = X;
Zenit.body.cylinder.main.Y = Y;
Zenit.body.cylinder.main.Z = Z;

Zenit.body.cylinder.main.basePosition = R(1,:);
Zenit.body.cylinder.main.radius = r_main;
Zenit.body.cylinder.main.height = h_main;
Zenit.body.cylinder.main.N = 100;
%% Secondary Cylinder

 [X,Y,Z] = cylinder2P(r_s,100,[0 0 h_s/2]+R(2,:),[0 0 -h_s/2]+R(2,:));
 % R is radius of cylider, difference between r1 and r2 is height

 if plotNum > 0
surf(X,Y,Z,'facecolor',[0.5 0.5 0.5],'LineStyle','none');
fill3(X(1,:),Y(1,:),Z(1,:),[0.5 0.5 0.5])
fill3(X(2,:),Y(2,:),Z(2,:),[0.5 0.5 0.5])   % draw cube
xlabel('X')
ylabel('Y')
zlabel('Z')
 end 

Zenit.body.cylinder.secondary.X = X;
Zenit.body.cylinder.secondary.Y = Y;
Zenit.body.cylinder.secondary.Z = Z;
Zenit.body.cylinder.secondary.basePosition = R(2,:);
Zenit.body.cylinder.secondary.radius = r_main;
Zenit.body.cylinder.secondary.height = h_main;

%% Tertiary Cylinders
for ii = 1:4
    [X,Y,Z] = cylinder2P(r_t,100,[0 0 h_t/2]+R(ii+2,:),[0 0 -h_s/2]+R(ii+2,:));
    % R is radius of cylider, difference between r1 and r2 is height

     if plotNum > 0
        surf(X,Y,Z,'facecolor',[0.5 0.5 0.5],'LineStyle','none');
        fill3(X(1,:),Y(1,:),Z(1,:),[0.5 0.5 0.5])
        fill3(X(2,:),Y(2,:),Z(2,:),[0.5 0.5 0.5])   % draw cube
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        %daspect([1 1 1])
    end 
    Zenit.body.cylinder.tertiary(ii).X = X;
    Zenit.body.cylinder.tertiary(ii).Y = Y;
    Zenit.body.cylinder.tertiary(ii).Z = Z;
    Zenit.body.cylinder.tertiary(ii).basePosition = R(ii,:);
    Zenit.body.cylinder.tertiary(ii).radius = r_t;
    Zenit.body.cylinder.tertiary(ii).height = h_t;
end 







