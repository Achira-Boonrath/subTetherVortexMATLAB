% Created Liam Field 10/4/22
% This file reads information from Vortex output files for the Net
% Detumbling vortex code. It does all the relevant plotting and
% postprocessing.
clear all
close all
clc
mu = 3.986*10^5;

%% Load sub-tether data

load("subTetherMRAC_v2.mat")

% load("idset2_TxtOut/opt_vars_idset2_3var_2pt_v2_1.mat")
Func = 1;

close all
Chaser = [X1,Y1,Z1,VX1,VY1,VZ1,omega1_I,0*omega1_I,quar1_B1,quar1_B2,quar1_B3,quar1_B4];
Target = [X2,Y2,Z2,VX2,VY2,VZ2,omega2_I,0*omega2_I,quar2_B1,quar2_B2,quar2_B3,quar2_B4];
% NetAtt = [X3,Y3,Z3,VX3,VY3,VZ3];

% Check if N_mt_nodes exists, if not default to 1 (legacy support)
if ~exist('N_mt_nodes', 'var')
    if exist('args','var') && isfield(args,'N_mt_nodes')
        N_mt_nodes = args.N_mt_nodes;
    else
        N_mt_nodes = 1;
    end
end
fprintf('Processing for %d Main Tether Nodes...\n', N_mt_nodes);

% Extract all nodes
% state_vec indices for nodes start at 27
% Format: [X, Y, Z, VX, VY, VZ] for each node
NodeX = zeros(length(tspan), N_mt_nodes);
NodeY = zeros(length(tspan), N_mt_nodes);
NodeZ = zeros(length(tspan), N_mt_nodes);
NodeVX = zeros(length(tspan), N_mt_nodes);
NodeVY = zeros(length(tspan), N_mt_nodes);
NodeVZ = zeros(length(tspan), N_mt_nodes);

idx_nodes_start = 27;
for i = 1:N_mt_nodes
    curr_idx = idx_nodes_start + (i-1)*6;
    NodeX(:,i)  = state_vec(:, curr_idx);
    NodeY(:,i)  = state_vec(:, curr_idx+1);
    NodeZ(:,i)  = state_vec(:, curr_idx+2);
    NodeVX(:,i) = state_vec(:, curr_idx+3);
    NodeVY(:,i) = state_vec(:, curr_idx+4);
    NodeVZ(:,i) = state_vec(:, curr_idx+5);
end

% Legacy support for "X3" access if needed elsewhere, assume last node (End of MT) allows connection
X3 = NodeX(:,end);
Y3 = NodeY(:,end);
Z3 = NodeZ(:,end);
VX3 = NodeVX(:,end);
VY3 = NodeVY(:,end);
VZ3 = NodeVZ(:,end);

% NodeX = X3;
% NodeY = Y3;
% NodeZ = Z3;
% NodeVX = VX3;
% NodeVY = VY3;
% NodeVZ = VZ3;
Time = tspan;
% state_vec

% 4 ST
[Rx,Ry,Rz,Vx, Vy, Vz] = postCompConnPt(state_vec,massPoint1, debrisM, debrisI,chaserM, chaserI, chaserSideLength, debrisSideLengthX, debrisSideLengthY, debrisSideLengthZ, Kvec, l0vec, cVec, muEarth, 0.0);

% 2 ST
% [Rx,Ry,Rz,Vx, Vy, Vz] = postCompConnPt_2pt(state_vec,massPoint1, debrisM, debrisI,chaserM, chaserI, chaserSideLength, debrisSideLengthX, debrisSideLengthY, debrisSideLengthZ, Kvec, l0vec, cVec, muEarth, 0.0);

%%
% Initialize indices and parameters
centerPartIndex = N_mt_nodes; % Net acts on the last node
val = 1;
numNode = N_mt_nodes;

% Extract initial positions and velocities for Chaser and Target
pos0 = [Chaser(val,1:3); Target(val,1:3)];
vel0 = [Chaser(val,4:6) Target(val,4:6)];

% Extract angular velocities for Chaser and Target
wc = Chaser(val,7:9);
wt = Target(val,7:9);

% Normalize quaternions for Chaser and Target
qc = Chaser(val,13:16)./norm(Chaser(val,13:16));
qt = Target(val,13:16)./norm(Target(val,13:16));

% Compute rotation matrices for Chaser and Target
% RotationMatrixChsTgt_Post;
targb_R_eci0Func=@(qt) [ qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2  2*(qt(2)*qt(3)-qt(1)*qt(4))      2*(qt(2)*qt(4)+qt(1)*qt(3));
    2*(qt(2)*qt(3)+qt(1)*qt(4))     qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2   2*(qt(4)*qt(3)-qt(1)*qt(2));
    2*(qt(2)*qt(4)-qt(3)*qt(1))     2*(qt(3)*qt(4)+qt(1)*qt(2))      qt(1)^2-qt(2)^2-qt(3)^2+qt(4)^2]';
targb_R_eci0 = targb_R_eci0Func(qt);

chsb_R_eci0Func=@(qc) [ qc(1)^2+qc(2)^2-qc(3)^2-qc(4)^2  2*(qc(2)*qc(3)-qc(1)*qc(4))      2*(qc(2)*qc(4)+qc(1)*qc(3));
    2*(qc(2)*qc(3)+qc(1)*qc(4))     qc(1)^2-qc(2)^2+qc(3)^2-qc(4)^2   2*(qc(4)*qc(3)-qc(1)*qc(2));
    2*(qc(2)*qc(4)-qc(3)*qc(1))     2*(qc(3)*qc(4)+qc(1)*qc(2))      qc(1)^2-qc(2)^2-qc(3)^2+qc(4)^2]';
chsb_R_eci0 = chsb_R_eci0Func(qc);
% Set Chaser side length and attachment point
ChaserSide = 1.5;
chsAtt = [0 0 -ChaserSide/2]';
RchsAtt = pos0(1,:)'+chsb_R_eci0'*chsAtt;

% Get net attachment point from node data
NetAtt = [NodeX(val,centerPartIndex) NodeY(val,centerPartIndex) NodeZ(val,centerPartIndex)]';

% Create Zenit body and apply rotation/translation
Zenit = ZenitBody(0);
Zenit = ZenitRotated(Zenit,targb_R_eci0');
Zenit = ZenitTranslated(Zenit,pos0(2,1:3)'-0.0*targb_R_eci0(:,3));

%% Calculate target angular velocity and segment length
wt = Target(:,7:9);
l_seg = 6.5;

% Loop through all Chaser states to compute target attitude and vectors
for ii = 1:length(Chaser())
    % Normalize quaternions for current timestep
    qc = Chaser(ii,13:16)./norm(Chaser(ii,13:16));
    qt = Target(ii,13:16)./norm(Target(ii,13:16));

    % Compute rotation matrices for Chaser and Target
    % RotationMatrixChsTgt_Post;
    targb_R_eci0 = targb_R_eci0Func(qt);
    chsb_R_eci0 = chsb_R_eci0Func(qc);
    NetAtt = [NodeX(ii,centerPartIndex) NodeY(ii,centerPartIndex) NodeZ(ii,centerPartIndex)]';

    % Transform target angular velocity to body frame
    TargAng(ii,1:3) =  targb_R_eci0*wt(ii,1:3)';

    % Compute vector from net attachment to target
    TargetVec = Target(ii,1:3)-NetAtt';
end

%% Video Plotting in LVLH Frame

% Loop through all Chaser states to compute LVLH frame coordinates
for ii = 1:length(Chaser)
    % Compute LVLH frame axes
    R = Target(ii,1:3)'/norm(Target(ii,1:3));
    W = cross(R,Target(ii,4:6)'/norm(Target(ii,4:6)));
    S = cross(W,R);

    LVLH_R_ECI = [R S W]';

    % Transform positions to LVLH frame
    TargLVLH(ii,1:3) =  LVLH_R_ECI*(Target(ii,1:3)'-Target(ii,1:3)');
    ChaserLVLH(ii,1:3) =  LVLH_R_ECI*(Chaser(ii,1:3)'-Target(ii,1:3)');
    for jj = 1:size(NodeX,2)
        NodeLVLH(jj,1:3) = LVLH_R_ECI*([NodeX(ii,jj) NodeY(ii,jj) NodeZ(ii,jj)]'-Target(ii,1:3)');
    end
    RchsAtt = ChaserLVLH(ii,1:3)'+LVLH_R_ECI*chsb_R_eci0'*chsAtt;

    % Transform connection points to LVLH frame
    for kk = 1:5
        RV(kk,1:3) = LVLH_R_ECI*([Rx(ii,kk) Ry(ii,kk) Rz(ii,kk)]'-Target(ii,1:3)');
    end

    % Normalize quaternions for current timestep
    qc = Chaser(ii,13:16)./norm(Chaser(ii,13:16));
    qt = Target(ii,13:16)./norm(Target(ii,13:16));

    % Compute rotation matrices for Chaser and Target in LVLH frame
    % RotationMatrixChsTgt_Post;
    targb_R_eci0 = targb_R_eci0Func(qt);
    chsb_R_eci0 = chsb_R_eci0Func(qc);
    Rchs(:,:,ii) = LVLH_R_ECI*chsb_R_eci0';
    Rtgt(:,:,ii) = LVLH_R_ECI*targb_R_eci0';

    % Generate Chaser patch object for plotting
    ChsPatch(ii).Chs = RB_ObjectsFunc(ChaserLVLH(ii,1:3)',Rchs(:,:,ii),ChaserSide);

    % Store node positions in LVLH frame
    NodeX_LVLH(ii,:) = NodeLVLH(:,1)';
    NodeY_LVLH(ii,:) = NodeLVLH(:,2)';
    NodeZ_LVLH(ii,:) = NodeLVLH(:,3)';

    % Store connection point positions in LVLH frame
    RXV(ii,:) = RV(:,1)';
    RYV(ii,:) = RV(:,2)';
    RZV(ii,:) = RV(:,3)';
end

% Prepare arrays for plotting node positions in LVLH frame
XV = [NodeX_LVLH];
YV = [NodeY_LVLH];
ZV = [NodeZ_LVLH];

% Create Zenit body for plotting
Zenit = ZenitBody(0);

%% video
close all


figure
vidObj = VideoWriter('VideoVortexPostCapture_LVLH_Correct_FORCE');

vidObj.FrameRate = 20;

open(vidObj);
for hh = 1:10:round(length(Time)/1)
    % for hh = 1375
    % for hh = 2751
    % for hh = [1375, 2751]
    % for hh = [901, 1801, 2751]
    % for hh = [688, 1375, 2063, 2751]
    scatter3(XV(hh,:),YV(hh,:),ZV(hh,:),6,'ko','MarkerFaceColor','k');
    %     scatter3(X(k,:),Y(k,:),Z(k,:),4,'ro','MarkerFaceColor','r');
    %
    hold on

    Zenit = ZenitBody(0);
    Zenit = ZenitRotated(Zenit,Rtgt(:,:,hh));
    Zenit = ZenitTranslated(Zenit,TargLVLH(hh,1:3)'-0.0*Rtgt(:,3,hh));
    ZenitPlot(Zenit,0)

    plot3([XV(hh,end),RXV(hh,2)],[YV(hh,end),RYV(hh,2)],[ZV(hh,end),RZV(hh,2)],'b-','LineWidth',1.0);
    plot3([XV(hh,end),RXV(hh,3)],[YV(hh,end),RYV(hh,3)],[ZV(hh,end),RZV(hh,3)],'b-','LineWidth',1.0);
    plot3([XV(hh,end),RXV(hh,4)],[YV(hh,end),RYV(hh,4)],[ZV(hh,end),RZV(hh,4)],'b-','LineWidth',1.0);
    plot3([XV(hh,end),RXV(hh,5)],[YV(hh,end),RYV(hh,5)],[ZV(hh,end),RZV(hh,5)],'b-','LineWidth',1.0);

    patch(ChsPatch(hh).Chs)


    % Draw Main Tether (Chaser -> Node 1 -> ... -> Node N)
    % Segment 1: Chaser Attach -> Node 1
    % We need Chaser Attachment point in LVLH.
    % RchsAtt is computed above but not stored for plotting directly?
    % Let's recompute for plot or use RchsAtt from memory if valid
    % RchsAtt is 3x1

    % Chaser Body Attach Point in LVLH:
    % RchsAtt = ChaserLVLH(ii,1:3)' + LVLH_R_ECI * chsb_R_eci0' * chsAtt;
    % Re-calculate for safely inside loop using 'hh' index
    % Recalc Att Point
    qc_hh = Chaser(hh,13:16)./norm(Chaser(hh,13:16));
    chsb_R_eci0_hh = chsb_R_eci0Func(qc_hh);
    % LVLH at hh
    R_hh = Target(hh,1:3)'/norm(Target(hh,1:3));
    W_hh = cross(R_hh,Target(hh,4:6)'/norm(Target(hh,4:6)));
    S_hh = cross(W_hh,R_hh);
    LVLH_R_ECI_hh = [R_hh S_hh W_hh]';

    RchsAtt_hh = ChaserLVLH(hh,1:3)' + LVLH_R_ECI_hh * chsb_R_eci0_hh' * chsAtt;

    % Plot Chaser to Node 1
    plot3([RchsAtt_hh(1), XV(hh,1)], [RchsAtt_hh(2), YV(hh,1)], [RchsAtt_hh(3), ZV(hh,1)], 'r-','LineWidth',1.5);

    % Plot Nodes 1 to N
    for kk = 1:numNode-1
        p1 = [XV(hh,kk), YV(hh,kk), ZV(hh,kk)];
        p2 = [XV(hh,kk+1), YV(hh,kk+1), ZV(hh,kk+1)];
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r-','LineWidth',1.5);
    end

    % Net Sub-Tethers connect to Last Node (Node N)
    % This is handled above by: plot3([XV(hh,1),RXV(hh,2)]...)
    % BUT centerPartIndex used for extracting NodeX(val, centerPartIndex) for NetAtt.
    % XV contains NodeX_LVLH.
    % We need to make sure the sub-tethers attach to the LAST node column of XV.
    % XV is [Time x N_nodes]. So XV(hh, end) is the last node.

    % Re-draw sub-tethers extending from the LAST node
    % Overwrite the previous hardcoded XV(hh,1) plots with XV(hh,end)
    % Note: I am replacing lines 184-187 logic here or below?
    % The user asked to modify...

    % Since I cannot easily delete lines 184-187 from here without a huge block, I will assume the previous tool call
    % logic for those lines was kept or I should have included them in the chunk.
    % Wait, the chunk I am replacing is 192-196.
    % The lines 184-187 use XV(hh,1). If N>1, this is wrong. It should be XV(hh,end).
    % I will do another replacement for those lines 184-187 to fix them.

    %     scatter3(TetherMatVec(2:end-1,1,hh),TetherMatVec(2:end-1,2,hh),TetherMatVec(2:end-1,3,hh),'b')
    %     for kk = 1:numNode+1
    %     plot3(TetherMatVec(kk:kk+1,1,hh),TetherMatVec(kk:kk+1,2,hh),TetherMatVec(kk:kk+1,3,hh),'r')
    %     end
    if hh > 1
        %         plot3(ForceVec(:,1,hh-1),ForceVec(:,2,hh-1),ForceVec(:,3,hh-1),'k')
    end
    grid on

    hold off

    %axis([ min(TargetAttachPoint(:,1))-10 max(TargetAttachPoint(:,1))+10  -10 max(TargetAttachPoint(:,2))+10  min(TargetAttachPoint(:,3))-10 max(TargetAttachPoint(:,3))+10])
    Xax = [TargLVLH(hh,1) - 14 TargLVLH(hh,1) + 14];
    %             Yax = [min(ChaserLVLH(hh,2))-5 TargLVLH(hh,2) + 10];
    Yax = [TargLVLH(hh,1)- 45 TargLVLH(hh,1) + 14];
    %             Yax = [TargLVLH(hh,1)- 20 TargLVLH(hh,1) + 8];
    Zax = [TargLVLH(hh,3) - 11 TargLVLH(hh,3) + 11];
    axis([Xax Yax Zax])
    %viewVec = cross(Rtgt(:,3,hh)');
    %view(viewVec);
    % str1 = ['t = ' num2str(floor(Time(hh))) 's'];
    str1 = ['sub_F' num2str(Func) '_' num2str(floor(Time(hh))) 's'];

    % title(str1)
    xlabel('Radial, m','fontsize',18,'interpreter','latex')
    ylabel('Along-Track, m','fontsize',18,'interpreter','latex')
    zlabel('Cross-Track, m','fontsize',18,'interpreter','latex')
    daspect([1 1 1])
    currframe = getframe(gcf);
    % print(gcf,str1,'-depsc')

    writeVideo(vidObj,currframe);

end


close(vidObj);
