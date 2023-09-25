clear all
close all


%str{1} = 'NetCompare/ThrustChange/'; % Incorrect
%str{1} = 'NetCompare/ThrustChange2/'; % Correct Open Loop Thrust
%str{1} = 'NetCompare/ThrustChange3/'; % OL Tether Direction
%str{1} = 'NetCompare/';
%str{1} = 'NetCompare/PD_Chaservel/';
%str{1} = 'NetCompare/iniCondChange3/'; % Quadrant 4 Tether Direction
%str{1} = 'NetCompare/iniCondChange4/'; % Tether Along Radial Direction
%str{1} = 'NetCompare/InitialConditionsCheck/'; % Tether Along Radial Direction
str{1} = 'NetCompare/ThrustChangeResults/Rbar2/';
str{1} = 'NetCompare/ThrustChangeResults/Rbar_LongSim/';
%str{1} = 'NetCompare/ThrustChangeResults/Vbar_Opposite/';
str{1} = 'NetCompare/ThrustChangeResults/Vbar_Pos/';
str{1} = 'NetCompare/ThrustChangeResults/Vbar_Correct/';
str{1} = 'NetCompare/ThrustChangeResults/Vbar_MatlabIC/';


HH = 1;
Chaser = readmatrix([str{1},'ChaserDynamics.txt']); % <- with 100N -vc force
Target = readmatrix([str{1},'TargetDynamics.txt']); % and k = k/((100/k)/(max(l(above)-30)))
%NodeP = readmatrix([str{HH},'NodesPos.txt']);
%NodeV = readmatrix([str{HH},'NodesVel.txt']);
Tension = readmatrix([str{1},'Tension.txt']);
outPD = readmatrix([str{1},'outPD.txt']);
t = readmatrix([str{1},'IntegrationTime.txt']);
t = t(:,1);
Style = {'k-','b--','r-.'};
outPD_Thrust = outPD(:,1:3);

massC = 1600;
massT = 3000;

% For no initial conditions ouput in the nodes
pos0 = [Chaser(:,1:3) Target(:,1:3)];
vel0 = [Chaser(:,4:6) Target(:,4:6)];

wc = Chaser(:,7:9);
wt = Target(:,7:9);
%qe = SMC(:,7:10);


num_bod0 = size(pos0,2)/3;
if num_bod0 > 2
    numNode = num_bod0-2;
else
    numNode = 0;
end 
num_bod = num_bod0;
l_seg = 30/num_bod0;
num_seg = num_bod-1;

AttPoints = [.5 0 0; 0 -1.75/2 0];
chsAtt = AttPoints(1,:)';
targAtt = AttPoints(2,:)';

%t = (.1:(length(Chaser)-1))/10;
%t = 0:.1:(length(Chaser)-1)/10;
Time = t;
for ii = 1:length(pos0)
    
    qc = Chaser(ii,13:16)./norm(Chaser(ii,13:16));
    qt = Target(ii,13:16)./norm(Target(ii,13:16));
    
    RotationMatrixChsTgt_Post;
    
    TethAtt_C = chsb_R_eci0'*AttPoints(1,:)';
    TethAtt_T = targb_R_eci0'*AttPoints(2,:)';
    
    TC(ii,1) = norm(TethAtt_C);
    TT(ii,2) = norm(TethAtt_T);
    TargAng(ii,1:3) = targb_R_eci0*wt(ii,:)';
    ChsAng(ii,1:3) = chsb_R_eci0*wc(ii,:)';
    ElongByNodes;
    L(ii,1:3) = pos0(ii,4:6)'+TethAtt_T -pos0(ii,1:3)'-TethAtt_C;
    errorVar(ii) = .01+30-l_SegOUT(ii);
    errorVarDeriv(ii) = -(-vel0(ii,1:3)'-chsb_R_eci0'*cross(wc(ii,:)',AttPoints(1,:)')+vel0(ii,4:6)'+targb_R_eci0'*cross(wt(ii,:)',AttPoints(2,:)'))'*L(ii,1:3)'/norm(L(ii,1:3));
    
    U(ii,1:3) = -(errorVar(ii)*300+errorVarDeriv(ii)*2000)*L(ii,1:3)'/norm(L(ii,1:3));
    Align(ii) = AlignmentCalculator(AttPoints(1,:)',AttPoints(2,:)',pos0(ii,1:3),pos0(ii,4:6),qc,qt);
    
    PeakChaser = max(vecnorm(pos0(1:3),2,2));
    %ChaserPNormalized(ii) = 
% TensionCalcPost;
if ii > 1
    TensionInt(ii-1,1:3) = L(ii,1:3)*Tension(ii-1)/norm(L(ii,1:3));
    IntTest(ii-1) = dot(TensionInt(ii-1,:)',Chaser(ii,4:6)')/(norm(TensionInt(ii-1,:)')*norm(Chaser(ii,4:6)'));
    AngIntTest(ii-1) = acosd(dot(TensionInt(ii-1,:)',Chaser(ii,4:6)')/(norm(TensionInt(ii-1,:)')*norm(Chaser(ii,4:6)')));
end 
    %     l_p = pos0(ii,4:6)+TethAtt_T'-pos0(ii,1:3)-TethAtt_C';
%     l(ii) = norm(l_p);
% CoM(ii) = norm(pos0(ii,4:6)-pos0(ii,1:3));
end 

PeakChaser = max(vecnorm(pos0(:,4:6),2,2));
ChaserPNormalized = vecnorm(pos0(:,1:3),2,2)/PeakChaser;
TargetPNormalized = vecnorm(pos0(:,4:6),2,2)/PeakChaser;
figure
plot(Time,ChaserPNormalized,Time,TargetPNormalized)

figure
plot(Time,TargetPNormalized-ChaserPNormalized)

figure
plot(l_SegOUT)

figure
plot(Align)
%% Desired Images:
% - Center of mass positions for system and individual bodies
% - Orbital Velocity
% - Tensions


% Part centers of mass:
% Chaser
figure(1)
plot(t,vecnorm(pos0(:,1:3),2,2),Style{HH})
hold on
xlabel('Time (s)')
ylabel('Radius of Chaser (m)')
%legend('Individual')
set(gca,'Fontsize',12)

% Target
figure(2)
plot(t,vecnorm(pos0(:,end-2:end),2,2),Style{HH})
hold on
xlabel('Time (s)')
ylabel('Radius of Target (m)')
set(gca,'Fontsize',12)


figure(3)
plot(t,vecnorm(pos0(:,1:3)-pos0(:,end-2:end),2,2),Style{HH})
hold on
xlabel('Time (s)')
ylabel('CoM Difference (m)')
set(gca,'Fontsize',12)


% Nodes
figure(4)
plot(t(2:end),Tension,Style{HH})
xlabel('Time (s)')
ylabel('Tension (N)')
set(gca,'Fontsize',12)


figure(5)
plot(t(2:end),vecnorm(outPD_Thrust,2,2),Style{HH})
xlabel('Time (s)')
ylabel('Thrust (N)')
set(gca,'Fontsize',12)


% figure(6)
% plot(t(1:end),vecnorm(U,2,2),Style{HH})
% xlabel('Time (s)')
% ylabel('Thrust (N)')
% set(gca,'Fontsize',12)

if size(outPD,2) > 3
figure(7)
plot(t(2:end),errorVar(2:end)'-outPD(:,4),Style{HH})
xlabel('Time (s)')
ylabel('errorVar')
set(gca,'Fontsize',12)

figure(8)
plot(t(2:end),errorVarDeriv(2:end)'-outPD(:,5),Style{HH})
xlabel('Time (s)')
ylabel('errorVarDeriv')
set(gca,'Fontsize',12)



figure(8)
plot(errorVar(1:end)',Style{HH})
hold on
plot(outPD(:,4))
xlabel('Time (s)')
ylabel('errorVar')
set(gca,'Fontsize',12)

figure(8)
plot(errorVarDeriv(1:end)',Style{HH})
hold on
plot(outPD(:,5))
xlabel('Time (s)')
ylabel('errorVarDeriv')
set(gca,'Fontsize',12)
end

% Tensions
% figure(4)
% for ll = 1:num_seg
% subplot(num_seg,1,ll)
% plot(t,Tension(:,ll),Style{HH})
% hold on
% xlabel('Time (s)')
% ylabel(['Tensions (N)'])
% legend('Individual','Center of Mass','Modified-CoM')
% set(gca,'Fontsize',12)
% end 



% figure
% plot(Time(2:end),vecnorm(TensionInt,2,2))
% hold on
% plot(Time(2:end),Tension)

figure
plot(Time(2:end),IntTest)

figure
plot(Time(2:end),AngIntTest)


%trapz(Time(3:58352),IntTest(2:58351))

    Struct(HH).R = pos0;
    Struct(HH).V = vel0;

ChaserSides = [1 1 1]';
TargetSides = [1.25 1.75 1.25]';

for ii = 1:length(Chaser)
    
    %if ii == 1
     R = Target(ii,1:3)'/norm(Target(ii,1:3));
     W = cross(R,Target(ii,4:6)'/norm(Target(ii,4:6)));
     S = cross(W,R);
      LVLH_R_ECI = [R S W]';
    %end
     
    
    plot_LVLHX(ii,:) = R;
    plot_LVLHY(ii,:) = S;
    plot_LVLHZ(ii,:) = W;

    %LVLH_R_ECI = [R S W]';
    checkVec(ii) = norm(cross(plot_LVLHX(ii,:),plot_LVLHY(ii,:)));
   
        
        TargLVLH(ii,1:3) =  LVLH_R_ECI*(Target(ii,1:3)'-Target(ii,1:3)');
        ChaserLVLH(ii,1:3) =  LVLH_R_ECI*(Chaser(ii,1:3)'-Target(ii,1:3)');
        
        qc = Chaser(ii,13:16)./norm(Chaser(ii,13:16));
        qt = Target(ii,13:16)./norm(Target(ii,13:16));
        RotationMatrixChsTgt_Post;
        RchsAtt = ChaserLVLH(ii,1:3)'+LVLH_R_ECI*chsb_R_eci0'*chsAtt;
        RtargAtt = TargLVLH(ii,1:3)'+LVLH_R_ECI*targb_R_eci0'*targAtt;
        
        TetherMatVec(:,:,ii) = [RchsAtt' ; RtargAtt'];
        TethVec(ii,:) = RtargAtt-RchsAtt;
        TethVecECI(ii,:) = -LVLH_R_ECI'*TethVec(ii,:)'; 
    
        
        Rchs(:,:,ii) = LVLH_R_ECI*chsb_R_eci0';
        Rtgt(:,:,ii) = LVLH_R_ECI*targb_R_eci0';
%       ChsPatch(ii).Chs = RB_ObjectsFunc(ChaserLVLH(ii,1:3)',Rchs(:,:,ii),ChaserSides);
%       TargPatch(ii).T = RB_ObjectsFunc(TargLVLH(ii,1:3)',Rtgt(:,:,ii),TargetSides);
        
        OrbElemChs(ii,:) = eciPosVel2OrbitalElem(pos0(ii,1:3)'/1000,vel0(ii,1:3)'/1000,398600);
        OrbElemTarg(ii,:) = eciPosVel2OrbitalElem(pos0(ii,4:6)'/1000,vel0(ii,4:6)'/1000,398600);

        
    if ii < length(Chaser)
        PD_angle(ii) = acos(dot(outPD_Thrust(ii,:),TethVec(ii,:))./(norm(TethVec(ii,:))*norm(outPD_Thrust(ii,:))))*180/pi;
        outPD_Thrust_LVLH(ii,:) = LVLH_R_ECI*outPD_Thrust(ii,:)'/norm(outPD_Thrust(ii,:));
        U_LVLH(ii,:) = LVLH_R_ECI*U(ii+1,:)'/norm(U(ii+1,:));
    end 
    jj = ii;
    RB_Objects;


    % Center of Mass Calculations
    CenterOfMassPosition(ii,:) = (massC*Chaser(ii,1:3)+massT*Target(ii,1:3))/(massC+massT);
    CenterOfMassVelocity(ii,:) = (massC*Chaser(ii,4:6)+massT*Target(ii,4:6))/(massC+massT);
    
    % Calculating Tether Libration Angle
    

end 
    figure
    plot(Time,ChaserLVLH(:,1),Time,ChaserLVLH(:,2),Time,ChaserLVLH(:,3))
    xlabel('Time (s)')
    ylabel('Chaser Position (m)')
    legend('LVLH-X','LVLH-Y','LVLH-Z')
    print('ChaserLVLH.jpg','-djpeg')

     [~,apoapsisInd] = max(vecnorm(Chaser(:,1:3),2,2));
        [~,periapsisInd] = min(vecnorm(Chaser(:,1:3),2,2));
    figure
    plot(ChaserLVLH(:,1),ChaserLVLH(:,2))
    hold on
    scatter(ChaserLVLH(1,1),ChaserLVLH(1,2))
    scatter(ChaserLVLH(end,1),ChaserLVLH(end,2))
    %scatter(ChaserLVLH(periapsisInd,1),ChaserLVLH(periapsisInd,2))
    %scatter(ChaserLVLH(apoapsisInd,1),ChaserLVLH(apoapsisInd,2))
    tVecLVLH = TargLVLH(1,:)'+ Rtgt(:,:,1)*targAtt;
    plot([ChaserLVLH(1,1)  tVecLVLH(1)],[ChaserLVLH(1,2) tVecLVLH(2)])
    legend('traj','start','end','tether','Location','best')
    % legend('traj','start','end','apo','peri','tether','Location','best')
    xlabel('LVLH-X')
    ylabel('LVLH-Y')
    daspect([1 1 1])
    savefig([str{1},'LVLH_XY.fig'])
    print('LVLH_XY.jpg','-djpeg')


%     figure
%     plot3(pos0(:,1),pos0(:,2),pos0(:,3),Style{HH})
%     hold on
%     for ii = 1:51:length(Time)
%     plot3([pos0(ii,1);1000000*plot_LVLHX(ii,1)+pos0(ii,1)],[pos0(ii,2);1000000*plot_LVLHX(ii,2)+pos0(ii,2)],[pos0(ii,3);1000000*plot_LVLHX(ii,3)+pos0(ii,3)],'r')
%     plot3([pos0(ii,1);10000000*plot_LVLHZ(ii,1)+pos0(ii,1)],[pos0(ii,2);10000000*plot_LVLHZ(ii,2)+pos0(ii,2)],[pos0(ii,3);10000000*plot_LVLHZ(ii,3)+pos0(ii,3)],'b')
%     %plot(norm(cross(plot_LVLHX(ii,:),plot_LVLHY(ii,:))))
%     hold on
%     end 
%     xlabel('Time (s)')
%     ylabel('Radius of Target (m)')
%     set(gca,'Fontsize',12)

%     figure
%     plot(checkVec)

    figure
    plot(ChaserLVLH(:,1),ChaserLVLH(:,2))
    xlabel('LVLH-X')
    ylabel('LVLH-Y')
    hold on
    for ii = 2:10:length(Chaser)
        plot([ChaserLVLH(ii,1);3*outPD_Thrust_LVLH(ii-1,1)+ChaserLVLH(ii,1)],[ChaserLVLH(ii,2);3*outPD_Thrust_LVLH(ii-1,2)+ChaserLVLH(ii,2)],'r')
        plot([ChaserLVLH(ii,1);3*U_LVLH(ii-1,1)+ChaserLVLH(ii,1)],[ChaserLVLH(ii,2);3*U_LVLH(ii-1,2)+ChaserLVLH(ii,2)],'b')

    end 

    daspect([1 1 1])
    
    % Plotting Orbit and Keplerian Elements
    figure
    plot3(pos0(:,1),pos0(:,2),pos0(:,3),Style{HH})
    hold on 
    plot3(pos0(:,4),pos0(:,5),pos0(:,6))
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    figure
    plot3(pos0(:,1),pos0(:,2),pos0(:,3),Style{HH})
    hold on
    %    plot3(pos0(:,4),pos0(:,5),pos0(:,6))
    for ii = 1:1001:length(Time)
    plot3([pos0(ii,1);100000*TethVecECI(ii,1)+pos0(ii,1)],[pos0(ii,2);100000*TethVecECI(ii,2)+pos0(ii,2)],[pos0(ii,3);100000*TethVecECI(ii,3)+pos0(ii,3)],'r')
    %plot3([pos0(ii,1);10000000*plot_LVLHZ(ii,1)+pos0(ii,1)],[pos0(ii,2);10000000*plot_LVLHZ(ii,2)+pos0(ii,2)],[pos0(ii,3);10000000*plot_LVLHZ(ii,3)+pos0(ii,3)],'b')
    %plot(norm(cross(plot_LVLHX(ii,:),plot_LVLHY(ii,:))))
    hold on
    end 
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca,'Fontsize',12)



    figure
    plot3(pos0(:,4),pos0(:,5),pos0(:,6),Style{HH})
    hold on
    %    plot3(pos0(:,4),pos0(:,5),pos0(:,6))
    for ii = 1:1001:length(Time)
    plot3([pos0(ii,4);100000*TethVecECI(ii,1)+pos0(ii,4)],[pos0(ii,5);100000*TethVecECI(ii,2)+pos0(ii,5)],[pos0(ii,6);100000*TethVecECI(ii,3)+pos0(ii,6)],'r')
    %plot3([pos0(ii,1);10000000*plot_LVLHZ(ii,1)+pos0(ii,1)],[pos0(ii,2);10000000*plot_LVLHZ(ii,2)+pos0(ii,2)],[pos0(ii,3);10000000*plot_LVLHZ(ii,3)+pos0(ii,3)],'b')
    %plot(norm(cross(plot_LVLHX(ii,:),plot_LVLHY(ii,:))))
    pause(.5)
    end 
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca,'Fontsize',12)
    print('Orbit_withTether.jpg','-djpeg')


    
    %Orbital E
    OrbElemLabels = {'a (km)','e','i (rad)','Omega (rad)','Arg of Peri (rad)','theta (rad)'};

    for ll = [1:4 6]
        figure
        plot(Time,OrbElemChs(:,ll),'k')
        hold on
        plot(Time,OrbElemTarg(:,ll),'b--')
        legend('Chaser','Target','Location','best')
        ylabel(OrbElemLabels{ll})
        xlabel('Time (s)')
        print(['Vbar_Pos_',OrbElemLabels{ll},'.jpg'],'-djpeg')
        
    end 
    
    a_Aslanov = (vecnorm(outPD_Thrust,2,2).*(OrbElemChs(2:end,1).*(1-OrbElemChs(2:end,2).^2  )).^3 )./ ( 30*398600*1600*(1-OrbElemChs(2:end,2)).^4    );

    for ii = 1:1000:length(Chaser)
        
    end 
    
    
%     figure
%     vidObj = VideoWriter('Video_PD_Rbar_12000s');
% 
%     vidObj.FrameRate = 20;
% 
%     open(vidObj);
%     for hh = 1:1:length(Time)
% 
%           
% %     scatter3(XV(hh,:),YV(hh,:),ZV(hh,:),1,'ko','MarkerFaceColor','k');
% %     scatter3(X(k,:),Y(k,:),Z(k,:),4,'ro','MarkerFaceColor','r'); 
% %     
%     
%        
%   
%     
% % % 
% %     for i = 1:1:Nlinks
% %         plot3([XV(hh,Conn_table(i,1)),XV(hh,Conn_table(i,2))],[YV(hh,Conn_table(i,1)),YV(hh,Conn_table(i,2))],[ZV(hh,Conn_table(i,1)),ZV(hh,Conn_table(i,2))],'k-','LineWidth',0.1);
% %     end
%     
%     
%     
%     if numNode >0
%         scatter3(TetherMatVec(2:end-1,1,hh),TetherMatVec(2:end-1,2,hh),TetherMatVec(2:end-1,3,hh),'b')
%     hold on
%     end
%             
%      for kk = 1:numNode+1
%      plot3(-TetherMatVec(kk:kk+1,1,hh),-TetherMatVec(kk:kk+1,2,hh),-TetherMatVec(kk:kk+1,3,hh),'r')
%      hold on
%      end
%      
%          %plot3(ChaserLVLH(1:hh,1),ChaserLVLH(1:hh,2),ChaserLVLH(1:hh,3),'k')
% 
% 
% %     if hh > 1
% %         plot3(ForceVec(:,1,hh-1),ForceVec(:,2,hh-1),ForceVec(:,3,hh-1),'k')
% %     end
%         grid on
%         
%         hold off
%         
%         %axis([ min(TargetAttachPoint(:,1))-10 max(TargetAttachPoint(:,1))+10  -10 max(TargetAttachPoint(:,2))+10  min(TargetAttachPoint(:,3))-10 max(TargetAttachPoint(:,3))+10])
% %         Xax = [min(ChaserLVLH(:,1))-2 TargLVLH(hh,1) + 10];
% %         Yax = [min(ChaserLVLH(:,2))-2 max(ChaserLVLH(:,2))+2];
% %         Zax = [TargLVLH(hh,3) - 10 TargLVLH(hh,3) + 10];
%         Xax = [min(ChaserLVLH(:,1))-2 max(ChaserLVLH(:,1))+2];
%         Yax = [min(ChaserLVLH(:,2))-2 max(ChaserLVLH(:,2))+2];
%         Zax = [min(ChaserLVLH(:,3))-2 max(ChaserLVLH(:,3))+2];
% %         if norm(ChaserLVLH(hh,2)-TargLVLH(hh,2)) < 10
% %             Yax = [TargLVLH(hh,2)-15 TargLVLH(hh,2) +
% %         elseif Yax(1) > Yax(2)
% %             Yax = [min(ChaserLVLH(hh,2))-5 TargLVLH(hh,2) + 10];
% %         end
% %         if ChaserLVLH(hh,1) < -5
% %             Xax = [ChaserLVLH(hh,1)-5 TargLVLH(hh,1) + 10];
% %         end 
% %         if Yax(2) < Yax(1)
% %             Yax = [TargLVLH(hh,2) - 10 max(ChaserLVLH(hh,2))+5];
% %         end
%         patch(Chs(hh))
%         patch(Targ(hh))
%         
%         axis([Xax Yax Zax])
%         %viewVec = cross(Rtgt(:,3,hh)');
%         %view(viewVec);
%         str1 = ['t = ' num2str(floor(Time(hh))) 's'];
% 
%         title(str1)
%         xlabel('Radial (m)')
%         ylabel('Along-Track (m)')
%         zlabel('Cross-Track (m)')
%         daspect([1 1 1])
%         currframe = getframe(gcf);
% 
%         writeVideo(vidObj,currframe);
% 
%     end
% 
% 
% 
% 
%     close(vidObj);
    


%% Center of Mass LVLH
for ii = 1:length(Chaser)

    % Center of Mass Calculations
    CenterOfMassPosition(ii,:) = (massC*Chaser(ii,1:3)+massT*Target(ii,1:3))/(massC+massT);
    CenterOfMassVelocity(ii,:) = (massC*Chaser(ii,4:6)+massT*Target(ii,4:6))/(massC+massT);


    %if ii == 1
     R = CenterOfMassPosition(ii,:)'/norm(CenterOfMassPosition(ii,:));
     W = cross(R,CenterOfMassVelocity(ii,:)'/norm(CenterOfMassVelocity(ii,:)));
     S = cross(W,R);
      LVLH_R_ECI = [R S W]';
    %end
     
    
    plot_LVLHX(ii,:) = R;
    plot_LVLHY(ii,:) = S;
    plot_LVLHZ(ii,:) = W;

    %LVLH_R_ECI = [R S W]';
    checkVec(ii) = norm(cross(plot_LVLHX(ii,:),plot_LVLHY(ii,:)));
   
        
        TargLVLH(ii,1:3) =  LVLH_R_ECI*(Target(ii,1:3)'-CenterOfMassPosition(ii,:)');
        ChaserLVLH(ii,1:3) =  LVLH_R_ECI*(Chaser(ii,1:3)'-CenterOfMassPosition(ii,:)');
        
        qc = Chaser(ii,13:16)./norm(Chaser(ii,13:16));
        qt = Target(ii,13:16)./norm(Target(ii,13:16));
        RotationMatrixChsTgt_Post;
        RchsAtt = ChaserLVLH(ii,1:3)'+LVLH_R_ECI*chsb_R_eci0'*chsAtt;
        RtargAtt = TargLVLH(ii,1:3)'+LVLH_R_ECI*targb_R_eci0'*targAtt;
        
        TetherMatVec(:,:,ii) = [RchsAtt' ; RtargAtt'];
        TethVec(ii,:) = RtargAtt-RchsAtt;
        TethVecECI(ii,:) = -LVLH_R_ECI'*TethVec(ii,:)'; 
    
        
        Rchs(:,:,ii) = LVLH_R_ECI*chsb_R_eci0';
        Rtgt(:,:,ii) = LVLH_R_ECI*targb_R_eci0';
%       ChsPatch(ii).Chs = RB_ObjectsFunc(ChaserLVLH(ii,1:3)',Rchs(:,:,ii),ChaserSides);
%       TargPatch(ii).T = RB_ObjectsFunc(TargLVLH(ii,1:3)',Rtgt(:,:,ii),TargetSides);
        
        OrbElemCoM(ii,:) = eciPosVel2OrbitalElem(CenterOfMassPosition(ii,:)'/1000,CenterOfMassVelocity(ii,:)'/1000,398600);

        
    if ii < length(Chaser)
        PD_angle(ii) = acos(dot(outPD_Thrust(ii,:),TethVec(ii,:))./(norm(TethVec(ii,:))*norm(outPD_Thrust(ii,:))))*180/pi;
        outPD_Thrust_LVLH(ii,:) = LVLH_R_ECI*outPD_Thrust(ii,:)'/norm(outPD_Thrust(ii,:));
        U_LVLH(ii,:) = LVLH_R_ECI*U(ii+1,:)'/norm(U(ii+1,:));
    end 
    jj = ii;
    RB_ObjectsCoM;
    
    AngleXY(ii) = acosd(dot( TethVecECI(ii,:), R')/(norm(R)*norm(TethVecECI(ii,:))));
    AngleZ(ii) = acosd(dot( TethVec(ii,:), W')/(norm(W)*norm(TethVec(ii,:))));

    AngleXY_R(ii) = acosd(dot( RchsAtt', [0 1 0]')/(norm([0 1 0]')*norm(RchsAtt)));

    RchsAttOut(ii,:) = RchsAtt;
    % Calculating Time Derivative of AngleXY_R
    
    % Need Attachment Point Velocity w/r to CoM
    R_dot(ii,:) = LVLH_R_ECI*(Chaser(ii,4:6)'+chsb_R_eci0'*(cross(Chaser(ii,7:9)',chsAtt))-CenterOfMassVelocity(ii,:)');
    Argument1(ii) = dot( R_dot(ii,:)', [0 1 0]')/(norm([0 1 0]')*norm(RchsAtt));
    Argument2(ii) = -dot(R_dot(ii,:)',RchsAtt')/(norm([0 1 0]')*norm(RchsAtt)^3)*dot( RchsAtt', [0 1 0]');
    AngleXY_R_d(ii) = -sqrt(1-(dot( RchsAtt', [0 1 0]')/(norm([0 1 0]')*norm(RchsAtt)))^2)^-1 * (Argument1(ii) + Argument2(ii) );
    
end 
    
    LibAtan = atan2(RchsAttOut(:,2),RchsAttOut(:,1))*180/pi;
    
    figure
    plot(ChaserLVLH(:,1),ChaserLVLH(:,2))
    hold on
    scatter(ChaserLVLH(1,1),ChaserLVLH(1,2))
    scatter(ChaserLVLH(end,1),ChaserLVLH(end,2))
    %scatter(ChaserLVLH(periapsisInd,1),ChaserLVLH(periapsisInd,2))
    %scatter(ChaserLVLH(apoapsisInd,1),ChaserLVLH(apoapsisInd,2))
    tVecLVLH = TargLVLH(1,:)'+ Rtgt(:,:,1)*targAtt;
    plot([ChaserLVLH(1,1)  tVecLVLH(1)],[ChaserLVLH(1,2) tVecLVLH(2)])
    legend('traj','start','end','tether','Location','best')
    % legend('traj','start','end','apo','peri','tether','Location','best')
    xlabel('LVLH-X')
    ylabel('LVLH-Y')
    daspect([1 1 1])
    savefig([str{1},'LVLH_COM_XY.fig'])
    print('LVLH_COM_XY.jpg','-djpeg')

    figure
    plot(t,AngleXY,t,LibAtan)
    xlabel('Time (s)')
    ylabel('In-plane Libration Angle (deg)')
    print('LibAngle.jpg','-djpeg')
    
    figure
    plot(t,AngleXY_R_d)
    xlabel('Time (s)')
    ylabel('In-plane Libration Angle Time Derivative (deg/s)')
    
    
    for ll = [1:4 5 6]
        figure
        plot(Time,OrbElemCoM(:,ll),'k')
        ylabel(OrbElemLabels{ll})
        xlabel('Time (s)')
        print(['Vbar_Pos_COM_',OrbElemLabels{ll},'.jpg'],'-djpeg')
        
    end 
    
    
  VortexR.Orb = OrbElemCoM;
  VortexR.Lib = LibAtan;
    
    
    figure
    vidObj = VideoWriter('Video_PD_RbarCoM_12000s');

    vidObj.FrameRate = 20;

    open(vidObj);
    
    
    
    for hh = 1:1:length(Time)

          
%     scatter3(XV(hh,:),YV(hh,:),ZV(hh,:),1,'ko','MarkerFaceColor','k');
%     scatter3(X(k,:),Y(k,:),Z(k,:),4,'ro','MarkerFaceColor','r'); 
%     
    
       
  
    
% % 
%     for i = 1:1:Nlinks
%         plot3([XV(hh,Conn_table(i,1)),XV(hh,Conn_table(i,2))],[YV(hh,Conn_table(i,1)),YV(hh,Conn_table(i,2))],[ZV(hh,Conn_table(i,1)),ZV(hh,Conn_table(i,2))],'k-','LineWidth',0.1);
%     end
    
    
    
    if numNode >0
        scatter3(TetherMatVec(2:end-1,1,hh),TetherMatVec(2:end-1,2,hh),TetherMatVec(2:end-1,3,hh),'b')
    hold on
    end
            
     for kk = 1:numNode+1
     plot3(TetherMatVec(kk:kk+1,1,hh),TetherMatVec(kk:kk+1,2,hh),TetherMatVec(kk:kk+1,3,hh),'r')
     hold on
     end
     
     scatter3(CenterOfMassPosition(hh,1)-CenterOfMassPosition(hh,1),CenterOfMassPosition(hh,2)-CenterOfMassPosition(hh,2),CenterOfMassPosition(hh,3)-CenterOfMassPosition(hh,3),'k','filled')
         %plot3(ChaserLVLH(1:hh,1),ChaserLVLH(1:hh,2),ChaserLVLH(1:hh,3),'k')


%     if hh > 1
%         plot3(ForceVec(:,1,hh-1),ForceVec(:,2,hh-1),ForceVec(:,3,hh-1),'k')
%     end
        grid on
        
        hold off
        
        %axis([ min(TargetAttachPoint(:,1))-10 max(TargetAttachPoint(:,1))+10  -10 max(TargetAttachPoint(:,2))+10  min(TargetAttachPoint(:,3))-10 max(TargetAttachPoint(:,3))+10])
%         Xax = [min(ChaserLVLH(:,1))-2 TargLVLH(hh,1) + 10];
%         Yax = [min(ChaserLVLH(:,2))-2 max(ChaserLVLH(:,2))+2];
%         Zax = [TargLVLH(hh,3) - 10 TargLVLH(hh,3) + 10];
        Xax = [min(ChaserLVLH(:,1))-2 max(ChaserLVLH(:,1))+2];
        Yax = [min(ChaserLVLH(:,2))-2 max(ChaserLVLH(:,2))+2];
        Zax = [min(ChaserLVLH(:,3))-2 max(ChaserLVLH(:,3))+2];
%         if norm(ChaserLVLH(hh,2)-TargLVLH(hh,2)) < 10
%             Yax = [TargLVLH(hh,2)-15 TargLVLH(hh,2) +
%         elseif Yax(1) > Yax(2)
%             Yax = [min(ChaserLVLH(hh,2))-5 TargLVLH(hh,2) + 10];
%         end
%         if ChaserLVLH(hh,1) < -5
%             Xax = [ChaserLVLH(hh,1)-5 TargLVLH(hh,1) + 10];
%         end 
%         if Yax(2) < Yax(1)
%             Yax = [TargLVLH(hh,2) - 10 max(ChaserLVLH(hh,2))+5];
%         end
        patch(Chs(hh))
        patch(Targ(hh))
        
        axis([Xax Yax Zax])
        %axis([Xax Yax])
        %viewVec = cross(Rtgt(:,3,hh)');
        %view(viewVec);
        str1 = ['t = ' num2str(floor(Time(hh))) 's'];

        title(str1)
        xlabel('Radial (m)')
        ylabel('Along-Track (m)')
        zlabel('Cross-Track (m)')
        daspect([1 1 1])
        currframe = getframe(gcf);

        writeVideo(vidObj,currframe);

    end




    close(vidObj);





