function [Rx,Ry,Rz,Vx, Vy, Vz] = postCompConnPt(s_mod_code2,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec, mu, FT)
%% Post compute MATLAB tension

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

    for i = 1:length(X1)


        % connPos_mt = [];
        % connVel_mt = [];
        % connPos_st = [];
        % connVel_st = [];


        %Tension in links
        
        posChaser = [X1(i),Y1(i),Z1(i)]';
        velChaser = [VX1(i),VY1(i),VZ1(i)]';
        quar_C = [quar1_B1(i),quar1_B2(i),quar1_B3(i),quar1_B4(i)]';
        qc=quar_C;
        omegaChaser = [omega1_BX(i), omega1_BY(i), omega1_BZ(i)]';
    
        posTarget = [X2(i),Y2(i),Z2(i)]';
        velTarget = [VX2(i),VY2(i),VZ2(i)]';
        quar_T = [quar2_B1(i),quar2_B2(i),quar2_B3(i),quar2_B4(i)]';
        qt=quar_T;
        omegaTarget = [omega2_BX(i), omega2_BY(i), omega2_BZ(i)]';
    
        posPoint1 = [X3(i),Y3(i),Z3(i)]';
        velPoint1 = [VX3(i),VY3(i),VZ3(i)]';
    
        Lc = chaserSideLength;
    
        %% Rotation Matris
        %From Liam and SJ book
        rotMat_C_A_I=[ qc(1)^2+qc(2)^2-qc(3)^2-qc(4)^2  2*(qc(2)*qc(3)-qc(1)*qc(4))      2*(qc(2)*qc(4)+qc(1)*qc(3));
                          2*(qc(2)*qc(3)+qc(1)*qc(4))     qc(1)^2-qc(2)^2+qc(3)^2-qc(4)^2   2*(qc(4)*qc(3)-qc(1)*qc(2));
                          2*(qc(2)*qc(4)-qc(3)*qc(1))     2*(qc(3)*qc(4)+qc(1)*qc(2))      qc(1)^2-qc(2)^2-qc(3)^2+qc(4)^2]';
    
        rotMat_D_A_I=[ qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2  2*(qt(2)*qt(3)-qt(1)*qt(4))      2*(qt(2)*qt(4)+qt(1)*qt(3));
                          2*(qt(2)*qt(3)+qt(1)*qt(4))     qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2   2*(qt(4)*qt(3)-qt(1)*qt(2));
                          2*(qt(2)*qt(4)-qt(3)*qt(1))     2*(qt(3)*qt(4)+qt(1)*qt(2))      qt(1)^2-qt(2)^2-qt(3)^2+qt(4)^2]';
    
%% N2L Chaser
        
        % For Tension in links
        distAttPt_to_C = 0.5*Lc*[0 0 -1]';
        % distAttPt_to_C = 0.5*Lc*[0 0 0]';
        l_mt_vec = posPoint1 - (posChaser + rotMat_C_A_I'*(distAttPt_to_C));
        l_mt = norm(l_mt_vec);
        evec_mt = l_mt_vec/l_mt;
        VR_mt = velPoint1 - (velChaser + rotMat_C_A_I'*(cross(omegaChaser,distAttPt_to_C)));
        
        %Compute Tension
        Tvec_mt = zeros(3,1);
        if (l_mt > l0vec(1))
            Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
        %         Tvec_mt = Tmag_mt*evec_mt;
            if (Tmag_mt > 0)
                Tvec_mt = Tmag_mt*evec_mt;
            end
        end

        connPos_mt = (posChaser + rotMat_C_A_I'*(distAttPt_to_C));
        connVel_mt = (velChaser + rotMat_C_A_I'*(cross(omegaChaser,distAttPt_to_C)));

    %% N2L Target ST

        appTorqueTarget = zeros(3,1) ;
        Tvec_st = zeros(3,4);
    
        for j = [1:4]
        % Tension in links
            if j == 1
                distAttPt_to_D = ([-0.5*targetSideLengthY, 0.0,  -5.2 ]');
            elseif j == 2
                distAttPt_to_D = ([-0.5*targetSideLengthY,0.0,  5.2 ]');
            elseif j == 3
                distAttPt_to_D = ([0.5*targetSideLengthY, 0.0, -5.2 ]');
            elseif j == 4
                distAttPt_to_D = ([0.5*targetSideLengthY,0.0,  5.2 ]');
            end 
    
            % For Tension in links
            l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
            l_st = norm(l_st_vec);
            evec_st = l_st_vec/l_st;
            VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));

            connPos_st(:,j) = posTarget + rotMat_D_A_I'*distAttPt_to_D;
            connVel_st(:,j) = velTarget + rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
        
            %Compute Tension
            if (l_st > l0vec(j+1))
                Tmag_st = Kvec(j+1)*(l_st - l0vec(j+1))+ cVec(j+1)* dot(VR_st,evec_st );
    %             Tvec_st(:,i) = Tmag_st*evec_st;
                if (Tmag_st > 0)
                    Tvec_st(:,j) = Tmag_st*evec_st;
                end
            end
    
            appTorqueTarget = appTorqueTarget + cross(distAttPt_to_D, rotMat_D_A_I*Tvec_st(:,j));
        end

    %% Save values
        Rx(i,:) = [connPos_mt(1),connPos_st(1,1),connPos_st(1,2),connPos_st(1,3),connPos_st(1,4)];
        Ry(i,:) = [connPos_mt(2),connPos_st(2,1),connPos_st(2,2),connPos_st(2,3),connPos_st(2,4)];
        Rz(i,:) = [connPos_mt(3),connPos_st(3,1),connPos_st(3,2),connPos_st(3,3),connPos_st(3,4)];

        Vx(i,:) = [connVel_mt(1),connVel_st(1,1),connVel_st(1,2),connVel_st(1,3),connVel_st(1,4)];
        Vy(i,:) = [connVel_mt(2),connVel_st(2,1),connVel_st(2,2),connVel_st(2,3),connVel_st(2,4)];
        Vz(i,:) = [connVel_mt(3),connVel_st(3,1),connVel_st(3,2),connVel_st(3,3),connVel_st(3,4)];
        
    end
end