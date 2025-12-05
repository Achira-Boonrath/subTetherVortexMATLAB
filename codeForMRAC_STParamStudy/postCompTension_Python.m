function [Tx,Ty,Tz, appTorqueSave] = postCompTension_Python(chaserInfo, targetInfo, NodeX, NodeY, NodeZ, NodeVX, NodeVY, NodeVZ, chaserSideLength, targetSideLengthY, N_k, Kvec, l0vec, cVec, mu, FT)
%% Post compute MATLAB tension

    X1 = chaserInfo(:,1);    Y1 = chaserInfo(:,2);    Z1 = chaserInfo(:,3);
    VX1 = chaserInfo(:,4);    VY1 = chaserInfo(:,5);    VZ1 = chaserInfo(:,6);
    
    X2 = targetInfo(:,1);    Y2 = targetInfo(:,2);    Z2= targetInfo(:,3);
    VX2 = targetInfo(:,4);    VY2 = targetInfo(:,5);    VZ2= targetInfo(:,6);

    if N_k < 1
        X3 = NodeX;    Y3 = NodeY;    Z3= NodeZ;
        VX3 = NodeVX;    VY3 = NodeVY;    VZ3= NodeVZ;
    else
        X3 = NodeX(:,(N_k^2+1)/2);    Y3 = NodeY(:,(N_k^2+1)/2);    Z3= NodeZ(:,(N_k^2+1)/2);
        VX3 = NodeVX(:,(N_k^2+1)/2);    VY3 = NodeVY(:,(N_k^2+1)/2);    VZ3= NodeVZ(:,(N_k^2+1)/2);
    end
   
    % chaser
    quar1_B1 = chaserInfo(:,7);    quar1_B2 = chaserInfo(:,8);
    quar1_B3 = chaserInfo(:,9);    quar1_B4 = chaserInfo(:,10);
    omega1_BX = chaserInfo(:,11);    omega1_BY = chaserInfo(:,12);
    omega1_BZ= chaserInfo(:,13);

    % target
    quar2_B1 = targetInfo(:,7);    quar2_B2 = targetInfo(:,8);
    quar2_B3 = targetInfo(:,9);    quar2_B4 = targetInfo(:,10);
    omega2_BX = targetInfo(:,11);    omega2_BY = targetInfo(:,12);
    omega2_BZ= targetInfo(:,13);
    
    for i = 1:length(X1)
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
        l_mt_vec = posPoint1 - (posChaser + rotMat_C_A_I'*(distAttPt_to_C));
        l_mt = norm(l_mt_vec);
        evec_mt = l_mt_vec/l_mt;
        VR_mt = velPoint1 - (velChaser + rotMat_C_A_I'*(cross(omegaChaser,distAttPt_to_C)));
        
        %Compute Tension
        Tvec_mt = zeros(3,1);
        if (l_mt > l0vec(1))
            Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
            % Tmag_mt = Kvec(1)*(l_mt - l0vec(1))
        %         Tvec_mt = Tmag_mt*evec_mt;
            if (Tmag_mt > 0)
                Tvec_mt = Tmag_mt*evec_mt;
            end
        end

        %% Computing Torques
        distAttPt_to_D = ([0.0, 0.0,  0.0]');
        appTorqueTarget = ([0.0, 0.0,  0.0]');
        Tvec_st = zeros(3,4);
        for k = [1:4]
            if length(Kvec) == 5
                if k == 1
                    distAttPt_to_D = ([-0.5*targetSideLengthY, 0.0,  -5.1 ]');
                elseif k == 2
                    distAttPt_to_D = ([-0.5*targetSideLengthY,0.0,  5.1 ]');
                elseif k == 3
                    distAttPt_to_D = ([0.5*targetSideLengthY, 0.0, -5.1 ]');
                elseif k == 4
                    distAttPt_to_D = ([0.5*targetSideLengthY,0.0,  5.1 ]');
                end 
                    % For Tension in links
                    l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
                    l_st = norm(l_st_vec);
                    evec_st = l_st_vec/l_st;
                    VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
            elseif length(Kvec) == 3
                if k == 1
                    distAttPt_to_D = ([0.0, 0.0,  -5.1 ]');
                            % For Tension in links
                    l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
                    l_st = norm(l_st_vec);
                    evec_st = l_st_vec/l_st;
                    VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
                elseif k == 2
                    distAttPt_to_D = ([0.0, 0.0,  5.1 ]');
                                    % For Tension in links
                    l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
                    l_st = norm(l_st_vec);
                    evec_st = l_st_vec/l_st;
                    VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
                else
                    distAttPt_to_D = ([0.0, 0.0,  0.0 ]');
                                    l_st_vec = zeros(3,1);
                    l_st = norm(l_st_vec);
                    evec_st = zeros(3,1);
                    VR_st = zeros(3,1);
                end 
            else
                distAttPt_to_D = ([0.0, 0.0,  0.0 ]');
                        % For Tension in links
                    l_st_vec = zeros(3,1);
                    l_st = norm(l_st_vec);
                    evec_st = zeros(3,1);
                    VR_st = zeros(3,1);
            end 
            %Compute Tension
            if (l_st > l0vec(k+1))
                Tmag_st = Kvec(k+1)*(l_st - l0vec(k+1))+ cVec(k+1)* dot(VR_st,evec_st );
                if (Tmag_st > 0)
                    Tvec_st(:,k) = Tmag_st*evec_st;
                end
            end

            appTorqueTarget = appTorqueTarget + cross(distAttPt_to_D, rotMat_D_A_I*Tvec_st(:,k));
        end
        appTorqueSave(i,:) = appTorqueTarget;
    %% Save values
        Tx(i,:) = [Tvec_mt(1)];
        Ty(i,:) = [Tvec_mt(2)];
        Tz(i,:) = [Tvec_mt(3)];
        
    end
end