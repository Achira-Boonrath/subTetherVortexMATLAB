function [ds] = stateDeriv(t,s,massPoint1, targetM,targetI,chaserM, chaserI, chaserSideLength, targetSideLengthX, targetSideLengthY,targetSideLengthZ, Kvec, l0vec, cVec)

    posChaser = s(1:3);
    velChaser = s(4:6);
    quar_C = s(7:10);
    omegaChaser = s(11:13);

    posTarget = s(14:16);
    velTarget = s(17:19);
    quar_T = s(20:23);
    omegaTarget = s(24:26);

    posPoint1 = s(27:29);
    velPoint1 = s(30:32);

    Lc = chaserSideLength;

    %From Liam Thesis
%     quar_CDot = 0.5*[[-quar_C(2) -quar_C(3) -quar_C(4)];[quar_C(1) -quar_C(4) quar_C(3)];...
%         [quar_C(4) quar_C(1) -quar_C(2)];[-quar_C(3) quar_C(2) quar_C(1)]]*omegaChaser;
%    
%     quar_TDot = 0.5*[[-quar_T(2) -quar_T(3) -quar_T(4)];[quar_T(1) -quar_T(4) quar_T(3)];...
%         [quar_T(4) quar_T(1) -quar_T(2)];[-quar_T(3) quar_T(2) quar_T(1)]]*omegaTarget ;

    %From Stadnyk and Howell 2020,
    skewSymEpsilon_C =[0 -quar_C(3) quar_C(2) ; quar_C(3) 0 -quar_C(1) ; -quar_C(2) quar_C(1) 0 ];
    skewSymEpsilon_T =[0 -quar_T(3) quar_T(2) ; quar_T(3) 0 -quar_T(1) ; -quar_T(2) quar_T(1) 0 ];
    
    quar_CDot = 0.5*[(skewSymEpsilon_C+quar_C(4)*eye(3))*omegaChaser; -[quar_C(1); quar_C(2); quar_C(3)]'*omegaChaser];
   
    quar_TDot = 0.5*[(skewSymEpsilon_T+quar_T(4)*eye(3))*omegaTarget; -[quar_T(1); quar_T(2); quar_T(3)]'*omegaTarget];

    %% Rot Mat

    %From Stadnyk and Howell 2020,
    rotMat_C_A_I =[ 1-2*quar_C(2)^2-2*quar_C(3)^2,  2*(quar_C(1)*quar_C(2)+quar_C(3)*quar_C(4)),      2*(quar_C(1)*quar_C(3)-quar_C(4)*quar_C(2));
              2*(quar_C(1)*quar_C(2)-quar_C(4)*quar_C(3)),    1-2*quar_C(1)^2-2*quar_C(3)^2,   2*(quar_C(2)*quar_C(3)+quar_C(4)*quar_C(1));
              2*(quar_C(1)*quar_C(3)+quar_C(4)*quar_C(2)),    2*(quar_C(2)*quar_C(3)-quar_C(4)*quar_C(1)),     1-2*quar_C(1)^2-2*quar_C(2)^2];

    rotMat_D_A_I =[ 1-2*quar_T(2)^2-2*quar_T(3)^2,  2*(quar_T(1)*quar_T(2)+quar_T(3)*quar_T(4)),      2*(quar_T(1)*quar_T(3)-quar_T(4)*quar_T(2));
              2*(quar_T(1)*quar_T(2)-quar_T(4)*quar_T(3)),    1-2*quar_T(1)^2-2*quar_T(3)^2,   2*(quar_T(2)*quar_T(3)+quar_T(4)*quar_T(1));
              2*(quar_T(1)*quar_T(3)+quar_T(4)*quar_T(2)),    2*(quar_T(2)*quar_T(3)-quar_T(4)*quar_T(1)),     1-2*quar_T(1)^2-2*quar_T(2)^2];
%%
    appTorqueChaser = zeros(3,1) ;
    appTorqueTarget = zeros(3,1) ;
%% N2L Chaser

    % Tension in links
    l_mt_vec = posPoint1 - posChaser - rotMat_C_A_I'*(0.5*Lc*[1 0 0]');
    l_mt = norm(l_mt_vec);
    evec_mt = l_mt_vec/l_mt;
    VR_mt = velPoint1 - velChaser - rotMat_C_A_I'*(cross(omegaChaser,0.5*Lc*[1 0 0]'));

    Tvec_mt = zeros(3,1);
    if (l_mt > l0vec(1))
        Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
        if (Tmag_mt > 0)
            Tvec_mt = Tmag_mt*evec_mt;
        end
    end

    Fthrust = [-100 0 0]';

    accChaser = (Tvec_mt + Fthrust)/chaserM;

    appTorqueChaser = cross((0.5*Lc*[1 0 0]'), rotMat_C_A_I*Tvec_mt);
%% N2L Target

    Tvec_st = zeros(3,4);

    for i = [1:4]
    % Tension in links

        if i == 1
            distAttPt_to_D = (0.5*[-targetSideLengthX -targetSideLengthY -targetSideLengthZ ]');
        elseif i == 2
            distAttPt_to_D = (0.5*[-targetSideLengthX -targetSideLengthY targetSideLengthZ ]');
        elseif i == 3
            distAttPt_to_D = (0.5*[-targetSideLengthX targetSideLengthY -targetSideLengthZ ]');
        elseif i == 4
            distAttPt_to_D = (0.5*[-targetSideLengthX targetSideLengthY targetSideLengthZ ]');
        end 
        
        l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
        l_st = norm(l_st_vec);
        evec_st = l_st_vec/l_st;
        VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
    
        if (l_st > l0vec(i+1))
            Tmag_st = Kvec(i+1)*(l_st - l0vec(i+1))+ cVec(1)* dot(VR_st,evec_st );
            if (Tmag_st > 0)
                Tvec_st(:,i) = Tmag_st*evec_st;
            end
        end

        appTorqueTarget = appTorqueTarget + cross(distAttPt_to_D, rotMat_D_A_I*Tvec_st(:,i));
    end

    accTarget = (Tvec_st(:,1)+Tvec_st(:,2)+Tvec_st(:,3)+Tvec_st(:,4))/targetM;

%% N2L Connection Point

    s(30:32);
    accPoint1 = -(Tvec_st(:,1)+Tvec_st(:,2)+Tvec_st(:,3)+Tvec_st(:,4) + Tvec_mt)/massPoint1;
%%

    omegaDotChaser = inv(chaserI)*(appTorqueChaser - cross(omegaChaser,chaserI*omegaChaser ) );

    omegaDotTarget = inv(targetI)*(appTorqueTarget - cross(omegaTarget,targetI*omegaTarget ) );

    %chaser state derivatives
    ds(1:3) = s(4:6); %linear vel
    ds(4:6) = accChaser; %acc vel
    ds(7:10) = quar_CDot; %ang vel
    ds(11:13) = omegaDotChaser; %ang acc

    %target state derivatives
    ds(14:16) = s(17:19); %linear vel
    ds(17:19) = accTarget; %acc vel
    ds(20:23) = quar_TDot; %ang vel
    ds(24:26) = omegaDotTarget; %ang acc

    %connection state derivatives
    ds(27:29) = s(30:32); %linear vel
    ds(30:32) = accPoint1; %acc vel

    ds= ds';

end