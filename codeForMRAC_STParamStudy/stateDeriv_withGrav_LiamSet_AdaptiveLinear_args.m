function [ds] = stateDeriv_withGrav_LiamSet_AdaptiveLinear_args(t,s, args)
%% Achira Boonrath, MAE 566 Project
    
    massPoint1 =args.massPoint1; targetM=args.targetM; targetI=args.targetI;
    chaserM =args.chaserM; chaserI=args.chaserI; chaserSideLength=args.chaserSideLength; 
    targetSideLengthX =args.targetSideLengthX; targetSideLengthY=args.targetSideLengthY;targetSideLengthZ=args.targetSideLengthZ; 
    Kvec =args.Kvec; l0vec=args.l0vec; cVec=args.cVec;
    mu=args.mu; FT =args.FT;

    %Extract States
    s(1:6)   = s(1:6  ) * args.ODEscale;
    s(14:19) = s(14:19) * args.ODEscale;
    s(27:32) = s(27:32) * args.ODEscale;

    posChaser = s(1:3);    velChaser = s(4:6);
    q_C = s(7:10);    omegaChaser = s(11:13);

    posTarget = s(14:16);    velTarget = s(17:19);
    q_D = s(20:23);    omegaTarget = s(24:26);

    posPoint1 = s(27:29);    velPoint1 = s(30:32);
    ds = zeros(1,39);
    Lc = chaserSideLength;

    %From Field 2022 Thesis
    quar_CDot = 0.5*[[-q_C(2) -q_C(3) -q_C(4)];[q_C(1) -q_C(4) q_C(3)];...
        [q_C(4) q_C(1) -q_C(2)];[-q_C(3) q_C(2) q_C(1)]]*omegaChaser;
   
    quar_TDot = 0.5*[[-q_D(2) -q_D(3) -q_D(4)];[q_D(1) -q_D(4) q_D(3)];...
        [q_D(4) q_D(1) -q_D(2)];[-q_D(3) q_D(2) q_D(1)]]*omegaTarget ;

    %% Rotation Matris

    %From Field 2022 Thesis
    rotMat_C_A_I=[ q_C(1)^2+q_C(2)^2-q_C(3)^2-q_C(4)^2  2*(q_C(2)*q_C(3)-q_C(1)*q_C(4))      2*(q_C(2)*q_C(4)+q_C(1)*q_C(3));
                      2*(q_C(2)*q_C(3)+q_C(1)*q_C(4))     q_C(1)^2-q_C(2)^2+q_C(3)^2-q_C(4)^2   2*(q_C(4)*q_C(3)-q_C(1)*q_C(2));
                      2*(q_C(2)*q_C(4)-q_C(3)*q_C(1))     2*(q_C(3)*q_C(4)+q_C(1)*q_C(2))      q_C(1)^2-q_C(2)^2-q_C(3)^2+q_C(4)^2]';

    rotMat_D_A_I=[ q_D(1)^2+q_D(2)^2-q_D(3)^2-q_D(4)^2  2*(q_D(2)*q_D(3)-q_D(1)*q_D(4))      2*(q_D(2)*q_D(4)+q_D(1)*q_D(3));
                      2*(q_D(2)*q_D(3)+q_D(1)*q_D(4))     q_D(1)^2-q_D(2)^2+q_D(3)^2-q_D(4)^2   2*(q_D(4)*q_D(3)-q_D(1)*q_D(2));
                      2*(q_D(2)*q_D(4)-q_D(3)*q_D(1))     2*(q_D(3)*q_D(4)+q_D(1)*q_D(2))      q_D(1)^2-q_D(2)^2-q_D(3)^2+q_D(4)^2]';

%% N2L Chaser
    appTorqueChaser = zeros(3,1) ;
    
    % For Tension in links
    distAttPt_to_C = 0.5*Lc*[0 0 -1]';
    l_mt_vec = posPoint1 - (posChaser + rotMat_C_A_I'*(distAttPt_to_C));
    l_mt = norm(l_mt_vec);
    evec_mt = l_mt_vec/l_mt;
    VR_mt = velPoint1 - (velChaser + rotMat_C_A_I'*(cross(omegaChaser,distAttPt_to_C)));

    %Compute Tension
    Tvec_mt = zeros(3,1); maskMTreal = 0;
    if (l_mt > l0vec(1))
        Tmag_mt = Kvec(1)*(l_mt - l0vec(1))+ cVec(1)* dot(VR_mt,evec_mt );
        if (Tmag_mt > 0)
            Tvec_mt = Tmag_mt*evec_mt;
            maskMTreal = 1;
        end
    end
    
    % slopeTime = 50.0;
    % if (t < (slopeTime))
    %     ThrustConstantFinal = FT * (t / (slopeTime));
    % else
    %     ThrustConstantFinal = FT;
    % end
    % 
    % Fthrust = -ThrustConstantFinal*(velChaser/norm(velChaser));

    %% Adaptive Control - Linear w/ dynamics inversion

    x1_m = s(33);
    x1dot_m = s(34);
    hhat = s(35);
    ahat_1 = s(36);
    ahat_2 = s(37);
    Theta_hat = s(38:39);       % take first two entries

    % ==========================
    % Initialize adaptive gains
    % ==========================
    Kx_hat    = [ahat_1; ahat_2];        % (2x1 vector)
    Kr_hat    = hhat;                    % scalar in MATLAB
    % Theta_hat = Theta_hat(:);            % ensure column vector (2x1)
    
    % ==========================
    % System states
    % ==========================
    x1 = (l_mt - l0vec(1));
    x1dot = dot(VR_mt,evec_mt );
    ds(33) = x1dot_m;

    % ==========================
    % Desired elongation scaling
    % ==========================
    x1_m0  = args.x1_m0 ;
    tOnChaser_fromt0  = args.tOnChaser_fromt0 ;
    slopeTime = args.slopeTime;
    desElong  = args.desElong ;
    ThrustSaturation = args.ThrustSaturation;
    Gamma_x =  args.Gamma_x;
    Gamma_r = args.Gamma_r;
    Gamma_theta = args.Gamma_theta;
    P  = args.P;
    B_linear = args.B_linear ;
    sigmaMRACLin = args.sigmaMRACLin ;

    % args.x1_m0 = 0.0; 
    % args.tOnChaser_fromt0 = 0.0; 
    % args.slopeTime = 60; 
    % args.desElong = 0.01;
    % args.ThrustSaturation = 850;
    % args.Gamma_x = [[5e+7 ,   0];       [  0 ,  5e+7]];
    % args.Gamma_r = 4.1e+10;
    % args.Gamma_theta = [[5e+7,   0];       [  0 ,  5e+4]];
    % args.P = [[465.0415   ,   5.2612624];       [  5.2612624,   6.547933 ]];
    % args.B_linear = [0, 1/chaserM]';
    % args.sigmaMRACLin = 0.000003;

    desElongScale = min(max((t - tOnChaser_fromt0)/slopeTime, 0.0), 1.0);
    desElonVary   = x1_m0 + desElongScale * ((desElong - x1_m0) / 1.0);
    
    % ==========================
    % Reference model dynamics
    % ==========================
    Kp = args.Kp; Kd = args.Kd;
    ds(34) = - max([0, (cVec(1)/chaserM)*x1dot_m + (Kvec(1)/chaserM)*max([0, x1_m]) ]) ...
        + min(max(Kp*(desElonVary-x1_m) + Kd*(-x1dot_m), -ThrustSaturation/chaserM), ThrustSaturation/chaserM); % linear ref model
    
    % ==========================
    % Error and regressor vectors
    % ==========================
    e      = [x1 - x1_m; x1dot - x1dot_m];    % (2x1)
    Phi    = [x1*(double(maskMTreal) - 1); x1dot*(double(maskMTreal) - 1)]; % (2x1)
    x_vec  = [x1; x1dot];                     % (2x1)
    
    % ==========================
    % Control input
    % ==========================
    delta = Kx_hat' *x_vec + Kr_hat*desElonVary - Theta_hat' *Phi;
    
    % ==========================
    % Adaptive laws
    % ==========================
    % gamma = 45; ThrustSaturation = 825;
    % Gamma_x = chaserM * 500000 * gamma * [[400 ,   0];       [  0 ,  1/Kvec(1)]];
    % Gamma_r = chaserM * 7000000 * gamma * 1;
    % Gamma_theta = chaserM * 500000 * gamma * [[10 ,   0];       [  0 ,  1/Kvec(1)]];
    % P = [[442.9909   ,   2.3888493];       [  2.388844 ,  20.87838  ]];
    % B_linear = [0, 1/chaserM]';
    
    % Param adaptation
    % Kx_hat_dot    = -(Gamma_x * x_vec * (e'  * P * B_linear))' ;
    % Kr_hat_dot    = -(Gamma_r * desElonVary * (e'  * P * B_linear))' ;
    % Theta_hat_dot = -(Gamma_theta * Phi * (e'  * P * B_linear))' ;

    Kx_hat_dot    = (-Gamma_x * x_vec * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_x *Kx_hat)' ;
    Kr_hat_dot    = (-Gamma_r * desElonVary * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_r* desElonVary*Kr_hat)' ;
    Theta_hat_dot = (-Gamma_theta * Phi * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_theta *Theta_hat)' ;
    
    % ==========================
    % Thrust along +cZ direction
    % ==========================
    clipped_delta = min(max(delta, -ThrustSaturation), ThrustSaturation);
    Fthrust = clipped_delta * ( rotMat_C_A_I' * [0.0,0.0,1.0]' );

    maskFT = (delta*delta < ThrustSaturation*ThrustSaturation );
    ds(35) = Kr_hat_dot*double(maskFT);
    ds(36:37) = Kx_hat_dot*double(maskFT);
    ds(38:39) = Theta_hat_dot*double(maskFT);

    %% PID
    
%     errorInt = s(33);
%     Kp = 6000; Kd = 2000; Ki = 1000; desElong = 0.1;
%     elongError = desElong - (l_mt - l0vec(1));
%     ds(33) = elongError;
% 
%     elongErrorDot = -dot(VR_mt,evec_mt);
%     F_PD = Kp*elongError + Kd*elongErrorDot + Ki*errorInt;
%     Fthrust = -F_PD*(velChaser/(norm(velChaser)));

    %% with gravity;
    J2 = 1.08263e-3;
    rEarth = 6378.0e3;
    posChaserNorm = norm(posChaser);
    coeffT = (3 / 2) * J2 * mu * (rEarth ^ 2 / posChaserNorm ^ 4);

    gravChaser = - mu*chaserM*posChaser/(posChaserNorm^3);

    J2_Chaser = args.J2on*chaserM * coeffT * [ ...
    (5 * (posChaser(3) / posChaserNorm)^2 - 1) * (posChaser(1) / posChaserNorm), ...
    (5 * (posChaser(3) / posChaserNorm)^2 - 1) * (posChaser(2) / posChaserNorm), ...
    (5 * (posChaser(3) / posChaserNorm)^2 - 3) * (posChaser(3) / posChaserNorm) ...
    ];

    accChaser = (Tvec_mt + Fthrust + gravChaser + J2_Chaser')/chaserM;

    %% Sliding Att Control

    % Vortex
%     k_Loc = -evec_mt;
    k_Loc = - (velChaser/norm(velChaser));
    h_vec = cross(posChaser,velChaser);
    i_Loc = cross(k_Loc,posChaser./norm(posChaser));
    j_Loc = cross(k_Loc,i_Loc);
    RotMat_LocCh = [i_Loc,j_Loc,k_Loc];

    K = 16;    epsilon = 0.001;
    G = eye(3);    sat = @(x, delta) min(max(x/delta, -1), 1);

    quat_desCh = rotm2quat(RotMat_LocCh)';
    
    Qmat_desCh = [[-quat_desCh(2) -quat_desCh(3) -quat_desCh(4)];[quat_desCh(1) -quat_desCh(4) quat_desCh(3)];...
        [quat_desCh(4) quat_desCh(1) -quat_desCh(2)];[-quat_desCh(3) quat_desCh(2) quat_desCh(1)]];

    Qmat_actCh = [[-q_C(2) -q_C(3) -q_C(4)];[q_C(1) -q_C(4) q_C(3)];...
        [q_C(4) q_C(1) -q_C(2)];[-q_C(3) q_C(2) q_C(1)]];

    q_eCh = Qmat_desCh'*q_C;
    q_esCh = q_C'*quat_desCh;

    slidingVec = (omegaChaser-omegaChaser*0) + K*sign(q_esCh)*q_eCh;

    tau_Att_Ch = cross(omegaChaser,chaserI*omegaChaser) + ...
        chaserI*(0.5*K*sign(q_esCh)*( Qmat_actCh'*Qmat_desCh*omegaChaser*0 - Qmat_desCh'*Qmat_actCh*omegaChaser) + omegaChaser*0 - G*sat(slidingVec,epsilon) );
    % with SMC
    appTorqueChaser = cross((distAttPt_to_C), rotMat_C_A_I*Tvec_mt) + tau_Att_Ch;
    %% MODIFY ST ATTACHMENT PT HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    appTorqueTarget = zeros(3,1) ;    Tvec_st = zeros(3,4);

    distAttPt_to_D = ([0.0, 0.0,  0.0]');
    for i = [1:4]
        if i == 1
            distAttPt_to_D = ([-0.5*targetSideLengthY, 0.0,  -5.1 ]');
        elseif i == 2
            distAttPt_to_D = ([-0.5*targetSideLengthY,0.0,  5.1 ]');
        elseif i == 3
            distAttPt_to_D = ([0.5*targetSideLengthY, 0.0, -5.1 ]');
        elseif i == 4
            distAttPt_to_D = ([0.5*targetSideLengthY,0.0,  5.1 ]');
        end 

        % For Tension in links
        l_st_vec = posPoint1 - posTarget - rotMat_D_A_I'*distAttPt_to_D;
        l_st = norm(l_st_vec);
        evec_st = l_st_vec/l_st;
        VR_st = velPoint1 - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
    
        %Compute Tension
        if (l_st > l0vec(i+1))
            Tmag_st = Kvec(i+1)*(l_st - l0vec(i+1))+ cVec(i+1)* dot(VR_st,evec_st );
            if (Tmag_st > 0)
                Tvec_st(:,i) = Tmag_st*evec_st;
            end
        end

        appTorqueTarget = appTorqueTarget + cross(distAttPt_to_D, rotMat_D_A_I*Tvec_st(:,i));
    end

    gravTarget = - mu*targetM*posTarget/(norm(posTarget)^3);
    accTarget = (sum(Tvec_st,2) + gravTarget )/targetM;
    
%% N2L Connection Point

    gravPoint1 = - mu*massPoint1*posPoint1/(norm(posPoint1)^3);
    accPoint1 = (-(sum(Tvec_st,2) + Tvec_mt) + gravPoint1)/massPoint1;
    
%%

    omegaDotChaser = (chaserI)\(appTorqueChaser - cross(omegaChaser,chaserI*omegaChaser ) );
    omegaDotTarget = (targetI)\(appTorqueTarget - cross(omegaTarget,targetI*omegaTarget ) );

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

    ds(1:6)   = ds(1:6  ) / args.ODEscale;
    ds(14:19) = ds(14:19) / args.ODEscale;
    ds(27:32) = ds(27:32) / args.ODEscale;

    ds= ds';

end