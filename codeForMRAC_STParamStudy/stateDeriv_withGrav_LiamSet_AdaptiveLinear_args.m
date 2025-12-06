function [ds] = stateDeriv_withGrav_LiamSet_AdaptiveLinear_args(t,s, args)
    %% Achira Boonrath, MAE 566 Project
    
    massPoint1 =args.massPoint1; targetM=args.targetM; targetI=args.targetI;
    chaserM =args.chaserM; chaserI=args.chaserI; chaserSideLength=args.chaserSideLength;
    targetSideLengthX =args.targetSideLengthX; targetSideLengthY=args.targetSideLengthY;targetSideLengthZ=args.targetSideLengthZ;
    Kvec =args.Kvec; l0vec=args.l0vec; cVec=args.cVec;
    mu=args.mu; FT =args.FT;
    N_mt_nodes = args.N_mt_nodes;
    
    %Extract States
    s(1:6)   = s(1:6  ) * args.ODEscale;
    s(14:19) = s(14:19) * args.ODEscale;
    s(27 : 26 + 6*N_mt_nodes) = s(27 : 26 + 6*N_mt_nodes) * args.ODEscale;
    
    posChaser = s(1:3);    velChaser = s(4:6);
    q_C = s(7:10);    omegaChaser = s(11:13);
    
    posTarget = s(14:16);    velTarget = s(17:19);
    q_D = s(20:23);    omegaTarget = s(24:26);
    
    % Extract Nodes
    nodes_pos = zeros(3, N_mt_nodes);
    nodes_vel = zeros(3, N_mt_nodes);
    idx_start = 27;
    for i = 1:N_mt_nodes
        idx = idx_start + (i-1)*6;
        nodes_pos(:,i) = s(idx : idx+2);
        nodes_vel(:,i) = s(idx+3 : idx+5);
    end
    
    % MRAC States indices
    mrac_idx = idx_start + 6*N_mt_nodes;
    
    % Total states = 26 + 6*N + 7
    ds = zeros(1, mrac_idx + 7 - 1);
    
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
    
    %% N2L Chaser & MT Nodes
    distAttPt_to_C = 0.5*Lc*[0 0 -1]';
    pos_Att = posChaser + rotMat_C_A_I'*(distAttPt_to_C);
    vel_Att = velChaser + rotMat_C_A_I'*(cross(omegaChaser,distAttPt_to_C));
    
    % Initialize Forces
    Force_nodes = zeros(3, N_mt_nodes);
    Force_Chaser_Tether = zeros(3,1);
    Tvec_mt_seg1 = zeros(3,1); % For logic use
    
    % Loop over N segments
    % Seg 1: Chaser -> Node 1
    % Seg k: Node k-1 -> Node k
    for k = 1:N_mt_nodes
        if k == 1
            p_prev = pos_Att;
            v_prev = vel_Att;
        else
            p_prev = nodes_pos(:, k-1);
            v_prev = nodes_vel(:, k-1);
        end
    
        p_curr = nodes_pos(:, k);
        v_curr = nodes_vel(:, k);
    
        l_vec = p_curr - p_prev;
        l_mag = norm(l_vec);
        e_vec = l_vec / l_mag;
        vr_vec = v_curr - v_prev;
    
        Tvec = zeros(3,1);
    
        % Tension Calculation
        if (l_mag > l0vec(1))
            Tmag = Kvec(1)*(l_mag - l0vec(1)) + cVec(1)*dot(vr_vec, e_vec);
            if Tmag > 0
                Tvec = Tmag * e_vec;
            end
        end
    
        % Apply Forces
        % Tvec pulls 'curr' towards 'prev' (-Tvec on curr, +Tvec on prev)
        Force_nodes(:, k) = Force_nodes(:, k) - Tvec;
    
        if k == 1
            Force_Chaser_Tether = Force_Chaser_Tether + Tvec;
            % Save Segment 1 info for Control Law
            l_mt_seg1 = l_mag;
            VR_mt_seg1 = vr_vec; % Node1 - Chaser
            evec_mt_seg1 = e_vec;
            Tvec_mt_seg1 = Tvec;
            maskMTreal = (l_mag > l0vec(1)) && (norm(Tvec) > 0);
        else
            Force_nodes(:, k-1) = Force_nodes(:, k-1) + Tvec;
        end
    end
    
    % Gravity on Nodes
    m_node = massPoint1 / N_mt_nodes; % Distribute mass
    for i = 1:N_mt_nodes
        pos = nodes_pos(:, i);
        grav = -mu * m_node * pos / (norm(pos)^3);
        Force_nodes(:, i) = Force_nodes(:, i) + grav;
    end
    
    %% Adaptive Control - Linear w/ dynamics inversion
    
    x1_m = s(mrac_idx);
    x1dot_m = s(mrac_idx+1);
    hhat = s(mrac_idx+2);
    ahat_1 = s(mrac_idx+3);
    ahat_2 = s(mrac_idx+4);
    Theta_hat = s(mrac_idx+5:mrac_idx+6);       % take first two entries
    
    % ==========================
    % Initialize adaptive gains
    % ==========================
    Kx_hat    = [ahat_1; ahat_2];        % (2x1 vector)
    Kr_hat    = hhat;                    % scalar in MATLAB
    % Theta_hat = Theta_hat(:);            % ensure column vector (2x1)
    
    % ==========================
    % System states (Segment 1)
    % ==========================
    x1 = (l_mt_seg1 - l0vec(1));
    x1dot = dot(VR_mt_seg1, evec_mt_seg1);
    ds(mrac_idx) = x1dot_m;
    
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
    
    desElongScale = min(max((t - tOnChaser_fromt0)/slopeTime, 0.0), 1.0);
    desElonVary   = x1_m0 + desElongScale * ((desElong - x1_m0) / 1.0);
    
    % ==========================
    % Reference model dynamics
    % ==========================
    Kp = args.Kp; Kd = args.Kd;
    ds(mrac_idx+1) = - max([0, (cVec(1)/chaserM)*x1dot_m + (Kvec(1)/chaserM)*max([0, x1_m]) ]) ...
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
    
    Kx_hat_dot    = (-Gamma_x * x_vec * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_x *Kx_hat)' ;
    Kr_hat_dot    = (-Gamma_r * desElonVary * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_r* desElonVary*Kr_hat)' ;
    Theta_hat_dot = (-Gamma_theta * Phi * (e'  * P * B_linear)  - sigmaMRACLin*abs(e' * P * B_linear)*Gamma_theta *Theta_hat)' ;
    
    % ==========================
    % Thrust along +cZ direction
    % ==========================
    clipped_delta = min(max(delta, -ThrustSaturation), ThrustSaturation);
    Fthrust = clipped_delta * ( rotMat_C_A_I' * [0.0,0.0,1.0]' );
    
    maskFT = 0;%((abs(delta)+ 1e-1)*(abs(delta)+ 1e-1) < ThrustSaturation*ThrustSaturation );
    ds(mrac_idx+2) = Kr_hat_dot*double(maskFT);
    ds(mrac_idx+3:mrac_idx+4) = Kx_hat_dot*double(maskFT);
    ds(mrac_idx+5:mrac_idx+6) = Theta_hat_dot*double(maskFT);
    
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
    
    accChaser = (Force_Chaser_Tether + Fthrust + gravChaser + J2_Chaser')/chaserM;
    
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
    % Tvec_mt_seg1 (from Chaser to Node 1) acts at distAttPt
    appTorqueChaser = cross((distAttPt_to_C), rotMat_C_A_I*Tvec_mt_seg1) + tau_Att_Ch;
    
    %% MODIFY ST ATTACHMENT PT HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Connects to Node N
    posNodeN = nodes_pos(:, N_mt_nodes);
    velNodeN = nodes_vel(:, N_mt_nodes);
    
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
        l_st_vec = posNodeN - posTarget - rotMat_D_A_I'*distAttPt_to_D; % Node N to Debris AttPt
        l_st = norm(l_st_vec);
        evec_st = l_st_vec/l_st;
        VR_st = velNodeN - velTarget - rotMat_D_A_I'*(cross(omegaTarget,distAttPt_to_D));
    
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
    
    %% N2L Connection Point (Node N)
    
    % Force on Node N needs ST forces.
    Force_nodes(:, N_mt_nodes) = Force_nodes(:, N_mt_nodes) - sum(Tvec_st, 2);
    
    % Calculate Accelerations for all Nodes
    nodes_acc = zeros(3, N_mt_nodes);
    for i = 1:N_mt_nodes
        nodes_acc(:, i) = Force_nodes(:, i) / m_node;
    end
    
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
    
    %nodes state derivatives
    for i = 1:N_mt_nodes
        idx = idx_start + (i-1)*6;
        ds(idx : idx+2) = nodes_vel(:, i);
        ds(idx+3 : idx+5) = nodes_acc(:, i);
    end
    
    % Scale back
    ds(1:6)   = ds(1:6  ) / args.ODEscale;
    ds(14:19) = ds(14:19) / args.ODEscale;
    % ds(27:32) = ds(27:32) / args.ODEscale;
    % Scale N nodes
    ds(idx_start : idx_start + 6*N_mt_nodes - 1) = ds(idx_start : idx_start + 6*N_mt_nodes - 1) / args.ODEscale;
    
    ds= ds';

end