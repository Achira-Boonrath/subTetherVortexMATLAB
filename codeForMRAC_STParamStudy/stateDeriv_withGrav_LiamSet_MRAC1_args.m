function [ds] = stateDeriv_withGrav_LiamSet_MRAC1_args(t,s, args)
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
    %ds = zeros(1, mrac_idx + 7 - 1);
    ds = zeros(1, mrac_idx + 9 - 1);
    
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
    
    % Vectorized Loop over N segments
    % Seg 1: Chaser -> Node 1
    % Seg k: Node k-1 -> Node k
    
    % Prep Pos/Vel Matrices for Predecessor (Prev) and Current (Curr) points
    % Prev: [pos_Att, Node 1, Node 2 ... Node N-1]
    P_prev = [pos_Att, nodes_pos(:, 1:end-1)];
    V_prev = [vel_Att, nodes_vel(:, 1:end-1)];
    
    % Curr: [Node 1, Node 2 ... Node N]
    P_curr = nodes_pos;
    V_curr = nodes_vel;
    
    % Vectorized Force Calc
    L_vecs = P_curr - P_prev;
    L_mags = sqrt(sum(L_vecs.^2, 1));
    E_vecs = L_vecs ./ L_mags;
    VR_vecs = V_curr - V_prev;
    
    deltas = L_mags - l0vec(1);
    vr_dots = sum(VR_vecs .* E_vecs, 1);
    
    Tmags = Kvec(1) .* deltas + cVec(1) .* vr_dots;
    
    % Masks
    maskLength = deltas > 0;
    maskTension = Tmags > 0;
    activeMask = maskLength & maskTension;
    
    Tmags(~activeMask) = 0;
    Tvecs = E_vecs .* Tmags; % 3xN
    
    % Apply Forces
    % Force on 'curr' (Node k) gets -Tvec (Pulls back to Prev)
    Force_nodes = Force_nodes - Tvecs;
    
    % Force on 'prev' (Node k-1 or Chaser) gets +Tvec (Pulls fwd to Curr)
    Force_Chaser_Tether = Force_Chaser_Tether + Tvecs(:, 1);
    if N_mt_nodes > 1
        Force_nodes(:, 1:end-1) = Force_nodes(:, 1:end-1) + Tvecs(:, 2:end);
    end
    
    % Capture Segment 1 info for Control logic
    l_mt_seg1 = L_mags(1);
    VR_mt_seg1 = VR_vecs(:,1);
    evec_mt_seg1 = E_vecs(:,1);
    Tvec_mt_seg1 = Tvecs(:,1);
    maskMTreal = activeMask(1);
    
    % Gravity on Nodes
    m_node = massPoint1 / N_mt_nodes; % Distribute mass
    for i = 1:N_mt_nodes
        pos = nodes_pos(:, i);
        grav = -mu * m_node * pos / (norm(pos)^3);
        Force_nodes(:, i) = Force_nodes(:, i) + grav;
    end
    
    %% Adaptive Control
    x1_m = s(mrac_idx);
    x1dot_m = s(mrac_idx+1);
    hhat = s(mrac_idx+2);
    ahat_1 = s(mrac_idx+3);
    ahat_2 = s(mrac_idx+4);
    
    L0    = args.L0;
    kA    = args.kA;
    gamma = args.gamma;
    sigmaMRAC_h  = args.sigmaMRAC_h ;
    sigmaMRAC_a1 = args.sigmaMRAC_a1;
    sigmaMRAC_a2 = args.sigmaMRAC_a2;
    
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
    sliding = (x1dot - x1dot_m) + L0*(x1 - x1_m);
    
    % ==========================
    % Desired elongation scaling
    % ==========================
    desElongScale = min(max((t - tOnChaser_fromt0)/slopeTime, 0.0), 1.0);
    desElonVary   = x1_m0 + desElongScale * ((desElong - x1_m0) / 1.0);
    
    % ==========================
    % Reference model dynamics
    % ==========================
    Kp = args.Kp; Kd = args.Kd;
    ds(mrac_idx+1) = - max([0, (cVec(1)/chaserM)*x1dot_m + (Kvec(1)/chaserM)*max([0, x1_m]) ]) ...
        + min(max(Kp*(desElonVary-x1_m) + Kd*(-x1dot_m), -ThrustSaturation/chaserM), ThrustSaturation/chaserM); % linear ref model
    
    x_rDdot = ds(mrac_idx+1) - L0*(x1dot - x1dot_m);
    delta = hhat*x_rDdot + maskMTreal*( x1*ahat_1 + (x1dot)*ahat_2 ) - kA*sliding;
    
    % ==========================
    % Thrust along +cZ direction
    % ==========================
    clipped_delta = min(max(delta, -ThrustSaturation), ThrustSaturation);
    Fthrust = clipped_delta * ( rotMat_C_A_I' * [0.0,0.0,1.0]' );
    
    maskFT = (delta*delta < ThrustSaturation*ThrustSaturation );
    ds(mrac_idx+2) = maskFT*( -1*gamma*sliding*x_rDdot       - sigmaMRAC_h*((sliding).^2)*hhat );
    ds(mrac_idx+3) = maskFT*( -1*gamma*sliding*x1*maskMTreal - sigmaMRAC_a1*((sliding).^2)*ahat_1 );
    ds(mrac_idx+4) = maskFT*( -1*gamma*sliding*x1dot*maskMTreal - sigmaMRAC_a2*((sliding).^2)*ahat_2 );
    ds(mrac_idx+5:mrac_idx+6) = [0, 0];
    ds(mrac_idx+7:mrac_idx+8) = [x1dot, clipped_delta];
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
    nodes_acc = Force_nodes / m_node;
    
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
    temp_node_states = [nodes_vel; nodes_acc]; % 6 x N
    ds(idx_start : idx_start + 6*N_mt_nodes - 1) = temp_node_states(:);
    
    % Scale back
    ds(1:6)   = ds(1:6  ) / args.ODEscale;
    ds(14:19) = ds(14:19) / args.ODEscale;
    % ds(27:32) = ds(27:32) / args.ODEscale;
    % Scale N nodes
    ds(idx_start : idx_start + 6*N_mt_nodes - 1) = ds(idx_start : idx_start + 6*N_mt_nodes - 1) / args.ODEscale;
    
    ds= ds';

end