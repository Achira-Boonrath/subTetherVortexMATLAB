function [ds_mrac, Fthrust, MRAC_1_updateParams, MRAC_2_updateParams] = ...
    calculateAdaptiveControl_Unified(t, s, args, mrac_idx, x1, x1dot, maskMTreal, rotMat_C_A_I)
    % calculateAdaptiveControl_Unified
    %
    % Computes the adaptive control law and state derivatives for the Model
    % Reference Adaptive Control (MRAC) system applied to the tethered system.
    % This function supports two versions of MRAC logic, selected via args.MRAC_v.
    %
    % Inputs:
    %   t            - Current simulation time (scalar)
    %   s            - State vector (linear array)
    %   args         - Structure containing system parameters and control gains:
    %                  * args.MRAC_v: Control version flag
    %                  * args.chaserM: Mass of the chaser
    %                  * args.cVec, args.Kvec: Damping and stiffness arrays
    %                  * args.l0vec: Unstretched tether lengths
    %                  * args.L0, args.kA, args.gamma: MRAC-1 specific gains
    %                  * args.Gamma_x, args.Gamma_r, args.Gamma_theta: MRAC 2 update gains
    %                  * args.P, args.B_linear: Lyapunov equation parameters
    %                  * args.ThrustSaturation: Maximum allowable thrust
    %                  * ... and other desElong/slopeTime parameters.
    %   mrac_idx     - Index for the start of MRAC states in vector 's'
    %   l_mt_seg1    - Current length of the first tether segment (scalar)
    %   VR_mt_seg1   - Relative velocity vector of the first tether segment (3x1)
    %   evec_mt_seg1 - Unit vector along the first tether segment (3x1)
    %   maskMTreal   - Boolean flag indicating if the tether segment is taut
    %   rotMat_C_A_I - Rotation matrix from Inertial to Chaser Attachment frame (3x3)
    %
    % Outputs:
    %   ds_mrac      - Derivative of the MRAC states (1x9 vector).
    %                  Contains derivatives for reference model, adaptive gains, and tracked variables.
    %   Fthrust      - Calculated thrust force vector (3x1) in the Inertial frame.
    %
    
    %% Extract Parameters needed locally
    chaserM = args.chaserM;
    cVec = args.cVec;
    Kvec = args.Kvec;
    l0vec = args.l0vec;
    
    % Initialize output state derivative vector (size 9 to match allocations)
    ds_mrac_1 = zeros(1, 9);
    ds_mrac_2 = zeros(1, 9);
    ds_mrac = zeros(1, 9);
    Fthrust = zeros(3, 1);

    %% 1. Derivative of Reference Model Position

    x1_m    = s(mrac_idx);      % Reference model state (position/elongation)
    x1dot_m = s(mrac_idx+1);    % Reference model state (rate)

    ds_mrac_1(1) = x1dot_m;
    ds_mrac_2(1) = ds_mrac_1(1);
    % --- Desired Trajectory Generation ---
    % Ramps up desired elongation from x1_m0 to desElong over slopeTime
    x1_m0  = args.x1_m0 ;
    tOnChaser_fromt0  = args.tOnChaser_fromt0 ;
    slopeTime = args.slopeTime;
    desElong  = args.desElong ;
    ThrustSaturation = args.ThrustSaturation;
    
    % Compute time-varying desired elongation
    desElongScale = min(max((t - tOnChaser_fromt0)/slopeTime, 0.0), 1.0);
    desElonVary   = x1_m0 + desElongScale * ((desElong - x1_m0) / 1.0);

    % --- Reference Model Dynamics ---
    Kp = args.Kp; Kd = args.Kd;
    % The reference model is a mass-spring-damper system driven by PD control to track desElonVary.
    % It includes a saturation term to represent physical thrust limits.
    % ds_mrac_1(2) = - max([0, (cVec(1)/chaserM)*x1dot_m + (Kvec(1)/chaserM)*max([0, x1_m]) ]) ...
    %     + min(max(Kp*(desElonVary-x1_m) + Kd*(-x1dot_m),
    %     -ThrustSaturation/chaserM), ThrustSaturation/chaserM); old, elong
    %     measured using first segment only
    
    N_mt_nodes = args.N_mt_nodes;
    ds_mrac_1(2) = - max([0, (cVec(1)/chaserM)*x1dot_m/(N_mt_nodes+0) + (Kvec(1)/chaserM)*max([0, x1_m])/(N_mt_nodes+0) ]) ...
        + min(max(Kp*(desElonVary-x1_m) + Kd*(-x1dot_m), -ThrustSaturation/chaserM), ThrustSaturation/chaserM);
    ds_mrac_2(2) = ds_mrac_1(2);
    %% ==================================================================================================================================================
    % MRAC Version 1:
    % ==================================================================================================================================================
    
    % --- Extract current Adaptive States ---
    hhat    = s(mrac_idx+2);    % Adaptive parameter estimate: h_hat
    ahat_1  = s(mrac_idx+3);    % Adaptive parameter estimate: a_hat_1
    ahat_2  = s(mrac_idx+4);    % Adaptive parameter estimate: a_hat_2
    
    % --- Extract Gains for MRAC 1 ---
    L0    = args.L0;            % Sliding surface gain / Decay rate
    kA    = args.kA;            % Control gain
    gamma = args.gamma;         % Adaptation rate
    sigmaMRAC_h  = args.sigmaMRAC_h ;  % Sigma-modification term for h
    sigmaMRAC_a1 = args.sigmaMRAC_a1;  % Sigma-modification term for a1
    sigmaMRAC_a2 = args.sigmaMRAC_a2;  % Sigma-modification term for a2
    
    % --- Compute System States for Segment 1 ---
    % x1: Elongation error (Current length - Unstretched length)
    % x1 = (l_mt_seg1 - l0vec(1));
    % % x1dot: Rate of change of elongation (projected relative velocity)
    % x1dot = dot(VR_mt_seg1, evec_mt_seg1);

    % Sliding surface variable 's' (not to be confused with state vector s)
    sliding = (x1dot - x1dot_m) + L0*(x1 - x1_m);
    
    % --- Control Law Calculation ---
    x_rDdot = ds_mrac_1(2) - L0*(x1dot - x1dot_m);
    delta = hhat*x_rDdot + maskMTreal*( x1*ahat_1 + (x1dot)*ahat_2 ) - kA*sliding;
    
    % --- Thrust Calculation ---
    % Saturate the calculated control effort
    clipped_delta = min(max(delta, -ThrustSaturation), ThrustSaturation);
    % Apply thrust in the Chaser's body frame (+Z direction), rotated to Inertial frame
    Fthrust_1 = clipped_delta * ( rotMat_C_A_I' * [0.0,0.0,1.0]' );
    
    % --- Adaptive Laws Update ---
    % Only update adaptive parameters if not saturated to prevent wind-up
    maskFT = ((abs(delta)+ 1e-1)*(abs(delta)+ 1e-1) < ThrustSaturation*ThrustSaturation );
    
    % Update laws with Sigma-modification for robustness
    ds_mrac_1(3) = maskFT*( -1*gamma*sliding*x_rDdot       - sigmaMRAC_h*((sliding).^2)*hhat );
    ds_mrac_1(4) = maskFT*( -1*gamma*sliding*x1*maskMTreal - sigmaMRAC_a1*((sliding).^2)*ahat_1 );
    ds_mrac_1(5) = maskFT*( -1*gamma*sliding*x1dot*maskMTreal - sigmaMRAC_a2*((sliding).^2)*ahat_2 );
    
    MRAC_1_updateParams = [maskFT; -1*gamma*sliding*x_rDdot; - sigmaMRAC_h*((sliding).^2)*hhat ;...
        -1*gamma*sliding*x1*maskMTreal; - sigmaMRAC_a1*((sliding).^2)*ahat_1 ;...
        -1*gamma*sliding*x1dot*maskMTreal; - sigmaMRAC_a2*((sliding).^2)*ahat_2];
    
    % Unused states for this version (fillers)
    ds_mrac_1(6:7) = [0, 0];
    
    % Record tracked variables for debugging/plotting
    ds_mrac_1(8:9) = [x1dot, clipped_delta];
    
    
    %% ==================================================================================================================================================
    % MRAC Version 2:
    % ==================================================================================================================================================
    
    % --- Extract States ---
    hhat = s(mrac_idx+2);        % Adaptive gain: K_r_hat (Command feedforward)
    ahat_1 = s(mrac_idx+3);      % Adaptive gain: K_x_hat component 1
    ahat_2 = s(mrac_idx+4);      % Adaptive gain: K_x_hat component 2
    Theta_hat = s(mrac_idx+5:mrac_idx+6); % Adaptive gain: Theta_hat (
    
    % Reconstruct gain vectors
    Kx_hat    = [ahat_1; ahat_2];
    Kr_hat    = hhat;
    
    % --- System States (Segment 1) ---
    % x1 = (l_mt_seg1 - l0vec(1));
    % x1dot = dot(VR_mt_seg1, evec_mt_seg1);
    
    % Extract MRAC 2 specific gains
    Gamma_x =  args.Gamma_x;
    Gamma_r = args.Gamma_r;
    Gamma_theta = args.Gamma_theta;
    P  = args.P;                % Solution to Lyapunov equation
    B_linear = args.B_linear ;
    sigmaMRACLin = args.sigmaMRACLin ;
    
    % --- Error and Regressor Definitions ---
    e      = [x1 - x1_m; x1dot - x1dot_m];    % Tracking error vector e = x - x_m
    % Phi is the regressor vector. It zeroes out terms when the tether is slack (maskMTreal=0)
    Phi    = [x1*(double(maskMTreal) - 1); x1dot*(double(maskMTreal) - 1)];
    x_vec  = [x1; x1dot];                     % State vector x
    
    % --- Control Input Calculation ---
    % u = K_x'x + K_r*r - Theta'*Phi
    delta = Kx_hat' *x_vec + Kr_hat*desElonVary - Theta_hat' *Phi;
    
    % --- Adaptive Laws Update ---
    % Updates based on Lyapunov stability analysis (e'PB term drives adaptation)
    % Includes sigma-modification for robustness.
    common_term = (e' * P * B_linear); % Scalar term appearing in all updates
    
    Kx_hat_dot    = (-Gamma_x * x_vec * common_term - sigmaMRACLin*abs(common_term)*Gamma_x *Kx_hat)' ;
    Kr_hat_dot    = (-Gamma_r * desElonVary * common_term - sigmaMRACLin*abs(common_term)*Gamma_r* desElonVary*Kr_hat)' ;
    Theta_hat_dot = (-Gamma_theta * Phi * common_term - sigmaMRACLin*abs(common_term)*Gamma_theta *Theta_hat)' ;
    
    % --- Thrust Calculation ---
    clipped_delta = min(max(delta, -ThrustSaturation), ThrustSaturation);
    Fthrust_2 = clipped_delta * ( rotMat_C_A_I' * [0.0,0.0,1.0]' );
    
    % Mask updates if saturated
    maskFT = ((abs(delta)+ 1e-1)*(abs(delta)+ 1e-1) < ThrustSaturation*ThrustSaturation );
    
    MRAC_2_updateParams = [maskFT; -Gamma_x * x_vec * common_term; - sigmaMRACLin*abs(common_term)*Gamma_x *Kx_hat; ...
        -Gamma_r * desElonVary * common_term; - sigmaMRACLin*abs(common_term)*Gamma_r* desElonVary*Kr_hat; ...
        -Gamma_theta * Phi * common_term; - sigmaMRACLin*abs(common_term)*Gamma_theta *Theta_hat ];
    
    % Store derivatives
    ds_mrac_2(3) = Kr_hat_dot*double(maskFT);      % d(hhat)/dt
    ds_mrac_2(4:5) = Kx_hat_dot*double(maskFT);    % d(ahat_1)/dt, d(ahat_2)/dt
    ds_mrac_2(6:7) = Theta_hat_dot*double(maskFT); % d(Theta_hat)/dt
    ds_mrac_2(8:9) = [x1dot, clipped_delta];       % Tracked vars
    
    
    if args.MRAC_v == 1
        Fthrust = Fthrust_1;
        ds_mrac = ds_mrac_1;
    elseif args.MRAC_v == 2
        Fthrust = Fthrust_2;
        ds_mrac = ds_mrac_2;
    end

end
