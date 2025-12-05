function [cost] = nehaST_MRAC_script(wIn2)%(wIn)
% Opt_mainFunc_SMC_3kCG_newAttPt([3 6.8 3 1]) or [3 8 3 1]
    % 
    % Calculates the cost value for a given input parameter vector `w` as part of an optimization routine. 
    %% Func Description
    % w: 3x1 decision variable vector
    close all
    % clear all
    % clc

    %% opt comb 1 
    %threadDamping = 10^(wIn(3)*3.5);
    %threadStiffness = 10^(wIn(1)*3.5);

    %L_min = 5.5; L_max = 9.5;
    % set lower and upper bounds of the decision variables
    %lb = L_min; ub = L_max;
    %l0_st = wIn(2)*(ub - lb) +  lb;
    wIn = [0.923832631579215   0.153706128897234   0.818002917495584];

    %% opt combo 2  
    %threadDamping = 10^(wIn(3)*3.5);
    %threadStiffness = 10^(wIn(1)*3.5);

    %L_min = 5.5; L_max = 9.5;
    % set lower and upper bounds of the decision variables
    %lb = L_min; ub = L_max;
    %l0_st = wIn(2)*(ub - lb) +  lb;
    % wIn = [1.0	0.8228146347919569	0.007893587065757102]; %from opt
    % wIn =[0.985	0.8228146347919569	0.007893587065757102]; %match spikes
    %% Import data 
    % load("dataIC_ForSTrun_Neha.mat");
    load("dataIC_ForSTrun_Neha_002.mat");
    % load("dataIC_ForSTrun_Neha_007.mat");
    % load("dataForNI1_009_1MT_compare.mat")

    for GG = [1]
        %% Adaptive states
        % Set feedback control mode to MRAC (Model Reference Adaptive Control)
        fbControl = 'MRAC';
        MRAC_v = 2; % Version of MRAC being used
        
        % Set thrust magnitude from MRAC parameters
        FT_const = dataMRAC.MRACparams(8);
        
        % Compute the vector from chaser center to attachment point (assumed at -Z face)
        distAttPt_to_C = 0.5 * chaserSideLength * [0 0 -1]';
        
        % Calculate the vector from chaser attachment point to connection point
        l_mt_vec = states0_connPoint1(1:3) - (pos0chaser + rotMat_C_A_I' * (distAttPt_to_C));
        l_mt = norm(l_mt_vec); % Length of the main tether
        evec_mt = l_mt_vec / l_mt; % Unit vector along the main tether
        
        % Compute relative velocity between connection point and chaser attachment point
        VR_mt = states0_connPoint1(4:6) - (vel0chaser + rotMat_C_A_I' * (cross(omega0chaser, distAttPt_to_C)));
        
        if MRAC_v == 1
        % Initialize adaptive states for the ODE solver - MRAC-1
            s0(33) = (l_mt - l0vec(1)); 
            s0(34) = 0.0; 
            % ${\hat{h}}$ 
            s0(35) = chaserM;
            % $\hat{\mathbf{a}}=[{\hat{a}}_1, {\hat{a}}_2]^T$
            s0(36) = 1.0 * Kvec(1);
            s0(37) = 1.0 * cVec(1);
            s0(38:39) = [0, 0];

            % Good 1
            args.L0 = 15*0.9;
            args.kA = 1*chaserM*3.8*(1.1);
            args.gamma = 30;
            args.sigmaMRAC_h  = 0.05; %1.0e+03; %0.25 * 0.1  *3*4.0*(args.L0/15.0);
            args.sigmaMRAC_a1 = 0.0005; %0.25 * 0.1  *3*4.0*(args.L0/15.0);
            args.sigmaMRAC_a2 = 0.05; %0.25 * 0.01 *3*4.0*(args.L0/15.0);

            % wIn2=[0.687367280667017                   0   0.489991127634287]; % opt solu 1
            % lb1 = 15*0.6; ub1 = 15*1.1;
            % args.L0 = wIn2(1)*(ub1 - lb1) +  lb1;
            % lb2 = chaserM*3.8*(1.0); ub2 = chaserM*3.8*(2.0);
            % args.kA = wIn2(2)*(ub2 - lb2) +  lb2;
            % lb3 = 27; ub3 = 33;
            % args.gamma = wIn2(3)*(ub3 - lb3) +  lb3;

            % wIn2=[0   0.101953473828612   0.972077265457035]; % opt solu 2
            % lb1 = 15*0.7; ub1 = 15*1.1;
            % args.L0 = wIn2(1)*(ub1 - lb1) +  lb1;
            % lb2 = chaserM*3.8*(0.8); ub2 = chaserM*3.8*(1.5);
            % args.kA = wIn2(2)*(ub2 - lb2) +  lb2;
            % lb3 = 28; ub3 = 32;
            % args.gamma = wIn2(3)*(ub3 - lb3) +  lb3;

            % wIn2=[0.761115429591623   0.800889939488221                   0]; % opt solu 3
            % lb1 = 15*0.5; ub1 = 15*1.0;
            % args.L0 = wIn2(1)*(ub1 - lb1) +  lb1;
            % lb2 = chaserM*3.8*(0.9); ub2 = chaserM*3.8*(1.5);
            % args.kA = wIn2(2)*(ub2 - lb2) +  lb2;
            % lb3 = 28; ub3 = 32;
            % args.gamma = wIn2(3)*(ub3 - lb3) +  lb3;

            args.sigmaMRAC_h  = (args.gamma/30)*0.05; 
            args.sigmaMRAC_a1 = (args.gamma/30)*0.0005; 
            args.sigmaMRAC_a2 = (args.gamma/30)*0.05; 
    
        else
        % Initialize adaptive states for the ODE solver - MRAC-2
            s0(33) = (l_mt - l0vec(1)); 
            s0(34) = 0.0; 
            
            % MODIFY THESE INITIAL GUESS VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %$\hat{k}_r $
            s0(35) = 37500; 
            %$\hat{\mathbf{K}}_z=[\hat{k}_{z,1}, \hat{k}_{z,2}]^T$
            s0(36:37) = -1*[37500, 56250]; 
            %$\hat{{\boldsymbol{\Theta}}} =[\hat{{\boldsymbol{\Theta}}}_{1}, \hat{{\boldsymbol{\Theta}}}_{2}]^T$
            s0(38:39) = -0.5*[30925, 14.78]; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            args.Gamma_x = [[5e+7 ,   0];       [  0 ,  5e+7]];
            args.Gamma_r = 4.1e+10;
            args.Gamma_theta = [[5e+7,   0];       [  0 ,  1e+4]];
            args.P = [[465.0415   ,   5.2612624];       [  5.2612624,   6.547933 ]];
            args.B_linear = [0, 1/chaserM]';
            args.sigmaMRACLin = 0.000003;
        end
        %% Set Sub-Tether Parameters

        % Extract main tether parameters from data
        l0_mt = data.inertialMTParams(end);        
        threadDamping = 10^(wIn(3)*3.5); % Compute damping coefficient
        threadStiffness = 10^(wIn(1)*3.5); % Compute stiffness coefficient

        % Define bounds for sub-tether length
        L_min = 5.5; L_max = 9.5;
        lb = L_min; ub = L_max; % Lower and upper bounds
        l0_st = wIn(2)*(ub - lb) +  lb; % Compute sub-tether length

        % Define vectors for tether parameters
        l0vec = [l0_mt, l0_st*ones(1,4)]; % Initial lengths
        Kvec = [data.inertialMTParams(end-2), threadStiffness, threadStiffness, threadStiffness, threadStiffness]; % Stiffness values
        cVec = [data.inertialMTParams(end-1), threadDamping, threadDamping, threadDamping, threadDamping]; % Damping values


        %% Set Sim Params
        % system params %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        args.massPoint1= massPoint1; 
        args.targetM= debrisM;         args.targetI= debrisI;
        args.chaserM= chaserM;        args.chaserI= chaserI; 
        args.chaserSideLength= chaserSideLength; 
        args.targetSideLengthX= debrisSideLengthX; 
        args.targetSideLengthY= debrisSideLengthY;
        args.targetSideLengthZ= debrisSideLengthZ; 
        args.Kvec= Kvec;         args.l0vec=l0vec; 
        args.cVec= cVec;        args.mu= muEarth; 
        args.FT= FT_const;
        % Define gravitational parameter for Earth
        muEarth = 3.986000e14; 
        
        % desired control profile %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        args.x1_m0 = 0.0;         args.tOnChaser_fromt0 = 0.0; 
        % args.slopeTime = 60;         args.desElong = 0.01;
        args.slopeTime = 1e-4;         args.desElong = 0.01;
        args.ThrustSaturation = 850;
        args.Kp = 6000; args.Kd = 9000;
        args.J2on = 0;
        
        ODEscale = 1;
        args.ODEscale = ODEscale;
        s0(1:6)   = s0(1:6  ) / ODEscale;
        s0(14:19) = s0(14:19) / ODEscale;
        s0(27:32) = s0(27:32) / ODEscale;
        %% Solve ODE for System Dynamics

        % Set ODE solver options
        options = odeset('RelTol', 2e-8,'AbsTol', 2e-8,'Stats','off');
        % tspan = data.IntegrationTime(Idx0:IdxF) - data.IntegrationTime(Idx0); % Time span for integration
        tspan = data.IntegrationTime(Idx0:(1200+Idx0) ) - data.IntegrationTime(Idx0); % Time span for integration
        
        % Start timer for ODE solver
        tStart = tic; 
        % Solve the system of ODEs using ode23
        if MRAC_v == 1
            [t_mod_code2, state_vec] = ode23(@(t,s) stateDeriv_withGrav_LiamSet_MRAC1_args_mex(t,s,args), tspan, s0, options);
        else 
            [t_mod_code2, state_vec] = ode23(@(t,s) stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_mex(t,s,args), tspan, s0, options);
        end

        % Measure time taken for integration
        t_ode45 = toc(tStart);
        fprintf('It took %.4f s to integrate \n', t_ode45);
        
        % Scale state vectors for output
        state_vec(:, 1:6)   = state_vec(:, 1:6  ) * ODEscale;
        state_vec(:, 14:19) = state_vec(:, 14:19) * ODEscale;
        state_vec(:, 27:32) = state_vec(:, 27:32) * ODEscale;

        %% Extract Quantities from ODE Outputs

        % Extract positions and velocities of the chaser, debris, and connection point
        X1 = state_vec(:, 1); Y1 = state_vec(:, 2); Z1 = state_vec(:, 3);
        VX1 = state_vec(:, 4); VY1 = state_vec(:, 5); VZ1 = state_vec(:, 6);
        
        X2 = state_vec(:, 14); Y2 = state_vec(:, 15); Z2 = state_vec(:, 16);
        VX2 = state_vec(:, 17); VY2 = state_vec(:, 18); VZ2 = state_vec(:, 19);
        
        X3 = state_vec(:, 27); Y3 = state_vec(:, 28); Z3 = state_vec(:, 29);
        VX3 = state_vec(:, 30); VY3 = state_vec(:, 31); VZ3 = state_vec(:, 32);
        
        t_plot = t_mod_code2; % Time vector for plotting
        
        % Extract quaternion and angular velocity for chaser and debris
        quar1_B1 = state_vec(:, 7); quar1_B2 = state_vec(:, 8);
        quar1_B3 = state_vec(:, 9); quar1_B4 = state_vec(:, 10);
        omega1_BX = state_vec(:, 11); omega1_BY = state_vec(:, 12);
        omega1_BZ = state_vec(:, 13);
        
        quar2_B1 = state_vec(:, 20); quar2_B2 = state_vec(:, 21);
        quar2_B3 = state_vec(:, 22); quar2_B4 = state_vec(:, 23);
        omega2_BX = state_vec(:, 24); omega2_BY = state_vec(:, 25);
        omega2_BZ = state_vec(:, 26);
        
        omega2_I = zeros(length(t_plot), 3); % Initialize debris angular velocity in inertial frame
        omega1_I = zeros(length(t_plot), 3); % Initialize chaser angular velocity in inertial frame

        %% Post-Process Results

        % compute alignment angle and elongation for the MT
        [alignAng, elong_mt] = postAlignAng(state_vec(:, 1:13), state_vec(:, 14:26), X3 ,Y3 ,Z3, VX3 ,VY3 ,VZ3, chaserSideLength, 0, Kvec, l0vec, cVec, muEarth, FT_const);
        
        % Compute tension components from MATLAB simulation results
        [Tx,Ty,Tz] = postCompTension_Python(state_vec(:, 1:13), state_vec(:, 14:26), X3 ,Y3 ,Z3, VX3 ,VY3 ,VZ3, chaserSideLength, 0, 0, Kvec, l0vec, cVec, muEarth, FT_const);
        % post compute thrust, from total chaser acceleration
        % Loop through each time step to compute tension magnitude and thrust force
        for k = 1:length(Tx)
            % Compute the magnitude of the sub-tether tension vector
            TmagST(k) = norm([Tx(k, 1), Ty(k, 1), Tz(k, 1)]);

            % Compute the state derivative at the current time step based on the MRAC version
            if MRAC_v == 1
            % Use MRAC-1 model for state derivative computation
            ds_k = stateDeriv_withGrav_LiamSet_MRAC1_args_mex(t_mod_code2(k), state_vec(k, :)', args);
            else
            % Use Adaptive Linear model for state derivative computation
            ds_k = stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_mex(t_mod_code2(k), state_vec(k, :)', args);
            end

            % Define constants for Earth's gravitational and J2 perturbation effects
            J2 = 1.08263e-3;            % Earth's J2 coefficient
            rEarth = 6378.0e3;          % Earth's radius in meters
            posChaser = state_vec(k, 1:3); posChaserNorm = norm(posChaser); % Compute the norm of the chaser position vector

            % Compute gravitational force acting on the chaser
            gravChaser = -muEarth * chaserM * posChaser / (posChaserNorm^3);

            % Compute J2 perturbation force acting on the chaser
            coeffT = (3 / 2) * J2 * muEarth * (rEarth^2 / posChaserNorm^4);
                J2_Chaser = args.J2on * chaserM * coeffT * [ ...
                (5 * (posChaser(3) / posChaserNorm)^2 - 1) * (posChaser(1) / posChaserNorm), ...
                (5 * (posChaser(3) / posChaserNorm)^2 - 1) * (posChaser(2) / posChaserNorm), ...
                (5 * (posChaser(3) / posChaserNorm)^2 - 3) * (posChaser(3) / posChaserNorm) ...
            ];

            % Compute the total gravitational force acting on the chaser
            gtotChaser = (gravChaser' + J2_Chaser');

            % Compute the control acceleration by subtracting gravitational and tension forces
            accControl = ds_k(4:6) - ...
            ([Tx(k, 1), Ty(k, 1), Tz(k, 1)]' + gtotChaser) / chaserM;

            % Compute the magnitude of the thrust force
            FmagST(k) = norm(accControl * chaserM);
        end

        %% optional Plotting
        % if LiamSet == 0
        %     MRACterms_Python = dataMRAC.MRAC;
        %     plotThrustAndMRAC(t_plot, FInfo, MRACterms_Python, fbControl, saveName, path_png, path_fig)
        % end
        
        % Call function to compute RMSE and other post-processing results
        % [sumRMSE_vec_AngPos, RMSEangvel_t, RMSEangvel_c, RMSErelPos, RMSErelVel, ...
        % omega2_I, omega1_I, TensionInfo, FInfo, alignAng_Python, elong_mt_Python, ...
        % chaserAngEng_Python, debrisAngEng_Python, TmagST, FmagST, alignAng, elong_mt, ...
        % chaserAngEng, debrisAngEng, MRACterms_Python] = computeRMSEandPostResults( ...
        % t_plot, ...
        % quar1_B1, quar1_B2, quar1_B3, quar1_B4, omega1_BX, omega1_BY, omega1_BZ, ...
        % quar2_B1, quar2_B2, quar2_B3, quar2_B4, omega2_BX, omega2_BY, omega2_BZ, ...
        % data, Idx0, IdxF, X1, Y1, Z1, VX1, VY1, VZ1, ...
        % X2, Y2, Z2, VX2, VY2, VZ2, X3, Y3, Z3, VX3, VY3, VZ3, ...
        % chaserSideLength, N_k, Kvec, l0vec, cVec, muEarth, FT_const, ...
        % fbControl, chaserM, dataMRAC, LiamSet, chaserI, debrisI, state_vec);

        %% Plotting 
        runFuncForOpt = 0;
        if runFuncForOpt == 0
            save(['subTetherMRAC_v',num2str((MRAC_v)),'.mat'])

            % %fuel consumed %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % thrustIntHist{count} = dataMRAC.MRACintStates(:,1)*(double(dataMRAC.g0) * double(dataMRAC.Isp));
            % % figure; plot(thrustMag)
            % % thrustIntHist{count} = cumtrapz(tspan, thrustMag(end-length(tspan)+1:end));
            % thrustIntHist{count} = cumtrapz(tspan, abs(thrustMag(end-length(tspan)+1:end)));

            % % control error %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % elongError = dataMRAC.MRAC(end-length(tspan)+1:end,5) - elong_mt_Python(end-length(tspan)+1:end);
            % slopeTime = dataMRAC.slopeTime; desElong = dataMRAC.MRACparams(3);
            % elongError = desElong*min(max((tspan)/slopeTime, 0.0), 1.0)- elong_mt_Python(end-length(tspan)+1:end);
            % % elongErrorHist{count} = cumtrapz(tspan, elongError(end-length(tspan)+1:end));
            % elongErrorHist{count} = cumtrapz(tspan, abs(elongError(end-length(tspan)+1:end)));

            % % change in debris rot. energy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % debrisAngEngChange_PythonHist = debrisAngEng_Python(end-length(tspan)+1:end) - debrisAngEng_Python(end-length(tspan)+1);
            % % debrisAngEngChange_PythonIntHist{count} = cumtrapz(tspan, debrisAngEngChange_PythonHist);
            % debrisAngEngChange_PythonIntHist{count} = cumtrapz(tspan, (debrisAngEngChange_PythonHist));

            % tSkip = round(1/(tspan(3)-tspan(2)));
            figure; tSkip = 1;
            % subplot(2, 1, 1)
            % plot(TmagST(1:tSkip:end)) 
            % hold on;
            % yline(0.01 * data.inertialMTParams(end-2))
            % xlabel("Time, s", 'fontsize', 13,'interpreter','latex')
            % ylabel("MT Tension, N", 'fontsize', 13,'interpreter','latex')
            % grid minor; 
            % subplot(2, 1, 2)
            % Plot MT Elongation and Thrust Force %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            subplot(2, 1, 1)
            plot(t_plot, elong_mt(1:tSkip:end)) 
            hold on;
            % yline(0.01)
            plot(t_plot, state_vec(1:tSkip:end,33),"--")
            ylim([0 0.03])
            legend("Actual","Desired")
            xlabel("Time, s", 'fontsize', 13,'interpreter','latex')
            ylabel("MT Elongation, m", 'fontsize', 13,'interpreter','latex')
            grid minor; 
            subplot(2, 1, 2)
            plot(t_plot, FmagST(1:tSkip:end)) 
            ylim([0 900])
            xlabel("Time, s", 'fontsize', 13,'interpreter','latex')
            ylabel("$|F_T|$ N", 'fontsize', 13,'interpreter','latex')
            grid minor; 
            % Save the figure
            saveas(gcf, 'MT_Elongation_Thrust.png');

            % Plot Adaptive Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            figure; 
            subplot(3, 2, 1)
            plot(t_plot, state_vec(1:tSkip:end,35))
            xlabel("Time, s", 'fontsize', 13,'interpreter','latex'); 
            ylabel("$\hat{K}_r$, kg", 'fontsize', 13,'interpreter','latex')
            grid minor; 
            subplot(3, 2, 2)
            plot(t_plot, state_vec(1:tSkip:end,36))
            xlabel("Time, s",'fontsize', 13,'interpreter','latex')
            ylabel("$\hat{K}_{z,1}$, N/m",'Interpreter','latex')
            grid minor
            subplot(3, 2, 3)
            plot(t_plot, state_vec(1:tSkip:end,37))
            xlabel("Time, s",'fontsize', 13,'interpreter','latex')
            ylabel("$\hat{K}_{z,2}$, Ns/m",'Interpreter','latex')
            grid minor
            subplot(3, 2, 4)
            plot(t_plot, state_vec(1:tSkip:end,38))
            xlabel("Time, s",'fontsize', 13,'interpreter','latex')
            ylabel("$\hat{\Theta}_{1}$, N/m",'Interpreter','latex')
            grid minor
            subplot(3, 2, 5)
            plot(t_plot, state_vec(1:tSkip:end,39))
            grid minor
            xlabel("Time, s",'fontsize', 13,'interpreter','latex')
            ylabel("$\hat{\Theta}_{2}$, Ns/m",'Interpreter','latex')
            % Save the figure
            saveas(gcf, 'Adaptive_Parameters.png');

            % Plot MT State Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            figure;
            subplot(2, 1, 1)
            plot(t_plot, state_vec(1:tSkip:end,33))
            xlabel("Time, s", 'fontsize', 13,'interpreter','latex')
            ylabel("$z_m$, m", 'fontsize', 13,'interpreter','latex')
            ylim([0 0.02])
            grid minor
            subplot(2, 1, 2)
            plot(t_plot, state_vec(1:tSkip:end,34))
            xlabel("Time, s",'fontsize', 13,'interpreter','latex')
            ylabel("$dz_m/dt$, m/s",'Interpreter','latex')
            grid minor
            % Save the figure
            saveas(gcf, 'MT_State_Variables.png');
    
        end
    end
    cost = rmse(state_vec(200:end,33)', elong_mt(200:end))*(1e+4); % gain tune
    
    % cost = rmse(elong_mt_Python(1:700), elong_mt(1:700)); % param ID
end





