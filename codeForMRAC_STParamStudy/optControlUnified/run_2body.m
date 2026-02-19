function run_2body
    % Wrapper for the 2-body unconstrained problem
    
    %% 1. Problem Configuration
    
    % Physical Constants
    muVal = 398600;
    r0 = 780 + 6378;
    rf = 1770 + 6378;
    tf = 1.6 * 3600 * 0.5;
    
    problem.x0 = [r0 0 0 sqrt(muVal/r0)]';
    problem.xf = [-rf 0 0 -sqrt(muVal/rf)]';
    problem.tf = tf;
    problem.lambda0_guess = [0; 0; 0; -1];
    problem.useParticleSwarm = false;
    
    % Store constants for symbolic setup
    problem.constants.muVal = muVal;
    problem.constants.meanMotion = 0.001; 
    
    %% 2. Function Handles
    problem.setupSyms = @() setupSyms_2Body(problem.constants);
    problem.terminalConst = @(z_tf, xf) z_tf(5:8) - xf; % Match state x (indices 5-8 in z=[lam; x])
    problem.postProcess = @postProcess_2Body;
    problem.plotFn = @plot_2Body;
    
    %% 3. Run Solver
    [t, z, u, problem] = optControl_singleShoot_Unified(problem);
    
end

function [Lvec, X, Xnum, U, f, V, optUSet, symParams] = setupSyms_2Body(consts)
    syms x y vx vy real
    syms ax ay real
    syms L1 L2 L3 L4 real
    syms x1 x2 x3 x4 real
    syms muEarth real
    
    X = [x; y; vx; vy];
    Xnum = [x1; x2; x3; x4];
    U = [ax; ay];
    Lvec = [L1; L2; L3; L4];
    
    r_sq = x^2 + y^2;
    f = [vx; ...
         vy; ...
         -x*muEarth/(r_sq^(3/2)) + ax; ...
         -y*muEarth/(r_sq^(3/2)) + ay];
         
    V = 0.5 * (U.' * U);
    optUSet = []; % Default solve
    
    symParams.muEarth = consts.muVal;
end

function u = postProcess_2Body(t, z, problem)
    % z = [lambda; x]
    % lambda is first 4, x is next 4.
    lambda = z(:, 1:4);
    % u = -lambda_v = -lambda(3:4)
    u = -lambda(:, 3:4);
end

function plot_2Body(t, z, u, problem)
    x = z(:, 5:8);
    figure;
    subplot(3,1,1); plot(t, x(:,1)); ylabel('x'); title('2-Body Unified Results');
    subplot(3,1,2); plot(t, x(:,2)); ylabel('y');
    subplot(3,1,3); plot(t, u); ylabel('u'); xlabel('Time');
end
