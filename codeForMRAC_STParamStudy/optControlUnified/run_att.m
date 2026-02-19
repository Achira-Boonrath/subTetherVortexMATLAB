function run_att
    % Wrapper for the Attitude Control Problem
    
    %% 1. Parameters
    tf = 3600 * 1.5;
    
    % Initial State: [Pos(0), Vel(0), q(0), w(0)]
    % Pos/Vel = 0 relative?
    x0_trans = [0; -500; 0; 0; 0; 0];
    q0 = [1; 0; 0; 0]; % Identity (assuming scalar first or last? MATLAB quat is scalar first usually [w x y z])
    % Original used rotm2quat(rotz(0)).
    % Let's use the same initial conditions as original file L24
    % x (m), y (m), vx (m/s), vy (m/s) ?? 
    % Original L24: x0 = [[0 -500 0 0 0 0], rotm2quat(rotz(0)), [0 0 0]]';
    
    x0 = [0; -500; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]; 
    % Note: rotm2quat([1 0 0; 0 1 0; 0 0 1]) is [1 0 0 0].
    
    % Target:
    xf = [0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0];
    % rotm2quat(rotz(180)) -> [cos(90) 0 0 sin(90)] -> [0 0 0 1].
    
    problem.x0 = x0;
    problem.xf = xf;
    problem.tf = tf;
    problem.lambda0_guess = zeros(13, 1);
    
    problem.constants.meanMotion = 0.001;
    problem.constants.Jx = 100;
    problem.constants.Jy = 100;
    problem.constants.Jz = 100;
    
    %% 2. Setup
    problem.setupSyms = @() setupSyms_Att(problem.constants);
    problem.terminalConst = @(z_tf, xf) z_tf(14:end) - xf; 
    % z = [lambda(13); x(13)]. x is 14:26.
    
    problem.postProcess = @postProcess_Att;
    problem.plotFn = @plot_Att;
    problem.solverOpts = optimoptions('fsolve', 'Display', 'iter', 'MaxIterations', 40);
    
    %% 3. Run
    [t, z, u, problem] = optControl_singleShoot_Unified(problem);
end

function [Lvec, X, Xnum, U, f, V, optUSet, symParams] = setupSyms_Att(c)
    syms x y z vx vy vz real
    syms q1 q2 q3 q4 wx wy wz real
    syms ax ay az tx ty tz real
    syms n Jx Jy Jz real
    
    X = [x; y; z; vx; vy; vz; q1; q2; q3; q4; wx; wy; wz];
    syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 real
    Xnum = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12; x13];
    
    U = [ax; ay; az; tx; ty; tz];
    Lvec = sym('L', [13 1], 'real');
    
    J = diag([Jx, Jy, Jz]);
    omega = [wx; wy; wz];
    torque = [tx; ty; tz];
    
    % Linearized CWH (Relative Motion)
    f_trans = [vx; ...
               vy; ...
               vz; ...
               3*n^2*x + 2*n*vy + ax; ...
               -2*n*vx + ay; ...
               -n^2*z + az];
    
    f_quat = 0.5 * [[-q2 -q3 -q4]; [q1 -q4 q3]; [q4 q1 -q2]; [-q3 q2 q1]] * omega;
    
    % Rotational Dynamics
    % J w_dot + w x Jw = torque
    f_omega = J \ (torque - cross(omega, J*omega));
    
    f = [f_trans; f_quat; f_omega];
    
    V = 0.5 * (U.' * U);
    optUSet = [];
    
    symParams.n = c.meanMotion;
    symParams.Jx = c.Jx;
    symParams.Jy = c.Jy;
    symParams.Jz = c.Jz;
end

function u = postProcess_Att(t, z, problem)
    
    % Let's verify loop.
    u = zeros(length(t), 6);
    J = diag([problem.constants.Jx, problem.constants.Jy, problem.constants.Jz]);
    Jinv = inv(J);
    
    for i = 1:length(t)
        L = z(i, 1:13)';
        u_acc = -L(4:6);
        L_omega = L(11:13);
        u_torque = -Jinv * L_omega;
        u(i,:) = [u_acc; u_torque]';
    end
end

function plot_Att(t, z, u, problem)
    pos = z(:, 14:16);
    q = z(:, 20:23);
    
    figure;
    subplot(3,1,1); plot(t, pos); ylabel('Position'); title('Attitude+Orbit Results'); legend('x','y','z');
    subplot(3,1,2); plot(t, q); ylabel('Quaternions'); legend('q1','q2','q3','q4');
    subplot(3,1,3); plot(t, u(:,4:6)); ylabel('Torque'); legend('tx','ty','tz');
end
