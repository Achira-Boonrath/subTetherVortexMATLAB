function run_2body_constrained
    % Wrapper for the constrained thrust 2-body problem
    
    %% 1. Parameters
    muVal = 398600;
    r0 = 780 + 6378;
    rf = 1770 + 6378;
    aTrans = (r0 + rf)/2;
    period = 2*pi*sqrt( (aTrans^3) /muVal  );
    tf = period*0.5;
    
    x0 = [r0 0 0 0 sqrt(muVal/r0) 0 1000]'; % [r v m]
    xf = [-rf 0 0 0 -sqrt(muVal/rf) 0 800]'; % Mass free
    
    problem.x0 = x0;
    problem.xf = xf;
    problem.tf = tf;
    problem.lambda0_guess = 1e-5 * ones(7,1);
    
    consts.muVal = muVal;
    consts.Tmax = 425;
    consts.Isp = 230;
    consts.g0 = 9.81;
    consts.epsilon = 1;
    problem.constants = consts;
    
    %% 2. Setup
    problem.setupSyms = @() setupSyms_Constrained(consts);
    
    % Custom ODE function to handle switching logic explicitly
    problem.odeFunc = @(t, z, ~) ode_Constrained(t, z, consts);
    
    % Terminal Constraint: Match position (1:3) and velocity (4:6). Ignore mass (7) and costates.
    problem.terminalConst = @(z_tf, xf) [z_tf(8:13) - xf(1:6); z_tf(7)]; 
    
    problem.postProcess = @postProcess_Constrained;
    problem.plotFn = @plot_Constrained;
    problem.useParticleSwarm = true; 
    
    %% 3. Run
    [t, z, u, problem] = optControl_singleShoot_Unified(problem);
    
end

function [Lvec, X, Xnum, U, f, V, optUSet, symParams] = setupSyms_Constrained(c)
    syms x y z vx vy vz mC real
    syms ax ay az uThrottle real
    syms muEarth Tmax Isp g0 real
    
    X = [x; y; z; vx; vy; vz; mC];
    syms x1 x2 x3 x4 x5 x6 x7 real
    Xnum = [x1; x2; x3; x4; x5; x6; x7];
    U = [ax; ay; az; uThrottle];
    
    Lvec = sym('L', [7 1], 'real');
    
    r_sq = x^2 + y^2 + z^2;
    f = [vx; vy; vz; ...
        -x*muEarth/(r_sq^(3/2)) + ax*(Tmax/mC)*uThrottle; ...
        -y*muEarth/(r_sq^(3/2)) + ay*(Tmax/mC)*uThrottle; ...
        -z*muEarth/(r_sq^(3/2)) + az*(Tmax/mC)*uThrottle; ...
        -(Tmax/Isp*g0)*uThrottle];
        
    V = 0.5 * uThrottle^2; % Dummy V? Original used U(end)'*U(end)
    
    % Optimal Control Direction (Symbolic placeholder)
    % Logic is handled in ODE, but for completeness:
    optUSet = []; 
    
    symParams.muEarth = c.muVal;
    symParams.Tmax = c.Tmax;
    symParams.Isp = c.Isp;
    symParams.g0 = c.g0;
end

function dz = ode_Constrained(t, s, c)
    % s = [lambda; x]
    % Matches hamiltonian_odeConstT logic
    L = s(1:7);
    x = s(8:end);
    
    muEarth = c.muVal; Tmax = c.Tmax; Isp = c.Isp; g0 = c.g0; epsilon = c.epsilon;
    
    nLv = (L(4)^2 + L(5)^2 + L(6)^2);
    nPos = (x(1)^2 + x(2)^2 + x(3)^2);
    r_mag = sqrt(nPos);
    
    rho = 1 - (Isp * g0 * sqrt(nLv))/(x(7)) - L(7);
    
    uSwitch = 0.5 - rho/(2*epsilon);
    if rho > epsilon
        uSwitch = 0;
    elseif rho < -epsilon
        uSwitch = 1;
    end
    
    Tmag = Tmax * uSwitch;
    
    ds = zeros(14,1);
    
    % Dynamics equations (Manual implementation for speed/correctness as per original)
    % Costates
    force_term = 3*muEarth/r_mag^5;
    ds(1) = -( -force_term*x(1)*(L(4)*x(1) + L(5)*x(2) + L(6)*x(3)) + muEarth/r_mag^3*L(4) ); 
    
    % Re-implementing original lines 304-323
    % ds(1)..ds(3) are -dL/dx (adjoint)
    term1 = (2*L(4)*x(1)^2 + 3*L(5)*x(1)*x(2) + 3*L(6)*x(1)*x(3) - L(4)*x(2)^2 - L(4)*x(3)^2);
    ds(1) = -(muEarth*term1)/nPos^(5/2);
    
    term2 = (- L(5)*x(1)^2 + 3*L(4)*x(1)*x(2) + 2*L(5)*x(2)^2 + 3*L(6)*x(2)*x(3) - L(5)*x(3)^2);
    ds(2) = -(muEarth*term2)/nPos^(5/2);
    
    term3 = (- L(6)*x(1)^2 + 3*L(4)*x(1)*x(3) - L(6)*x(2)^2 + 3*L(5)*x(2)*x(3) + 2*L(6)*x(3)^2);
    ds(3) = -(muEarth*term3)/nPos^(5/2);
    
    ds(4) = -L(1);
    ds(5) = -L(2);
    ds(6) = -L(3);
    
    ds(7) = -(Tmag*sqrt(nLv))/x(7)^2;
    
    % States
    ds(8) = x(4);
    ds(9) = x(5);
    ds(10)= x(6);
    
    L_v = L(4:6);
    nLv_val = norm(L_v);
    
    if nLv_val ~= 0 
        u_dir = -L_v / nLv_val;
    else
        u_dir = [0;0;0];
    end
    
    acc_thrust = (Tmag / x(7)) * u_dir;
    gravity = -muEarth * x(1:3) / r_mag^3;
    
    ds(11:13) = gravity + acc_thrust;
    ds(14) = -(Tmag / (Isp * g0));
    
    ds(1) = -( -force_term*x(1)*(L(4)*x(1) + L(5)*x(2) + L(6)*x(3)) + muEarth/r_mag^3*L(4) ); % dL1/dt
    
    ds = [ds(1); ds(2); ds(3); ds(4); ds(5); ds(6); ds(7); ...
          ds(8); ds(9); ds(10); ds(11); ds(12); ds(13); ds(14)];
end

function u = postProcess_Constrained(t, z, problem)
    consts = problem.constants;
    u = zeros(length(t), 4);
    for i = 1:length(t)
        L = z(i, 1:7)';
        x = z(i, 8:14)';
        
        L_v = L(4:6);
        nLv = norm(L_v);
        
        if nLv ~= 0
            curr_u = -L_v / nLv;
        else
            curr_u = [0;0;0];
        end
        
        eta = 1 - consts.Isp * consts.g0 * nLv / x(7) - L(7);
        sw = 0.5 - eta/(2*consts.epsilon);
        if eta > consts.epsilon, sw=0; elseif eta < -consts.epsilon, sw=1; end
        
        u(i,:) = [curr_u', sw];
    end
end

function plot_Constrained(t, z, u, problem)
    x = z(:, 8:10); % Pos
    figure;
    subplot(3,1,1); plot(t, x(:,1)); ylabel('x'); title('Constrained Thrust Results');
    subplot(3,1,2); plot(t, x(:,2)); ylabel('y');
    subplot(3,1,3); plot(t, u(:,4)); ylabel('Throttle'); xlabel('Time');
end
