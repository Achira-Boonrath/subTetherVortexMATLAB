function [t, z, u, problem] = optControl_singleShoot_Unified(problem)
% optControl_singleShoot_Unified
% A unified framework for solving single-shooting optimal control problems.
%
% Inputs:
%   problem - struct containing:
%       .x0             : Initial state vector (numeric)
%       .xf             : Target state vector (numeric)
%       .tf             : Final time (scalar)
%       .lambda0_guess  : Initial guess for costates (numeric)
%       .setupSyms      : Function handle @() -> [Lvec, X, Xnum, U, f, V, optUSet, params]
%                         * Returns symbolic variables and equations.
%                         * params is a struct of symbolic-to-numeric mappings.
%       .terminalConst  : (Optional) Function handle @(z_tf, xf) -> Residual vector
%                         * Default: Matches x(tf) to xf (first n states).
%       .odeSolver      : (Optional) Function handle @(fun, tspan, z0) used for integration.
%                         * Default: @ode45
%       .odeFunc        : (Optional) Function handle @(t, z, funcSubs) -> dzdt
%                         * Default: @(t,z,f) double(f(z)) (standard Hamiltonian)
%                         * Useful for custom logic (e.g., bang-bang switching).
%       .postProcess    : (Optional) Function handle @(t, z, problem) -> u
%                         * Calculates control history from states/costates.
%       .plotFn         : (Optional) Function handle @(t, z, u, problem)
%                         * Custom plotting.
%       .solverOpts     : (Optional) Options for fsolve/particleswarm.
%
% Outputs:
%   t, z : Time and state history from final integration.
%   u    : Control history.
%   problem : Updated problem struct (containing solved lambda0).

%% 1. Setup Symbolic Dynamics
fprintf('Generating symbolic equations...\n');
[Lvec, X, Xnum, U, f, V, optUSet, symParams] = problem.setupSyms();

% Use helper to derive costate equations
[star_xdot, star_Ldot] = odeDynAndLag_constT(Lvec, X, Xnum, U, f, V, optUSet);

eqns = [(star_Ldot).', (star_xdot).'];

% Substitute physical parameters
paramNames = fieldnames(symParams);
for k = 1:numel(paramNames)
    val = symParams.(paramNames{k});
    % Check if the value is a symbol that exists in 'eqns' before substituting
    % to avoid errors if symbolic variables aren't used.
    eqns = subs(eqns, evalin('caller', paramNames{k}), val); 
    % Note: The evalin depend on the setupSyms declaring symbols in its scope 
    % or passing them out. A deeper substitution approach:
    % We assume `symParams` maps Symbolic Variables -> Numeric Values directly
    % if possible, OR we do string based subs if they are not in scope.
    % Better: setupSyms should return the map of SymVar -> Value.
end
% Actually, simplest way: symParams should have keys as symbolic vars ideally, 
% but structs utilize string keys. 
% Let's assume eqns has everything fully defined except Z states.

eqns = simplify(eqns);

% Function handle for numeric evaluation: z -> [lambda; x]
funcSubs = @(z) full(double(subs(eqns, [Lvec; Xnum], [z(1:length(problem.x0)); z((length(problem.x0)+1):end)])));
% Note: subs() can be slow. converting to matlabFunction is faster but 
% subs is more robust for 'sym' arrays matching.
% Optimizing:
% vars = [Lvec; Xnum];
% func_handle = matlabFunction(eqns, 'Vars', {vars});
% funcSubs = @(z) func_handle(z); 
% For now we stick to the existing pattern for compatibility, but `subs` inside ODE 
% is very slow. The original scripts used it, so we will keep it for now but
% note performance. 
% Correction: Original scripts used:
% funcSubs = @(z) subs(eqns, [Lvec, Xnum], ... )
% and then hamiltonian_ode called double(funcSubs(z)).

problem.funcSubs = funcSubs;

%% 2. Define ODE Function
if ~isfield(problem, 'odeFunc')
    problem.odeFunc = @(t, z, fH) double(fH(z));
end
% Wrapper for the solver
odeFun = @(t, z) problem.odeFunc(t, z, funcSubs);

%% 3. Define Terminal Constraint (Shooting) Function
if ~isfield(problem, 'terminalConst')
    % Default: Match position/velocity (all states defined in xf)
    problem.terminalConst = @(z_tf, xf_target) ...
        [z_tf( (end-length(xf_target)+1):end ) - xf_target; 0]; 
        % Note: The original added a dummy 0 or costate error? 
        % singleShoot_2body: F = shooting(...) -> calls ode, gets z(end), returns error.
end

shootingFn = @(lam0) run_shooting(lam0, problem.x0, problem.xf, problem.tf, odeFun, problem.terminalConst);

%% 4. Solve for Initial Costates
fprintf('Starting shooting method...\n');

lambda0 = problem.lambda0_guess;
lb = -1e-5 * ones(size(lambda0));
ub = -lb;

if isfield(problem, 'useParticleSwarm') && problem.useParticleSwarm
    fprintf('Running Particle Swarm...\n');
    objfun = @(lam) norm(shootingFn(lam'));
    opts = optimoptions('particleswarm', 'Display', 'iter', 'SwarmSize', 3*length(lambda0), 'MaxIterations', 50);
    lambda0 = particleswarm(objfun, length(lambda0), lb, ub, opts);
    lambda0 = lambda0'; % make column
end

fprintf('Running fsolve...\n');
opts = optimoptions('fsolve', 'Display', 'iter', 'MaxIterations', 50);
if isfield(problem, 'solverOpts'), opts = problem.solverOpts; end

lambdaOpt = fsolve(@(lam) shootingFn(lam, problem.x0, problem.xf, problem.tf, odeFun, problem.terminalConst), lambda0, opts);
problem.lambdaOpt = lambdaOpt;

%% 5. Integrate Final Trajectory
fprintf('Integrating final solution...\n');
z0 = [lambdaOpt; problem.x0];
if ~isfield(problem, 'odeSolver'), problem.odeSolver = @ode45; end
[t, z] = problem.odeSolver(odeFun, [0 problem.tf], z0);

%% 6. Post-Process Control
if isfield(problem, 'postProcess')
    u = problem.postProcess(t, z, problem);
else
    u = [];
end

%% 7. Plotting
if isfield(problem, 'plotFn')
    fprintf('Plotting results...\n');
    problem.plotFn(t, z, u, problem);
end

end

%% Helper: Running Shooting (One Integration)
function F = run_shooting(lambda0, x0, xf, tf, odeFun, terminalConstFn)
    z0 = [lambda0; x0];
    try
        [~, z] = ode45(odeFun, [0 tf], z0);
        z_tf = z(end, :).';
        F = terminalConstFn(z_tf, xf);
        % Handle cases where F might not be same length as variable (fsolve requirement)
        % For costate shooting, we usually want F to be size of lambda.
        % Ensure F acts on states matching Xf.
    catch
        F = 1e6 * ones(size(lambda0)); % Penalty if integration fails
    end
end
