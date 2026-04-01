% This program is to demo training surrogate models
% and using it for surrogate-based optimization

clear
close all
clc
addpath(genpath(pwd))

%% Defining problem params
% % Branin function... 
% % The number of variables n = 2; -5<x1<10 , 0<x2<15
% % f = (x(2)-(5.1/(4*pi^2))*x(1)^2+5*x(1)/pi-6)^2+10*(1-1/(8*pi))*cos(x(1))+10;
% n = 2;
% LB = [-5,0];
% UB = [10,15];

% Goldstein and Price function 
n = 2;
% LB = [-2,-2];
% UB = [2,2];

LB = [-1.5,-0.5];
UB = [1.5,2.5];


%% Visualizing function (just for illustration)
disp('Creating visuals of the problem function')
x1 = LB(1):0.2:UB(1);
x2 = LB(2):0.2:UB(2);
for i=1:length(x1)
    for j=1:length(x2)
        y_func(j,i) = TestFunction([x1(i) x2(j)]);
    end
end

figure(1);
surf(x1,x2,y_func);
title('Actual Function');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Design of Experiments
disp('Generating samples')
N = 300; % number of training samples
x = lhsdesign(N,n);
for i=1:n
    x_in(:,i) = LB(i) + (UB(i)-LB(i))*x(:,i);
end

for i=1:length(x_in(:,1))
    y_out(i) = TestFunction(x_in(i,:));
end

y_out = y_out';


%% Fitting the QRSM (quadratic response surface)
disp('Fitting QRSM')
beta = rsm(x_in,y_out);


%% Fitting the RBF (radial basis function)
disp('Fitting RBF')
rho = 0.9;
rbf_coeff = rbf(x_in,y_out,rho,'Multiquadric');


%% Fitting the ANN (artificial neural nets)
disp('Fitting ANN')
% net = dlnetwork;
% inputSize = size(x_in,2); % number of variables
% outputSize = size(y_out,2); % number of output functions -- ANN can do MISO and MIMO models
% % Constructing a Feedforward MLP or ReLU network
% layers = [
%     featureInputLayer(inputSize)
%     fullyConnectedLayer(128)
%     reluLayer(Name="relu1")
%     fullyConnectedLayer(128)
%     reluLayer(Name="relu2")
%     % fullyConnectedLayer(20)
%     % reluLayer(Name="relu3")
%     % fullyConnectedLayer(20)
%     % reluLayer(Name="relu4")
%     fullyConnectedLayer(outputSize)];
% net = addLayers(net,layers);
% ann = fitrnet(x_in,y_out,Network=net,Standardize=true);

%ann = fitrnet(x_in,y_out,"Standardize",true,"LayerSizes",[128 128]);
ann = fitrnet(x_in,y_out,"Standardize",true,"LayerSizes",[128 128 128]*1);


%% Plotting the fitted QRS on the test data
disp('Plotting QRS')
for i=1:length(x1)
    for j=1:length(x2)
        xt = [x1(i) x2(j)];
        y_QRS(j,i) = rsm_approx(xt,beta);
    end
end

figure(2);
surf(x1,x2,y_QRS);
title('Trained Quadratic Response Surface');

%% Plotting the fitted RBF on the test data
disp('Plotting RBF')
for i=1:length(x1)
    for j=1:length(x2)
        xt = [x1(i) x2(j)];
        y_RBF(j,i) = rbf_approx(x_in,xt,rbf_coeff,rho,'Multiquadric');
    end
end

figure(3);
surf(x1,x2,y_RBF);
title('Trained Radial Basis Function');

%% Plotting the fitted ANN on the test data
disp('Plotting ANN')
for i=1:length(x1)
    for j=1:length(x2)
        xt = [x1(i) x2(j)];
        y_ANN(j,i) = predict(ann,xt);
    end
end

figure(4);
surf(x1,x2,y_ANN);
title('Trained Artificial Neural Network');


%% optimizing with QRSM model
disp('Running Optimization with QRS')
%options = optimset('Display','off');
%[xopt1,fopt1]=fmincon(@objfunsm_qrs,[0,0],[],[],[],[],LB,UB,@nonlcon,options,beta);
gaOptions = optimoptions('ga', ...
    'OutputFcn', @gaOutputFcn, ...
    'Display', 'off', 'PlotFcn', {@gaplotbestf, @gaplotstopping}, ...
    'FunctionTolerance',1e-06,'ConstraintTolerance',1e-06);
[xopt1, fopt1, exitflagGA1, outputGA1] = ga(@(x) objfunsm_qrs(x, beta), 2, [], [], [],...
    [], LB, UB, @nonlcon, gaOptions);

disp('Optimum given by QRS-based optimization:')
disp(xopt1)
disp(fopt1)


%% optimizing with RBF model
disp('Running Optimization with RBF')
%[xopt2,fopt2]=fmincon(@objfunsm_rbf,[0,0],[],[],[],[],LB,UB,@nonlcon,options,x_in,rbf_coeff,rho);
[xopt2, fopt2, exitflagGA2, outputGA2] = ga(@(x) objfunsm_rbf(x, x_in, rbf_coeff, rho), 2, [], [], [],...
    [], LB, UB, @nonlcon, gaOptions);

disp('Optimum given by RBF-based optimization:')
disp(xopt2)
disp(fopt2)


%% optimizing with ANN model
disp('Running Optimization with ANN')
%[xopt3,fopt3]=fmincon(@objfunsm_ann,[0,0],[],[],[],[],LB,UB,@nonlcon,options,ann);
[xopt3, fopt3, exitflagGA3, outputGA3] = ga(@(x) objfunsm_ann(x, ann), 2, [], [], [],...
    [], LB, UB, @nonlcon, gaOptions);

disp('Optimum given by ANN-based optimization:')
disp(xopt3)
disp(fopt3)
%%

function [state, options, optchanged] = gaOutputFcn(options, state, flag)
    optchanged = false;
    if strcmp(flag, 'iter')
        % Send iteration number to base workspace
        assignin('base', 'currentGeneration', state.Generation);
        
        % Optional: also save best function value
        assignin('base', 'bestFval', state.Best(end));
        
        % fprintf('Generation: %d | Best fval: %.6f\n', ...
        %         state.Generation, state.Best(end));
    end
end

function stop = psoOutputFcn(optimValues, state)
    stop = false;
    if strcmp(state, 'iter')
        % Send iteration number to base workspace
        assignin('base', 'currentIteration', optimValues.iteration);
        
        % Optional: also save best function value
        assignin('base', 'bestFval', optimValues.bestfval);
        
        fprintf('Iteration: %d | Best fval: %.6f\n', ...
                optimValues.iteration, optimValues.bestfval);
    end
end











