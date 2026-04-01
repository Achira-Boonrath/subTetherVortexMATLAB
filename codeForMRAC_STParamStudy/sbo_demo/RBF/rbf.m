function rbf_coeff = rbf(x_in,y_out,C,KernelType,varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to train the RBF Interpolation surrogate model.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created by: Ali Mehmani
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% last update: 25-Nov-2016
% Modified by: Souma Chowdhury
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fprintf('Training RBF: %s\n',KernelType)
x = x_in';

n_p = size(x,2);          % number of data points
%n_dim = size(x,1);        % number of dimensions

if nargin == 5,
    index_Tr = varargin{1};
end

%% Compute or read (if already available) the Euclidean dist between pts
if exist('rbf_dist_mat.mat','file') && exist('index_Tr','var')
    load rbf_dist_mat; % load the dx1x2 values if they have already been computed before (saving time)
else
    dx1x2 = zeros(n_p);
    for i = 1:n_p-1
        for j = i+1:n_p
            dx1x2(i,j) = norm(x(:,i)-x(:,j));
            dx1x2(j,i) = dx1x2(i,j);
        end
    end
end

%% Create radial basis function approximation & linear system of equations
a = zeros(n_p);
if exist('index_Tr','var')
    for i = 1:n_p
    % if pre-computed Euclidean dist exist, read index of training points
        ii = index_Tr(i);
        for j = i:n_p
        % if pre-computed Euclidean dist exist, read index of training points
            jj = index_Tr(j);
            a(i,j) = radbas_norm(dx1x2(ii,jj),C,KernelType);
            a(j,i) = a(i,j);
        end
    end
else
    for i = 1:n_p
        % use all points as traning points
        for j = i:n_p
            a(i,j) = radbas_norm(dx1x2(i,j),C,KernelType);
            a(j,i) = a(i,j);
        end
    end
end

%% solving the linear system of equations
rbf_coeff = a\y_out;


