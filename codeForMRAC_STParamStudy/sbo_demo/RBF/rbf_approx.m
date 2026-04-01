function f = rbf_approx(x_in,x_test,rbf_coeff,C,KernelType)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to execute a trained RBF Interpolation surrogate model.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created by: Ali Mehmani
% ali.mehmani@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% last update: 25-Nov-2016
% Modified by: Souma Chowdhury
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alpha = rbf_coeff;

x = x_in';
x_t = x_test';

n_tr = size(x,2);          % number of data points
%n_dim = size(x,1);        % number of dimensions

n_te = size(x_t,2);
%k = 0;
%rms_sum = 0;

% Evaluate the test data
f = zeros(1,n_te);
for i = 1:n_te        
        sum1 = 0; 
        
        for h = 1:n_tr
            dx1x2 = norm(x_t(:,i)-x(:,h));
            sum1 = sum1 + alpha(h)*radbas_norm(dx1x2,C,KernelType); 
        end  
        
        f(i) = sum1;
end
     
