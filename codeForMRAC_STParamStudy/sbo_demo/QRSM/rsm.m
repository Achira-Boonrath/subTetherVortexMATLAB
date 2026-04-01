function beta1 = rsm(x_in,y_out)
% This function is used construct the surrogate model using
% quadratic response surface method

x = x_in';
mu = y_out';

n_p = size(x,2);          % number of data points
n_dim = size(x,1);        % number of dimensions

for i = 1:n_p
    A(i,1) = 1;
    
    for j = 1:n_dim
        A(i,j+1) = x(j,i);
        A(i,1+n_dim+j) = x(j,i)^2;
    end
    
    n = 2 + 2*n_dim;
    
    for j = 1:n_dim-1
        for k = j+1:n_dim
            A(i,n) = x(j,i)*x(k,i);
            n = n + 1;
        end
    end     
end

%beta1 = (inv(A'*A))*A'*mu';
beta1 = pinv(A)*mu';



