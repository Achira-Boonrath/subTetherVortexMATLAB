function f = rsm_approx(x,beta1)
% This function is used to estimate the function value using Quadratic RSM.

n_dim = size(x,2);

sum1 = beta1(1);

for i = 2:n_dim+1
    sum1 = sum1 + beta1(i)*x(i-1);
end

n = 2 + n_dim;

for i = 1:n_dim
    sum1 = sum1 + beta1(n)*x(i)^2;
    n = n + 1;
end

n = 2 + 2*n_dim;

for i = 1:n_dim-1
    for j = i+1:n_dim
        sum1 = sum1 + beta1(n)*x(i)*x(j);
        n = n + 1;
    end
end

f = sum1;

