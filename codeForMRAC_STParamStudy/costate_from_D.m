function lambda = costate_from_D(X)
% COSTATE_FROM_X  Compute normalized costates λ(t0) from search variables X.
%
%   lambda = costate_from_X(X)
%
%   INPUT:
%       X : 7xN matrix, each column in [0,1]
%
%   OUTPUT:
%       lambda : 8xN matrix
%                [λ0;
%                 λr (3x1);
%                 λv (3x1);
%                 λm]

% Ensure correct dimension
if size(X,1) ~= 7
    error('Input X must be 7xN.');
end

% ----- Angle variables (Eq. 32) -----

beta = zeros(size(X));

% β1, β2, β3 ∈ [0, π/2]
beta(1:3,:) = (pi/2) .* X(1:3,:);

% β4, β5 ∈ [-π/2, π/2]
beta(4:5,:) = pi .* (X(4:5,:) - 1/2);

% β6, β7 ∈ [0, 2π]
beta(6:7,:) = 2*pi .* X(6:7,:);

% Precompute trig terms
c1 = cos(beta(1,:));  s1 = sin(beta(1,:));
c2 = cos(beta(2,:));  s2 = sin(beta(2,:));
c3 = cos(beta(3,:));  s3 = sin(beta(3,:));
c4 = cos(beta(4,:));  s4 = sin(beta(4,:));
c5 = cos(beta(5,:));  s5 = sin(beta(5,:));
c6 = cos(beta(6,:));  s6 = sin(beta(6,:));
c7 = cos(beta(7,:));  s7 = sin(beta(7,:));

% ----- Costates (Eq. 33) -----

% λ0
lambda0 = s1;

% λr
scale_r = c1 .* c2 .* c3;
lambda_r = scale_r .* [c4 .* c6;
                       c4 .* s6;
                       s4];

% λv
scale_v = c1 .* c2 .* s3;
lambda_v = scale_v .* [c5 .* c7;
                       c5 .* s7;
                       s5];

% λm
lambda_m = c1 .* s2;

% Assemble full vector
lambda = [lambda0;
          lambda_r;
          lambda_v;
          lambda_m];

end
