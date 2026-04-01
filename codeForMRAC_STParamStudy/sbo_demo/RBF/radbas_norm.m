function d = radbas_norm(dx1x2,C,KernelType)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes the basis function terms (based on given Euclidean
% distance between two points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Souma Chowdhury
% soumacho@buffalo.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dx1x2:      distance between points x1 and x2 (generally Euclidean norm)
% C:          RBF shape parameter
% KernelType: Type of Kernel function (Linear/Cubic/Tps/Gaussian/MultiQ)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch(KernelType)
    
    case 'Linear'
    d = dx1x2;
    case 'Cubic'
    d = (dx1x2)^3;
    case 'Tps'
    % Thin plate spline
    r=(dx1x2);
        if r < 1e-200
        d=0;
        else   
        d = r^2*log(r);
        end
    case 'Gaussian'
    d = exp(-(dx1x2)^2/(2*C^2));  
    case 'Multiquadric'
    d = ((dx1x2)^2+C^2)^0.5;
end