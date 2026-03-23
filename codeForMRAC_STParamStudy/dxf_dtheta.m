
function [F1 , F2] = dxf_dtheta(theta_f, at, et, it, Omega_t, omega_t, mu)
% from transCondFreeXf.m

F1 = [(at*(et^2 - 1)*(et*cos(Omega_t)*sin(omega_t) + cos(Omega_t)*cos(omega_t)*sin(theta_f) + et*sin(Omega_t)*cos(it)*cos(omega_t) - sin(Omega_t)*cos(it)*sin(omega_t)*sin(theta_f)) + at*cos(theta_f)*(cos(Omega_t)*sin(omega_t) + sin(Omega_t)*cos(it)*cos(omega_t))*(et^2 - 1))/(et*cos(theta_f) + 1)^2;...
(at*(et^2 - 1)*(et*sin(Omega_t)*sin(omega_t) + sin(Omega_t)*cos(omega_t)*sin(theta_f) - et*cos(Omega_t)*cos(it)*cos(omega_t) + cos(Omega_t)*cos(it)*sin(omega_t)*sin(theta_f)) + at*cos(theta_f)*(sin(Omega_t)*sin(omega_t) - cos(Omega_t)*cos(it)*cos(omega_t))*(et^2 - 1))/(et*cos(theta_f) + 1)^2;...
    -(at*sin(it)*(et^2 - 1)*(cos(omega_t + theta_f) + et*cos(omega_t)))/(et*cos(theta_f) + 1)^2];

F2 = [((mu/at)^(1/2)*(cos(Omega_t)*sin(omega_t)*sin(theta_f) - cos(Omega_t)*cos(omega_t)*cos(theta_f) + sin(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f) + sin(Omega_t)*cos(it)*cos(theta_f)*sin(omega_t)))/(1 - et^2)^(1/2);...
    -((mu/at)^(1/2)*(sin(Omega_t)*cos(omega_t)*cos(theta_f) - sin(Omega_t)*sin(omega_t)*sin(theta_f) + cos(Omega_t)*cos(it)*cos(omega_t)*sin(theta_f) + cos(Omega_t)*cos(it)*cos(theta_f)*sin(omega_t)))/(1 - et^2)^(1/2);...
    -(sin(omega_t + theta_f)*sin(it)*(mu/at)^(1/2))/(1 - et^2)^(1/2)];

% F= F1 + F2;
end