function gtot = computeGravJ2_multi(positions, mass, muEarth, args)
% computeGravJ2_multi  Vectorized gravity + J2 for multiple positions
%   positions: 3xN matrix of position vectors (m)
%   mass: scalar mass applied to all positions (kg)
%   muEarth: gravitational parameter (m^3/s^2)
%   args.J2on: toggle for J2 (0/1)

    J2 = 1.08263e-3;
    rEarth = 6378.0e3;

    p = positions; % 3xN
    r = sqrt(sum(p.^2, 1)); % 1xN

    % Avoid division by zero: mark zeros and set safe nonzero radius
    zeroMask = (r <= eps);
    rSafe = r;
    rSafe(zeroMask) = 1.0; % temporary to avoid NaNs

    % central gravity (3xN)
    grav = - muEarth * mass * (p ./ (rSafe.^3));

    % J2 perturbation (3xN)
    coeffT = (3/2) * J2 * muEarth .* (rEarth^2 ./ (rSafe.^4)); % 1xN
    zOverR = p(3, :) ./ rSafe; % 1xN

    J2_x = (5 * (zOverR).^2 - 1) .* (p(1, :) ./ rSafe);
    J2_y = (5 * (zOverR).^2 - 1) .* (p(2, :) ./ rSafe);
    J2_z = (5 * (zOverR).^2 - 3) .* (p(3, :) ./ rSafe);

    J2_vec = args.J2on * mass .* (coeffT .* [J2_x; J2_y; J2_z]);

    gtot = grav + J2_vec;

    % Zero-out any entries where original radius was zero
    if any(zeroMask)
        gtot(:, zeroMask) = 0;
    end
end
