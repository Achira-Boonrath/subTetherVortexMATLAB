function drawSphere(n, r, posX, posY, posZ)

    [x, y, z] = sphere(n);

    x = r*x + posX;
    y = r*y + posY;
    z = r*z + posZ;

    coords = [x(:), z(:), y(:)]';

    % rotCoords = rotMat * coords;

    % rotX = reshape(rotCoords(1,:), size(x));
    % rotY = reshape(rotCoords(2,:), size(y));
    % rotZ = reshape(rotCoords(3,:), size(z));
    
    map = [0.5 0.5 0.5];

    % surf(rotX, rotY, rotZ)
    surf(x, y, z, 'EdgeColor','none');
    colormap(map)

end