function drawSphereSpn(n, r, posX, posY, posZ, rotMat)

    [x, y, z] = sphere(n);

    x = r*x;
    y = r*y;
    z = r*z;

    coords = [x(:), y(:), z(:)]';

    rotCoords = rotMat * coords;

    rotX = reshape(rotCoords(1,:), size(x));
    rotY = reshape(rotCoords(2,:), size(y));
    rotZ = reshape(rotCoords(3,:), size(z));
    
    rotX = rotX + posX;
    rotY = rotY + posY;
    rotZ = rotZ + posZ;

    map = [0.5 0.5 0.5];

    surf(rotX, rotY, rotZ, 'EdgeColor','none')
    colormap(map)

end