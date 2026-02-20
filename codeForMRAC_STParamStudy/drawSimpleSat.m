function drawSimpleSat(targetPos, targetInfo, rotMat, k, ColorSide, ColorBase)
% draws the simple sattelite model using the parameters extracted from target info    

    rectLengthx = targetPos(18);
    rectHeightz = targetPos(19);
    rectWidthy = targetPos(20);
    rect2Lengthx = targetPos(21);
    rect2Heightz = targetPos(22);
    rect2Widthy = targetPos(23);
    rect3Lengthx = targetPos(24);
    rect3Heightz = targetPos(25);
    rect3Widthy = targetPos(26);
    RotMat = rotMat;
    
   % Center rect. prism labeled obj. 1
    Lside3 = rectHeightz; % bz length, m
    Lside2 = rectWidthy; % by length, m
    Lside1 = rectLengthx; % bx length, m
    RotMat =rotMat';
    drawBox(targetInfo(k,1), targetInfo(k,2), targetInfo(k,3), Lside1, Lside2, Lside3, RotMat)

    % side cube on the positive x-side (inertial) labeled obj. 2
    sidePos1 = [(0.5*(rectLengthx + rect2Lengthx)); 0; 0];
    rotSidePos1 = RotMat * sidePos1;
    drawBox(targetInfo(k,1) + rotSidePos1(1,1), targetInfo(k,2) + rotSidePos1(2,1), targetInfo(k,3) + rotSidePos1(3,1), rect2Lengthx, rect2Widthy, rect2Heightz, RotMat)

    % side cube on the neagtive x-side (inertial) labeled obj. 3
    sidePos2 = [-(0.5*(rectLengthx + rect2Lengthx)); 0; 0];
    rotSidePos2 = RotMat * sidePos2;
    drawBox(targetInfo(k,1) + rotSidePos2(1,1), targetInfo(k,2) + rotSidePos2(2,1), targetInfo(k,3) + rotSidePos2(3,1), rect3Lengthx, rect3Widthy, rect3Heightz, RotMat)
end