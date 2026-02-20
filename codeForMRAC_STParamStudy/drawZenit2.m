function drawZenit2(targetPos, targetInfo, rotMat, k, ColorSide, ColorBase)
% draws the Zenit-2 stage usign the extracted parameters from target info
    
    cylinderRadius = targetPos(10);
    cylinderHeight = targetPos(11);
    smallCylinderHeight = targetPos(12);
    smallCylinderRadius = targetPos(13);
    mainEngineHeight = targetPos(14);
    mainEngineRadius = targetPos(15);
    smallEngineHeight = targetPos(16);
    smallEngineRadius = targetPos(17);
    rotMat_D_A_I = rotMat;
    
    % Uncomment to draw Cylinder Target (Main Cylinder)
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius);
    ZtopCylinder = targetInfo(k,3) + cylinderHeight/2;
    ZbottomCylinder = targetInfo(k,3) - cylinderHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 2 (Side Cylinder 1)
    cylinderHeight2 = smallCylinderHeight;
    cylinderRadius2 = smallCylinderRadius;
    targetInfo2(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[cylinderRadius+cylinderRadius2,0, -(0.5*cylinderHeight-0.5*cylinderHeight2)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius2);
    ZtopCylinder = targetInfo2(k,3) + cylinderHeight2/2;
    ZbottomCylinder = targetInfo2(k,3) - cylinderHeight2/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo2(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo2(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo2(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo2(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 3 (Side Cylinder 2)
    targetInfo3(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[-cylinderRadius-cylinderRadius2,0, -(0.5*cylinderHeight-0.5*cylinderHeight2)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius2);
    ZtopCylinder = targetInfo3(k,3) + cylinderHeight2/2;
    ZbottomCylinder = targetInfo3(k,3) - cylinderHeight2/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo3(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo3(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo3(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo3(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 4 (Side Cylinder 3)
    targetInfo4(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[0,cylinderRadius+cylinderRadius2, -(0.5*cylinderHeight-0.5*cylinderHeight2)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius2);
    ZtopCylinder = targetInfo4(k,3) + cylinderHeight2/2;
    ZbottomCylinder = targetInfo4(k,3) - cylinderHeight2/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo4(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo4(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo4(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo4(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 5 (Side Cylinder 4)
    targetInfo5(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[0,-cylinderRadius-cylinderRadius2, -(0.5*cylinderHeight-0.5*cylinderHeight2)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius2);
    ZtopCylinder = targetInfo5(k,3) + cylinderHeight2/2;
    ZbottomCylinder = targetInfo5(k,3) - cylinderHeight2/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo5(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo5(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo5(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo5(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 6 (Main Engine)
    targetInfo6(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[0,0, -(0.5*cylinderHeight+0.5*mainEngineHeight)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(mainEngineRadius);
    ZtopCylinder = targetInfo6(k,3) + mainEngineHeight/2;
    ZbottomCylinder = targetInfo6(k,3) - mainEngineHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo6(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo6(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo6(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo6(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 7 (Small Engine 1)
    posSmEng_R = sqrt(((0.8*cylinderRadius)^2)/2);
    targetInfo7(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[posSmEng_R,posSmEng_R, -(0.5*cylinderHeight+0.5*smallEngineHeight)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(smallEngineRadius);
    ZtopCylinder = targetInfo7(k,3) + smallEngineHeight/2;
    ZbottomCylinder = targetInfo7(k,3) - smallEngineHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo7(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo7(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo7(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo7(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 8 (Small Engine 2)
    targetInfo8(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[posSmEng_R,-posSmEng_R, -(0.5*cylinderHeight+0.5*smallEngineHeight)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(smallEngineRadius);
    ZtopCylinder = targetInfo8(k,3) + smallEngineHeight/2;
    ZbottomCylinder = targetInfo8(k,3) - smallEngineHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo8(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo8(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo8(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo8(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 9 (Small Engine 3)
    targetInfo9(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[-posSmEng_R,posSmEng_R, -(0.5*cylinderHeight+0.5*smallEngineHeight)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(smallEngineRadius);
    ZtopCylinder = targetInfo9(k,3) + smallEngineHeight/2;
    ZbottomCylinder = targetInfo9(k,3) - smallEngineHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo9(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo9(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo9(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo9(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
    
    % Uncomment to draw Cylinder Target 10 (Small Engine 4)
    targetInfo10(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I'*[-posSmEng_R,-posSmEng_R, -(0.5*cylinderHeight+0.5*smallEngineHeight)]')';
    
    [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(smallEngineRadius);
    ZtopCylinder = targetInfo10(k,3) + smallEngineHeight/2;
    ZbottomCylinder = targetInfo10(k,3) - smallEngineHeight/2;
    % Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-zCylinder;
    Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo10(k,3);
    xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
    xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
    % R = rotx(90)*roty(45);
    % R = rotx(90)*roty(5*t(k));
    % R = rotx(thetaTargetX_t0 + thetaTargetX_rate*t(k))*roty(thetaTargetY_t0 + thetaTargetY_rate*t(k))*rotz(thetaTargetZ_t0 + thetaTargetZ_rate*t(k));
    R = rotMat_D_A_I';
    rotTopRow = R*xyzCyRow1;
    rotLowRow = R*xyzCyRow2;
    Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo10(k,1);
    Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo10(k,2);
    Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo10(k,3);
    % Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+xCylinder;
    % Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+yCylinder;
    % Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+zCylinder;
    %
    %
    hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
    fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
    fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);
end