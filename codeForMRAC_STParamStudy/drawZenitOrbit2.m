function drawZenitOrbit2(targetPos, targetInfo, RotMat_H_A_I, rotMat, k, ColorSide, ColorBase, alphaFace)
% Draw the apollo target from extracted target information

    rotMat_D_A_I = rotMat;

    %% Quick mafs
    cylinderRadius = 1.95;
    mainEngineRadius = 1.0;
    smallEngineRadius = 0.46; % Nomrally 0.3, adjusted to accomodate single inner node
    mainEngineHeight = 0.8;
    smallEngineHeight = 0.8;

    % Initializing dimension values for cylinders
    cylinderHeight1 = 10.2; % height of body cylinder
    cylinderHeight3 = mainEngineHeight; % height of main engine nozzle
    cylinderHeight4 = smallEngineHeight; % height of smaller engine nozzles
    
    cylinderRadius1 = cylinderRadius; % radius of body cylinder
    cylinderRadius3 = mainEngineRadius; % radius of main engine nozzle
    cylinderRadius4 = smallEngineRadius; % radius of smaller engine nozzles
    
       % hCyHalf = 0.5*cylinderHeight1 % half height of main cylinder body
    distFromCom2 = -5.7170 + 0.5*mainEngineHeight; % distance along the Z-axis from CoM of main body to main engine nozzle
    distFromCom3 = -5.7170 + 0.5*smallEngineHeight; % distance along the Z-axis  from CoM of main body to small engine nozzles
    posSmEng_R = sqrt(((0.8*cylinderRadius)^2)/2); % distance magnitude for small engine nozzles in the x-axis AND y-axis
    
    % Main Fuselage
    mb_pos = [[0.0]; [0.0]; [0.183]];
    
    % Main Engine
    % Defining position of this cylinder relative to the CoM
    bfDistFromCom6 = [[0.0]; [0.0]; [distFromCom2]];
    
    % Small Engine 1
    % Defining position of this cylinder relative to the CoM
    bfDistFromCom7 = [[posSmEng_R]; [posSmEng_R]; [distFromCom3]];
    
    % Small Engine 2
    % Defining position of this cylinder relative to the CoM
    bfDistFromCom8 = [[posSmEng_R]; [-posSmEng_R]; [distFromCom3]];
    
    % Small Engine 3
    % Defining position of this cylinder relative to the CoM
    bfDistFromCom9 = [[-posSmEng_R]; [posSmEng_R]; [distFromCom3]];
    
    % Small Engine 4
    % Defining position of this cylinder relative to the CoM
    bfDistFromCom10 = [[-posSmEng_R]; [-posSmEng_R]; [distFromCom3]];

        targetInfo0(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  mb_pos)';
        targetInfo2(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bfDistFromCom6)';
        targetInfo3(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bfDistFromCom7)';
        targetInfo4(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bfDistFromCom8)';
        targetInfo5(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bfDistFromCom9)';
        targetInfo6(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bfDistFromCom10)';

    %% Main body
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius1);
        ZtopCylinder = targetInfo0(k,3) + cylinderHeight1/2;
        ZbottomCylinder = targetInfo0(k,3) - cylinderHeight1/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo0(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo0(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo0(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo0(k,3);
        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);

    %% Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius3);
        ZtopCylinder = targetInfo2(k,3) + cylinderHeight3/2;
        ZbottomCylinder = targetInfo2(k,3) - cylinderHeight3/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo2(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo2(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo2(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo2(k,3);

        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);


    %% Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius4);
        ZtopCylinder = targetInfo3(k,3) + cylinderHeight4/2;
        ZbottomCylinder = targetInfo3(k,3) - cylinderHeight4/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo3(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo3(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo3(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo3(k,3);

        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);


    %% Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius4);
        ZtopCylinder = targetInfo4(k,3) + cylinderHeight4/2;
        ZbottomCylinder = targetInfo4(k,3) - cylinderHeight4/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo4(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo4(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo4(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo4(k,3);

        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);


    %% Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius4);
        ZtopCylinder = targetInfo5(k,3) + cylinderHeight4/2;
        ZbottomCylinder = targetInfo5(k,3) - cylinderHeight4/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo5(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo5(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo5(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo5(k,3);

        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);


    %% Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(cylinderRadius4);
        ZtopCylinder = targetInfo6(k,3) + cylinderHeight4/2;
        ZbottomCylinder = targetInfo6(k,3) - cylinderHeight4/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo6(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = RotMat_H_A_I*rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo6(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo6(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo6(k,3);

        % 
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide,'FaceAlpha',alphaFace, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase,'FaceAlpha',alphaFace);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase,'FaceAlpha',alphaFace);



end