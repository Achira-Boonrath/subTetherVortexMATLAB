function drawApolloOrbit2(targetPos, targetInfo, RotMat_H_A_I, rotMat, k, ColorSide, ColorBase)
% Draw the apollo target from extracted target information

    rectLengthx = targetPos(18);
    rectHeightz = targetPos(19);
    rectWidthy = targetPos(20);
    apolloBodyCylRadius = targetPos(28);
    apolloBodyCylHeight = targetPos(29);
    apolloEngineRadius = targetPos(30);
    apolloEngineHeight = targetPos(31);
    apolloBigNoseSphere = targetPos(32);
    apolloSmallNoseSphere = targetPos(33);
    rotMat_D_A_I = rotMat;

    % Quick mafs
        mb_pos = [-0.04610688, 0.01510257, 0.03870575]';
        eng_pos = [-0.04520139, 0.03137187, -2.27177424]';
        bigN_pos = [-0.02407987, 0.00752071, 2.45431766]';
        smN_pos = [-0.00916107, 0.01307450, 4.07125763]';
        cube_pos = [0.46561794,-0.65806733,-3.14877350]';

        eng_pos_rel = eng_pos;
        bigN_pos_rel = bigN_pos;
        smN_pos_rel = smN_pos;
        cube_pos_rel = cube_pos;

        targetInfo0(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  mb_pos)';
        targetInfo2(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  eng_pos_rel)';
        targetInfo3(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  bigN_pos_rel)';
        targetInfo4(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  smN_pos_rel)';
        targetInfo5(k,1:3) = targetInfo(k,1:3) + (RotMat_H_A_I*rotMat_D_A_I' *  cube_pos_rel)';


    % Main body
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(apolloBodyCylRadius);
        ZtopCylinder = targetInfo0(k,3) + apolloBodyCylHeight/2;
        ZbottomCylinder = targetInfo0(k,3) - apolloBodyCylHeight/2;
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
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);

    % Engine
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(apolloEngineRadius);
        ZtopCylinder = targetInfo2(k,3) + apolloEngineHeight/2;
        ZbottomCylinder = targetInfo2(k,3) - apolloEngineHeight/2;
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
        hSurf = surf(Xplot_cylinder,Yplot_cylinder,Zplot_cylinder, ColorSide, 'EdgeColor','none','LineStyle','none','FaceLighting','phong');
        fill3(Xplot_cylinder(1,:), Yplot_cylinder(1,:), Zplot_cylinder(1,:), ColorBase);
        fill3(Xplot_cylinder(2,:), Yplot_cylinder(2,:), Zplot_cylinder(2,:), ColorBase);

    % Lower nose cone
        drawSphereSpn(50, apolloBigNoseSphere, targetInfo3(k,1), targetInfo3(k,2), targetInfo3(k,3), R)
    
    % Upper nose cone
        drawSphereSpn(50, apolloSmallNoseSphere, targetInfo4(k,1), targetInfo4(k,2), targetInfo4(k,3), R)

    % Side cube
        Lside3 = rectHeightz; % bz length, m
        Lside2 = rectWidthy; % by length, m
        Lside1 = rectLengthx; % bx length, m
        RotMat = RotMat_H_A_I*rotMat_D_A_I'*rotz(-52.13142946);
        drawBox(targetInfo5(k,1), targetInfo5(k,2), targetInfo5(k,3), Lside1, Lside2, Lside3, RotMat)

end