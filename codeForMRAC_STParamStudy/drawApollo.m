function drawApolloOrbit(targetPos, targetInfo, RotMat_H_A_I, rotMat, k, ColorSide, ColorBase)
% Draw the apollo target from extracted target information

    apolloBodyCylRadius = targetPos(28);
    apolloBodyCylHeight = targetPos(29);
    apolloEngineRadius = targetPos(30);
    apolloEngineHeight = targetPos(31);
    apolloBigNoseSphere = targetPos(32);
    apolloSmallNoseSphere = targetPos(33);
    rotMat_D_A_I = rotMat;

    % Quick mafs
        mb_pos = [0.0, -0.1, -0.3]';
        eng_pos = [0.0, -0.084, -2.61]';
        bigN_pos = [0.0, -.108, 2.116]';
        smN_pos = [0.0, -.102, 3.733]';
        eng_pos_rel = -mb_pos + eng_pos;
        bigN_pos_rel = -mb_pos + bigN_pos;
        smN_pos_rel = -mb_pos + smN_pos;

        targetInfo2(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I' *  eng_pos_rel)';
        targetInfo3(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I' *  bigN_pos_rel)';
        targetInfo4(k,1:3) = targetInfo(k,1:3) + (rotMat_D_A_I' *  smN_pos_rel)';


    % Main body
        [Xplot_cylinder,Yplot_cylinder,Zplot_cylinder] = cylinder(apolloBodyCylRadius);
        ZtopCylinder = targetInfo(k,3) + apolloBodyCylHeight/2;
        ZbottomCylinder = targetInfo(k,3) - apolloBodyCylHeight/2;
        Zplot_cylinder = repmat([ZbottomCylinder; ZtopCylinder],1,length(Xplot_cylinder))-targetInfo(k,3);
        xyzCyRow1 = [Xplot_cylinder(1,:);Yplot_cylinder(1,:);Zplot_cylinder(1,:)];
        xyzCyRow2 = [Xplot_cylinder(2,:);Yplot_cylinder(2,:);Zplot_cylinder(2,:)];
        R = rotMat_D_A_I';
        rotTopRow = R*xyzCyRow1;
        rotLowRow = R*xyzCyRow2;
        Xplot_cylinder = [rotTopRow(1,:);rotLowRow(1,:);]+targetInfo(k,1);
        Yplot_cylinder = [rotTopRow(2,:);rotLowRow(2,:);]+targetInfo(k,2);
        Zplot_cylinder = [rotTopRow(3,:);rotLowRow(3,:);]+targetInfo(k,3);
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
        R = rotMat_D_A_I';
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

end