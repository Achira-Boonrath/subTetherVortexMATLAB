function drawCube(xc, yc, zc, Lside, RotMat) 
    % xc=XV(hh,end-3); yc=YV(hh,end-3); zc=ZV(hh,end-3);    % coordinated of the center
    L = Lside;
    W = Lside;
    H = Lside;
    % alpha=0.5;           % transparency (max=1=opaque)
    
    X_e = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
    Y_e = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
    Z_e = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];
    
    C=[0.7 0.8 1];                  % unicolor
    
    % X_e = L*(X_e-0.5) + xc;
    % Y_e = W*(Y_e-0.5) + yc;
    % Z_e = H*(Z_e-0.5) + zc;

    X_e = L*(X_e-0.5) ;
    Y_e = W*(Y_e-0.5) ;
    Z_e = H*(Z_e-0.5) ; 

    xyzCyRow1 = [X_e(1,:);Y_e(1,:);Z_e(1,:)];
    xyzCyRow2 = [X_e(2,:);Y_e(2,:);Z_e(2,:)];
    xyzCyRow3 = [X_e(3,:);Y_e(3,:);Z_e(3,:)];
    xyzCyRow4 = [X_e(4,:);Y_e(4,:);Z_e(4,:)];

    rotRow1 = RotMat*xyzCyRow1;
    rotRow2 = RotMat*xyzCyRow2;
    rotRow3 = RotMat*xyzCyRow3;
    rotRow4 = RotMat*xyzCyRow4;

    X_eFinal = [rotRow1(1,:);rotRow2(1,:);rotRow3(1,:);rotRow4(1,:)]+ xc;
    Y_eFinal = [rotRow1(2,:);rotRow2(2,:);rotRow3(2,:);rotRow4(2,:)]+ yc;
    Z_eFinal = [rotRow1(3,:);rotRow2(3,:);rotRow3(3,:);rotRow4(3,:)]+ zc; 

    fill3(X_eFinal,Y_eFinal,Z_eFinal,C)

    % direction = [1 0 0];
    % rotate(fill3(X_e,Y_e,Z_e,C),direction,0);    % draw cube

end