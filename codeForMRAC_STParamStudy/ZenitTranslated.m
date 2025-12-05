function ZenitOut = ZenitTranslated(Zenit,Rpos)

    % Translates Zenit by Rpos.
    
    if length(Rpos) > 3
        error('Input vector too large.')
    elseif size(Rpos,2) == 3
        Rpos = Rpos';
    end 
    ZenitOut = Zenit;
    X1 = Zenit.body.cylinder.main.X;
    Y1 = Zenit.body.cylinder.main.Y;
    Z1 = Zenit.body.cylinder.main.Z;
    N = Zenit.body.cylinder.main.N;
    CoM = Zenit.body.CoM;
     
    X2 = Zenit.body.cylinder.secondary.X;
    Y2 = Zenit.body.cylinder.secondary.Y;
    Z2 = Zenit.body.cylinder.secondary.Z;
    
    X3 = Zenit.body.cylinder.tertiary(1).X;
    Y3 = Zenit.body.cylinder.tertiary(1).Y;
    Z3 = Zenit.body.cylinder.tertiary(1).Z;
    
    X4 = Zenit.body.cylinder.tertiary(2).X;
    Y4 = Zenit.body.cylinder.tertiary(2).Y;
    Z4 = Zenit.body.cylinder.tertiary(2).Z;
    
    X5 = Zenit.body.cylinder.tertiary(3).X;
    Y5 = Zenit.body.cylinder.tertiary(3).Y;
    Z5 = Zenit.body.cylinder.tertiary(3).Z;
    
    X6 = Zenit.body.cylinder.tertiary(4).X;
    Y6 = Zenit.body.cylinder.tertiary(4).Y;
    Z6 = Zenit.body.cylinder.tertiary(4).Z;
    
    X1 = [X1(1,:) X1(2,:) X2(1,:) X2(2,:) X3(1,:) X3(2,:) X4(1,:) X4(2,:) X5(1,:) X5(2,:) X6(1,:) X6(2,:)];
    Y1 = [Y1(1,:) Y1(2,:) Y2(1,:) Y2(2,:) Y3(1,:) Y3(2,:) Y4(1,:) Y4(2,:) Y5(1,:) Y5(2,:) Y6(1,:) Y6(2,:)];
    Z1 = [Z1(1,:) Z1(2,:) Z2(1,:) Z2(2,:) Z3(1,:) Z3(2,:) Z4(1,:) Z4(2,:) Z5(1,:) Z5(2,:) Z6(1,:) Z6(2,:)];
    
    
    TransPos = [X1;Y1;Z1]+Rpos;
   
    ZenitOut.states.pos = Rpos;
    for ii = 1:6
      
        X = [TransPos(1,2*N*(ii-1)+1:N*(2*ii-1));TransPos(1,(2*ii-1)*N+1:2*N*ii)];
        Y = [TransPos(2,2*N*(ii-1)+1:N*(2*ii-1));TransPos(2,(2*ii-1)*N+1:2*N*ii)];
        Z = [TransPos(3,2*N*(ii-1)+1:N*(2*ii-1));TransPos(3,(2*ii-1)*N+1:2*N*ii)];
        
        if ii == 1
            ZenitOut.body.cylinder.main.X = X;
            ZenitOut.body.cylinder.main.Y = Y;
            ZenitOut.body.cylinder.main.Z = Z;
        elseif ii == 2
            ZenitOut.body.cylinder.secondary.X = X;
            ZenitOut.body.cylinder.secondary.Y = Y;
            ZenitOut.body.cylinder.secondary.Z = Z;
        else
            ZenitOut.body.cylinder.tertiary(ii-2).X = X;
            ZenitOut.body.cylinder.tertiary(ii-2).Y = Y;
            ZenitOut.body.cylinder.tertiary(ii-2).Z = Z;
        end 
            
    end 
       
    


