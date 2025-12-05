function ZenitPlot(Zenit,C)

    % Plots Zenit using the positions in the Zenit Struct. The input C
    % checks if the figure is currently open or not. Set it to 0 for a plot
    % that is currently open, or 1 for a new plot.
    
    
    ColorBase = [0.5 0.5 0.5];
    
    
    if C > 0
        figure;
        hold on;
    end 
    
    for ii = 1:6
      
        
        if ii == 1
             X = Zenit.body.cylinder.main.X;
             Y = Zenit.body.cylinder.main.Y;
             Z = Zenit.body.cylinder.main.Z;
        elseif ii == 2
            X = Zenit.body.cylinder.secondary.X;
            Y = Zenit.body.cylinder.secondary.Y;
            Z = Zenit.body.cylinder.secondary.Z;
        else
           X = Zenit.body.cylinder.tertiary(ii-2).X;
           Y = Zenit.body.cylinder.tertiary(ii-2).Y;
           Z = Zenit.body.cylinder.tertiary(ii-2).Z;
        end 
        % surf(X,Y,Z,'facecolor',[0.7 0.9 1 ],'LineStyle','none');
        surf(X,Y,Z,'facecolor',ColorBase,'LineStyle','none');
        % alpha 0.5
        alpha 0.7
        % fill3(X(1,:),Y(1,:),Z(1,:),[0.7 0.9 1 ])
        % fill3(X(2,:),Y(2,:),Z(2,:),[0.7 0.9 1 ])   % draw cube
        fill3(X(1,:),Y(1,:),Z(1,:),ColorBase)
        fill3(X(2,:),Y(2,:),Z(2,:),ColorBase)   % draw cube
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
    end 
    if C > 0
           hold off
    end 
    


