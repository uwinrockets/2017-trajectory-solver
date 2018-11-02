function [Fx,Fy] = calculateforce(g,Distance,alt,launchRodLength,Mass,Theta,launchAngle,Cd_R,airDensity, ...
    Acs,resultantVelocity,y_velocity,Thrust,Cd_drogue,Cd_main,R_drogue,R_spill,R_main,Deploy_main_alt,db, ...
    dn,lb,ln,lt,ls,lr)
    % Parachute Parameters
    A_drogue = pi*(R_drogue^2 - R_spill^2); % Area of drogue (m^2)
    A_main = pi*R_main^2;           % Area of main chute (m^2)
    
    % Normal Force
    if Distance <= launchRodLength
        Normal = Mass*g*cosd(launchAngle);
    else
        Normal = 0;
    end
    
    % Drag Forces
    Drag_flight = Cd_R*1/2*airDensity*Acs*(resultantVelocity^2);
	Drag_drogue = Cd_drogue*1/2*airDensity*A_drogue*(y_velocity^2);
    Drag_main = Cd_main*1/2*airDensity*A_main*(y_velocity^2);
    
    % Recovery Drag Force
    if y_velocity >= 0
        Drag_recovery = 0;
    elseif y_velocity < 0 && alt >= Deploy_main_alt
        Drag_recovery = Drag_drogue;
        Drag_flight = 0;
    else
        Drag_recovery = Drag_main;
        Drag_flight = 0;
    end 
    
    % Lift
%     L = 
    
    % Wind Forces
    A_side_body = db*lb*sind(Theta);
    A_side_nose = dn*ln*sind(Theta)/2;
    A_side_fin = 2*((lt + lr)*sind(Theta)*ls/2);
    A_side = A_side_body + A_side_nose + A_side_fin;
%     Wind = Cd_side*1/2*airDensity*U_wind^2*A_side;

    % Sum of Forces
    Fx = Thrust*cosd(Theta)-Drag_flight*cosd(Theta)...
        -Normal*sind(Theta); % -Wind;                            
    Fy = Thrust*sind(Theta)-(Mass*g)-...
        Drag_flight*sind(Theta)+Normal*cosd(Theta)+Drag_recovery;