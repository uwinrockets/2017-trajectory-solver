function [Thrust,Mass,fuelBurned] = launchphases(motorSpecs,motorSelection,totalMass,t,t_burn,slope,y_intercept)
% The different phases the rocket will go through during flight, from
% initial launch, to apogee, and recovery.
      
% Burn Phase
    if t <= max(t_burn)
        for i = 1:length(t_burn)-1    
            if t >= t_burn(i) && t < t_burn(i+1) 
                Thrust = slope(i)*t + y_intercept(i);
            else
            end
        end
        fuelBurned = motorSpecs(motorSelection,4)*t/max(t_burn); 
        
% Coast Phase
    elseif t >= max(t_burn)                        
        Thrust = 0;
        fuelBurned = motorSpecs(motorSelection,4);
    end

        Mass = totalMass - fuelBurned;    
        
end

