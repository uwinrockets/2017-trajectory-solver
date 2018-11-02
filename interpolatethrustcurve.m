function [slope,y_intercept] = interpolatethrustcurve(F_thrust,t_burn)

    for i = 1:length(F_thrust)-1
        slope(i) = (F_thrust(i+1)-F_thrust(i))/(t_burn(i+1)-t_burn(i));
        y_intercept(i) = F_thrust(i) - slope(i)*t_burn(i);
    end
    
end