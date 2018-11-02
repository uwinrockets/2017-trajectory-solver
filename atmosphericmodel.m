function [airTemp,airDensity,airPressure,dynamicViscosity,soundSpeed,g] = atmosphericmodel(yPosition,groundTemp,lapseRate,gamma,R,groundPressure,initialPosition)

    % Constants
    Mo = 0.0289644;
    Ru = 8.3142;
    g_0 = 9.79143;
    r_earth = 6371000;
    % Gravity
    g = g_0*(r_earth/(r_earth + yPosition))^2;
    
    % Air Temperature
    airTemp = groundTemp + lapseRate*(yPosition - initialPosition);

    % Air Pressure
    airPressure = groundPressure*(groundTemp/airTemp)^((g*Mo)/(Ru*lapseRate));

    % Air Density
    airDensity = airPressure/(R*airTemp);

    % Dynamic Viscosity (Sutherland's formula)
    dynamicViscosity = 0.1456E-5*(sqrt(airTemp)/(1 + 110/airTemp));

    % Local Speed of Sound
    soundSpeed = sqrt(gamma*R*airTemp);
    
end

