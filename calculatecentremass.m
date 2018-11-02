function [centreMass] = calculatecentremass(components,initialFuel,fuelBurned,totMass)
% Centre of mass calculation for the rocket, based on the given geometries
% inputted into the RocketDimensions excel sheet. Calculates based on
% inner component lengths and weights, and is all relative to the nose tip.

components(20,1) = initialFuel - fuelBurned;
centreMass = sum(components(:,1).*components(:,4))/totMass;

end

