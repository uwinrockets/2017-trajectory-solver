clear, clc, close all

%% Select a Motor (Based on Excel Sheet)
motorSelection = 14;

%%
%   ///////////////////////////////////////////////////
%   /////////////// IMPORT DATA ///////////////////////
%   ///////////////////////////////////////////////////

components = xlsread('RocketDimensions.xlsx','Components');
finGeometry = xlsread('RocketDimensions.xlsx','FinGeometry');
motorSpecs = xlsread('RocketDimensions.xlsx','MotorSpecs');
inputs = xlsread('RocketDimensions.xlsx','Inputs');
parasiticParameters = xlsread('RocketDimensions.xlsx','ParasiticParameters');
boundaryConditions = xlsread('RocketDimensions.xlsx','Inputs');
recovery = xlsread('RocketDimensions.xlsx','Recovery');
thrustCurve = xlsread('RocketDimensions.xlsx','Thrust Curve');

%%
%   ///////////////////////////////////////////////////
%   /////////////// INITIALIZATION ////////////////////
%   ///////////////////////////////////////////////////

% Technical Parameters
timeStep = 0.003;
Memory_Allocation = 80000;
n = 1;

% Memory Preallocations
t = zeros(1, Memory_Allocation);
thrust = zeros(1, Memory_Allocation);
mass = zeros(1, Memory_Allocation);
theta = zeros(1, Memory_Allocation);
normalForce = zeros(1, Memory_Allocation);
dragForce = zeros(1, Memory_Allocation);
x_force = zeros(1, Memory_Allocation);
y_force = zeros(1, Memory_Allocation);
x_acceleration = zeros(1, Memory_Allocation);
y_acceleration = zeros(1, Memory_Allocation);
resultantAcceleration = zeros(1, Memory_Allocation);
x_velocity = zeros(1, Memory_Allocation);
y_velocity = zeros(1, Memory_Allocation);
resultantVelocity = zeros(1, Memory_Allocation);
x_position = zeros(1, Memory_Allocation);
y_position = zeros(1, Memory_Allocation);
x_distance = zeros(1, Memory_Allocation);
y_distance = zeros(1, Memory_Allocation);
totalDistance = zeros(1, Memory_Allocation);

% Format
format short g

%%
%   ///////////////////////////////////////////////////
%   /////////////// DIMENSIONS ////////////////////////
%   ///////////////////////////////////////////////////

% Nose
ln = components(1,2);
dn = components(1,3);
% syms x_nose
% theta1 = acos(1 - 2*x_nose/ln);
% y_nose = ((dn/2)/sqrt(pi))*sqrt(theta1 - 1/2*sin(2*theta1));
% A_func = int(2*pi*y_nose,0,ln);
% A_nose_surface = vpa(A_func,6);
A_nose_surface = 0.289081;

% Body
lb = components(3,2) + components(4,2);
lc = 0;
lTR = lb + ln; 
db = components(3,3);       
du = db;
dd = db;
Ap_n = ln*dn/2; 
Ap_b = lb*db;   
Acs = (db^2)*pi/4;     

% Fin Dimensions
lt = finGeometry(1);                            
ls = finGeometry(2);                             
lr = finGeometry(3);                            
lw = finGeometry(4);                            
lm = finGeometry(5);    
lTS = finGeometry(6);                        
df = finGeometry(7);                          
wf = finGeometry(8);                           
Xf = finGeometry(9);                       
tf = finGeometry(10);                          
N = finGeometry(11);
Ae_f = finGeometry(12);
Ap_f = finGeometry(13);  

% Exposed Holes, Rail Buttons, Screws/Rivets, Shear Pins, & Surface Roughness Height		
N_hole = parasiticParameters(1);	
N_rail_button = parasiticParameters(2);		
A_rail_button = parasiticParameters(3);		
N_shear_pin = parasiticParameters(4);		
A_shear_pin = parasiticParameters(5);		
N_screw = parasiticParameters(6);		
Rs = parasiticParameters(7);
gamma = 1.4;
R = 287;

% Recovery
Cd_drogue = recovery(1,3);      % Drag coefficient of drogue
Cd_main = recovery(2,3);        % Drag coefficient of main chute
R_drogue = recovery(1,1) ;		% Radius of drogue (m)
R_spill = recovery(1,2);        % Radius of spill hole of drogue (m)
R_main = recovery(2,1);			% Radius main chute (m)

% Wind
% U_wind_mean = 
% xturb = 
% yturb = 
% stdev_turb = 

%%
%   ///////////////////////////////////////////////////
%   /////////////// PARAMETERS ////////////////////////
%   ///////////////////////////////////////////////////

% User Inputs and Initial Values                                      
groundTemp = inputs(1,1);
groundPressure = inputs(1,2);
y_position(1) = inputs(1,3);   
lapseRate = inputs(1,4);
surfaceRoughness = inputs(1,5)/(1e6);
launchRodLength = inputs(1,6);
theta(1) = 90 - inputs(1,7);
Deploy_main_alt = inputs(1,8);  % Altitude at main chute deployment (m)

airTemp(1) = groundTemp;
airPressure(1) = groundPressure;  
airDensity(1) = groundPressure/(R*groundTemp);
speedSound(1) = sqrt(R*gamma*groundTemp);
dynamicViscosity(1) = 0.1456E-5*(sqrt(groundTemp)/(1 + 110/groundTemp));            

x_velocity(1) = 0;              
y_velocity(1) = 0;                                            
x_position(1) = 0;      
alt(1) = 0;
                                                                                        
mass(1) = sum(components(:,1));                      
initialFuel = components(20,1);
centreMass(1) = calculatecentremass(components,initialFuel,0,mass(1));  

% Thrust Curve
t_burn = thrustCurve(3:end,1);
F_thrust = thrustCurve(3:end,2);
%%
%   ///////////////////////////////////////////////////
%   ///////////// ROCKET PHYSICS //////////////////////
%   ///////////////////////////////////////////////////

%% CENTER OF PRESSURE
centrePressure = calculatecentrepressure(df,ls,N,dn,lm,lr,lt,Xf,ln);

%% INTERPOLATE THRUST CURVE
[slope,y_intercept] = interpolatethrustcurve(F_thrust,t_burn);

while y_position(n) >= y_position(1)  
    
    %% TIME STEP
    n = n + 1;
    t(n)= (n - 1)*timeStep;                    
    
    %% ATMOSPHERIC MODEL
    [airTemp(n),airDensity(n),airPressure(n),dynamicViscosity(n),soundSpeed(n),g(n)] = ...
        atmosphericmodel(y_position(n-1),groundTemp,lapseRate,gamma,R,groundPressure,y_position(1));

    % Mach Number Calculation  
    machNo(n) = resultantVelocity(n-1)/speedSound; 

    
    %% LAUNCH PHASES
    [thrust(n),mass(n),fuelBurned(n)] = launchphases(motorSpecs,motorSelection,mass(1),t(n),t_burn,slope,y_intercept);
    
    %% CENTER OF MASS
    centreMass(n) = calculatecentremass(components,initialFuel,fuelBurned(n),mass(n));

    %% COEFFICIENT OF DRAG    
    % Coefficients of Drag
    [coeff_skinDrag(n),coeff_baseDrag(n),coeff_finDrag(n),coeff_interferenceDrag(n),coeff_parasitic(n)] = ...
        coefficientdrag(airDensity(n),dynamicViscosity(n),lTR,ln,lb,lc,lm,db,dd,df,tf,...
        resultantVelocity(n-1),gamma,machNo(n),Ap_f,Ae_f,Acs,A_shear_pin,A_rail_button,N,N_hole,...
        N_rail_button,N_shear_pin,N_screw,surfaceRoughness);   
    
    % Compressibility Correction
    [correctedCoeff_skinDrag(n),correctedCoeff_baseDrag(n),correctedCoeff_finDrag(n),correctedCoeff_interferenceDrag(n),correctedCoeff_parasiticDrag(n)] = ...
        compressibilitycorrection(coeff_skinDrag(n),coeff_baseDrag(n),coeff_finDrag(n),coeff_interferenceDrag(n),machNo(n),coeff_parasitic(n));
    
    % Total Drag Coefficient
    correctedCoeff_totalDrag(n) = correctedCoeff_skinDrag(n)+correctedCoeff_baseDrag(n)+ ...
        correctedCoeff_finDrag(n)+correctedCoeff_interferenceDrag(n)+correctedCoeff_parasiticDrag(n); 
    
    %% DYNAMICS     
    [x_force(n),y_force(n)] = calculateforce(g(n),totalDistance(n-1),alt(n-1),boundaryConditions(5),mass(n),theta(n-1),theta(1), ...
        correctedCoeff_totalDrag(n),airDensity(n),Acs,resultantVelocity(n-1),y_velocity(n-1),thrust(n),Cd_drogue, ...
        Cd_main,R_drogue,R_spill,R_main,Deploy_main_alt,db,dn,lb,ln,lt,ls,lr);
    
    %% KINEMATICS
    % Acceleration 
    x_acceleration(n)= x_force(n)/mass(n);
    y_acceleration(n)= y_force(n)/mass(n);
	resultantAcceleration(n) = sqrt(x_acceleration(n)^2 + y_acceleration(n)^2);
    
    % Velocity
    x_velocity(n)= x_velocity(n-1)+x_acceleration(n)*timeStep;
    y_velocity(n)= y_velocity(n-1)+y_acceleration(n)*timeStep;
	resultantVelocity(n) = sqrt(x_velocity(n)^2 + y_velocity(n)^2);
    
    % Position
    x_position(n)= x_position(n-1)+x_velocity(n)*timeStep;
    y_position(n)= y_position(n-1)+y_velocity(n)*timeStep;
    alt(n) = y_position(n) - y_position(1);
    
    % Distance  
    x_distance(n) = x_distance(n-1)+abs(x_velocity(n)*timeStep);
    y_distance(n) = y_distance(n-1)+abs(y_velocity(n)*timeStep); 
    totalDistance(n) = (x_distance(n)^2+y_distance(n)^2)^(1/2); 

    % Angle With Respect to Y-Axis
    theta(n)= atand(y_velocity(n)/x_velocity(n));
    
end

%%
%   ///////////////////////////////////////////////////
%   ///////////// IMPORTANT RESULT ////////////////////
%   ///////////////////////////////////////////////////

Apogee = max(alt);
Max_Velocity = max(resultantVelocity);
Max_Acceleration = max(resultantAcceleration);
CG_1 = centreMass(1);
CG_2 = min(centreMass);              
CP = centrePressure;    
Stability = (CP - CG_1)/db;
Total_Mass = mass(1);
tempmin = min(airTemp);
tempmax = max(airTemp);
tempavg = (tempmax+tempmin)/2;
tempC = tempavg-273.15;

%%
%   ///////////////////////////////////////////////////
%   ///////////////// FIGURES /////////////////////////
%   ///////////////////////////////////////////////////

% % Thrust Curve Plot
% plot(t(1:n),thrust(1:n),'color','k'); grid on;
% title('Thrust (N) vs Time (s)')
% xlabel('Time (s)')
% ylabel('Thrust (N)')
% xlim([0 max(t_burn)])

% Max_Thrust = max(F_thrust(2:end))

% Comparing to Experimental Data
expData = xlsread('TRS Flight Data IREC.xlsx');
expTime = expData(:,1)/1000;
expAlt = expData(:,2)*0.3048;

plot(t(1:n),alt(1:n),'color','b'); hold on; grid on;
title('Altitude (m) vs Time (s)')
xlabel('Time (s)')
ylabel('Altitude (m)')
ylim([0 inf])

plot(expTime,expAlt,'color','r');
hold off
legend('Model','Actual')
%%
%   ///////////////////////////////////////////////////
%   ///////////////// EXPORT TO EXCEL /////////////////
%   ///////////////////////////////////////////////////
% t = t';                  
% alt = alt';               
% mass = mass';
% thrust = thrust';
% x_velocity = x_velocity';
% y_velocity = y_velocity';
% resultantVelocity = resultantVelocity';
% x_distance = x_distance';
% y_distance = y_distance';
% totalDistance = totalDistance';
% x_position = x_position';
% y_position = y_position';
% theta = theta';
% g = g';                   
% airDensity = airDensity';
% airPressure = airPressure';
% airTemp = airTemp';
% dynamicViscosity = dynamicViscosity';
% soundSpeed = soundSpeed';
% machNo = machNo';
% correctedCoeff_totalDrag = correctedCoeff_totalDrag';
% correctedCoeff_skinDrag = correctedCoeff_skinDrag';
% correctedCoeff_baseDrag = correctedCoeff_baseDrag';
% correctedCoeff_finDrag = correctedCoeff_finDrag';
% correctedCoeff_interferenceDrag = correctedCoeff_interferenceDrag';
% correctedCoeff_parasiticDrag = correctedCoeff_parasiticDrag';
% Cd_rail_button = Cd_rail_button';
% Cd_hole = Cd_hole';
% Cd_shear_pin = Cd_shear_pin';
% Cd_screw = Cd_screw';
% Cd_parasitic = Cd_parasitic';
% filename = 'MATLAB_Model_IRECsim_July_11th.xlsx';
% xlswrite(filename,[t,alt,x_velocity,y_velocity,resultantVelocity,x_distance,y_distance,totalDistance,x_position,y_position,theta, ...
%     g,airDensity,airPressure,airTemp,dynamicViscosity,soundSpeed,machNo,mass,thrust])