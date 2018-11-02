function [Cd_fb,Cd_b,Cd_f,Cd_i,Cd_parasitic] = coefficientdrag(airDensity,dynamicViscosity,lTR,ln,lb,lc,lm,db,dd,df,tf,resultantVelocity,gamma,machNo,Ap_f,Ae_f,Acs,A_shear_pin,A_rail_button,N,N_hole,N_rail_button,N_shear_pin,N_screw,Rs)
% Drag cofficient calculation. The drags involved will be skin drag (Cd_fb), 
% base drag (Cd_b), fin drag (Cd_f), and interference drag (Cd_i).
% gamma,machNo,Ap_f,Ae_f,Acs,A_shear_pin,A_rail_button,N,N_hole,N_rail_button,N_shear_pin,N_screw,Rs
%% Flow Properties

    Re_b = (airDensity*resultantVelocity*lTR)/dynamicViscosity;   
    Re_f = (airDensity*resultantVelocity*lm)/dynamicViscosity;
    ReC_b = 51*(Rs/lTR)^(-1.039);
    ReC_f = 51*(Rs/lm)^(-1.039);

%% Skin Friction Coefficient 
   
%   Body
    if Re_b <= 10E4
       Cf_fb = 1.48E-2;
       
    elseif Re_b > 10E4 && Re_b <= ReC_b
       Cf_fb = 1/((1.5*log(Re_b)-5.6))^2;
       
    elseif Re_b > ReC_b
       Cf_fb = 0.032*(Rs/lTR)^0.2;
        
    end
    
%   Fin
    if Re_f <= 10E4
       Cf_f = 1.48E-2;
       
    elseif Re_f > 10E4 && Re_f <= ReC_f
       Cf_f = 1/((1.5*log(Re_f)-5.6)^2);
       
    elseif Re_f > ReC_f
       Cf_f = 0.032*(Rs/lm)^0.2;
        
    end

%% Skin Drag Coefficient
    B_fb = ReC_b*(0.074/Re_b^(1/5) - 1.328/sqrt(Re_b));
    Cd_fb =  (1 + 60/(lTR/db)^3 + 0.0025*(lb/db))*...
                (2.7*(ln/db) + 4*(lb/db) + 2*(1 - (dd/db))*(lc/db))*Cf_fb;
            
%     A_body_wet = A_nose_surface + lb*(pi*db) + N*Ae_f;
%     Cf_surface_rough_total = Cf_surface*(((1 + 2*lTR/db)*A_body_wet + (1 + 2*tf/lm)*2*Ae_f)/Acs);
            
%% Base Drag Coefficient
    Cd_b = 0.029*(dd/db)^3/sqrt(Cd_fb);
    
%% Fin Drag Coefficient
    B_ff = ReC_f*(0.074/Re_f^(1/5) - 1.328/sqrt(Re_f));
    Cd_f = 2*Cf_f*(1 + 2*(tf/lm))*((4*N*Ap_f)/(pi*df^2));

%% Interference Drag Coefficient
    Cd_i = 2*Cf_f*(1 + 2*(tf/lm))*((4*N)*(Ap_f - Ae_f)/(pi*df^2));
    
%% Parasitic Drag Coefficient
    
    % Apogee Components Airfoiled Rail Button
    if machNo ==0
        P_ratio = 0;
        C_pt = 0;
    else
        P_ratio = (1+machNo^2*(gamma-1)/2)^(gamma/(gamma-1));
        C_pt = (P_ratio - 1)*(2/(gamma*machNo^2));
    end
    
    Rail_Button_Efficiency = 0.3/0.57;
    Cd_rail_button = 0.8*Rail_Button_Efficiency*A_rail_button*C_pt/Acs;

    % Vent Hole
    Cd_hole = 0.0041*(pi/4*0.003^2)*C_pt/Acs;
    
    % Shear Pin (Round rivet head)
    Cd_shear_pin = 0.32*A_shear_pin*C_pt/Acs;
    
    % Screw Head
    Cd_screw = 0.42*(pi/4*0.01905^2)*C_pt/Acs;
    
    % Total Parasitic Drag
    Cd_parasitic = N_hole*Cd_hole + N_rail_button*Cd_rail_button + N_shear_pin*Cd_shear_pin + N_screw*Cd_screw;
%    Cd_parasitic = 0;
    
%% Wave Drag

    % Cd_pressure = a/(ln/dn +)^b;
    
    
end