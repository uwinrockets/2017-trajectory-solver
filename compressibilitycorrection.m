function [Cd_fbi,Cd_bi,Cd_fi,Cd_ii,Cd_parasitici] = compressibilitycorrection(coeff_skinDrag,coeff_baseDrag,coeff_finDrag,coeff_interferenceDrag, machNo,coeff_parasitic)
% Compressibility correction calculation for subsonic (Mach < 1) and
% supersonic (Mach > 1) regime, using the Prandtl-Glauert rule.

    % Subsonic
    if machNo <= 1 && machNo >= 0
        Cd_fbi = coeff_skinDrag/sqrt(1-machNo^2);
        Cd_bi = coeff_baseDrag/sqrt(1-machNo^2);
        Cd_fi = coeff_finDrag/sqrt(1-machNo^2);
        Cd_ii = coeff_interferenceDrag/sqrt(1-machNo^2);
        Cd_parasitici = coeff_parasitic/sqrt(1 - machNo^2);

    % Supersonic        
    elseif machNo > 1
        Cd_fbi = coeff_skinDrag/sqrt(machNo^2-1);
        Cd_bi = coeff_baseDrag/sqrt(machNo^2-1);
        Cd_fi = coeff_finDrag/sqrt(machNo^2-1);
        Cd_ii = coeff_interferenceDrag/sqrt(machNo^2-1);
        Cd_parasitici = coeff_parasitic/sqrt(machNo^2-1);
        
    end

end

