function [Xcpr_R] = calculatecentrepressure(df,ls,N,dn,lm,lr,lt,Xf,ln)
% Centre of pressure calculation for the rocket, based only on the 
% outer geometry. This only includes nose and fins, and ignores body lift.


    % Parts Stability Derivative
    Kfb = 1 + (df/2)/(ls+(df/2));
    Cna_f = Kfb*(4*N*(ls/dn)^2)/(1+ sqrt(1+(2*lm/(lr+lt))^2));

    Cna_P = [
        2
        Cna_f
    ];

    % Parts Centre of Pressure
    Xcpr_f = Xf + (lm*(lr+2*lt))/(3*(lr+lt)) + (1/6)*(lr+lt-(lr*lt/(lr+lt)));

    Xcpr_P = [
        (0.5)*ln
        Xcpr_f
    ];

    % Rocket Centre of Pressure
    Cna_R = sum(Cna_P);
    Xcpr_R = sum(Cna_P(:,1).*Xcpr_P(:,1))/Cna_R;

end
