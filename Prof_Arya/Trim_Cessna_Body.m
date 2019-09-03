function F = Trim_Cessna_Body(x)

m = 1043.26;
S_ref = 16.1651;
d = 1.225;
V = 60;
g = 9.8;

C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_delta_e = 0.3476;
        
C_D_0 = 0.036;
C_D_alpha = 0.13;
C_D_delta_e = 0.06;
        
C_M_0 = -0.02;
C_M_alpha = -1.8;
C_M_delta_e = -1.28;

C_L = C_L_0 + C_L_alpha*x(1) + C_L_delta_e*x(2);
% C_D = C_D_0 + C_D_alpha*abs(x(1)) + C_D_delta_e*abs(x(2));
C_D = C_D_0 + 0.3*C_L^2;
C_M = C_M_0 + C_M_alpha*x(1) + C_M_delta_e*x(2);

F = [C_M;
    0.5*d*V*V*S_ref*(-C_D*cos(x(1)) + C_L*sin(x(1))) - m*g*sin(x(1)) + x(3);
    0.5*d*V*V*S_ref*(-C_D*sin(x(1)) - C_L*cos(x(1))) + m*g*cos(x(1))];