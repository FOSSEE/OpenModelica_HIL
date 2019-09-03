clear all
close all
clc

global C_L_0 C_L_alpha C_L_q C_L_delta_e
global C_D_0 C_D_alpha C_D_beta C_D_delta_e k
global C_Y_beta C_Y_p C_Y_r C_Y_delta_a
global C_l_beta C_l_p C_l_r C_l_delta_a C_l_delta_r
global C_M_0 C_M_alpha C_M_alpha_dot C_M_delta_e
global C_N_beta C_N_p C_N_r C_N_delta_a C_N_delta_r

global m C_bar b S_ref
global I_xx I_yy I_zz I_xy I_yz I_xz

g = 9.8;
rho = 1.225;

m = 1043.26;
C_bar = 1.493;
b = 10.911;
S_ref = 16.1651;

I_xx = 1285.31;
I_yy = 1824.93;
I_zz = 2666.893;
I_xz = 0;
I_yz = 0;
I_xy = 0;

C_L_0 = 0.25;
C_L_alpha = 4.47;
C_L_q = 1.7;
C_L_delta_e = 0.3476;

C_D_0 = 0.036;
k = 0.3;
% C_D_alpha = 0.13;
% C_D_beta = 0.17;
% C_D_delta_e = 0.06;

C_Y_beta = -0.31;
C_Y_p = -0.037;
C_Y_r = 0.21;
C_Y_delta_a = 0;

C_l_beta = -0.089;
C_l_p = -0.47;
C_l_r = 0.096;
C_l_delta_a = -0.09;
C_l_delta_r = 0.0147;

C_M_0 = -0.02;
C_M_alpha = -1.8;
C_M_alpha_dot = -12.4;
C_M_delta_e = -1.28;

C_N_beta = 0.065;
C_N_p = -0.03;
C_N_r = -0.99;
C_N_delta_a = -0.0053;
C_N_delta_r = -0.0657;

V_trim = 60;
V = 60;

alpha_delta_e_thrust = fsolve(@Trim_Cessna_Body, [0.035 -0.065 1500]);

alpha = alpha_delta_e_thrust(1,1);
delta_e = alpha_delta_e_thrust(1,2);
thrust = alpha_delta_e_thrust(1,3);

x = 0;
y = 0;
z = 0;

delta_a = 0;
delta_r = 0;
beta = 0;

u = V_trim*cos(alpha)*cos(beta);
v = V_trim*sin(beta);
w = V_trim*sin(alpha)*cos(beta);

phi = 0;
theta = alpha;
psi = 0;

p = 0;
q = 0;
r = 0;

statenames = getstatenames('Cessna_6DOF_trim_ha');
display(statenames)