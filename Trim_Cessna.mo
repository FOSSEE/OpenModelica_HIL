model Trim_Cessna


import Modelica.SIunits.*;
import Modelica.Math.Matrices.*;

parameter Real m = 1043.26;//
parameter Real s = 16.1651;//reference area
parameter Real cbar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force



parameter Real CD0    = 0.036;//
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.31;//for cessna
parameter Real Cy_p  = -0.037;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna

// lift
parameter Real CL0 = 0.25;   //for cessna
parameter Real CL_alpha = 4.47;//for cessna
parameter Real CL_q = 3.9;//for cessna
parameter Real CL_delta_e = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta = -0.089;//for cessna
parameter Real Cl_p = -0.47;//for cessna
parameter Real Cl_r = 0.096;//for cessna
parameter Real Cl_delta_a= -0.09;//for cessna
parameter Real Cl_delta_r = 0.0147;//for cessna

// pitching moment
parameter Real Cm0   = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q  = -12.4;//for cessna
parameter Real Cm_delta_e= -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna

parameter Real rho = 1.225;
parameter Real[3,3] J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}};
Real L;
Real D;

Real Q;
  

Real V;

parameter Real[3] omega = {0,0.0,0};

Real CL;
Real CD;
parameter Real alpha = 0.1;
Real de;//To be pasted in delta[2] in the TestFm file
Real thrust;//To be pasted in thrust[1] in the TestFm file
Real theta = alpha;


equation


Q=0.5*rho*V^2;

0  = Cm0+Cm_alpha*alpha+((Cm_q*omega[2]*cbar)/(2*V))+Cm_delta_e*de;
CL = CL0+CL_alpha*alpha+((CL_q*omega[2]*cbar)/(2*V))+CL_delta_e*de;
//CD = CD0+CD_alpha*alpha+((CD_q*omega[2]*cbar)/(2*V))+CD_delta_e*abs(de);// + CDbeta * beta + CDdeltae * 
CD = CD0 + K_drag*CL^2;

//Elevator;
// forces and moments

L = CL*s*Q;
D = CD*s*Q;



0 = -D*cos(alpha)+L*sin(alpha)+thrust - m*9.81*sin(theta);
0 = -D*sin(alpha)-L*cos(alpha)+m*9.81*cos(theta);

end Trim_Cessna;