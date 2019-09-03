model WindForceMoment

import Modelica.Math.Matrices.*;
import Modelica.SIunits.*;
import Modelica.Blocks.Interfaces.*;
import Modelica.Math.Vectors.*;



parameter Real rho = 1.225;
parameter Real g = 9.81;
parameter Real m = 1043.26;//1.56 for zagi
parameter Real S_ref = 16.1651;//reference area
parameter Real C_bar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span

parameter Real CD0    = 0.036;//= 0.01631;for Zagi
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
parameter Real Cm0 = -0.02;//for cessna
parameter Real Cm_alpha = -1.8;//for cessna
parameter Real Cm_q   = -12.4;//for cessna
parameter Real Cm_delta_e = -1.28;//for cessna

// yawing moment
parameter Real Cn_beta = 0.065;//for cessna
parameter Real Cn_p  = -0.03;//for cessna
parameter Real Cn_r = -0.99;//for cessna
parameter Real Cn_delta_a = -0.0053;//for cessna
parameter Real Cn_delta_r = -0.0657;//for cessna

Real CL;
Real CD;
Real CY;
Real Cl;
Real Cm;
Real Cn;
Real CX;
Real CZ;

RealInput thrust annotation(   Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force
    
RealInput[3] delta annotation(
    Placement(visible = true, transformation(origin = {-110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
 
RealInput[3] VAB annotation(    Placement(visible = true, transformation(origin = {-33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));//V, alpha, beta

RealInput[3] omega annotation(    Placement(visible = true, transformation(origin = {33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));//Angular velocities

 
 
Real V = VAB[1];
Real alpha = VAB[2];
Real beta = VAB[3];
 
Real p = omega[1];
Real q = omega[2];
Real r = omega[3]; 
    
Real deltaE = delta[1];
Real deltaR = delta[2];
Real deltaA = delta[3];

Real qbar = 0.5*rho*V^2;



RealOutput Force[3] annotation(   Placement(visible = true, transformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force

RealOutput Moment[3] annotation(   Placement(visible = true, transformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force



equation
CL = CL0+CL_alpha*alpha+((CL_q*q*C_bar)/(2*V))+CL_delta_e*deltaE;
//CD =  CD0+CD_alpha*alpha+((CD_q*q*C_bar)/(2*V))+CD_delta_e*abs(deltaE)  ;
CD = CD0 + K_drag*CL^2;
CY = Cy_beta * beta + Cy_p * (p*b)/(2*V) + Cy_r *(r*b)/(2*V) + Cy_delta_a * deltaA + Cy_delta_r*deltaR;//Sideslip coeff


Cl = Cl_beta * beta + Cl_p*(p*b)/(2*V) + Cl_r *(r*b)/(2*V) + Cl_delta_a * deltaA + Cl_delta_r * deltaR;//Rolling coeff

Cm  = Cm0+Cm_alpha*alpha+((Cm_q*q*C_bar)/(2*V))+Cm_delta_e*deltaE;//pitching coeff

Cn = Cn_beta * beta + Cn_p * (p*b)/(2*V) + Cn_r *(r*b) /(2*V) + Cn_delta_a * deltaA + Cn_delta_r * deltaR;//Yawing coeff

CX = -CD*cos(alpha) + CL*sin(alpha);
CZ = -CD*sin(alpha) - CL*cos(alpha);

Force[1] = thrust*cos(alpha)*cos(beta)-0.5*rho*V^2*S_ref*(CD*cos(beta)-CY*sin(beta));
Force[2] = -thrust*cos(alpha)*sin(beta)+0.5*rho*V^2*S_ref*(CY*cos(beta)+CD*sin(beta));
Force[3] = 0.5*rho*V^2*S_ref*CL+thrust*sin(alpha);


Moment[2] = Cm*qbar*S_ref*C_bar;
Moment[1] = Cl*qbar*S_ref*b;
Moment[3] = Cn*qbar*S_ref*b;

end WindForceMoment;