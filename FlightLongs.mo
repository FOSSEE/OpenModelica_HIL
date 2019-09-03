model FlightLongs

import Modelica.Blocks.Interfaces.*;

parameter Real rho = 1.225;
parameter Real m = 1043.26;//1.56 for zagi
parameter Real S_ref = 16.1651;//reference area
parameter Real C_bar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
parameter Real g  = 9.81;//gravitational force
//parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;



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


//Initial conditions. (delta[2], thrust[1] and the others are straightforward)

parameter Real I_yy = 1824.93;


Real CL;
Real CD;
Real Cm;
Real CX;
Real CZ;


  RealInput del (start = -0.15625) annotation( Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  RealInput thrust (start = 1112.82) annotation( Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

      
  Modelica.Blocks.Interfaces.RealOutput q (start = 0) annotation(Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110,-50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  
  Modelica.Blocks.Interfaces.RealOutput V (start = 39.8858) annotation(Placement(visible = true, transformation(origin = {110, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110,100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));


  Modelica.Blocks.Interfaces.RealOutput theta (start = 0.1) annotation(Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110,0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

Real x (start = 0.1);
  Modelica.Blocks.Interfaces.RealOutput z (start = 100) annotation(Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110,50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

Real alpha (start = 0.1) ;
Real gamma = alpha - theta;


Real qbar = 0.5*rho*V^2;

Real Vdot;
Real alphadot;
Real gammadot;
Real qdot;
Real xdot;
Real zdot;


equation
CL = CL0+CL_alpha*alpha+CL_delta_e*del;
CD = CD0 + K_drag*CL^2;
Cm  = Cm0+Cm_alpha*alpha+Cm_delta_e*del;
CX = -CD*cos(alpha) + CL*sin(alpha);
CZ = -CD*sin(alpha) - CL*cos(alpha);


Vdot = der(V);
alphadot = der(alpha);
gammadot = der(gamma);
qdot = der(q);
xdot = der(x);
zdot = der(z);



Vdot = (1/m) *(thrust*cos(alpha) -qbar*S_ref*CD - m*g*sin(gamma));
alphadot = q - (-(g/V)*cos(gamma) + (qbar*S_ref*CL + thrust*sin(alpha))/(m*V));
qdot = (1/I_yy)*(qbar*S_ref*C_bar*Cm);
gammadot = q;
xdot=V*cos(gamma);
zdot=-V*sin(gamma);

 annotation(experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-3, Interval = 0.02),
    uses(Modelica(version = "3.2.2")));
end FlightLongs;