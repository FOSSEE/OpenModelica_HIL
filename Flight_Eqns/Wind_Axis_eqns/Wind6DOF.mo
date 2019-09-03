block Wind6DOF

import Modelica.Math.Matrices.*;
import Modelica.SIunits.*;
import Modelica.Blocks.Interfaces.*;
import Modelica.Math.Vectors.*;


RealInput thrust annotation(
    Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force
    
RealInput[3] delta annotation(
    Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    

Modelica.Blocks.Interfaces.RealOutput V annotation(start =39.8858,
    Placement(visible = true, transformation(origin = {110, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0))); //Linear velocity

Modelica.Blocks.Interfaces.RealOutput alpha annotation(start =0.1,
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0))); //Angle of attack

Modelica.Blocks.Interfaces.RealOutput beta annotation(start = 0,
    Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0))); //Angle of sideslip
        
    
Modelica.Blocks.Interfaces.RealOutput pos[3]annotation(
    Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //Position (Displacement) //Displacement
  
Modelica.Blocks.Interfaces.RealOutput omega[3] annotation(
    Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //Angular velocity around the CM
Modelica.Blocks.Interfaces.RealOutput angles[3] annotation(
    Placement(visible = true, transformation(origin = {110, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  //mu, gamma, and chi
    





parameter Real rho;
parameter Real m;//1.56 for zagi
parameter Real S_ref;//reference area
parameter Real C_bar ;//average chord
parameter Real b ;//span
parameter Real g  = 9.81;//gravitational force
//parameter Real b= 1.4224, C_bar = 0.3302,s = 0.2589;



parameter Real CD0;//= 0.01631;for Zagi
parameter Real K_drag ;//for cessna
parameter Real CD_beta;//for cessna
parameter Real CD_alpha;
parameter Real CD_q;
parameter Real CD_delta_e;

//side force
parameter Real Cy_beta;//for cessna
parameter Real Cy_p;//for cessna
parameter Real Cy_r;//for cessna
parameter Real Cy_delta_r; //for cessna
parameter Real Cy_delta_a;     //for cessna

// lift
parameter Real CL0 ;   //for cessna
parameter Real CL_alpha;//for cessna
parameter Real CL_q;//for cessna
parameter Real CL_delta_e;//for cessna

// rolling moment
parameter Real Cl_beta;//for cessna
parameter Real Cl_p;//for cessna
parameter Real Cl_r;//for cessna
parameter Real Cl_delta_a;//for cessna
parameter Real Cl_delta_r ;//for cessna

// pitching moment
parameter Real Cm0;//for cessna
parameter Real Cm_alpha ;//for cessna
parameter Real Cm_q;//for cessna
parameter Real Cm_delta_e;//for cessna

// yawing moment
parameter Real Cn_beta;//for cessna
parameter Real Cn_p;//for cessna
parameter Real Cn_r;//for cessna
parameter Real Cn_delta_a ;//for cessna
parameter Real Cn_delta_r;//for cessna


//Initial conditions. (delta[2], thrust[1] and the others are straightforward)

parameter Real I_xx;
parameter Real I_yy;
parameter Real I_zz;

Real CL;
Real CD;
Real CY;
Real Cl;
Real Cm;
Real Cn;
Real CX;
Real CZ;
//Params
//parameter Real delta[1] = 0;
//parameter Real delta[3] = 0;

//Modelica.Blocks.Sources.RealExpression delta[2] (y = if time > 100 and time < 105 then  -0.15625+3.1412 /180 else -0.15625)  annotation(    Placement(visible = true, transformation(origin = {-112, 1}, extent = {{-26, -47}, {26, 47}}, rotation = 0)));
//parameter Real delta[2] = -0.15625;

//parameter  Real thrust = 1112.82;



Real qbar = 0.5*rho*V^2;

Real Vdot;
Real alphadot;
Real betadot;

Real pdot;
Real qdot;
Real rdot;

Real mudot;
Real gammadot;
Real chidot;

Real xdot;
Real ydot;
Real zdot;


equation
CL = CL0+CL_alpha*alpha+((CL_q*omega[2]*C_bar)/(2*V))+CL_delta_e*delta[2];
//CD =  CD0+CD_alpha*alpha+((CD_q*omega[2]*C_bar)/(2*V))+CD_delta_e*abs(delta[2]);// + CDbeta * beta + CDdelta[2] * Elevator;
CD = CD0 + K_drag*CL^2;
CY = Cy_beta * beta + Cy_p * (omega[1]*b)/(2*V) + Cy_r *(omega[3]*b)/(2*V) + Cy_delta_a * delta[1] + Cy_delta_r*delta[3];//Sideslip coeff


Cl = Cl_beta * beta + Cl_p*(omega[1]*b)/(2*V) + Cl_r *(omega[3]*b)/(2*V) + Cl_delta_a * delta[1] + Cl_delta_r * delta[3];//Rolling coeff

Cm  = Cm0+Cm_alpha*alpha+((Cm_q*omega[2]*C_bar)/(2*V))+Cm_delta_e*delta[2];//pitching coeff

Cn = Cn_beta * beta + Cn_p * (omega[1]*b)/(2*V) + Cn_r *(omega[3]*b) /(2*V) + Cn_delta_a * delta[1] + Cn_delta_r * delta[3];//Yawing coeff

CX = -CD*cos(alpha) + CL*sin(alpha);
CZ = -CD*sin(alpha) - CL*cos(alpha);


Vdot = der(V);
alphadot = der(alpha);
betadot = der(beta);

pdot = der(omega[1]);
qdot = der(omega[2]);
rdot = der(omega[3]);

xdot = der(pos[1]);
ydot = der(pos[2]);
zdot = der(pos[3]);

mudot = der(angles[1]);
gammadot = der(angles[2]);
chidot = der(angles[3]);




Vdot = 1/m*(thrust*cos(alpha)*cos(beta)-0.5*rho*V^2*S_ref*(CD*cos(beta)-CY*sin(beta))-m*g*sin(angles[2]));

alphadot = omega[2]-1/cos(beta)*((omega[1]*cos(alpha)+omega[3]*sin(alpha))*sin(beta)-g/V*cos(angles[2])*cos(angles[1])+0.5*rho*V^2*S_ref*CL/(m*V)+thrust*sin(alpha)/(m*V));

betadot = (omega[1]*sin(alpha)-omega[3]*cos(alpha))+1/(m*V)*(-thrust*cos(alpha)*sin(beta)+0.5*rho*V^2*S_ref*(CY*cos(beta)+CD*sin(beta))+m*g*cos(angles[2])*sin(angles[1]));


pdot = (I_yy-I_zz)/I_xx*omega[2]*omega[3]+1/(2*I_xx)*rho*V^2*S_ref*b*Cl;
qdot = (I_zz-I_xx)/I_yy*omega[1]*omega[3]+1/(2*I_yy)*rho*V^2*S_ref*C_bar*Cm;
rdot = (I_xx-I_yy)/I_zz*omega[1]*omega[2]+1/(2*I_zz)*rho*V^2*S_ref*b*Cn;


xdot=V*cos(angles[2])*cos(angles[3]);
ydot=V*cos(angles[2])*sin(angles[3]);
zdot=-V*sin(angles[2]);

mudot = omega[1]+tan(angles[2])*sin(angles[1])*omega[2]+tan(angles[2])*cos(angles[1])*omega[3];
gammadot = cos(angles[1])*omega[2]-sin(angles[1])*omega[3];
chidot=(1/cos(angles[2]))*sin(angles[1])*omega[2]+(1/cos(angles[2]))*cos(angles[1])*omega[3];

end Wind6DOF;