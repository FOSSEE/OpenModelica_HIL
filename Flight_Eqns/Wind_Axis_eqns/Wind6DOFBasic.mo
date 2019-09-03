model Wind6DOFBasic

import Modelica.Math.Matrices.*;
import SI=Modelica.SIunits;
import Modelica.Blocks.Interfaces.*;



parameter Real g = 9.81;
parameter Real m = 1043.26;//1.56 for zagi
parameter Real[3,3] J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}};



//12 states
RealOutput[3] omega(start = {0.0,0.0,0})  annotation(    Placement(visible = true, transformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//omega

Real OMEGA[3,3] = skew(omega);//Skew symmetric matrix form of the angular velocity term


Real V (start =39.8858);
Real alpha (start =0.1);
Real beta (start = 0);

RealOutput[3] VAB annotation(    Placement(visible = true, transformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//V, alpha, beta


Real x (start = 0);
Real y (start = 0);
Real z (start = 100);

Real mu (start = 0); 
Real gamma (start = 0);
Real chi (start = 0); 


RealInput Force[3] annotation(   Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force

RealInput Moment[3] annotation(   Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));//Thrust force



Real Vdot;
Real alphadot;
Real betadot;

Real[3] omegadot;

Real mudot;
Real gammadot;
Real chidot;

Real xdot;
Real ydot;
Real zdot;


equation


Vdot = der(V);
alphadot = der(alpha);
betadot = der(beta);

omegadot= der(omega);

xdot = der(x);
ydot = der(y);
zdot = der(z);

mudot = der(mu);
gammadot = der(gamma);
chidot = der(chi);


Vdot = 1/m*(Force[1] -m*g*sin(gamma));

alphadot = omega[2]-1/cos(beta)*((omega[1]*cos(alpha)+omega[3]*sin(alpha))*sin(beta)-g/V*cos(gamma)*cos(mu)+Force[3]/(m*V));

betadot = (omega[1]*sin(alpha)-omega[3]*cos(alpha))+1/(m*V)*(Force[2]+m*g*cos(gamma)*sin(mu));


omegadot = inv(J) * (Moment- OMEGA* J*omega);

xdot=V*cos(gamma)*cos(chi);
ydot=V*cos(gamma)*sin(chi);
zdot=-V*sin(gamma);

mudot = omega[1]+tan(gamma)*sin(mu)*omega[2]+tan(gamma)*cos(mu)*omega[3];
gammadot = cos(mu)*omega[2]-sin(mu)*omega[3];
chidot=(1/cos(gamma))*sin(mu)*omega[2]+(1/cos(gamma))*cos(mu)*omega[3];

VAB = {V,alpha,beta};

end Wind6DOFBasic;