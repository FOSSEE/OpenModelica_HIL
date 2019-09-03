model SpringMassTest

parameter Real M = 1;
parameter Real k = 1; //Spring Constant
parameter Real b = 10;//Damping Coeff

Real vel;
Real dist;
Real Force = if time > 10 and time < 11 then 105 else 100;
equation
Force = M * der(vel) + k*dist + b*vel;
vel  = der(dist);




end SpringMassTest;