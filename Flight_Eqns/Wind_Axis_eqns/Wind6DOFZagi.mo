model Wind6DOFZagi
 
import Modelica.Math.Matrices.*;
import SI=Modelica.SIunits;
import Modelica.Blocks.Interfaces.*;

parameter Real rho = 1.225;
parameter Real m = 1.56;//1.56 for zagi
parameter Real S_ref = 0.2589;//reference area
parameter Real C_bar = 0.3302 ;//average chord
parameter Real b = 1.4224 ;//span
parameter Real g  = 9.81;//gravitational force
//parameter Real b= 1.4224, C_bar = 0.3302,s = 0.2589;

// lift
parameter Real CL0 = 0.09167;   //for cessna
parameter Real CL_alpha = 3.5016;//for cessna
parameter Real CL_q = 2.8932;//for cessna
parameter Real CL_delta_e = 0.2724;//for cessna

parameter Real CD0    = 0.01631;//= 0.01631;for Zagi
parameter Real K_drag  = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_0;
parameter Real Cy_beta  = -0.07359;//for cessna
parameter Real Cy_p  = 0;//for cessna
parameter Real Cy_r   = 0.21;//for cessna
parameter Real Cy_delta_r = 0.187; //for cessna
parameter Real Cy_delta_a= 0;     //for cessna



// rolling moment
parameter Real Cl_0 = 0;
parameter Real Cl_beta = -0.02854;//for cessna
parameter Real Cl_p = -0.3209;//for cessna
parameter Real Cl_r = 0.03066;//for cessna
parameter Real Cl_delta_a= 0.1682;//for cessna
parameter Real Cl_delta_r = 0;//for cessna

// pitching moment
parameter Real Cm0 = -0.02338;//for cessna
parameter Real Cm_alpha = -0.5675;//for cessna
parameter Real Cm_q   = -1.3990;//for cessna
parameter Real Cm_delta_e = -0.3254;//for cessna

// yawing moment
parameter Real Cn_0 = 0;
parameter Real Cn_beta = -0.00040;//for cessna
parameter Real Cn_p  = -0.01297;//for cessna
parameter Real Cn_r = -0.00434;//for cessna
parameter Real Cn_delta_a = -0.00328;//for cessna
parameter Real Cn_delta_r = 0;//for cessna


//Initial conditions. (deltaE, thrust[1] and the others are straightforward)

parameter Real[3,3] J = {{0.1147, 0.0, -0.0015}, {0.0, 0.0576, 0.0}, {-0.0015, 0.0, 0.2589}};

Real CL;
Real CD;
Real CY;
Real Cl;
Real Cm;
Real Cn;
Real CX;
Real CZ;
//Params
parameter Real deltaE = -0.246251;
parameter Real deltaR = 0;

//Modelica.Blocks.Sources.RealExpression deltaA (y = if time > 100 and time < 105 then  3.1412 /180 elseif time > 105 and time < 110 then -3.1412 /180 else 0)  annotation(    Placement(visible = true, transformation(origin = {-112, 1}, extent = {{-26, -47}, {26, 47}}, rotation = 0)));
 Modelica.Blocks.Sources.Constant deltaA (k =0)  annotation(    Placement(visible = true, transformation(origin = {-42, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));




parameter  Real thrust = 4.47729;

//12 states
Real p (start = 0);
Real q (start = 0);
Real r (start = 0);

Real OMEGA[3,3] = skew({p,q,r});//Skew symmetric matrix form of the angular velocity term


Real V (start =15.8114);
Real alpha (start =0.1);
Real beta (start = 0);


Real x (start = 0);
Real y (start = 0);
Real z (start = 100);

Real gamma (start = 0);
Real chi (start = 0); 
Real mu (start = 0); 

Real qbar = 0.5*rho*V^2;
Real [3] Moment;



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
CL = CL0+CL_alpha*alpha+((CL_q*q*C_bar)/(2*V))+CL_delta_e*deltaE;
CD =  CD0+CD_alpha*alpha+((CD_q*q*C_bar)/(2*V))+CD_delta_e*abs(deltaE)  ;
//CD = CD0 + K_drag*CL^2;
CY = Cy_beta * beta + Cy_p * (p*b)/(2*V) + Cy_r *(r*b)/(2*V) + Cy_delta_a * deltaA.k + Cy_delta_r*deltaR;//Sideslip coeff


Cl = Cl_beta * beta + Cl_p*(p*b)/(2*V) + Cl_r *(r*b)/(2*V) + Cl_delta_a * deltaA.k + Cl_delta_r * deltaR;//Rolling coeff

Cm  = Cm0+Cm_alpha*alpha+((Cm_q*q*C_bar)/(2*V))+Cm_delta_e*deltaE;//pitching coeff

Cn = Cn_beta * beta + Cn_p * (p*b)/(2*V) + Cn_r *(r*b) /(2*V) + Cn_delta_a * deltaA.k + Cn_delta_r * deltaR;//Yawing coeff

CX = -CD*cos(alpha) + CL*sin(alpha);
CZ = -CD*sin(alpha) - CL*cos(alpha);

Moment[2] = Cm*qbar*S_ref*C_bar;
Moment[1] = Cl*qbar*S_ref*b;
Moment[3] = Cn*qbar*S_ref*b;

Vdot = der(V);
alphadot = der(alpha);
betadot = der(beta);

omegadot[1] = der(p);
omegadot[2] = der(q);
omegadot[3] = der(r);

xdot = der(x);
ydot = der(y);
zdot = der(z);

mudot = der(mu);
gammadot = der(gamma);
chidot = der(chi);




Vdot = 1/m*(thrust*cos(alpha)*cos(beta)-0.5*rho*V^2*S_ref*(CD*cos(beta)-CY*sin(beta))-m*g*sin(gamma));

alphadot = q-1/cos(beta)*((p*cos(alpha)+r*sin(alpha))*sin(beta)-g/V*cos(gamma)*cos(mu)+0.5*rho*V^2*S_ref*CL/(m*V)+thrust*sin(alpha)/(m*V));

betadot = (p*sin(alpha)-r*cos(alpha))+1/(m*V)*(-thrust*cos(alpha)*sin(beta)+0.5*rho*V^2*S_ref*(CY*cos(beta)+CD*sin(beta))+m*g*cos(gamma)*sin(mu));


omegadot = inv(J) * (Moment- OMEGA* J*{p,q,r});



xdot=V*cos(gamma)*cos(chi);
ydot=V*cos(gamma)*sin(chi);
zdot=-V*sin(gamma);

mudot = p+tan(gamma)*sin(mu)*q+tan(gamma)*cos(mu)*r;
gammadot = cos(mu)*q-sin(mu)*r;
chidot=(1/cos(gamma))*sin(mu)*q+(1/cos(gamma))*cos(mu)*r;


annotation(experiment(StartTime = 0, StopTime = 500, Interval = 0.002),
    uses(Modelica(version = "3.2.2")));
end Wind6DOFZagi;