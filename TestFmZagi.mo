model TestFmZagi



parameter Real m = 1.56;
parameter Real W[3]  = m*{0,0, 9.81};//gravitational force
parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;



parameter Real CD0 = 0.01631;//     = 0.036;//for cessna
parameter Real K_drag ;// = 0.0830304;//for cessna
parameter Real CD_beta = 0.17;//for cessna
parameter Real CD_alpha= 0.2108;
parameter Real CD_q = 0;
parameter Real CD_delta_e= 0.3045;

//side force
parameter Real Cy_beta  = -0.07359 ;// = -0.31;//for cessna
parameter Real Cy_p = 0     ;// = -0.037;//for cessna
parameter Real Cy_r  = 0    ;// = 0.21;//for cessna
parameter Real Cy_delta_r= 0;// = 0.187; //for cessna
parameter Real Cy_delta_a = 0;//= 0;     //for cessna

// lift
parameter Real CL0 = 0.09167      ;// = 0.25;   //for cessna
parameter Real CL_alpha= 3.5016 ;//  = 4.47;//for cessna
parameter Real CL_q   = 2.8932   ;// = 3.9;//for cessna
parameter Real CL_delta_e= 0.2724;// = 0.3476;//for cessna

// rolling moment
parameter Real Cl_beta  = -0.02854   ;//= -0.089;//for cessna
parameter Real Cl_p  = -0.3209     ;//= -0.47;//for cessna
parameter Real Cl_r  = 0.03066     ;//= 0.096;//for cessna
parameter Real Cl_delta_a= 0.1682 ;//= -0.09;//for cessna
parameter Real Cl_delta_r= 0;// = 0.0147;//for cessna

// pitching moment
parameter Real Cm0  = -0.02338;// = -0.02;//for cessna
parameter Real Cm_alpha = -0.5675 ;// = -1.8;//for cessna
parameter Real Cm_q   = -1.3990    ;//= -12.4;//for cessna
parameter Real Cm_delta_e= -0.3254;// = -1.28;//for cessna

// yawing moment
parameter Real Cn_beta  = -0.00040   ;//= 0.065;//for cessna
parameter Real Cn_p = -0.01297     ;// = -0.03;//for cessna
parameter Real Cn_r = -0.00434      ;// = -0.99;//for cessna
parameter Real Cn_delta_a = -0.00328 ;//= -0.0053;//for cessna
parameter Real Cn_delta_r = 0;//= -0.0657;//for cessna

parameter  Real del[3] = {0,-0.246251,0};
parameter  Real thrust[3] = {4.47729 , 0, 0};
parameter Real alphazero = 0.1;
  
  ForceMoment_Gen forceMoment_Gen1( CD0 = CD0, CD_alpha = CD_alpha, CD_beta = CD_beta, CD_delta_e = CD_delta_e, CD_q = CD_q, CL0 = CL0, CL_alpha = CL_alpha, CL_delta_e = CL_delta_e, CL_q = CL_q, Cl_beta = Cl_beta, Cl_delta_a = Cl_delta_a, Cl_delta_r =Cl_delta_r, Cl_p = Cl_p, Cl_r = Cl_r, Cm0 = Cm0, Cm_alpha = Cm_alpha, Cm_delta_e = Cm_delta_e, Cm_q = Cm_q, Cn_beta = Cn_beta, Cn_delta_a = Cn_delta_a, Cn_delta_r = Cn_delta_r, Cn_p = Cn_p, Cn_r = Cn_r, Cy_beta = Cy_beta, Cy_delta_a = Cy_delta_a, Cy_delta_r = Cy_delta_r, Cy_p = Cy_p, Cy_r = Cy_r,W =m * {0, 0, 9.81},  b= b, cbar =cbar, g = 9.81, m = 1.56, rho = 1.225, s = 0.2589)  annotation(
    Placement(visible = true, transformation(origin = {-9, 1}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
  Flight6DOF flight6DOF1(J = {{0.1147, 0, -0.0015}, {0, 0.0576, 0}, {-0.0015, 0, 0.1712}}, g = {0,0, 9.81},mass = m, omega( fixed = true,start = {0, 0, 0}), pos(start = {0, 0, -1000}), v(start = {15.8114*cos(alphazero), 0, 15.8114*sin(alphazero)}))  annotation(
    Placement(visible = true, transformation(origin = {47, -1}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
  
Modelica.Blocks.Sources.RealExpression[3] realExpression1(y = if time > 10 and time < 20 then {0, -0.01304370748, 0} else del)  annotation(    Placement(visible = true, transformation(origin = {-93, -3}, extent = {{-15, -9}, {15, 9}}, rotation = 0)));

initial equation
forceMoment_Gen1.alpha = alphazero;
forceMoment_Gen1.angles[2] = alphazero;


equation
 //forceMoment_Gen1.delta = del;
 connect(forceMoment_Gen1.delta, realExpression1.y) annotation(    Line(points = {{-28, -4}, {-53.5, -4}, {-53.5, -3}, {-76.5, -3}}, color = {0, 0, 127}, thickness = 0.5));
 
 connect(flight6DOF1.angles, forceMoment_Gen1.angles) annotation(
    Line(points = {{61, -13}, {102, -13}, {102, 52}, {-18, 52}, {-18, 20}, {-17.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
 connect(flight6DOF1.v, forceMoment_Gen1.vel) annotation(
    Line(points = {{61, 11}, {90, 11}, {90, 42}, {-6, 42}, {-6, 20}, {-9, 20}}, color = {0, 0, 127}, thickness = 0.5));
 connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
    Line(points = {{10, 6}, {33, 6}}, color = {0, 0, 127}, thickness = 0.5));
 connect(flight6DOF1.Moment, forceMoment_Gen1.Moment) annotation(
    Line(points = {{33, -7}, {21.5, -7}, {21.5, -4}, {10, -4}}, color = {0, 0, 127}, thickness = 0.5));
 connect(flight6DOF1.omega, forceMoment_Gen1.omega) annotation(
    Line(points = {{61, -5}, {98, -5}, {98, 32}, {-0.5, 32}, {-0.5, 20}}, color = {0, 0, 127}, thickness = 0.5));
 forceMoment_Gen1.thrust = thrust;
 
  
  annotation(
    uses(Modelica(version = "3.2.2")));
    end TestFmZagi;