model CessnaTest




parameter Real m = 1043.26;//1.56 for zagi
parameter Real S_ref = 16.1651;//reference area
parameter Real C_bar = 1.493 ;//average chord
parameter Real b = 10.911 ;//span
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

parameter Real I_xx = 1285.31;
parameter Real I_yy = 1824.93;
parameter Real I_zz = 2666.893;


  Wind6DOF wind6DOF1 ( CD0 = CD0, CD_alpha = CD_alpha, CD_beta = CD_beta, CD_delta_e = CD_delta_e, CD_q = CD_q, CL0 = CL0, CL_alpha = CL_alpha, CL_delta_e = CL_delta_e, CL_q = CL_q, Cl_beta = Cl_beta, Cl_delta_a = Cl_delta_a, Cl_delta_r =Cl_delta_r, Cl_p = Cl_p, Cl_r = Cl_r, Cm0 = Cm0, Cm_alpha = Cm_alpha, Cm_delta_e = Cm_delta_e, Cm_q = Cm_q, Cn_beta = Cn_beta, Cn_delta_a = Cn_delta_a, Cn_delta_r = Cn_delta_r, Cn_p = Cn_p, Cn_r = Cn_r, Cy_beta = Cy_beta, Cy_delta_a = Cy_delta_a, Cy_delta_r = Cy_delta_r, Cy_p = Cy_p, Cy_r = Cy_r,  b= b, C_bar =C_bar, g = 9.81, m = m, I_xx = I_xx, I_yy = I_yy, I_zz = I_zz, rho = 1.225, S_ref = S_ref, alpha (start= 0.1), omega (start = {0, 0, 0}), pos (start = {0, 0, -100}), V (start =  39.8858)) annotation(    Placement(visible = true, transformation(origin = {41, 9}, extent = {{-27, -27}, {27, 27}}, rotation = 0)));
  
  
Modelica.Blocks.Sources.Constant thrust (k = 1112.82)  annotation(
    Placement(visible = true, transformation(origin = {-46, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    
  Modelica.Blocks.Sources.Constant [3] delta(k = {0, -0.15625, 0})  annotation(
    Placement(visible = true, transformation(origin = {-82, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(wind6DOF1.delta, delta.y) annotation(
    Line(points = {{12, 0}, {-30, 0}, {-30, -12}, {-70, -12}, {-70, -12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(wind6DOF1.thrust, thrust.y) annotation(
    Line(points = {{12, 18}, {-35, 18}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end CessnaTest;