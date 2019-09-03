package Flight_Dynamics
  extends Modelica.Icons.Package;

  class Components
    block ForceMoment_Gen
      import Modelica.Math.Matrices.*;
      import Modelica.SIunits.*;
      import Modelica.Blocks.Interfaces.*;
      import Modelica.Math.Vectors.*;
      parameter Real rho = 1.225;
      parameter Real g = 9.81;
      parameter Real m = 1043.26;
      //1.56 for zagi
      parameter Real S_ref = 16.1651;
      //reference area
      parameter Real C_bar = 1.493;
      //average chord
      parameter Real b = 10.911;
      //span
      parameter Real CD0 = 0.036;
      //= 0.01631;for Zagi
      parameter Real K_drag = 0.0830304;
      //for cessna
      parameter Real CD_beta = 0.17;
      //for cessna
      parameter Real CD_alpha = 0.2108;
      parameter Real CD_q = 0;
      parameter Real CD_delta_e = 0.3045;
      //side force
      parameter Real Cy_beta = -0.31;
      //for cessna
      parameter Real Cy_p = -0.037;
      //for cessna
      parameter Real Cy_r = 0.21;
      //for cessna
      parameter Real Cy_delta_r = 0.187;
      //for cessna
      parameter Real Cy_delta_a = 0;
      //for cessna
      // lift
      parameter Real CL0 = 0.25;
      //for cessna
      parameter Real CL_alpha = 4.47;
      //for cessna
      parameter Real CL_q = 3.9;
      //for cessna
      parameter Real CL_delta_e = 0.3476;
      //for cessna
      // rolling moment
      parameter Real Cl_beta = -0.089;
      //for cessna
      parameter Real Cl_p = -0.47;
      //for cessna
      parameter Real Cl_r = 0.096;
      //for cessna
      parameter Real Cl_delta_a = -0.09;
      //for cessna
      parameter Real Cl_delta_r = 0.0147;
      //for cessna
      // pitching moment
      parameter Real Cm0 = -0.02;
      //for cessna
      parameter Real Cm_alpha = -1.8;
      //for cessna
      parameter Real Cm_q = -12.4;
      //for cessna
      parameter Real Cm_delta_e = -1.28;
      //for cessna
      // yawing moment
      parameter Real Cn_beta = 0.065;
      //for cessna
      parameter Real Cn_p = -0.03;
      //for cessna
      parameter Real Cn_r = -0.99;
      //for cessna
      parameter Real Cn_delta_a = -0.0053;
      //for cessna
      parameter Real Cn_delta_r = -0.0657;
      //for cessna
      Real CL;
      Real CD;
      Real CY;
      Real Cl;
      Real Cm;
      Real Cn;
      Real CX;
      Real CZ;
      RealInput thrust annotation(
        Placement(visible = true, transformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //Thrust force
      RealInput[3] delta annotation(
        Placement(visible = true, transformation(origin = {-110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      RealInput[3] VAB annotation(
        Placement(visible = true, transformation(origin = {-33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      //V, alpha, beta
      RealInput[3] omega annotation(
        Placement(visible = true, transformation(origin = {33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {33, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      //Angular velocities
      Real V = VAB[1];
      Real alpha = VAB[2];
      Real beta = VAB[3];
      Real p = omega[1];
      Real q = omega[2];
      Real r = omega[3];
      Real deltaE = delta[1];
      Real deltaR = delta[2];
      Real deltaA = delta[3];
      Real qbar = 0.5 * rho * V ^ 2;
      RealOutput Force[3] annotation(
        Placement(visible = true, transformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //Thrust force
      RealOutput Moment[3] annotation(
        Placement(visible = true, transformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //Thrust force
    equation
      CL = CL0 + CL_alpha * alpha + CL_q * q * C_bar / (2 * V) + CL_delta_e * deltaE;
//CD =  CD0+CD_alpha*alpha+((CD_q*q*C_bar)/(2*V))+CD_delta_e*abs(deltaE)  ;
      CD = CD0 + K_drag * CL ^ 2;
      CY = Cy_beta * beta + Cy_p * (p * b) / (2 * V) + Cy_r * (r * b) / (2 * V) + Cy_delta_a * deltaA + Cy_delta_r * deltaR;
//Sideslip coeff
      Cl = Cl_beta * beta + Cl_p * (p * b) / (2 * V) + Cl_r * (r * b) / (2 * V) + Cl_delta_a * deltaA + Cl_delta_r * deltaR;
//Rolling coeff
      Cm = Cm0 + Cm_alpha * alpha + Cm_q * q * C_bar / (2 * V) + Cm_delta_e * deltaE;
//pitching coeff
      Cn = Cn_beta * beta + Cn_p * (p * b) / (2 * V) + Cn_r * (r * b) / (2 * V) + Cn_delta_a * deltaA + Cn_delta_r * deltaR;
//Yawing coeff
      CX = (-CD * cos(alpha)) + CL * sin(alpha);
      CZ = (-CD * sin(alpha)) - CL * cos(alpha);
      Force[1] = thrust * cos(alpha) * cos(beta) - 0.5 * rho * V ^ 2 * S_ref * (CD * cos(beta) - CY * sin(beta));
      Force[2] = (-thrust * cos(alpha) * sin(beta)) + 0.5 * rho * V ^ 2 * S_ref * (CY * cos(beta) + CD * sin(beta));
      Force[3] = 0.5 * rho * V ^ 2 * S_ref * CL + thrust * sin(alpha);
      Moment[2] = Cm * qbar * S_ref * C_bar;
      Moment[1] = Cl * qbar * S_ref * b;
      Moment[3] = Cn * qbar * S_ref * b;
    end ForceMoment_Gen;

    block Flight6DOF
      import Modelica.Math.Matrices.*;
      import SI = Modelica.SIunits;
      import Modelica.Blocks.Interfaces.*;
      parameter Real g = 9.81;
      parameter Real m = 1043.26;
      //1.56 for zagi
      parameter Real[3, 3] J = {{1285.31, 0.0, 0.0}, {0.0, 1824.93, 0.0}, {0.0, 0.0, 2666.893}};
      //12 states
      RealOutput[3] omega(start = {0.0, 0.0, 0}) annotation(
        Placement(visible = true, transformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //omega
      Real OMEGA[3, 3] = skew(omega);
      //Skew symmetric matrix form of the angular velocity term
      Real V(start = 39.8858);
      Real alpha(start = 0.1);
      Real beta(start = 0);
      RealOutput[3] VAB annotation(
        Placement(visible = true, transformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //V, alpha, beta
      Real x(start = 0);
      Real y(start = 0);
      Real z(start = 100);
      Real mu(start = 0);
      Real gamma(start = 0);
      Real chi(start = 0);
      RealInput Force[3] annotation(
        Placement(visible = true, transformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //Thrust force
      RealInput Moment[3] annotation(
        Placement(visible = true, transformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -33}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      //Thrust force
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
      omegadot = der(omega);
      xdot = der(x);
      ydot = der(y);
      zdot = der(z);
      mudot = der(mu);
      gammadot = der(gamma);
      chidot = der(chi);
      Vdot = 1 / m * (Force[1] - m * g * sin(gamma));
      alphadot = omega[2] - 1 / cos(beta) * ((omega[1] * cos(alpha) + omega[3] * sin(alpha)) * sin(beta) - g / V * cos(gamma) * cos(mu) + Force[3] / (m * V));
      betadot = omega[1] * sin(alpha) - omega[3] * cos(alpha) + 1 / (m * V) * (Force[2] + m * g * cos(gamma) * sin(mu));
      omegadot = inv(J) * (Moment - OMEGA * J * omega);
      xdot = V * cos(gamma) * cos(chi);
      ydot = V * cos(gamma) * sin(chi);
      zdot = -V * sin(gamma);
      mudot = omega[1] + tan(gamma) * sin(mu) * omega[2] + tan(gamma) * cos(mu) * omega[3];
      gammadot = cos(mu) * omega[2] - sin(mu) * omega[3];
      chidot = 1 / cos(gamma) * sin(mu) * omega[2] + 1 / cos(gamma) * cos(mu) * omega[3];
      VAB = {V, alpha, beta};
      annotation(
        uses(Modelica(version = "3.2.2")));
    end Flight6DOF;
  end Components;

  class Test_Cases
    model CessnaTrim
      // The initial values for delta[2] (elevator), alpha, thrust, and V are obtained by executing Trim_Conditions_Cessna.mo.
      parameter Real m = 1043.26;
      //1.56 for zagi
      parameter Real s = 16.1651;
      //reference area
      parameter Real cbar = 1.493;
      //average chord
      parameter Real b = 10.911;
      //span
      parameter Real W[3] = m * {0, 0, 9.81};
      //gravitational force
      //parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;
      parameter Real CD0 = 0.036;
      //= 0.01631;for Zagi
      parameter Real K_drag = 0.0830304;
      //for cessna
      parameter Real CD_beta = 0.17;
      //for cessna
      parameter Real CD_alpha = 0.2108;
      parameter Real CD_q = 0;
      parameter Real CD_delta_e = 0.3045;
      //side force
      parameter Real Cy_beta = -0.31;
      //for cessna
      parameter Real Cy_p = -0.037;
      //for cessna
      parameter Real Cy_r = 0.21;
      //for cessna
      parameter Real Cy_delta_r = 0.187;
      //for cessna
      parameter Real Cy_delta_a = 0;
      //for cessna
      // lift
      parameter Real CL0 = 0.25;
      //for cessna
      parameter Real CL_alpha = 4.47;
      //for cessna
      parameter Real CL_q = 3.9;
      //for cessna
      parameter Real CL_delta_e = 0.3476;
      //for cessna
      // rolling moment
      parameter Real Cl_beta = -0.089;
      //for cessna
      parameter Real Cl_p = -0.47;
      //for cessna
      parameter Real Cl_r = 0.096;
      //for cessna
      parameter Real Cl_delta_a = -0.09;
      //for cessna
      parameter Real Cl_delta_r = 0.0147;
      //for cessna
      // pitching moment
      parameter Real Cm0 = -0.02;
      //for cessna
      parameter Real Cm_alpha = -1.8;
      //for cessna
      parameter Real Cm_q = -12.4;
      //for cessna
      parameter Real Cm_delta_e = -1.28;
      //for cessna
      // yawing moment
      parameter Real Cn_beta = 0.065;
      //for cessna
      parameter Real Cn_p = -0.03;
      //for cessna
      parameter Real Cn_r = -0.99;
      //for cessna
      parameter Real Cn_delta_a = -0.0053;
      //for cessna
      parameter Real Cn_delta_r = -0.0657;
      //for cessna
      //Initial conditions. (delta[2], thrust[1] and the others are straightforward)
      parameter Real del[3] = {-0.15625, 0, 0};
      parameter Real thrust = 1112.82;
      Flight_Dynamics.Components.ForceMoment_Gen forceMoment_Gen1 annotation(
        Placement(visible = true, transformation(origin = {-18, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Flight_Dynamics.Components.Flight6DOF flight6DOF1 annotation(
        Placement(visible = true, transformation(origin = {24, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(forceMoment_Gen1.omega, flight6DOF1.omega) annotation(
        Line(points = {{-14, 16}, {-14, 16}, {-14, 40}, {62, 40}, {62, 0}, {36, 0}, {36, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(flight6DOF1.VAB, forceMoment_Gen1.VAB) annotation(
        Line(points = {{36, 8}, {42, 8}, {42, 32}, {-20, 32}, {-20, 16}, {-22, 16}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.Moment, flight6DOF1.Moment) annotation(
        Line(points = {{-6, 0}, {12, 0}, {12, 0}, {14, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
        Line(points = {{-6, 8}, {12, 8}, {12, 8}, {14, 8}}, color = {0, 0, 127}, thickness = 0.5));
      forceMoment_Gen1.thrust = thrust;
      forceMoment_Gen1.delta = del;
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-3, Interval = 0.001),
        uses(Modelica(version = "3.2.2")));
    end CessnaTrim;

    model CessnaPerturbed
      //The initial values for delta[2] (elevator), alpha, thrust, and V are obtained by executing Trim_Conditions_Cessna.mo.
      parameter Real m = 1043.26;
      //1.56 for zagi
      parameter Real s = 16.1651;
      //reference area
      parameter Real cbar = 1.493;
      //average chord
      parameter Real b = 10.911;
      //span
      parameter Real W[3] = m * {0, 0, 9.81};
      //gravitational force
      //parameter Real b= 1.4224, cbar = 0.3302,s = 0.2589;
      parameter Real CD0 = 0.036;
      //= 0.01631;for Zagi
      parameter Real K_drag = 0.0830304;
      //for cessna
      parameter Real CD_beta = 0.17;
      //for cessna
      parameter Real CD_alpha = 0.2108;
      parameter Real CD_q = 0;
      parameter Real CD_delta_e = 0.3045;
      //side force
      parameter Real Cy_beta = -0.31;
      //for cessna
      parameter Real Cy_p = -0.037;
      //for cessna
      parameter Real Cy_r = 0.21;
      //for cessna
      parameter Real Cy_delta_r = 0.187;
      //for cessna
      parameter Real Cy_delta_a = 0;
      //for cessna
      // lift
      parameter Real CL0 = 0.25;
      //for cessna
      parameter Real CL_alpha = 4.47;
      //for cessna
      parameter Real CL_q = 3.9;
      //for cessna
      parameter Real CL_delta_e = 0.3476;
      //for cessna
      // rolling moment
      parameter Real Cl_beta = -0.089;
      //for cessna
      parameter Real Cl_p = -0.47;
      //for cessna
      parameter Real Cl_r = 0.096;
      //for cessna
      parameter Real Cl_delta_a = -0.09;
      //for cessna
      parameter Real Cl_delta_r = 0.0147;
      //for cessna
      // pitching moment
      parameter Real Cm0 = -0.02;
      //for cessna
      parameter Real Cm_alpha = -1.8;
      //for cessna
      parameter Real Cm_q = -12.4;
      //for cessna
      parameter Real Cm_delta_e = -1.28;
      //for cessna
      // yawing moment
      parameter Real Cn_beta = 0.065;
      //for cessna
      parameter Real Cn_p = -0.03;
      //for cessna
      parameter Real Cn_r = -0.99;
      //for cessna
      parameter Real Cn_delta_a = -0.0053;
      //for cessna
      parameter Real Cn_delta_r = -0.0657;
      //for cessna
      //Initial conditions. (delta[2], thrust[1] and the others are straightforward)
      parameter Real del[3] = {-0.15625, 0, 0};
      parameter Real thrust = 1112.82;
      Flight_Dynamics.Components.ForceMoment_Gen forceMoment_Gen1 annotation(
        Placement(visible = true, transformation(origin = {-23, 7}, extent = {{-23, -23}, {23, 23}}, rotation = 0)));
      Flight_Dynamics.Components.Flight6DOF flight6DOF1 annotation(
        Placement(visible = true, transformation(origin = {52, 6}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression delta annotation(
        Placement(visible = true, transformation(origin = {-142, -5}, extent = {{-26, -47}, {26, 47}}, rotation = 0)));
    equation
      connect(delta.y, forceMoment_Gen1.delta) annotation(
        Line(points = {{-113, -5}, {-50, -5}, {-50, -4}, {-48, -4}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.omega, flight6DOF1.omega) annotation(
        Line(points = {{-16, 32}, {-16, 32}, {-16, 78}, {98, 78}, {98, -2}, {80, -2}, {80, -2}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.VAB, flight6DOF1.VAB) annotation(
        Line(points = {{-30, 32}, {-28, 32}, {-28, 64}, {84, 64}, {84, 14}, {80, 14}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.Force, flight6DOF1.Force) annotation(
        Line(points = {{2, 14}, {22, 14}, {22, 14}, {24, 14}}, color = {0, 0, 127}, thickness = 0.5));
      connect(forceMoment_Gen1.Moment, flight6DOF1.Moment) annotation(
        Line(points = {{2, 0}, {26, 0}, {26, -2}, {24, -2}}, color = {0, 0, 127}, thickness = 0.5));
      forceMoment_Gen1.thrust = thrust;
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-3, Interval = 0.001),
        uses(Modelica(version = "3.2.2")));
    end CessnaPerturbed;
  end Test_Cases;
  annotation(
    uses(Modelica(version = "3.2.2")));
end Flight_Dynamics;