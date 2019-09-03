model FlightPitchHold
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-12, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {64, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 0.005)  annotation(
    Placement(visible = true, transformation(origin = {31, -21}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant pitch_angle(k = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-120, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1112.82) annotation(
    Placement(visible = true, transformation(origin = {24, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-78, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 15, k = 0.2)  annotation(
    Placement(visible = true, transformation(origin = {-44, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold1(samplePeriod = 0.02)  annotation(
    Placement(visible = true, transformation(origin = {22, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold2(samplePeriod = 0.02) annotation(
    Placement(visible = true, transformation(origin = {64, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold3(samplePeriod = 0.02) annotation(
    Placement(visible = true, transformation(origin = {-4, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  connect(flightLongs1.q, zeroOrderHold2.u) annotation(
    Line(points = {{76, 12}, {86, 12}, {86, -20}, {76, -20}}, color = {0, 0, 127}));
  connect(gain2.u, zeroOrderHold2.y) annotation(
    Line(points = {{38, -20}, {53, -20}}, color = {0, 0, 127}));
  connect(flightLongs1.theta, zeroOrderHold3.u) annotation(
    Line(points = {{76, 16}, {96, 16}, {96, -40}, {8, -40}, {8, -40}}, color = {0, 0, 127}));
  connect(feedback2.u2, zeroOrderHold3.y) annotation(
    Line(points = {{-78, 4}, {-78, 4}, {-78, -40}, {-14, -40}, {-14, -40}}, color = {0, 0, 127}));
  connect(feedback1.y, zeroOrderHold1.u) annotation(
    Line(points = {{-2, 12}, {10, 12}, {10, 12}, {10, 12}}, color = {0, 0, 127}));
  connect(zeroOrderHold1.y, flightLongs1.del) annotation(
    Line(points = {{34, 12}, {52, 12}, {52, 12}, {54, 12}}, color = {0, 0, 127}));
  connect(PI.y, feedback1.u1) annotation(
    Line(points = {{-32, 12}, {-22, 12}, {-22, 12}, {-20, 12}}, color = {0, 0, 127}));
  connect(feedback2.y, PI.u) annotation(
    Line(points = {{-68, 12}, {-56, 12}, {-56, 12}, {-56, 12}}, color = {0, 0, 127}));
  connect(pitch_angle.y, feedback2.u1) annotation(
    Line(points = {{-109, 12}, {-99, 12}, {-99, 12}, {-87, 12}, {-87, 12}, {-87, 12}, {-87, 12}, {-87, 12}}, color = {0, 0, 127}));
  connect(gain2.y, feedback1.u2) annotation(
    Line(points = {{25.5, -21}, {-12, -21}, {-12, 4}}, color = {0, 0, 127}));
  connect(const.y, flightLongs1.thrust) annotation(
    Line(points = {{36, 58}, {48, 58}, {48, 19}, {53, 19}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
  
end FlightPitchHold;