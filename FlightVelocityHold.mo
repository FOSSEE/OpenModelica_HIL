model FlightVelocityHold
  FlightLongs flightLongs1 annotation(
    Placement(visible = true, transformation(origin = {46, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-70, 16}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 40)  annotation(
    Placement(visible = true, transformation(origin = {-114, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0.15625)  annotation(
    Placement(visible = true, transformation(origin = {6, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI annotation(
    Placement(visible = true, transformation(origin = {-38, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1 annotation(
    Placement(visible = true, transformation(origin = {8, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(PI.y, add1.u1) annotation(
    Line(points = {{-27, 16}, {-4, 16}}, color = {0, 0, 127}));
  connect(feedback1.y, PI.u) annotation(
    Line(points = {{-61, 16}, {-51, 16}}, color = {0, 0, 127}));
  connect(const.y, feedback1.u1) annotation(
    Line(points = {{-103, 16}, {-79, 16}, {-79, 16}, {-79, 16}}, color = {0, 0, 127}));
  connect(feedback1.u2, flightLongs1.V) annotation(
    Line(points = {{-70, 24}, {-70, 44}, {76, 44}, {76, 16}, {58, 16}}, color = {0, 0, 127}));
  connect(add1.y, flightLongs1.thrust) annotation(
    Line(points = {{20, 10}, {32, 10}, {32, 10}, {36, 10}}, color = {0, 0, 127}));
  connect(flightLongs1.del, const1.y) annotation(
    Line(points = {{36, 2}, {28, 2}, {28, -62}, {17, -62}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end FlightVelocityHold;