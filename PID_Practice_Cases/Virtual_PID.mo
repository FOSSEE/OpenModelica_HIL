//Provides a virtual PID against which the arduino PID can be compared.
class Virtual_PID
    Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-44, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, initType = Modelica.Blocks.Types.Init.NoInit, k = 1, y_start = 1) annotation(
    Placement(visible = true, transformation(origin = {54, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 50, freqHz = 0.5, offset = 150) annotation(
    Placement(visible = true, transformation(origin = {-88, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PID PID(Nd = 0.5, Td = 10 ^ 10, Ti = 10 ^ 10, k = 2) annotation(
    Placement(visible = true, transformation(origin = {0, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(PID.y, firstOrder1.u) annotation(
    Line(points = {{12, 22}, {42, 22}, {42, 22}, {42, 22}}, color = {0, 0, 127}));
  connect(feedback1.y, PID.u) annotation(
    Line(points = {{-34, 22}, {-12, 22}, {-12, 22}, {-12, 22}}, color = {0, 0, 127}));
  connect(sine1.y, feedback1.u1) annotation(
    Line(points = {{-76, 22}, {-52, 22}, {-52, 22}, {-52, 22}}, color = {0, 0, 127}));
  connect(firstOrder1.y, feedback1.u2) annotation(
    Line(points = {{66, 22}, {80, 22}, {80, 0}, {-46, 0}, {-46, 14}, {-44, 14}, {-44, 14}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")),
    Diagram);
end Virtual_PID;