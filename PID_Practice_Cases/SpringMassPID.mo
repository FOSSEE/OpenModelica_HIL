model SpringMassPID
  import Modelica.Mechanics.*;
  Modelica.Mechanics.Translational.Components.Mass mass1(L = 1, m = 1)  annotation(
    Placement(visible = true, transformation(origin = {36, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Force force1 annotation(
    Placement(visible = true, transformation(origin = {0, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed1 annotation(
    Placement(visible = true, transformation(origin = {94, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Pulse pulse1(amplitude = 158, period = 20, startTime = 10)  annotation(
    Placement(visible = true, transformation(origin = {-88, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor1 annotation(
    Placement(visible = true, transformation(origin = {-12, -56}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper1(c = 100, d = 10, s_rel0 = 1)  annotation(
    Placement(visible = true, transformation(origin = {66, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-61, 5}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.PID PID(Nd = 20, Td = 0.05, Ti = 0.1, k = 50) annotation(
    Placement(visible = true, transformation(origin = {-34, 6}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
equation
  connect(feedback1.y, PID.u) annotation(
    Line(points = {{-56.5, 5}, {-55.25, 5}, {-55.25, 4}, {-44, 4}}, color = {0, 0, 127}));
  connect(PID.y, force1.f) annotation(
    Line(points = {{-25, 4}, {-12, 4}}, color = {0, 0, 127}));
  connect(positionSensor1.s, feedback1.u2) annotation(
    Line(points = {{-23, -56}, {-61, -56}, {-61, 1}}, color = {0, 0, 127}));
  connect(pulse1.y, feedback1.u1) annotation(
    Line(points = {{-76, 4}, {-72, 4}, {-72, 5}, {-65, 5}}, color = {0, 0, 127}));
  connect(mass1.flange_b, positionSensor1.flange) annotation(
    Line(points = {{46, 4}, {46, 4}, {46, -56}, {-2, -56}, {-2, -56}}, color = {0, 127, 0}));
  connect(force1.flange, mass1.flange_a) annotation(
    Line(points = {{10, 4}, {26, 4}}, color = {0, 127, 0}));
  connect(springDamper1.flange_b, fixed1.flange) annotation(
    Line(points = {{76, 4}, {92, 4}, {92, 4}, {94, 4}}, color = {0, 127, 0}));
  connect(mass1.flange_b, springDamper1.flange_a) annotation(
    Line(points = {{46, 4}, {56, 4}, {56, 4}, {56, 4}}, color = {0, 127, 0}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end SpringMassPID;